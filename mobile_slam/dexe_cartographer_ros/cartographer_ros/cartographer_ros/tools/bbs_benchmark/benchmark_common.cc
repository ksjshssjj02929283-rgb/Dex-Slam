/*
 * benchmark_common.cc — 见 benchmark_common.h。
 */

#include "cartographer_ros/tools/bbs_benchmark/benchmark_common.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <sstream>
#include <tuple>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "glog/logging.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace cartographer_ros {
namespace benchmark_common {

namespace {

double HeaderStampToSeconds(const sensor_msgs::msg::LaserScan& scan) {
  return scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
}

// LaserScan 逐点经 3D 外参变换到 tracking frame 后投影 2D，追加到 points。
// 完整 3D 变换可正确处理倒装雷达（外参含 roll≈180°）；不去畸变。
void AppendScanToPointCloud(
    const sensor_msgs::msg::LaserScan& scan,
    const carto::transform::Rigid3d& sensor_to_tracking,
    std::vector<carto::sensor::RangefinderPoint>* points) {
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double angle = scan.angle_min + i * scan.angle_increment;
    const Eigen::Vector3d p =
        sensor_to_tracking *
        Eigen::Vector3d(r * std::cos(angle), r * std::sin(angle), 0.);
    points->push_back({Eigen::Vector3f(p.x(), p.y(), 0.f)});
  }
}

}  // namespace

std::shared_ptr<carto::mapping::Submap2D> LoadGlobalSubmap(
    const std::string& pbstream_path,
    carto::mapping::ValueConversionTables* conversion_tables) {
  LOG(INFO) << "Loading pbstream: " << pbstream_path;
  carto::io::ProtoStreamReader reader(pbstream_path);
  carto::io::ProtoStreamDeserializer deserializer(&reader);

  carto::mapping::proto::SerializedData record;
  while (deserializer.ReadNextSerializedData(&record)) {
    if (record.has_global_static_submap()) {
      return std::make_shared<carto::mapping::Submap2D>(
          record.global_static_submap(), conversion_tables);
    }
    record.Clear();
  }
  LOG(FATAL) << "no global_static_submap field in pbstream: " << pbstream_path;
  return nullptr;
}

GridStats ComputeGridStats(const carto::mapping::ProbabilityGrid& grid) {
  GridStats s;
  const auto& cells = grid.limits().cell_limits();
  s.total = cells.num_x_cells * cells.num_y_cells;
  for (int y = 0; y < cells.num_y_cells; ++y) {
    for (int x = 0; x < cells.num_x_cells; ++x) {
      const Eigen::Array2i idx(x, y);
      if (!grid.IsKnown(idx)) continue;
      ++s.known;
      const float p = grid.GetProbability(idx);
      if (p > kOccupiedProbability) ++s.occupied;
      else if (p < kFreeProbability) ++s.free;
    }
  }
  return s;
}

double RayCast(const carto::mapping::ProbabilityGrid& grid,
               const Eigen::Vector2d& origin, const Eigen::Vector2d& dir,
               const double max_range) {
  const double step = grid.limits().resolution() * 0.5;
  for (double r = step; r <= max_range; r += step) {
    const Eigen::Vector2f pt = (origin + r * dir).cast<float>();
    const Eigen::Array2i idx = grid.limits().GetCellIndex(pt);
    if (!grid.limits().Contains(idx)) return -1.;
    if (grid.IsKnown(idx) && grid.GetProbability(idx) > kOccupiedProbability) {
      return r;
    }
  }
  return -1.;
}

bool SampleFreePose(const carto::mapping::ProbabilityGrid& grid,
                    std::mt19937& rng, carto::transform::Rigid2d* pose) {
  const auto& limits = grid.limits();
  const auto& cells = limits.cell_limits();
  // 世界系包围盒：注意 cartographer 的 x 索引对应世界 -y 方向。
  const double x_max = limits.max().x();
  const double y_max = limits.max().y();
  const double x_min = x_max - limits.resolution() * cells.num_y_cells;
  const double y_min = y_max - limits.resolution() * cells.num_x_cells;
  std::uniform_real_distribution<double> ux(x_min, x_max);
  std::uniform_real_distribution<double> uy(y_min, y_max);
  std::uniform_real_distribution<double> uyaw(-M_PI, M_PI);

  for (int attempt = 0; attempt < 1000; ++attempt) {
    const Eigen::Vector2f pt(ux(rng), uy(rng));
    const Eigen::Array2i idx = limits.GetCellIndex(pt);
    if (!limits.Contains(idx) || !grid.IsKnown(idx)) continue;
    if (grid.GetProbability(idx) >= kFreeProbability) continue;  // 仅自由格
    *pose = carto::transform::Rigid2d(pt.cast<double>(), uyaw(rng));
    return true;
  }
  return false;
}

carto::sensor::PointCloud SynthesizeScan(
    const carto::mapping::ProbabilityGrid& grid,
    const carto::transform::Rigid2d& gt_pose, std::mt19937& rng,
    const SynthConfig& config) {
  std::normal_distribution<double> noise(0., config.range_noise_sigma);
  const double fov = config.fov_deg * M_PI / 180.;
  const double yaw = gt_pose.rotation().angle();
  std::vector<carto::sensor::RangefinderPoint> points;
  points.reserve(config.beams);
  for (int i = 0; i < config.beams; ++i) {
    const double beam_angle = -fov / 2. + fov * i / config.beams;
    const double world_angle = yaw + beam_angle;
    const Eigen::Vector2d dir(std::cos(world_angle), std::sin(world_angle));
    const double r = RayCast(grid, gt_pose.translation(), dir, config.max_range);
    if (r <= 0.) continue;
    const double rn = std::max(0.01, r + noise(rng));
    points.push_back({Eigen::Vector3f(rn * std::cos(beam_angle),
                                      rn * std::sin(beam_angle), 0.f)});
  }
  return carto::sensor::PointCloud(std::move(points));
}

carto::sensor::PointCloud ApplyMergedCloudFilters(
    const carto::sensor::PointCloud& cloud,
    const carto::preprocess::proto::DataPreprocessorOptions& dp_options) {
  carto::sensor::PointCloud out = carto::sensor::AdaptiveVoxelFilter(
      cloud, dp_options.adaptive_voxel_filter_options());
  if (dp_options.voxel_filter_size() > 0.f) {
    out = carto::sensor::VoxelFilter(out, dp_options.voxel_filter_size());
  }
  return out;
}

std::vector<std::string> SplitString(const std::string& s, const char delim) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

carto::transform::Rigid3d ParseExtrinsic(const std::string& s) {
  const auto parts = SplitString(s, ',');
  CHECK_EQ(parts.size(), 6u)
      << "extrinsic must be x,y,z,roll,pitch,yaw (m/deg): " << s;
  std::vector<double> v;
  for (const auto& p : parts) v.push_back(std::stod(p));
  const double d2r = M_PI / 180.;
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(v[5] * d2r, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(v[4] * d2r, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(v[3] * d2r, Eigen::Vector3d::UnitX());
  return carto::transform::Rigid3d(Eigen::Vector3d(v[0], v[1], v[2]), q);
}

std::vector<BagFrame> PrepareBagFrames(
    const BagPrepConfig& config,
    carto::preprocess::DataPreprocessor* preprocessor) {
  CHECK(!config.bag_filename.empty()) << "bag mode requires bag_filename";
  std::vector<std::string> topics = config.topics;
  CHECK(!topics.empty()) << "bag mode requires at least one scan topic";
  const std::string& primary = topics[0];

  rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

  // ===== 第一遍：收集 /tf_static 外参 + 各话题帧时间戳与 frame_id =====
  tf2::BufferCore tf_buffer;
  std::map<std::string, std::vector<double>> stamps;
  std::map<std::string, std::string> frame_ids;
  for (const auto& t : topics) stamps[t] = {};
  {
    rosbag2_cpp::Reader reader;
    reader.open(config.bag_filename);
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (msg->topic_name == "/tf_static") {
        rclcpp::SerializedMessage serialized(*msg->serialized_data);
        tf2_msgs::msg::TFMessage tf_msg;
        tf_serialization.deserialize_message(&serialized, &tf_msg);
        for (const auto& t : tf_msg.transforms) {
          tf_buffer.setTransform(t, "benchmark", true /*is_static*/);
        }
      } else if (stamps.count(msg->topic_name)) {
        rclcpp::SerializedMessage serialized(*msg->serialized_data);
        sensor_msgs::msg::LaserScan scan;
        serialization.deserialize_message(&serialized, &scan);
        stamps[msg->topic_name].push_back(HeaderStampToSeconds(scan));
        if (frame_ids[msg->topic_name].empty()) {
          frame_ids[msg->topic_name] = scan.header.frame_id;
        }
      }
    }
  }
  for (const auto& t : topics) {
    CHECK_GT(stamps[t].size(), 0u) << "topic not found in bag: " << t;
    LOG(INFO) << "topic " << t << ": " << stamps[t].size()
              << " frames, frame_id=" << frame_ids[t];
  }

  // ===== 外参解析：手动覆盖优先，否则从 /tf_static 查 =====
  std::map<std::string, carto::transform::Rigid3d> extrinsics;
  const auto manual = SplitString(config.scan_extrinsics, ';');
  if (!manual.empty()) {
    CHECK_EQ(manual.size(), topics.size())
        << "scan_extrinsics segment count must match topic count";
    for (size_t i = 0; i < topics.size(); ++i) {
      extrinsics[topics[i]] = ParseExtrinsic(manual[i]);
    }
  } else {
    for (const auto& t : topics) {
      try {
        const auto tf = tf_buffer.lookupTransform(
            config.tracking_frame, frame_ids[t], tf2::TimePointZero);
        extrinsics[t] = cartographer_ros::ToRigid3d(tf);
      } catch (const tf2::TransformException& e) {
        LOG(WARNING) << "tf_static missing " << config.tracking_frame << " <- "
                     << frame_ids[t] << " (" << e.what()
                     << "), treating this topic's extrinsic as identity; "
                     << "for multi-lidar check that the bag recorded /tf_static "
                     << "or pass scan_extrinsics manually";
        extrinsics[t] = carto::transform::Rigid3d::Identity();
      }
    }
  }
  for (const auto& t : topics) {
    LOG(INFO) << "extrinsic " << config.tracking_frame << " <- " << frame_ids[t]
              << ": " << extrinsics[t];
  }

  // ===== 主话题抽帧（时间随机 / 固定步长）+ 副话题时间配对 =====
  const auto& primary_stamps = stamps[primary];
  std::vector<size_t> primary_indices;
  if (config.bag_random_samples > 0) {
    std::vector<size_t> all(primary_stamps.size());
    std::iota(all.begin(), all.end(), 0);
    std::mt19937 rng(config.seed);
    std::shuffle(all.begin(), all.end(), rng);
    const size_t k = std::min(
        static_cast<size_t>(config.bag_random_samples), all.size());
    primary_indices.assign(all.begin(), all.begin() + k);
    std::sort(primary_indices.begin(), primary_indices.end());
    LOG(INFO) << "bag random time sampling: " << k << "/"
              << primary_stamps.size() << " frames (primary topic " << primary
              << ", seed=" << config.seed << ")";
  } else {
    for (size_t i = 0; i < primary_stamps.size() &&
                       primary_indices.size() <
                           static_cast<size_t>(config.max_bag_scans);
         i += std::max(1, config.bag_stride)) {
      primary_indices.push_back(i);
    }
  }

  struct PairedSample {
    std::map<std::string, size_t> frame_index;
    double t_rel;
  };
  std::vector<PairedSample> samples;
  std::map<std::string, std::set<size_t>> needed;
  for (const size_t pi : primary_indices) {
    PairedSample sample;
    sample.frame_index[primary] = pi;
    sample.t_rel = primary_stamps[pi] - primary_stamps.front();
    bool ok = true;
    for (size_t ti = 1; ti < topics.size(); ++ti) {
      const auto& s = stamps[topics[ti]];
      const auto it = std::lower_bound(s.begin(), s.end(), primary_stamps[pi]);
      size_t best = std::min<size_t>(it - s.begin(), s.size() - 1);
      if (best > 0 && std::abs(s[best - 1] - primary_stamps[pi]) <
                          std::abs(s[best] - primary_stamps[pi])) {
        --best;
      }
      if (std::abs(s[best] - primary_stamps[pi]) > config.pair_tolerance) {
        LOG(WARNING) << "primary frame #" << pi << " has no paired frame on "
                     << topics[ti] << " within " << config.pair_tolerance
                     << "s, sample skipped";
        ok = false;
        break;
      }
      sample.frame_index[topics[ti]] = best;
    }
    if (!ok) continue;
    for (const auto& [topic, idx] : sample.frame_index) needed[topic].insert(idx);
    samples.push_back(sample);
  }
  CHECK_GT(samples.size(), 0u)
      << "no usable samples (check topic names and pair tolerance)";

  // ===== 第二遍：只反序列化被选中的帧 =====
  std::map<std::string, std::map<size_t, sensor_msgs::msg::LaserScan>> stored;
  {
    rosbag2_cpp::Reader reader;
    reader.open(config.bag_filename);
    std::map<std::string, size_t> counters;
    while (reader.has_next()) {
      auto msg = reader.read_next();
      if (!stamps.count(msg->topic_name)) continue;
      const size_t idx = counters[msg->topic_name]++;
      if (!needed[msg->topic_name].count(idx)) continue;
      rclcpp::SerializedMessage serialized(*msg->serialized_data);
      serialization.deserialize_message(&serialized,
                                        &stored[msg->topic_name][idx]);
    }
  }

  // ===== 各话题点云经外参合并为一帧 =====
  // preprocessor != nullptr：组装 TimedPointCloudData 走线上 DataPreprocessor
  //（与 pose_estimator_2d 的 ProcessMultipleSensorData 同一份代码），取
  //  merged_filtered_cloud + merged_line_data——即 PoseInitializer 的真实输入。
  // nullptr：简化合并（无 range 裁剪/体素滤波），仅作快速测试。
  std::vector<BagFrame> frames;
  for (const auto& sample : samples) {
    BagFrame frame;
    frame.primary_frame_index = sample.frame_index.at(primary);
    frame.t_rel = sample.t_rel;
    if (preprocessor != nullptr) {
      std::map<std::string, carto::sensor::TimedPointCloudData> laser_data_map;
      carto::common::Time sync_time;
      for (const auto& [topic, idx] : sample.frame_index) {
        carto::sensor::PointCloudWithIntensities pcwi;
        carto::common::Time time;
        std::tie(pcwi, time) =
            cartographer_ros::ToPointCloudWithIntensities(stored[topic][idx]);
        const auto& T = extrinsics[topic];
        laser_data_map[topic] = carto::sensor::TimedPointCloudData{
            time, T.translation().cast<float>(), T,
            carto::sensor::TransformTimedPointCloud(pcwi.points, T.cast<float>()),
            pcwi.intensities};
        if (topic == primary) sync_time = time;
      }
      const auto merged_data =
          preprocessor->ProcessMultipleSensorData(laser_data_map, sync_time);
      if (merged_data.IsEmpty()) {
        LOG(WARNING) << "primary frame #" << frame.primary_frame_index
                     << " preprocessing output empty, sample skipped";
        continue;
      }
      frame.cloud = merged_data.merged_filtered_cloud;
      frame.line_data = merged_data.merged_line_data;
    } else {
      std::vector<carto::sensor::RangefinderPoint> merged;
      for (const auto& [topic, idx] : sample.frame_index) {
        AppendScanToPointCloud(stored[topic][idx], extrinsics[topic], &merged);
      }
      frame.cloud = carto::sensor::PointCloud(std::move(merged));
    }
    if (frame.cloud.empty()) continue;
    frames.push_back(std::move(frame));
  }
  CHECK_GT(frames.size(), 0u) << "no valid scan frames read from bag";
  return frames;
}

double Percentile(std::vector<double> v, const double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  const size_t idx =
      std::min(v.size() - 1, static_cast<size_t>(p / 100. * v.size()));
  return v[idx];
}

double Mean(const std::vector<double>& v) {
  if (v.empty()) return NAN;
  return std::accumulate(v.begin(), v.end(), 0.) / v.size();
}

}  // namespace benchmark_common
}  // namespace cartographer_ros
