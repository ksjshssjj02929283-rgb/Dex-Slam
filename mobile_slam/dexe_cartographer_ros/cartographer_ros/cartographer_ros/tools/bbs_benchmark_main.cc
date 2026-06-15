/*
 * bbs_benchmark_main.cc
 *
 * 2D 全局初始定位（BBS / MatchFullSubmap）离线基准测试工具。
 *
 * 加载与线上完全一致的地图（pbstream 的 global_static_submap 字段），
 * 用 instrumented 副本（见 bbs_benchmark/）执行全图分支定界搜索，
 * 输出各阶段耗时与搜索树规模统计。库代码零改动。
 *
 * 默认即【线上对齐模式】：自动加载本包 configuration_files/dexe_localization_2d.lua
 * （ament share 目录解析），min_score / branch_and_bound_depth / 预处理参数全部
 * 取自 lua，bag 模式复用线上 DataPreprocessor（双雷达融合 + AdaptiveVoxelFilter）。
 * 默认双雷达 /laser_scan/front + /laser_scan/rear，tracking_frame=base_footprint。
 *
 * 两种 scan 来源：
 *   --mode synth  从地图自由空间随机撒位姿，ray-cast 合成 scan（带真值，
 *                 可批量统计成功率与位姿误差）
 *   --mode bag    回放 rosbag2 中的 LaserScan（真实数据，测真实耗时分布）
 *   --mode both   两者都跑
 *
 * 输出：~/.data/localization/bbs_baseline/<地图名>_<时间戳>_<模式>.txt
 * （总结在前，每次测试的明细 CSV 附在文件末尾；目录可用 --output_dir 覆盖）
 *
 * 用法示例：
 *   ros2 run cartographer_ros bbs_benchmark_tool --map_name <地图名> \
 *     --mode synth --num_poses 50
 *
 *   ros2 run cartographer_ros bbs_benchmark_tool --map_name <地图名> \
 *     --mode bag --bag_filename <bag目录> --bag_random_samples 15
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/preprocess/data_preprocessor_options.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include "tf2_msgs/msg/tf_message.hpp"

#include "cartographer/mapping/internal/2d/scan_matching/batch_fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/gpu_batch_fast_correlative_scan_matcher_2d.h"
#include "cartographer_ros/tools/bbs_benchmark/instrumented_fast_correlative_scan_matcher_2d.h"

// --- 地图与匹配参数 ---
DEFINE_string(map_name, "", "地图名称，解析为 <maps_root>/<map_name>/map.pbstream");
DEFINE_string(maps_root, "", "地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(pbstream_filename, "", "直接指定 pbstream 文件路径（优先于 map_name）");
DEFINE_string(configuration_directory, "",
              "线上 lua 配置目录覆盖。默认自动解析本包 share 目录的 "
              "configuration_files（线上对齐模式始终启用：min_score、"
              "branch_and_bound_depth、预处理参数全部取自 lua，并复用线上 "
              "DataPreprocessor，强制 use_odometry=false 不去畸变）");
DEFINE_string(configuration_basename, "dexe_localization_2d.lua",
              "线上定位 lua 文件名");
DEFINE_double(min_score, 0.5,
              "全局定位最低得分阈值（lua 模式下被 lua 值覆盖）");
DEFINE_int32(branch_and_bound_depth, 7,
             "BBS 金字塔层数（lua 模式下被 lua 值覆盖）");
DEFINE_int32(pyramid_rebuilds, 3, "金字塔重复构建次数（测构建耗时稳定性）");

// --- matcher 版本（A/B 对照）---
DEFINE_string(matcher, "v0",
              "v0=原生 BBS（instrumented 副本）| v1=Phase A CPU 加速版"
              "（best-first + 攒批 OpenMP 并行）| v1.1=v1+角度金字塔分支"
              "（2D roto-translational branching，近似上界）| v2=Phase B GPU"
              "（best-first + CUDA 攒批 kernel，需 BUILD_GPU_MATCHER=ON）");
DEFINE_int32(num_threads, 4, "v1 并行打分线程数");
DEFINE_int32(batch_size, 0,
             "攒批大小；0=按版本默认（v1: 8192, v2: 32768）");
DEFINE_double(timeout_ms, 0., "v1/v2 超时（ms，<=0 不限时，超时返回当前最优）");

// --- 模式 ---
DEFINE_string(mode, "synth", "synth | bag | both");

// --- 合成模式参数 ---
DEFINE_int32(num_poses, 50, "合成模式：随机撒位姿数量");
DEFINE_uint64(seed, 42, "随机种子（保证可复现）");
DEFINE_int32(beams, 2400, "合成 scan 束数（按真实雷达角分辨率配置）");
DEFINE_double(fov_deg, 360.0, "合成 scan 视场角（度）");
DEFINE_double(max_range, 20.0, "合成 scan 最大量程（米）");
DEFINE_double(range_noise_sigma, 0.01, "合成 range 高斯噪声 σ（米）");
DEFINE_double(min_valid_beam_ratio, 0.3,
              "有效回波束数低于 beams*ratio 的采样位姿丢弃（如空旷区）");
DEFINE_double(success_trans_tol, 0.2, "成功判定：平移误差阈值（米）");
DEFINE_double(success_yaw_tol_deg, 3.0, "成功判定：角度误差阈值（度）");

// --- bag 模式参数 ---
DEFINE_string(bag_filename, "", "rosbag2 目录路径");
DEFINE_string(scan_topic, "/scan", "单雷达 LaserScan 话题名（scan_topics 置空时使用）");
DEFINE_string(scan_topics, "/laser_scan/front,/laser_scan/rear",
              "多雷达话题，逗号分隔（默认 W1 双雷达）。"
              "各雷达按外参组合到 tracking_frame 后合并为一帧再匹配（不去畸变）。"
              "第一个话题为主雷达（抽帧基准）；置空退化为 --scan_topic 单雷达");
DEFINE_string(tracking_frame, "base_footprint",
              "多雷达合并的目标坐标系（与线上 lua options.tracking_frame 一致）");
DEFINE_string(scan_extrinsics, "",
              "手动外参覆盖，按 scan_topics 顺序：\"x,y,z,roll,pitch,yaw;...\""
              "（米/度）。留空则从 bag 的 /tf_static 自动解析 frame_id → "
              "tracking_frame 变换");
DEFINE_double(pair_tolerance, 0.1,
              "副雷达与主雷达帧时间配对最大容差（秒），超差样本跳过");
DEFINE_int32(bag_stride, 20, "每 N 帧取 1 帧（bag_random_samples>0 时无效）");
DEFINE_int32(max_bag_scans, 30, "stride 抽帧模式最多测试帧数");
DEFINE_int32(bag_random_samples, 0,
             ">0 时启用时间随机抽帧：从整个 bag 时间轴无放回随机抽取该数量的帧"
             "（用 --seed 复现），替代 stride 模式");

// --- A4 自检 ---
DEFINE_bool(selftest_pyramid, false,
            "A4 金字塔序列化往返自检：构建→ToProto→FromProto→逐格 GetValue 比对，"
            "验证 pbstream 固化的金字塔与现场构建逐位一致");

// --- 输出 ---
DEFINE_string(output_dir, "",
              "报告输出目录，留空则使用 ~/.data/localization/bbs_baseline/。"
              "每个模式输出一个文件：<地图名>_<时间戳>_<模式>.txt"
              "（总结在前，每次测试明细 CSV 附在末尾）");

namespace {

namespace carto = ::cartographer;
using ::cartographer_ros::bbs_benchmark::BbsProfile;
using ::cartographer_ros::bbs_benchmark::InstrumentedFastCorrelativeScanMatcher2D;

constexpr float kOccupiedProbability = 0.65f;
constexpr float kFreeProbability = 0.196f;

// 单次测量记录（合成模式带真值，bag 模式 gt/err 字段为 NaN）。
struct RunRecord {
  std::string mode;
  int id = 0;
  double gt_x = NAN, gt_y = NAN, gt_yaw = NAN;
  double est_x = NAN, est_y = NAN, est_yaw = NAN;
  double trans_err = NAN, yaw_err = NAN;
  bool success = false;  // 合成模式：matched 且误差在阈值内；bag 模式：=matched
  BbsProfile profile;
};

std::string ResolvePbstreamPath() {
  if (!FLAGS_pbstream_filename.empty()) return FLAGS_pbstream_filename;
  CHECK(!FLAGS_map_name.empty())
      << "需要 --map_name 或 --pbstream_filename 指定地图";
  std::string root = FLAGS_maps_root;
  if (root.empty()) {
    const char* home = std::getenv("HOME");
    root = std::string(home ? home : "/tmp") + "/.dexe/data/maps";
  }
  return root + "/" + FLAGS_map_name + "/map.pbstream";
}

// 从 pbstream 读取 global_static_submap，构建与线上一致的 Submap2D。
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
  LOG(FATAL) << "no global_static_submap field in pbstream: "
             << pbstream_path;
  return nullptr;
}

struct GridStats {
  int known = 0, free = 0, occupied = 0, total = 0;
};

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

// 沿世界系方向 ray-march，命中占据格返回距离，否则返回 -1（无回波）。
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

// 在地图自由空间随机采样一个位姿（真值）。
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

// 给定真值位姿，ray-cast 合成传感器系 scan。
carto::sensor::PointCloud SynthesizeScan(
    const carto::mapping::ProbabilityGrid& grid,
    const carto::transform::Rigid2d& gt_pose, std::mt19937& rng) {
  std::normal_distribution<double> noise(0., FLAGS_range_noise_sigma);
  const double fov = FLAGS_fov_deg * M_PI / 180.;
  const double yaw = gt_pose.rotation().angle();
  std::vector<carto::sensor::RangefinderPoint> points;
  points.reserve(FLAGS_beams);
  for (int i = 0; i < FLAGS_beams; ++i) {
    const double beam_angle = -fov / 2. + fov * i / FLAGS_beams;
    const double world_angle = yaw + beam_angle;
    const Eigen::Vector2d dir(std::cos(world_angle), std::sin(world_angle));
    const double r = RayCast(grid, gt_pose.translation(), dir, FLAGS_max_range);
    if (r <= 0.) continue;
    const double rn = std::max(0.01, r + noise(rng));
    points.push_back({Eigen::Vector3f(rn * std::cos(beam_angle),
                                      rn * std::sin(beam_angle), 0.f)});
  }
  return carto::sensor::PointCloud(std::move(points));
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

std::vector<std::string> SplitString(const std::string& s, const char delim) {
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

// "x,y,z,roll,pitch,yaw"（米/度）→ Rigid3d
carto::transform::Rigid3d ParseExtrinsic(const std::string& s) {
  const auto parts = SplitString(s, ',');
  CHECK_EQ(parts.size(), 6u)
      << "外参格式应为 x,y,z,roll,pitch,yaw（米/度）: " << s;
  std::vector<double> v;
  for (const auto& p : parts) v.push_back(std::stod(p));
  const double d2r = M_PI / 180.;
  const Eigen::Quaterniond q =
      Eigen::AngleAxisd(v[5] * d2r, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(v[4] * d2r, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(v[3] * d2r, Eigen::Vector3d::UnitX());
  return carto::transform::Rigid3d(Eigen::Vector3d(v[0], v[1], v[2]), q);
}

double HeaderStampToSeconds(const sensor_msgs::msg::LaserScan& scan) {
  return scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9;
}

// 线上对齐模式：从线上 lua 取 BBS 参数 + 预处理参数，复用线上 DataPreprocessor，
// 使 benchmark 输入与 pose_estimator_2d → pose_initializer 的真实输入一致。
struct LuaPipeline {
  bool enabled = false;
  carto::preprocess::proto::DataPreprocessorOptions dp_options;
  carto::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
      fcsm_options;
  double min_score = 0.;
};

LuaPipeline LoadLuaPipeline() {
  LuaPipeline p;
  if (FLAGS_configuration_basename.empty()) {
    LOG(WARNING) << "--configuration_basename empty, online-alignment mode off: "
                 << "using flag params + simplified preprocessing (differs from "
                 << "online input, for quick debugging only)";
    return p;
  }
  // 默认从本包 ament share 目录解析 lua（安装后即 share/cartographer_ros/
  // configuration_files），无需手动传配置路径。
  std::string config_dir = FLAGS_configuration_directory;
  if (config_dir.empty()) {
    config_dir = ament_index_cpp::get_package_share_directory(
                     "cartographer_ros") +
                 "/configuration_files";
    LOG(INFO) << "lua config dir (auto-resolved): " << config_dir;
  }
  auto resolver =
      absl::make_unique<carto::common::ConfigurationFileResolver>(
          std::vector<std::string>{config_dir});
  const std::string code =
      "include \"" + FLAGS_configuration_basename + "\"\n" +
      "return { data_preprocessor = POSE_ESTIMATOR_2D.data_preprocessor, " +
      "pose_initializer = POSE_INITIALIZER_2D }";
  auto lua = carto::common::LuaParameterDictionary::NonReferenceCounted(
      code, std::move(resolver));
  p.dp_options = carto::preprocess::CreateDataPreprocessorOptions(
      lua->GetDictionary("data_preprocessor").get());
  auto pi = lua->GetDictionary("pose_initializer");
  p.min_score = pi->GetDouble("global_localization_min_score");
  p.fcsm_options =
      carto::mapping::scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
          pi->GetDictionary("fast_correlative_scan_matcher").get());
  // 离线基准约束：无里程计 → 关去畸变（数据输入约定为"不去畸变"）；
  // line 特征只服务 ceres 精匹配阶段，不影响 BBS 输入的 merged_filtered_cloud。
  p.dp_options.set_use_odometry(false);
  p.dp_options.set_use_deskew(false);
  p.dp_options.set_enable_feature_detection(false);
  p.enabled = true;
  LOG(INFO) << "online-alignment mode enabled: min_score=" << p.min_score
            << " branch_and_bound_depth="
            << p.fcsm_options.branch_and_bound_depth()
            << " preprocess min/max_range=" << p.dp_options.min_range() << "/"
            << p.dp_options.max_range()
            << " adaptive_voxel(max_length="
            << p.dp_options.adaptive_voxel_filter_options().max_length()
            << ", min_num_points="
            << p.dp_options.adaptive_voxel_filter_options().min_num_points()
            << ") (flag min_score/depth overridden by lua)";
  return p;
}

// 复刻 GetMergedData 对 merged_filtered_cloud 的滤波（用于合成模式对齐）。
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

// matcher 版本适配：v0 = 原生 instrumented 副本；v1 = Phase A 加速版。
// v1 的 BatchMatchStats 映射到 BbsProfile，两版本共用同一套统计与报告。
struct MatcherHolder {
  std::unique_ptr<InstrumentedFastCorrelativeScanMatcher2D> v0;
  std::unique_ptr<
      carto::mapping::scan_matching::BatchFastCorrelativeScanMatcher2D>
      v1;
  std::unique_ptr<
      carto::mapping::scan_matching::GpuBatchFastCorrelativeScanMatcher2D>
      v2;

  double precomputation_ms() const {
    if (v0) return v0->precomputation_ms();
    if (v1) return v1->precomputation_ms();
    return v2->precomputation_ms();
  }

  bool Match(const carto::sensor::PointCloud& cloud, const float min_score,
             float* score, carto::transform::Rigid2d* pose,
             BbsProfile* profile) const {
    if (v0) {
      return v0->MatchFullSubmap(cloud, min_score, score, pose, profile);
    }
    carto::mapping::scan_matching::BatchMatchStats stats;
    const bool ok =
        v1 ? v1->MatchFullSubmap(cloud, min_score, score, pose, &stats)
           : v2->MatchFullSubmap(cloud, min_score, score, pose, &stats);
    profile->t_rotate_ms = stats.t_rotate_ms;
    profile->t_discretize_ms = stats.t_discretize_ms;
    profile->t_gen_lowres_ms = stats.t_gen_lowres_ms;
    profile->t_score_lowres_ms = stats.t_score_lowres_ms;
    profile->t_bnb_ms = stats.t_search_ms;
    profile->t_total_ms = stats.t_total_ms;
    profile->num_rotated_scans = stats.num_rotated_scans;
    profile->points_per_scan = stats.points_per_scan;
    profile->num_initial_candidates = stats.num_initial_candidates;
    profile->num_expanded_nodes = stats.num_expanded_nodes;
    profile->num_scored_candidates = stats.num_scored_candidates;
    profile->num_point_lookups =
        stats.num_scored_candidates * stats.points_per_scan;
    profile->scored_per_depth = stats.scored_per_depth;
    profile->best_score = stats.best_score;
    profile->matched = stats.matched;
    if (stats.timed_out) LOG(WARNING) << "v1/v2 match timed out, returning current best";
    return ok;
  }
};

RunRecord RunMatch(const MatcherHolder& matcher,
                   const carto::sensor::PointCloud& cloud,
                   const float min_score, const std::string& mode,
                   const int id) {
  RunRecord rec;
  rec.mode = mode;
  rec.id = id;
  float score = 0.f;
  carto::transform::Rigid2d pose_estimate;
  matcher.Match(cloud, min_score, &score, &pose_estimate, &rec.profile);
  if (rec.profile.matched) {
    rec.est_x = pose_estimate.translation().x();
    rec.est_y = pose_estimate.translation().y();
    rec.est_yaw = pose_estimate.rotation().angle();
  }
  return rec;
}

double Percentile(std::vector<double> v, const double p) {
  if (v.empty()) return NAN;
  std::sort(v.begin(), v.end());
  const size_t idx = std::min(v.size() - 1,
                              static_cast<size_t>(p / 100. * v.size()));
  return v[idx];
}

double Mean(const std::vector<double>& v) {
  if (v.empty()) return NAN;
  return std::accumulate(v.begin(), v.end(), 0.) / v.size();
}

// 生成单模式统计汇总文本（终端与报告文件共用）。无该模式数据返回空串。
std::string BuildModeSummary(const std::vector<RunRecord>& records,
                             const std::string& mode) {
  std::vector<const RunRecord*> sel;
  for (const auto& r : records) {
    if (r.mode == mode) sel.push_back(&r);
  }
  if (sel.empty()) return "";
  std::ostringstream oss;

  auto collect = [&](auto getter) {
    std::vector<double> v;
    v.reserve(sel.size());
    for (const auto* r : sel) v.push_back(getter(*r));
    return v;
  };

  const auto t_total = collect([](const RunRecord& r) { return r.profile.t_total_ms; });
  const auto t_rotate = collect([](const RunRecord& r) { return r.profile.t_rotate_ms; });
  const auto t_disc = collect([](const RunRecord& r) { return r.profile.t_discretize_ms; });
  const auto t_gen = collect([](const RunRecord& r) { return r.profile.t_gen_lowres_ms; });
  const auto t_low = collect([](const RunRecord& r) { return r.profile.t_score_lowres_ms; });
  const auto t_bnb = collect([](const RunRecord& r) { return r.profile.t_bnb_ms; });

  const double mean_total = Mean(t_total);
  auto pct = [&](const std::vector<double>& v) { return Mean(v) / mean_total * 100.; };

  oss << "===== [" << mode << "] 模式汇总（" << sel.size() << " 次）=====\n";
  oss << "  MatchFullSubmap 总耗时 ms: mean=" << mean_total
      << " median=" << Percentile(t_total, 50)
      << " p95=" << Percentile(t_total, 95)
      << " max=" << Percentile(t_total, 100) << "\n";
  oss << "  阶段均值 ms（占比）:\n";
  oss << "    旋转scan生成 : " << Mean(t_rotate) << " (" << pct(t_rotate) << "%)\n";
  oss << "    离散化+收紧  : " << Mean(t_disc) << " (" << pct(t_disc) << "%)\n";
  oss << "    初始候选生成 : " << Mean(t_gen) << " (" << pct(t_gen) << "%)\n";
  oss << "    初始候选打分 : " << Mean(t_low) << " (" << pct(t_low) << "%)\n";
  oss << "    分支定界搜索 : " << Mean(t_bnb) << " (" << pct(t_bnb) << "%)\n";

  const auto n_init = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.num_initial_candidates);
  });
  const auto n_scored = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.num_scored_candidates);
  });
  const auto n_expanded = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.num_expanded_nodes);
  });
  const auto n_lookups = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.num_point_lookups);
  });
  const auto n_angles = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.num_rotated_scans);
  });
  const auto n_points = collect([](const RunRecord& r) {
    return static_cast<double>(r.profile.points_per_scan);
  });
  oss << "  搜索规模均值: 角度假设=" << Mean(n_angles)
      << " 点数/帧=" << Mean(n_points)
      << " 初始候选=" << Mean(n_init)
      << " 打分候选总数=" << Mean(n_scored)
      << " 展开节点=" << Mean(n_expanded)
      << " 逐点查表=" << Mean(n_lookups) << "\n";

  // 各层打分候选分布（GPU 批量可行性的关键数据）。
  if (!sel.empty()) {
    const size_t depths = sel[0]->profile.scored_per_depth.size();
    oss << "  各深度层打分候选均值: ";
    for (size_t d = 0; d < depths; ++d) {
      double sum = 0;
      for (const auto* r : sel) sum += r->profile.scored_per_depth[d];
      oss << "L" << d << "=" << static_cast<long>(sum / sel.size()) << " ";
    }
    oss << "\n";
  }

  int matched = 0, success = 0;
  std::vector<double> terr, yerr;
  for (const auto* r : sel) {
    if (r->profile.matched) ++matched;
    if (r->success) ++success;
    if (r->success && std::isfinite(r->trans_err)) {
      terr.push_back(r->trans_err);
      yerr.push_back(r->yaw_err);
    }
  }
  oss << "  匹配率(score>min): " << matched << "/" << sel.size() << "\n";
  if (mode == "synth") {
    oss << "  成功率(误差阈值内): " << success << "/" << sel.size() << "\n";
    oss << "  成功样本误差: 平移 mean=" << Mean(terr)
        << " m / 角度 mean=" << Mean(yerr) * 180. / M_PI << " deg\n";
  }
  return oss.str();
}

// 单模式报告文件：总结在前，每次测试的明细 CSV 附在末尾。
void WriteReport(const std::string& path, const std::string& header,
                 const std::string& summary,
                 const std::vector<RunRecord>& records,
                 const std::string& mode) {
  std::ofstream out(path);
  CHECK(out.is_open()) << "无法写入报告: " << path;
  out << header << "\n" << summary << "\n";
  out << "===== 每次测试明细（CSV）=====\n";
  out << "mode,id,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,trans_err,yaw_err,"
         "score,matched,success,t_total_ms,t_rotate_ms,t_discretize_ms,"
         "t_gen_lowres_ms,t_score_lowres_ms,t_bnb_ms,n_rotated_scans,"
         "n_points,n_init_cand,n_expanded,n_scored,n_point_lookups\n";
  size_t rows = 0;
  for (const auto& r : records) {
    if (r.mode != mode) continue;
    out << r.mode << "," << r.id << "," << r.gt_x << "," << r.gt_y << ","
        << r.gt_yaw << "," << r.est_x << "," << r.est_y << "," << r.est_yaw
        << "," << r.trans_err << "," << r.yaw_err << "," << r.profile.best_score
        << "," << r.profile.matched << "," << r.success << ","
        << r.profile.t_total_ms << "," << r.profile.t_rotate_ms << ","
        << r.profile.t_discretize_ms << "," << r.profile.t_gen_lowres_ms << ","
        << r.profile.t_score_lowres_ms << "," << r.profile.t_bnb_ms << ","
        << r.profile.num_rotated_scans << "," << r.profile.points_per_scan
        << "," << r.profile.num_initial_candidates << ","
        << r.profile.num_expanded_nodes << "," << r.profile.num_scored_candidates
        << "," << r.profile.num_point_lookups << "\n";
    ++rows;
  }
  LOG(INFO) << "report written: " << path << " (" << rows << " tests)";
}

void Run() {
  const std::string pbstream_path = ResolvePbstreamPath();

  carto::mapping::ValueConversionTables conversion_tables;
  const auto submap = LoadGlobalSubmap(pbstream_path, &conversion_tables);
  const auto* grid =
      dynamic_cast<const carto::mapping::ProbabilityGrid*>(submap->grid());
  CHECK(grid != nullptr) << "global_static_submap grid is not a ProbabilityGrid";

  if (FLAGS_selftest_pyramid) {
    carto::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D o;
    o.set_linear_search_window(7.);
    o.set_angular_search_window(M_PI / 6.);
    o.set_branch_and_bound_depth(FLAGS_branch_and_bound_depth);
    namespace sm = carto::mapping::scan_matching;
    sm::PrecomputationGridStack2D built(*grid, o);
    sm::PrecomputationGridStack2D loaded(built.ToProto());
    CHECK_EQ(built.num_levels(), loaded.num_levels());
    long long checked = 0, mismatch = 0;
    for (int d = 0; d < built.num_levels(); ++d) {
      const auto& g1 = built.Get(d);
      const auto& g2 = loaded.Get(d);
      const auto& wl = g1.wide_limits();
      const auto off = g1.offset();
      for (int y = 0; y < wl.num_y_cells; ++y) {
        for (int x = 0; x < wl.num_x_cells; ++x) {
          const Eigen::Array2i idx(off.x() + x, off.y() + y);
          if (g1.GetValue(idx) != g2.GetValue(idx)) ++mismatch;
          ++checked;
        }
      }
    }
    LOG(INFO) << "===== A4 pyramid serialization round-trip selftest =====";
    LOG(INFO) << "  levels: " << built.num_levels()
              << " | cells compared: " << checked << " | mismatches: " << mismatch;
    LOG(INFO) << (mismatch == 0
                      ? "  [OK] bit-identical: persisted pyramid == on-site build"
                      : "  [FAIL] mismatches found!");
    return;
  }

  const auto& cells = grid->limits().cell_limits();
  const GridStats stats = ComputeGridStats(*grid);
  LOG(INFO) << "===== map info =====";
  LOG(INFO) << "  grid: " << cells.num_x_cells << " x " << cells.num_y_cells
            << " @ " << grid->limits().resolution() << " m/cell";
  LOG(INFO) << "  physical size: " << cells.num_y_cells * grid->limits().resolution()
            << " x " << cells.num_x_cells * grid->limits().resolution() << " m";
  LOG(INFO) << "  cell stats: total=" << stats.total << " known=" << stats.known
            << " free=" << stats.free << " occupied=" << stats.occupied;

  // 线上对齐模式：BBS/预处理参数全部取自线上 lua。
  const LuaPipeline lua = LoadLuaPipeline();
  const float min_score = lua.enabled ? lua.min_score : FLAGS_min_score;
  std::unique_ptr<carto::preprocess::DataPreprocessor> preprocessor;
  if (lua.enabled) {
    preprocessor =
        absl::make_unique<carto::preprocess::DataPreprocessor>(lua.dp_options);
  }

  // 金字塔构建耗时（首次全局定位的隐藏成本，懒构造发生在线上首次调用时）。
  carto::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
      options;
  if (lua.enabled) {
    options = lua.fcsm_options;
  } else {
    // MatchFullSubmap 内部用 1e6 cells 窗口覆盖全图，这两个值仅影响 Match()，
    // 此处按上游默认填写。
    options.set_linear_search_window(7.);
    options.set_angular_search_window(M_PI / 6.);
    options.set_branch_and_bound_depth(FLAGS_branch_and_bound_depth);
  }

  CHECK(FLAGS_matcher == "v0" || FLAGS_matcher == "v1" ||
        FLAGS_matcher == "v1.1" || FLAGS_matcher == "v2" ||
        FLAGS_matcher == "v2.1")
      << "--matcher only supports v0 | v1 | v1.1 | v2 | v2.1";
  MatcherHolder matcher;
  LOG(INFO) << "===== pyramid build (matcher=" << FLAGS_matcher
            << ", branch_and_bound_depth=" << options.branch_and_bound_depth()
            << ") =====";
  for (int i = 0; i < std::max(1, FLAGS_pyramid_rebuilds); ++i) {
    if (FLAGS_matcher == "v0") {
      matcher.v0 = std::make_unique<InstrumentedFastCorrelativeScanMatcher2D>(
          *grid, options);
    } else if (FLAGS_matcher == "v1" || FLAGS_matcher == "v1.1") {
      matcher.v1 = std::make_unique<
          carto::mapping::scan_matching::BatchFastCorrelativeScanMatcher2D>(
          *grid, options);
      matcher.v1->set_num_threads(FLAGS_num_threads);
      if (FLAGS_batch_size > 0) matcher.v1->set_batch_size(FLAGS_batch_size);
      matcher.v1->set_timeout_ms(FLAGS_timeout_ms);
      matcher.v1->set_use_angular_branching(FLAGS_matcher == "v1.1");
    } else {
      matcher.v2 = std::make_unique<
          carto::mapping::scan_matching::GpuBatchFastCorrelativeScanMatcher2D>(
          *grid, options);
      if (FLAGS_batch_size > 0) matcher.v2->set_batch_size(FLAGS_batch_size);
      matcher.v2->set_timeout_ms(FLAGS_timeout_ms);
      matcher.v2->set_use_angular_branching(FLAGS_matcher == "v2.1");
    }
    LOG(INFO) << "  build #" << (i + 1) << ": "
              << matcher.precomputation_ms() << " ms";
  }

  std::vector<RunRecord> records;

  if (FLAGS_mode == "synth" || FLAGS_mode == "both") {
    std::mt19937 rng(FLAGS_seed);
    const double yaw_tol = FLAGS_success_yaw_tol_deg * M_PI / 180.;
    int generated = 0, attempts = 0;
    while (generated < FLAGS_num_poses && attempts < FLAGS_num_poses * 20) {
      ++attempts;
      carto::transform::Rigid2d gt_pose;
      if (!SampleFreePose(*grid, rng, &gt_pose)) break;
      carto::sensor::PointCloud cloud = SynthesizeScan(*grid, gt_pose, rng);
      if (static_cast<int>(cloud.size()) <
          FLAGS_beams * FLAGS_min_valid_beam_ratio) {
        continue;  // 回波太少（空旷/边缘区），不是有效采样
      }
      if (lua.enabled) {
        // 与线上 merged_filtered_cloud 同样的滤波，保证输入点密度一致
        cloud = ApplyMergedCloudFilters(cloud, lua.dp_options);
      }
      RunRecord rec = RunMatch(matcher, cloud, min_score, "synth", generated);
      rec.gt_x = gt_pose.translation().x();
      rec.gt_y = gt_pose.translation().y();
      rec.gt_yaw = gt_pose.rotation().angle();
      if (rec.profile.matched) {
        rec.trans_err = std::hypot(rec.est_x - rec.gt_x, rec.est_y - rec.gt_y);
        rec.yaw_err = std::abs(carto::common::NormalizeAngleDifference(
            rec.est_yaw - rec.gt_yaw));
        rec.success =
            rec.trans_err < FLAGS_success_trans_tol && rec.yaw_err < yaw_tol;
      }
      records.push_back(rec);
      ++generated;
      LOG(INFO) << "[synth " << generated << "/" << FLAGS_num_poses << "] "
                << "t=" << rec.profile.t_total_ms << " ms"
                << " score=" << rec.profile.best_score
                << " trans_err=" << rec.trans_err
                << " success=" << rec.success;
    }
    CHECK_GT(generated, 0) << "未能在自由空间采到有效位姿，检查地图与阈值";
  }

  if (FLAGS_mode == "bag" || FLAGS_mode == "both") {
    CHECK(!FLAGS_bag_filename.empty()) << "bag 模式需要 --bag_filename";

    // 话题列表：scan_topics（多雷达，逗号分隔）优先；空则退化单雷达 scan_topic。
    std::vector<std::string> topics = SplitString(FLAGS_scan_topics, ',');
    if (topics.empty()) topics.push_back(FLAGS_scan_topic);
    const std::string& primary = topics[0];

    rclcpp::Serialization<sensor_msgs::msg::LaserScan> serialization;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serialization;

    // ===== 第一遍：收集 /tf_static 外参 + 各话题帧时间戳与 frame_id =====
    tf2::BufferCore tf_buffer;
    std::map<std::string, std::vector<double>> stamps;     // topic → 各帧时间
    std::map<std::string, std::string> frame_ids;          // topic → frame_id
    for (const auto& t : topics) stamps[t] = {};
    {
      rosbag2_cpp::Reader reader;
      reader.open(FLAGS_bag_filename);
      while (reader.has_next()) {
        auto msg = reader.read_next();
        if (msg->topic_name == "/tf_static") {
          rclcpp::SerializedMessage serialized(*msg->serialized_data);
          tf2_msgs::msg::TFMessage tf_msg;
          tf_serialization.deserialize_message(&serialized, &tf_msg);
          for (const auto& t : tf_msg.transforms) {
            tf_buffer.setTransform(t, "bbs_benchmark", true /*is_static*/);
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
    const auto manual = SplitString(FLAGS_scan_extrinsics, ';');
    if (!manual.empty()) {
      CHECK_EQ(manual.size(), topics.size())
          << "--scan_extrinsics segment count must match topic count";
      for (size_t i = 0; i < topics.size(); ++i) {
        extrinsics[topics[i]] = ParseExtrinsic(manual[i]);
      }
    } else {
      for (const auto& t : topics) {
        try {
          const auto tf = tf_buffer.lookupTransform(
              FLAGS_tracking_frame, frame_ids[t], tf2::TimePointZero);
          extrinsics[t] = cartographer_ros::ToRigid3d(tf);
        } catch (const tf2::TransformException& e) {
          LOG(WARNING) << "tf_static missing " << FLAGS_tracking_frame << " <- "
                       << frame_ids[t] << " (" << e.what()
                       << "), treating this topic's extrinsic as identity; "
                       << "for multi-lidar check that the bag recorded /tf_static "
                       << "or pass --scan_extrinsics manually";
          extrinsics[t] = carto::transform::Rigid3d::Identity();
        }
      }
    }
    for (const auto& t : topics) {
      LOG(INFO) << "extrinsic " << FLAGS_tracking_frame << " <- " << frame_ids[t]
                << ": " << extrinsics[t];
    }

    // ===== 主话题抽帧（时间随机 / 固定步长）+ 副话题时间配对 =====
    const auto& primary_stamps = stamps[primary];
    std::vector<size_t> primary_indices;
    if (FLAGS_bag_random_samples > 0) {
      std::vector<size_t> all(primary_stamps.size());
      std::iota(all.begin(), all.end(), 0);
      std::mt19937 rng(FLAGS_seed);
      std::shuffle(all.begin(), all.end(), rng);
      const size_t k = std::min(
          static_cast<size_t>(FLAGS_bag_random_samples), all.size());
      primary_indices.assign(all.begin(), all.begin() + k);
      std::sort(primary_indices.begin(), primary_indices.end());
      LOG(INFO) << "bag random time sampling: " << k << "/" << primary_stamps.size()
                << " frames (primary topic " << primary << ", seed=" << FLAGS_seed << ")";
    } else {
      for (size_t i = 0; i < primary_stamps.size() &&
                         primary_indices.size() <
                             static_cast<size_t>(FLAGS_max_bag_scans);
           i += std::max(1, FLAGS_bag_stride)) {
        primary_indices.push_back(i);
      }
    }

    // 每个样本 = 主帧 + 各副话题时间最近帧（超差丢弃整个样本）。
    struct PairedSample {
      std::map<std::string, size_t> frame_index;  // topic → 帧索引
      double t_rel;                               // 主帧相对 bag 起始秒
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
        const auto it = std::lower_bound(s.begin(), s.end(),
                                         primary_stamps[pi]);
        size_t best = std::min<size_t>(it - s.begin(), s.size() - 1);
        if (best > 0 && std::abs(s[best - 1] - primary_stamps[pi]) <
                            std::abs(s[best] - primary_stamps[pi])) {
          --best;
        }
        if (std::abs(s[best] - primary_stamps[pi]) > FLAGS_pair_tolerance) {
          LOG(WARNING) << "primary frame #" << pi << " has no paired frame on "
                       << topics[ti] << " within " << FLAGS_pair_tolerance
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
    CHECK_GT(samples.size(), 0u) << "没有可用样本（检查话题名与配对容差）";

    // ===== 第二遍：只反序列化被选中的帧 =====
    std::map<std::string, std::map<size_t, sensor_msgs::msg::LaserScan>> stored;
    {
      rosbag2_cpp::Reader reader;
      reader.open(FLAGS_bag_filename);
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

    // ===== 评估：各话题点云经外参合并为一帧 =====
    // lua 模式：组装 TimedPointCloudData 走线上 DataPreprocessor（与
    // pose_estimator_2d L480 的 ProcessMultipleSensorData 同一份代码），
    // 取 merged_filtered_cloud——即 PoseInitializer 的真实输入。
    // 非 lua 模式：简化合并（无 range 裁剪/体素滤波），仅作快速测试。
    int tested = 0;
    for (const auto& sample : samples) {
      carto::sensor::PointCloud cloud;
      if (lua.enabled) {
        std::map<std::string, carto::sensor::TimedPointCloudData>
            laser_data_map;
        carto::common::Time sync_time;
        for (const auto& [topic, idx] : sample.frame_index) {
          carto::sensor::PointCloudWithIntensities pcwi;
          carto::common::Time time;
          std::tie(pcwi, time) =
              cartographer_ros::ToPointCloudWithIntensities(stored[topic][idx]);
          const auto& T = extrinsics[topic];
          laser_data_map[topic] = carto::sensor::TimedPointCloudData{
              time, T.translation().cast<float>(), T,
              carto::sensor::TransformTimedPointCloud(pcwi.points,
                                                      T.cast<float>()),
              pcwi.intensities};
          if (topic == primary) sync_time = time;
        }
        const auto merged_data =
            preprocessor->ProcessMultipleSensorData(laser_data_map, sync_time);
        if (merged_data.IsEmpty()) {
          LOG(WARNING) << "primary frame #" << sample.frame_index.at(primary)
                       << " preprocessing output empty, sample skipped";
          continue;
        }
        cloud = merged_data.merged_filtered_cloud;
      } else {
        std::vector<carto::sensor::RangefinderPoint> merged;
        for (const auto& [topic, idx] : sample.frame_index) {
          AppendScanToPointCloud(stored[topic][idx], extrinsics[topic],
                                 &merged);
        }
        cloud = carto::sensor::PointCloud(std::move(merged));
      }
      if (cloud.empty()) continue;
      RunRecord rec = RunMatch(matcher, cloud, min_score, "bag", tested);
      rec.success = rec.profile.matched;
      records.push_back(rec);
      ++tested;
      LOG(INFO) << "[bag " << tested << "/" << samples.size() << "] "
                << "frame#" << sample.frame_index.at(primary)
                << " bag_t=+" << sample.t_rel << "s"
                << " t=" << rec.profile.t_total_ms << " ms"
                << " score=" << rec.profile.best_score
                << " points=" << rec.profile.points_per_scan;
    }
    CHECK_GT(tested, 0) << "bag 中没有读到有效 scan";
  }

  // ===== 汇总输出：终端 + 报告文件（总结在前，明细 CSV 附在末尾）=====
  std::time_t now = std::time(nullptr);
  char timestamp[32];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S",
                std::localtime(&now));

  std::ostringstream header;
  header << "===== BBS 全局定位基线基准 =====\n"
         << "生成时间: " << timestamp << "\n"
         << "地图: " << (FLAGS_map_name.empty() ? pbstream_path : FLAGS_map_name)
         << " | 栅格 " << cells.num_x_cells << "x" << cells.num_y_cells
         << " @" << grid->limits().resolution() << " m/cell | 物理 "
         << cells.num_y_cells * grid->limits().resolution() << "x"
         << cells.num_x_cells * grid->limits().resolution() << " m\n"
         << "cell 统计: total=" << stats.total << " known=" << stats.known
         << " free=" << stats.free << " occupied=" << stats.occupied << "\n"
         << "matcher: " << FLAGS_matcher
         << (FLAGS_matcher == "v1"
                 ? "（Phase A: best-first+攒批 OpenMP, threads=" +
                       std::to_string(FLAGS_num_threads) + "）"
                 : FLAGS_matcher == "v2"
                       ? "（Phase B: best-first+CUDA 攒批 kernel）"
                       : "（原生 BBS）")
         << "\n"
         << "参数: min_score=" << min_score << " branch_and_bound_depth="
         << options.branch_and_bound_depth()
         << (lua.enabled ? "（线上对齐模式: " + FLAGS_configuration_basename +
                               "，复用 DataPreprocessor，不去畸变）"
                         : "（flag 简化模式，与线上输入有差异）")
         << "\n"
         << "scan: 双雷达=" << FLAGS_scan_topics
         << " tracking_frame=" << FLAGS_tracking_frame
         << " | synth beams=" << FLAGS_beams << " seed=" << FLAGS_seed << "\n"
         << "金字塔构建（最后一次）: " << matcher.precomputation_ms()
         << " ms（线上为懒构造，计入首次全局定位延迟）\n";

  LOG(INFO) << "";
  LOG(INFO) << "########## baseline benchmark summary ##########";
  LOG(INFO) << "\n" << header.str();

  // 输出目录：默认 ~/.data/localization/bbs_baseline/
  std::string out_dir = FLAGS_output_dir;
  if (out_dir.empty()) {
    const char* home = std::getenv("HOME");
    out_dir = std::string(home ? home : "/tmp") +
              "/.dexe/localization/bbs_baseline";
  }
  std::filesystem::create_directories(out_dir);

  const std::string map_tag =
      FLAGS_map_name.empty() ? "map" : FLAGS_map_name;
  for (const std::string& mode : {"synth", "bag"}) {
    const std::string summary = BuildModeSummary(records, mode);
    if (summary.empty()) continue;
    LOG(INFO) << "\n" << summary;
    const std::string path = out_dir + "/" + map_tag + "_" + timestamp + "_" +
                             FLAGS_matcher + "_" + mode + ".txt";
    WriteReport(path, header.str(), summary, records, mode);
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "2D 全局初始定位（BBS）离线基准测试工具，详见文件头注释。");
  google::ParseCommandLineFlags(&argc, &argv, true);
  Run();
  return 0;
}
