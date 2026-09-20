/*
 * map_transform_common.cc
 *
 * 见 map_transform_common.h。大部分实现从 reset_map_origin_main.cc 原样抽取
 * （匿名 namespace → cartographer_ros::map_transform），行为不变。
 */

#include "cartographer_ros/tools/map_transform_common.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <utility>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "glog/logging.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

namespace cartographer_ros {
namespace map_transform {

namespace fs = std::filesystem;

PbstreamContents LoadPbstream(const std::string& pbstream_file) {
  LOG(INFO) << "Loading pbstream: " << pbstream_file;
  ::cartographer::io::ProtoStreamReader reader(pbstream_file);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  PbstreamContents contents;
  // header + pose_graph + all_trajectory_builder_options 由 deserializer 自动读
  contents.header = deserializer.header();
  contents.pose_graph = deserializer.pose_graph();
  contents.traj_options = deserializer.all_trajectory_builder_options();

  // 后续 SerializedData 流式读
  ::cartographer::mapping::proto::SerializedData record;
  while (deserializer.ReadNextSerializedData(&record)) {
    contents.records.push_back(std::move(record));
    record.Clear();
  }
  CHECK(reader.eof()) << "Failed to fully read pbstream: " << pbstream_file;
  LOG(INFO) << "Loaded pbstream: " << contents.records.size()
            << " SerializedData records";
  return contents;
}

void WritePbstream(const std::string& pbstream_file,
                   const PbstreamContents& contents) {
  const std::string tmp_pbstream = pbstream_file + ".tmp";
  {
    ::cartographer::io::ProtoStreamWriter writer(tmp_pbstream);
    writer.WriteProto(contents.header);

    ::cartographer::mapping::proto::SerializedData pg_data;
    *pg_data.mutable_pose_graph() = contents.pose_graph;
    writer.WriteProto(pg_data);

    ::cartographer::mapping::proto::SerializedData opts_data;
    *opts_data.mutable_all_trajectory_builder_options() = contents.traj_options;
    writer.WriteProto(opts_data);

    for (const auto& rec : contents.records) {
      writer.WriteProto(rec);
    }
    CHECK(writer.Close()) << "Failed to close pbstream writer";
  }
  CHECK_EQ(std::rename(tmp_pbstream.c_str(), pbstream_file.c_str()), 0)
      << "Failed to rename tmp pbstream to " << pbstream_file;
  LOG(INFO) << "Pbstream updated: " << pbstream_file;
}

void TransformLineDataProto(::cartographer::mapping::proto::LineData* line,
                            const ::cartographer::transform::Rigid2d& T_2d) {
  const Eigen::Vector2d old_b(line->begin_x(), line->begin_y());
  const Eigen::Vector2d old_e(line->end_x(), line->end_y());
  const Eigen::Vector2d new_b = T_2d * old_b;
  const Eigen::Vector2d new_e = T_2d * old_e;
  line->set_begin_x(new_b.x());
  line->set_begin_y(new_b.y());
  line->set_end_x(new_e.x());
  line->set_end_y(new_e.y());
  if (line->has_normal()) {
    const Eigen::Vector2d old_n(line->normal_x(), line->normal_y());
    const Eigen::Vector2d new_n = T_2d.rotation() * old_n;
    line->set_normal_x(new_n.x());
    line->set_normal_y(new_n.y());
  }
}

void TransformStaticSubmapNonGrid(
    ::cartographer::mapping::proto::Submap2D* submap_2d,
    const ::cartographer::transform::Rigid3d& T_inv) {
  // 1. local_pose（全局位姿）
  const auto old_lp =
      ::cartographer::transform::ToRigid3(submap_2d->local_pose());
  *submap_2d->mutable_local_pose() =
      ::cartographer::transform::ToProto(T_inv * old_lp);

  // 2. cloud / highlight_cloud（点云在全局坐标系，用 3D 刚体变换）
  const auto transform_cloud =
      [&T_inv](::cartographer::mapping::proto::PointCloudData* cloud) {
        for (auto& pt : *cloud->mutable_points()) {
          const Eigen::Vector3d old_p(pt.x(), pt.y(), pt.z());
          const Eigen::Vector3d new_p = T_inv * old_p;
          pt.set_x(new_p.x());
          pt.set_y(new_p.y());
          pt.set_z(new_p.z());
        }
      };
  if (submap_2d->has_cloud()) transform_cloud(submap_2d->mutable_cloud());
  if (submap_2d->has_highlight_cloud()) {
    transform_cloud(submap_2d->mutable_highlight_cloud());
  }

  // 3. line_map_2d_list（线特征在全局坐标系，用 2D 刚体变换 begin/end，旋转 normal）
  if (submap_2d->has_line_map_2d_list()) {
    const auto T_2d = ::cartographer::transform::Project2D(T_inv);
    auto* lml = submap_2d->mutable_line_map_2d_list();
    for (auto& map_line : *lml->mutable_line_map_data()) {
      if (map_line.has_line_data()) {
        TransformLineDataProto(map_line.mutable_line_data(), T_2d);
      }
      for (auto& sub_line : *map_line.mutable_line_data_list()) {
        TransformLineDataProto(&sub_line, T_2d);
      }
    }
  }
}

void RerasterizeGridProto(::cartographer::mapping::proto::Grid2D* grid_proto,
                          const ::cartographer::transform::Rigid3d& T_inv) {
  if (!grid_proto->has_probability_grid_2d()) {
    LOG(WARNING) << "RerasterizeGridProto: only ProbabilityGrid is supported, "
                    "skipping.";
    return;
  }

  ::cartographer::mapping::ValueConversionTables conv;
  ::cartographer::mapping::ProbabilityGrid old_grid(*grid_proto, &conv);

  const auto& old_lim = old_grid.limits();
  const double res = old_lim.resolution();
  const int num_x = old_lim.cell_limits().num_x_cells;
  const int num_y = old_lim.cell_limits().num_y_cells;
  const Eigen::Vector2d old_max = old_lim.max();
  const Eigen::Vector2d old_min(old_max.x() - num_x * res,
                                old_max.y() - num_y * res);

  // 用旧栅格的 4 个角推出新栅格的轴对齐 bbox
  Eigen::AlignedBox2d new_bbox;
  for (double x : {old_min.x(), old_max.x()}) {
    for (double y : {old_min.y(), old_max.y()}) {
      const Eigen::Vector3d v = T_inv * Eigen::Vector3d(x, y, 0.0);
      new_bbox.extend(Eigen::Vector2d(v.x(), v.y()));
    }
  }
  // 对齐到栅格尺度并加一格 padding，避免边缘 cell 被裁
  const auto snap_up = [res](double v) { return std::ceil(v / res) * res; };
  const auto snap_dn = [res](double v) { return std::floor(v / res) * res; };
  const Eigen::Vector2d new_max(snap_up(new_bbox.max().x()) + res,
                                snap_up(new_bbox.max().y()) + res);
  const Eigen::Vector2d new_min(snap_dn(new_bbox.min().x()) - res,
                                snap_dn(new_bbox.min().y()) - res);
  const int new_num_x =
      static_cast<int>(std::round((new_max.x() - new_min.x()) / res));
  const int new_num_y =
      static_cast<int>(std::round((new_max.y() - new_min.y()) / res));

  ::cartographer::mapping::ProbabilityGrid new_grid(
      ::cartographer::mapping::MapLimits(
          res, new_max,
          ::cartographer::mapping::CellLimits(new_num_x, new_num_y)),
      &conv);

  // 逐 cell 重栅格化：first-wins，冲突时取较高占用概率
  // （SetProbability 要求 cell 处于 unknown，所以先收集到临时 buffer 再统一写入）
  std::vector<float> buf(new_num_x * new_num_y, -1.0f);
  size_t copied = 0, dropped = 0, conflict = 0;
  for (int i = 0; i < num_y; ++i) {
    for (int j = 0; j < num_x; ++j) {
      const Eigen::Array2i old_idx(j, i);
      if (!old_grid.IsKnown(old_idx)) continue;
      const float prob = old_grid.GetProbability(old_idx);
      const Eigen::Vector2f old_c = old_lim.GetCellCenter(old_idx);
      const Eigen::Vector3d new_p =
          T_inv * Eigen::Vector3d(old_c.x(), old_c.y(), 0.0);
      const Eigen::Array2i new_idx = new_grid.limits().GetCellIndex(
          Eigen::Vector2f(new_p.x(), new_p.y()));
      if (!new_grid.limits().Contains(new_idx)) {
        ++dropped;
        continue;
      }
      const int flat = new_num_x * new_idx.y() + new_idx.x();
      if (buf[flat] < 0.0f) {
        buf[flat] = prob;
        ++copied;
      } else {
        ++conflict;
        if (prob > buf[flat]) buf[flat] = prob;
      }
    }
  }
  for (int i = 0; i < new_num_y; ++i) {
    for (int j = 0; j < new_num_x; ++j) {
      const int flat = new_num_x * i + j;
      if (buf[flat] < 0.0f) continue;
      new_grid.SetProbability(Eigen::Array2i(j, i), buf[flat]);
    }
  }
  new_grid.FinishUpdate();

  *grid_proto = new_grid.ToProto();
  LOG(INFO) << "  Grid rerasterized: copied=" << copied
            << " dropped=" << dropped << " conflict=" << conflict
            << " new_size=" << new_num_x << "x" << new_num_y;
}

::cartographer::mapping::proto::Grid2D BuildFreshGridFromNodes(
    const ::cartographer::mapping::proto::PoseGraph& pose_graph,
    const std::vector<::cartographer::mapping::proto::SerializedData>& records,
    const ::cartographer::mapping::proto::AllTrajectoryBuilderOptions&
        traj_options,
    const double resolution, const double lidar_range_pad) {
  // 1. 收集 node global pose（已经过变换）
  std::map<std::pair<int, int>, ::cartographer::transform::Rigid3d>
      node_global_poses;
  for (const auto& traj : pose_graph.trajectory()) {
    for (const auto& node : traj.node()) {
      node_global_poses[{traj.trajectory_id(), node.node_index()}] =
          ::cartographer::transform::ToRigid3(node.pose());
    }
  }
  CHECK(!node_global_poses.empty())
      << "BuildFreshGridFromNodes: pose_graph has no nodes";

  // 2. 计算 bbox：节点位姿凸包再加激光射程 padding
  Eigen::AlignedBox2d bbox;
  for (const auto& [id, pose] : node_global_poses) {
    bbox.extend(Eigen::Vector2d(pose.translation().x(),
                                pose.translation().y()));
  }
  const Eigen::Vector2d pad = Eigen::Vector2d::Constant(lidar_range_pad);
  const Eigen::Vector2d bbox_min = bbox.min() - pad;
  const Eigen::Vector2d bbox_max = bbox.max() + pad;
  const int num_x = static_cast<int>(
      std::ceil((bbox_max.x() - bbox_min.x()) / resolution));
  const int num_y = static_cast<int>(
      std::ceil((bbox_max.y() - bbox_min.y()) / resolution));

  // 3. 创建空的 ProbabilityGrid，origin/local_pose 设为 bbox 右上角
  ::cartographer::mapping::ValueConversionTables conv;
  const Eigen::Vector2f origin_2d(static_cast<float>(bbox_max.x()),
                                  static_cast<float>(bbox_max.y()));
  auto submap = std::make_shared<::cartographer::mapping::Submap2D>(
      origin_2d,
      std::unique_ptr<::cartographer::mapping::Grid2D>(
          static_cast<::cartographer::mapping::Grid2D*>(
              std::make_unique<::cartographer::mapping::ProbabilityGrid>(
                  ::cartographer::mapping::MapLimits(
                      resolution, bbox_max,
                      ::cartographer::mapping::CellLimits(num_x, num_y)),
                  &conv)
                  .release())),
      &conv,
      std::make_unique<::cartographer::sensor::PointCloud>(),
      std::make_unique<::cartographer::sensor::PointCloud>(),
      0, std::map<int, ::cartographer::mapping::MapLine2D>{});

  // 4. 配置 range_data_inserter：优先用 traj_options 中的真实 hit/miss 概率
  ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D
      pg_opts;
  bool used_default_opts = true;
  if (!traj_options.options_with_sensor_ids().empty()) {
    const auto& rdi_opts = traj_options.options_with_sensor_ids(0)
                               .trajectory_builder_options()
                               .trajectory_builder_2d_options()
                               .submaps_options()
                               .range_data_inserter_options();
    if (rdi_opts.range_data_inserter_type() ==
        ::cartographer::mapping::proto::RangeDataInserterOptions::
            PROBABILITY_GRID_INSERTER_2D) {
      pg_opts = rdi_opts.probability_grid_range_data_inserter_options_2d();
      used_default_opts = false;
    }
  }
  if (used_default_opts) {
    pg_opts.set_hit_probability(0.65);
    pg_opts.set_miss_probability(0.48);
    pg_opts.set_insert_free_space(true);
    LOG(WARNING) << "BuildFreshGridFromNodes: traj_options has no "
                    "PROBABILITY_GRID_INSERTER_2D, using defaults "
                    "(hit=0.65, miss=0.48)";
  }
  auto inserter = std::make_unique<
      ::cartographer::mapping::ProbabilityGridRangeDataInserter2D>(pg_opts);

  // 5. 逐 Node 取 cloud → 变换到新坐标系 → 用近似 origin 插入 grid
  size_t inserted = 0, skipped_no_pose = 0, skipped_empty_cloud = 0;
  for (const auto& rec : records) {
    if (!rec.has_node()) continue;
    const auto& node_proto = rec.node();
    const auto it = node_global_poses.find(
        {node_proto.node_id().trajectory_id(),
         node_proto.node_id().node_index()});
    if (it == node_global_poses.end()) {
      ++skipped_no_pose;
      continue;
    }
    const auto& global_pose = it->second;
    const auto& node_data = node_proto.node_data();
    if (!node_data.has_filtered_gravity_aligned_point_cloud()) {
      ++skipped_empty_cloud;
      continue;
    }

    // Decompress + 变换到新全局系（global_pose 已是新系）
    ::cartographer::sensor::CompressedPointCloud compressed(
        node_data.filtered_gravity_aligned_point_cloud());
    const auto cloud_local = compressed.Decompress();
    if (cloud_local.empty()) {
      ++skipped_empty_cloud;
      continue;
    }
    const auto cloud_global = ::cartographer::sensor::TransformPointCloud(
        cloud_local, global_pose.cast<float>());

    ::cartographer::sensor::RangeData range_data;
    range_data.origin = global_pose.translation().cast<float>();
    range_data.returns = cloud_global;
    range_data.misses = ::cartographer::sensor::PointCloud{};

    submap->InsertRangeData(range_data, inserter.get());
    ++inserted;
  }
  submap->Finish();

  LOG(INFO) << "BuildFreshGridFromNodes: " << inserted << " nodes inserted ("
            << skipped_no_pose << " no pose, " << skipped_empty_cloud
            << " empty cloud), grid " << num_x << "x" << num_y
            << " @ res=" << resolution << "m, hit=" << pg_opts.hit_probability()
            << " miss=" << pg_opts.miss_probability();

  return submap->grid()->ToProto();
}

void RenderSingleStaticSubmap(
    const ::cartographer::mapping::proto::Submap2D& submap_2d_proto,
    const double occ_thresh, const double free_thresh, cv::Mat* out_mat,
    Eigen::Vector2d* out_origin, double* out_resolution) {
  // 反序列化得到 Submap2D，复用 cartographer 标准 ToResponseProto 流程
  ::cartographer::mapping::ValueConversionTables conv;
  ::cartographer::mapping::Submap2D submap(submap_2d_proto, &conv);

  ::cartographer::mapping::proto::SubmapQuery::Response resp;
  submap.ToResponseProto(submap.local_pose(), &resp);
  CHECK(!resp.textures().empty())
      << "global_static_submap has no texture (empty grid?)";
  const auto& texture = resp.textures(0);
  const auto pixels = ::cartographer::io::UnpackTextureData(
      texture.cells(), texture.width(), texture.height());
  const double resolution = texture.resolution();

  // 单 submap 包装成 SubmapSlice
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  const ::cartographer::mapping::SubmapId virtual_id{0, 0};
  auto& slice = submap_slices[virtual_id];
  slice.pose = submap.local_pose();
  slice.width = texture.width();
  slice.height = texture.height();
  slice.resolution = resolution;
  slice.slice_pose =
      ::cartographer::transform::ToRigid3(texture.slice_pose());
  slice.cairo_data.clear();
  slice.surface = ::cartographer::io::DrawTexture(
      pixels.intensity, pixels.alpha, texture.width(), texture.height(),
      &slice.cairo_data);

  auto paint =
      ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  ::cartographer::io::Image image(std::move(paint.surface));

  *out_mat = ::cartographer_ros::GeneratePngMat(image, occ_thresh, free_thresh);
  *out_origin = Eigen::Vector2d(
      -paint.origin.x() * resolution,
      (paint.origin.y() - image.height()) * resolution);
  *out_resolution = resolution;
  LOG(INFO) << "Rendered global_static_submap: " << image.width() << "x"
            << image.height() << " px, origin=(" << out_origin->x() << ", "
            << out_origin->y() << ")";
}

::cartographer::mapping::LineMap2DList LineMap2DListFromProto(
    const ::cartographer::mapping::proto::LineMap2DList& proto) {
  // proto → 运行时 LineMap2DList 的轻量重建（submap_2d.cc FromProto 的子集）：
  // 只重建主线段数据、不重放 line_data_list 观测历史 —— 与
  // io/internal/pbstream_check.cc 的 ALIGN-01 检查同源。
  std::map<int, ::cartographer::mapping::MapLine2D> line_map_data;
  for (const auto& map_line_proto : proto.line_map_data()) {
    const auto& line_data_proto = map_line_proto.line_data();
    const ::cartographer::sensor::LineData line_data(
        Eigen::Vector2d(line_data_proto.begin_x(), line_data_proto.begin_y()),
        Eigen::Vector2d(line_data_proto.end_x(), line_data_proto.end_y()),
        line_data_proto.weight());
    ::cartographer::mapping::MapLine2D map_line(
        ::cartographer::mapping::MapLine2D::static_line,
        map_line_proto.global_id(), line_data);
    map_line.SetLocalId(map_line_proto.local_id());
    map_line.SetInliers(map_line_proto.inliers());
    line_map_data.insert({map_line_proto.local_id(), std::move(map_line)});
  }
  return ::cartographer::mapping::LineMap2DList(proto.latest_local_id(),
                                                std::move(line_map_data));
}

std::string CurrentTimestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf;
  localtime_r(&t, &tm_buf);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(buf);
}

std::string BackupMapFiles(
    const std::string& map_filestem,
    const std::vector<std::string>& candidate_extensions) {
  const fs::path stem(map_filestem);
  std::error_code ec;

  bool any_exists = false;
  for (const auto& ext : candidate_extensions) {
    if (fs::exists(map_filestem + ext, ec)) {
      any_exists = true;
      break;
    }
  }
  if (!any_exists) return "";

  // 带时间戳的备份目录，子文件保持原名
  const fs::path backup_dir = stem.parent_path() / "backups" /
                              ("backup_" + CurrentTimestamp());
  fs::create_directories(backup_dir, ec);
  CHECK(!ec) << "Failed to create backup directory " << backup_dir << ": "
             << ec.message();

  for (const auto& ext : candidate_extensions) {
    const fs::path src(map_filestem + ext);
    if (!fs::exists(src, ec)) continue;
    const fs::path dst = backup_dir / src.filename();
    fs::copy_file(src, dst, fs::copy_options::overwrite_existing, ec);
    CHECK(!ec) << "Failed to backup " << src << " → " << dst << ": "
               << ec.message();
    LOG(INFO) << "Backed up: " << src << " → " << dst;
  }
  return backup_dir.string();
}

bool TransformPcdFile(const std::string& pcd_file,
                      const ::cartographer::transform::Rigid3d& T_inv) {
  std::error_code ec;
  if (!fs::exists(pcd_file, ec)) return false;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) < 0) {
    LOG(WARNING) << "Failed to load PCD (skipping transform): " << pcd_file;
    return false;
  }
  for (auto& pt : cloud->points) {
    const Eigen::Vector3d new_p =
        T_inv * Eigen::Vector3d(pt.x, pt.y, pt.z);
    pt.x = static_cast<float>(new_p.x());
    pt.y = static_cast<float>(new_p.y());
    pt.z = static_cast<float>(new_p.z());
  }
  // is_dense=false 安全：保留原有点云组织性标记由 load 决定，写回时沿用 cloud 属性
  pcl::io::savePCDFileBinary(pcd_file, *cloud);
  LOG(INFO) << "Transformed PCD: " << pcd_file << " (" << cloud->size()
            << " points)";
  return true;
}

}  // namespace map_transform
}  // namespace cartographer_ros
