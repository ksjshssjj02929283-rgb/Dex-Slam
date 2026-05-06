/*
 * reset_map_origin_main.cc
 *
 * 离线工具：变更地图原点。
 *
 * 用法（推荐）：
 *   ros2 run cartographer_ros reset_map_origin_tool \
 *     --map_name <地图名称> \
 *     --new_x 1.5 --new_y -2.0 --new_yaw 0.785
 *
 *   会从 ~/.dexe/data/maps/<map_name> 加载地图（可用 --maps_root 改默认根目录）。
 *
 * 说明：
 *   传入地图目录（含 map.pbstream、map.yaml、map.png）以及新原点在旧地图坐标系下的
 *   位姿（x, y, yaw）。工具将：
 *     1. 加载 map.pbstream，反序列化 submap 切片
 *     2. 对所有 submap 的全局位姿应用坐标变换（新原点的逆变换）
 *     3. 重新渲染并保存 map.png + map.yaml（覆盖原文件）
 *     4. 将变换后的 submap pose 也写回 map.pbstream（使多次调用的变换可累积）
 *
 * 路径解析：
 *   --map_name <name>         → ~/.dexe/data/maps/<name>（或 --maps_root 指定的根目录）
 *   --map_dir  <name|path>    → 完整路径或相对 maps_root 的名称（向后兼容）
 *   --pbstream_filename <p>   → 直接指定 pbstream 文件（更细粒度控制）
 *
 * 注意：
 *   - 输入 yaw 单位为弧度
 *   - 原文件会被覆盖，请提前备份
 */

#include <cstdio>
#include <string>
#include <map>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <cmath>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_name, "",
              "地图名称（推荐用法）。最终目录解析为 <maps_root>/<map_name>，需含 "
              "map.pbstream / map.yaml / map.png");
DEFINE_string(maps_root, "",
              "默认地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(map_dir, "",
              "地图目录完整路径（与 --map_name 二选一）。"
              "若为相对名称，将相对 maps_root 解析（向后兼容旧用法）");
DEFINE_string(pbstream_filename, "", "pbstream 文件路径（可选，默认为 map_dir/map.pbstream）");
DEFINE_string(map_filestem, "", "输出文件名 stem（可选，默认为 map_dir/map）");
DEFINE_double(new_x, 0.0, "新原点在旧地图坐标系中的 X 坐标（米）");
DEFINE_double(new_y, 0.0, "新原点在旧地图坐标系中的 Y 坐标（米）");
DEFINE_double(new_yaw, 0.0, "新原点在旧地图坐标系中的旋转角度（弧度）");
DEFINE_double(resolution, 0.05, "输出地图分辨率（米/格）");
DEFINE_double(occ_thresh, 0.65, "占用阈值（>此值为障碍物）");
DEFINE_double(free_thresh, 0.196, "空闲阈值（<此值为自由空间）");
DEFINE_string(navigation_map_filter_mode, "normal",
              "导航地图噪声过滤强度（normal/high/super），与 node 的 save_map 一致");
DEFINE_bool(save_origin_map, false,
            "是否同时保存未经噪声过滤的原始地图为 *_origin.png");
DEFINE_bool(save_debug_map, false, "是否保存噪声过滤的中间调试图（debug_map/）");

namespace cartographer_ros {
namespace {

// 对 LineData proto 应用 2D 刚体变换（begin/end 用刚体，normal 仅旋转）
void TransformLineDataProto(
    ::cartographer::mapping::proto::LineData* line,
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

// 重新栅格化 Grid2D：把每个已知 cell 的中心位置经 T_inv 变换后写入新栅格。
// 因为旋转会让原单元格不再轴对齐，需要分配新的 MapLimits 并按 first-wins 写入。
void RerasterizeGridProto(
    ::cartographer::mapping::proto::Grid2D* grid_proto,
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

// 用单个变换后的 global_static_submap 渲染地图（与 MapBuilderBridge::SaveGridMapFromRange
// 完全一致：把 submap 的 grid 包装成一个 SubmapSlice，然后调 PaintSubmapSlices）。
// submap_2d_proto 的 grid/local_pose 需在调用前已经过 T_inv 变换。
void RenderSingleStaticSubmap(
    const ::cartographer::mapping::proto::Submap2D& submap_2d_proto,
    const double occ_thresh, const double free_thresh,
    cv::Mat* out_mat, Eigen::Vector2d* out_origin, double* out_resolution) {
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

// 对 global_static_submap 的非 grid 数据应用 T_inv：
//   local_pose、cloud、highlight_cloud、line_map_2d_list
// grid 不在这里处理 —— 由调用方决定走"从 node 重新生成"还是"重栅格化"路径
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

// 把所有 trajectory node 的 filtered_gravity_aligned_point_cloud 在新坐标系中
// 重新插入到一张全新的 ProbabilityGrid，得到一张干净、无重影的最终栅格地图。
// 与 MapBuilderBridge::GenerateFinalSubmap 的 range_data 重插逻辑保持一致，
// 区别是 pbstream 不存原始 range_datas，只能用 filtered cloud + 节点位姿做近似
// （origin = node.global_pose.translation()，misses 留空）。
//
// 参数：
//   pose_graph        已应用 T_inv 的 pose_graph（node.pose 在新系下）
//   records           SerializedData 列表（用于取 Node 的压缩点云）
//   traj_options      用于读取 hit/miss 概率参数
//   resolution        输出栅格分辨率
//   lidar_range_pad   额外 padding（米），覆盖激光最大有效距离，避免裁掉边缘
// 返回：proto::Grid2D，可直接赋给 submap_2d->mutable_grid()
::cartographer::mapping::proto::Grid2D BuildFreshGridFromNodes(
    const ::cartographer::mapping::proto::PoseGraph& pose_graph,
    const std::vector<::cartographer::mapping::proto::SerializedData>& records,
    const ::cartographer::mapping::proto::AllTrajectoryBuilderOptions&
        traj_options,
    const double resolution, const double lidar_range_pad) {
  // 1. 收集 node global pose（已经过 T_inv 变换）
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

void Run(const std::string& pbstream_file,
         const std::string& map_filestem,
         const double new_x,
         const double new_y,
         const double new_yaw,
         const double resolution,
         const double occ_thresh,
         const double free_thresh,
         const std::string& navigation_map_filter_mode,
         const bool save_origin_map,
         const bool save_debug_map) {
  // === Step 1: 单次读盘，把 pbstream 全部数据加载到内存 ===
  LOG(INFO) << "Loading pbstream: " << pbstream_file;
  ::cartographer::io::ProtoStreamReader reader(pbstream_file);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  // header + pose_graph + all_trajectory_builder_options 由 deserializer 自动读
  const auto header = deserializer.header();
  auto pose_graph = deserializer.pose_graph();
  auto traj_options = deserializer.all_trajectory_builder_options();

  // 后续 SerializedData 流式读
  std::vector<::cartographer::mapping::proto::SerializedData> all_records;
  ::cartographer::mapping::proto::SerializedData record;
  while (deserializer.ReadNextSerializedData(&record)) {
    all_records.push_back(std::move(record));
    record.Clear();
  }
  CHECK(reader.eof()) << "Failed to fully read pbstream";
  LOG(INFO) << "Loaded pbstream: " << all_records.size()
            << " SerializedData records";

  // === Step 2: 构造变换 T_inv（旧系 → 新系）===
  const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()));
  const ::cartographer::transform::Rigid3d T_new_origin(
      Eigen::Vector3d(new_x, new_y, 0.0), rotation);
  const ::cartographer::transform::Rigid3d T_inv = T_new_origin.inverse();

  LOG(INFO) << "New origin in old map frame: x=" << new_x
            << " y=" << new_y << " yaw=" << new_yaw << " rad";
  LOG(INFO) << "Transform (T_inv): translation=("
            << T_inv.translation().x() << ", " << T_inv.translation().y()
            << "), rotation=" << T_inv.rotation().coeffs().transpose();

  // === Step 3: 应用 T_inv 到 pose_graph 内所有全局位姿 ===
  size_t n_submap = 0, n_node = 0, n_landmark = 0;
  for (auto& trajectory : *pose_graph.mutable_trajectory()) {
    for (auto& submap : *trajectory.mutable_submap()) {
      const auto old_pose =
          ::cartographer::transform::ToRigid3(submap.pose());
      *submap.mutable_pose() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_submap;
    }
    for (auto& node : *trajectory.mutable_node()) {
      const auto old_pose = ::cartographer::transform::ToRigid3(node.pose());
      *node.mutable_pose() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_node;
    }
  }
  for (auto& landmark : *pose_graph.mutable_landmark_poses()) {
    const auto old_pose =
        ::cartographer::transform::ToRigid3(landmark.global_pose());
    *landmark.mutable_global_pose() =
        ::cartographer::transform::ToProto(T_inv * old_pose);
    ++n_landmark;
  }
  LOG(INFO) << "Pose graph transformed: " << n_submap << " submap poses, "
            << n_node << " node poses, " << n_landmark << " landmark poses";

  // === Step 4: 应用 T_inv 到 SerializedData ===
  //   - global_static_submap：local_pose + cloud + line（grid 在 Step 4b 重新生成）
  //   - trajectory_data：fixed_frame_origin_in_map
  //   - 其余记录（submap/node/imu/odometry/landmark_data/fixed_frame_pose_data）保持原值
  ::cartographer::mapping::proto::Submap2D* static_submap_proto = nullptr;
  size_t n_static = 0, n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
      TransformStaticSubmapNonGrid(rec.mutable_global_static_submap(), T_inv);
      static_submap_proto = rec.mutable_global_static_submap();
      ++n_static;
    } else if (rec.has_trajectory_data() &&
               rec.trajectory_data().has_fixed_frame_origin_in_map()) {
      auto* td = rec.mutable_trajectory_data();
      const auto old_pose = ::cartographer::transform::ToRigid3(
          td->fixed_frame_origin_in_map());
      *td->mutable_fixed_frame_origin_in_map() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_traj_data;
    }
  }
  LOG(INFO) << "SerializedData transformed (non-grid): " << n_static
            << " global_static_submap (local_pose + cloud + line), "
            << n_traj_data << " trajectory_data fixed_frame_origin";

  // === Step 4b: 从 trajectory node 的点云重新生成最终栅格地图 ===
  //   理由：直接拼接各 trajectory submap 切片渲染会在重叠区出现重影；
  //         重新插入所有 node cloud 到一张干净 ProbabilityGrid，结果与
  //         MapBuilderBridge::SaveMap 内 GenerateFinalSubmap 的产物一致。
  //   仅在 pose_graph 至少有 1 个 node 时执行，否则保留旧 grid 并按
  //   重栅格化方式回退（用于纯 pose-graph、无原始 cloud 的特殊 pbstream）。
  bool grid_rebuilt = false;
  if (static_submap_proto != nullptr) {
    bool has_nodes = false;
    for (const auto& traj : pose_graph.trajectory()) {
      if (traj.node_size() > 0) { has_nodes = true; break; }
    }
    if (has_nodes) {
      // padding 30m，覆盖典型激光最大距离；可后续暴露成 flag
      *static_submap_proto->mutable_grid() = BuildFreshGridFromNodes(
          pose_graph, all_records, traj_options, resolution,
          /*lidar_range_pad=*/30.0);
      grid_rebuilt = true;
    } else if (static_submap_proto->has_grid()) {
      LOG(WARNING) << "pose_graph has no nodes; falling back to "
                      "rerasterize existing global_static_submap.grid";
      RerasterizeGridProto(static_submap_proto->mutable_grid(), T_inv);
    }
  }

  // === Step 5: 渲染地图（与 MapBuilderBridge::SaveMap 一致）===
  //   优先用变换后的 global_static_submap（cached merged map，从 range_data 累积出来），
  //   缺失则回退到把 trajectory submap 切片拼接（旧行为）。
  cv::Mat origin_mat;
  Eigen::Vector2d map_origin;
  double render_resolution = resolution;

  if (static_submap_proto != nullptr) {
    LOG(INFO) << "Rendering from cached global_static_submap (matches "
                 "MapBuilderBridge::SaveMap behavior)";
    RenderSingleStaticSubmap(*static_submap_proto, occ_thresh, free_thresh,
                             &origin_mat, &map_origin, &render_resolution);
  } else {
    LOG(WARNING) << "pbstream has no global_static_submap, falling back to "
                    "stitching trajectory submap slices";
    std::map<::cartographer::mapping::SubmapId,
             ::cartographer::io::SubmapSlice>
        submap_slices;
    ::cartographer::mapping::ValueConversionTables conv;
    // 用变换后的 submap.pose 重新填充 slices
    std::map<::cartographer::mapping::SubmapId,
             ::cartographer::transform::Rigid3d>
        submap_poses;
    for (const auto& trajectory : pose_graph.trajectory()) {
      for (const auto& sm : trajectory.submap()) {
        submap_poses[{trajectory.trajectory_id(), sm.submap_index()}] =
            ::cartographer::transform::ToRigid3(sm.pose());
      }
    }
    for (const auto& rec : all_records) {
      if (!rec.has_submap()) continue;
      const auto& sm = rec.submap();
      const ::cartographer::mapping::SubmapId id{
          sm.submap_id().trajectory_id(), sm.submap_id().submap_index()};
      auto it = submap_poses.find(id);
      if (it == submap_poses.end()) continue;
      ::cartographer::io::FillSubmapSlice(it->second, sm,
                                           &submap_slices[id], &conv);
    }
    auto paint =
        ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    ::cartographer::io::Image image(std::move(paint.surface));
    origin_mat = ::cartographer_ros::GeneratePngMat(image, occ_thresh, free_thresh);
    map_origin = Eigen::Vector2d(
        -paint.origin.x() * resolution,
        (paint.origin.y() - image.height()) * resolution);
    render_resolution = resolution;
  }

  // === Step 6: 写出 png/yaml（统一走 WriteNavigationMap，与 node 端 save_map 一致）===
  const std::string png_file = map_filestem + ".png";
  const std::string yaml_file = map_filestem + ".yaml";

  std::string map_dir;
  {
    const size_t last_slash = map_filestem.find_last_of("/");
    map_dir = (last_slash != std::string::npos)
                  ? map_filestem.substr(0, last_slash)
                  : ".";
  }

  WriteNavigationMap(origin_mat, png_file,
                     navigation_map_filter_mode, save_debug_map, map_dir);

  ::cartographer::io::StreamFileWriter yaml_writer(yaml_file);
  WriteYaml(render_resolution, map_origin, map_filestem, &yaml_writer,
            occ_thresh, free_thresh);

  if (save_origin_map) {
    const std::string origin_png = map_filestem + "_origin.png";
    cv::imwrite(origin_png, origin_mat);
    LOG(INFO) << "Saved origin map to: " << origin_png;
  }

  LOG(INFO) << "Saved map to: " << map_filestem << ".png/.yaml";

  // === Step 7: 写回 pbstream（含全部已变换的 pose_graph + records）===
  const std::string tmp_pbstream = pbstream_file + ".tmp";
  {
    ::cartographer::io::ProtoStreamWriter writer(tmp_pbstream);
    writer.WriteProto(header);

    ::cartographer::mapping::proto::SerializedData pg_data;
    *pg_data.mutable_pose_graph() = pose_graph;
    writer.WriteProto(pg_data);

    ::cartographer::mapping::proto::SerializedData opts_data;
    *opts_data.mutable_all_trajectory_builder_options() = traj_options;
    writer.WriteProto(opts_data);

    for (const auto& rec : all_records) {
      writer.WriteProto(rec);
    }
    CHECK(writer.Close()) << "Failed to close pbstream writer";
  }
  CHECK_EQ(std::rename(tmp_pbstream.c_str(), pbstream_file.c_str()), 0)
      << "Failed to rename tmp pbstream to " << pbstream_file;
  LOG(INFO) << "Pbstream updated: " << pbstream_file;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_name.empty() || !FLAGS_map_dir.empty() ||
        !FLAGS_pbstream_filename.empty())
      << "Need one of: --map_name, --map_dir, or --pbstream_filename";

  // 默认地图根目录：用户传 --maps_root 优先，否则 ~/.dexe/data/maps
  std::string maps_root = FLAGS_maps_root;
  if (maps_root.empty()) {
    const char* home = std::getenv("HOME");
    CHECK(home != nullptr) << "HOME environment variable not set";
    maps_root = std::string(home) + "/.dexe/data/maps";
  }

  // 解析地图目录优先级：
  //   1. --map_name        → <maps_root>/<map_name>（推荐）
  //   2. --map_dir         → 完整路径或相对 maps_root 的名称（向后兼容旧用法）
  //   3. --pbstream_filename 单独使用，map_dir 留空
  std::string map_dir;
  if (!FLAGS_map_name.empty()) {
    map_dir = maps_root + "/" + FLAGS_map_name;
    if (access((map_dir + "/map.pbstream").c_str(), F_OK) != 0) {
      LOG(FATAL) << "Map not found: " << map_dir << "/map.pbstream\n"
                 << "  --map_name = " << FLAGS_map_name << "\n"
                 << "  --maps_root = " << maps_root;
    }
    LOG(INFO) << "Resolved map directory: " << map_dir;
  } else if (!FLAGS_map_dir.empty() && FLAGS_pbstream_filename.empty()) {
    // --map_dir 同时支持完整路径和相对 maps_root 的名称
    const std::string candidate_default = maps_root + "/" + FLAGS_map_dir;
    const std::string candidate_direct = FLAGS_map_dir;

    const bool default_exists =
        (access((candidate_default + "/map.pbstream").c_str(), F_OK) == 0);
    const bool direct_exists =
        (access((candidate_direct + "/map.pbstream").c_str(), F_OK) == 0);

    if (default_exists) {
      map_dir = candidate_default;
      LOG(INFO) << "Found map in default path: " << map_dir;
    } else if (direct_exists) {
      map_dir = candidate_direct;
      LOG(INFO) << "Found map at given path: " << map_dir;
    } else {
      LOG(FATAL) << "Map not found.\n"
                 << "  Tried default path: " << candidate_default << "\n"
                 << "  Tried given path:   " << candidate_direct << "\n"
                 << "Please check --map_name / --map_dir / --pbstream_filename.";
    }
  }

  std::string pbstream_file = FLAGS_pbstream_filename;
  std::string map_filestem = FLAGS_map_filestem;

  if (pbstream_file.empty()) {
    pbstream_file = map_dir + "/map.pbstream";
  }
  if (map_filestem.empty()) {
    if (!map_dir.empty()) {
      map_filestem = map_dir + "/map";
    } else {
      // 从 pbstream 路径推断
      const std::string& p = pbstream_file;
      const size_t dot = p.rfind(".pbstream");
      map_filestem = (dot != std::string::npos) ? p.substr(0, dot) : p;
    }
  }

  ::cartographer_ros::Run(
      pbstream_file, map_filestem,
      FLAGS_new_x, FLAGS_new_y, FLAGS_new_yaw,
      FLAGS_resolution, FLAGS_occ_thresh, FLAGS_free_thresh,
      FLAGS_navigation_map_filter_mode,
      FLAGS_save_origin_map,
      FLAGS_save_debug_map);

  rclcpp::shutdown();
  return 0;
}
