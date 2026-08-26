/*
 * map_ops.cc
 *
 * 地图离线工具共享库实现。见 map_ops.h 头部注释。
 * 本文件中的加载 / 匹配逻辑抽取自 merge_maps_main.cc，变换 / 重写 / 渲染逻辑
 * 抽取自 reset_map_origin_main.cc；为避免改动现有工具，这里是独立副本（命名空间
 * cartographer_ros::map_ops），行为与原实现保持一致。
 */

#include "cartographer_ros/tools/map_ops/map_ops.h"

#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>

#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "glog/logging.h"
#include "opencv2/highgui.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace cartographer_ros {
namespace map_ops {

// ──────────────────────────────────────────────────
// 加载
// ──────────────────────────────────────────────────

MapData LoadMap(const std::string& pbstream_file, bool build_slices) {
  MapData data;

  ::cartographer::io::ProtoStreamReader reader(pbstream_file);

  // 手动读取前三个固定字段（跳过 ProtoStreamDeserializer 的轨迹数量 CHECK）。
  CHECK(reader.ReadProto(&data.header)) << "Failed to read SerializationHeader";

  ::cartographer::mapping::proto::SerializedData pg_record;
  CHECK(reader.ReadProto(&pg_record)) << "Failed to read PoseGraph record";
  CHECK(pg_record.has_pose_graph()) << "Expected PoseGraph as 2nd record";
  data.pose_graph = pg_record.pose_graph();

  ::cartographer::mapping::proto::SerializedData opts_record;
  CHECK(reader.ReadProto(&opts_record))
      << "Failed to read AllTrajectoryBuilderOptions record";
  CHECK(opts_record.has_all_trajectory_builder_options())
      << "Expected AllTrajectoryBuilderOptions as 3rd record";
  data.traj_options = opts_record.all_trajectory_builder_options();

  ::cartographer::mapping::proto::SerializedData record;
  while (reader.ReadProto(&record)) {
    data.all_records.push_back(record);
    record.Clear();
  }

  // 提取 global_static_submap（用于 FCSM 全局匹配）。必须使用成员 conv_tables，
  // 避免悬空指针（ProbabilityGrid 内部持有其指针）。
  for (const auto& rec : data.all_records) {
    if (rec.has_global_static_submap()) {
      data.global_submap_conv_tables =
          std::make_unique<::cartographer::mapping::ValueConversionTables>();
      data.global_submap = std::make_unique<::cartographer::mapping::Submap2D>(
          rec.global_static_submap(), data.global_submap_conv_tables.get());
      LOG(INFO) << "Loaded global_static_submap from pbstream";
      data.global_submap->RebuildIkdTrees();
      break;
    }
  }
  if (!data.global_submap) {
    LOG(WARNING) << "No global_static_submap found in pbstream.";
  }

  // 可选：构建 submap_slices（用于拼接渲染；替换类工具通常不需要）。
  if (build_slices) {
    for (const auto& rec : data.all_records) {
      if (rec.has_submap()) {
        const auto& submap_proto = rec.submap();
        const ::cartographer::mapping::SubmapId id{
            submap_proto.submap_id().trajectory_id(),
            submap_proto.submap_id().submap_index()};
        ::cartographer::transform::Rigid3d global_pose =
            ::cartographer::transform::Rigid3d::Identity();
        for (const auto& traj : data.pose_graph.trajectory()) {
          if (traj.trajectory_id() == id.trajectory_id) {
            for (const auto& sm : traj.submap()) {
              if (sm.submap_index() == id.submap_index) {
                global_pose = ::cartographer::transform::ToRigid3(sm.pose());
                break;
              }
            }
          }
        }
        ::cartographer::io::FillSubmapSlice(global_pose, submap_proto,
                                            &data.submap_slices[id],
                                            &data.conversion_tables);
      }
    }
  }

  return data;
}

// ──────────────────────────────────────────────────
// 提取节点点云
// ──────────────────────────────────────────────────

std::vector<NodeCloudEntry> ExtractNodeClouds(const MapData& data_b) {
  std::map<std::pair<int, int>, ::cartographer::transform::Rigid3d>
      node_global_poses;
  for (const auto& traj : data_b.pose_graph.trajectory()) {
    for (const auto& node : traj.node()) {
      node_global_poses[{traj.trajectory_id(), node.node_index()}] =
          ::cartographer::transform::ToRigid3(node.pose());
    }
  }

  std::vector<NodeCloudEntry> result;
  for (const auto& rec : data_b.all_records) {
    if (!rec.has_node()) continue;
    const auto& node_proto = rec.node();
    const int traj_id = node_proto.node_id().trajectory_id();
    const int node_idx = node_proto.node_id().node_index();
    const auto key = std::make_pair(traj_id, node_idx);
    if (node_global_poses.find(key) == node_global_poses.end()) continue;

    const auto& nd = node_proto.node_data();
    if (!nd.has_filtered_gravity_aligned_point_cloud()) continue;

    const ::cartographer::sensor::CompressedPointCloud compressed_cloud(
        nd.filtered_gravity_aligned_point_cloud());

    ::cartographer::sensor::PointCloud pc;
    for (const auto& pt : compressed_cloud) {
      pc.push_back(pt);
    }
    if (pc.empty()) continue;

    NodeCloudEntry entry;
    entry.trajectory_id = traj_id;
    entry.node_index = node_idx;
    entry.global_pose_b = node_global_poses.at(key);
    entry.point_cloud = std::move(pc);
    for (const auto& line_proto : nd.lines()) {
      entry.line_data_list.push_back(
          ::cartographer::sensor::FromProto(line_proto));
    }
    result.push_back(std::move(entry));
  }

  LOG(INFO) << "Extracted " << result.size() << " node point clouds";
  return result;
}

// ──────────────────────────────────────────────────
// 全局重定位
// ──────────────────────────────────────────────────

std::pair<bool, ::cartographer::transform::Rigid3d> FindTransformBinA(
    const MapData& data_a,
    const std::vector<NodeCloudEntry>& node_clouds_b,
    const double min_score, const int consecutive_needed) {
  CHECK(data_a.global_submap != nullptr)
      << "target map has no global_static_submap in pbstream. "
         "Cannot do global FCSM matching.";
  const ::cartographer::mapping::Grid2D* global_grid =
      data_a.global_submap->grid();
  CHECK(global_grid != nullptr) << "global_static_submap has no grid";

  const ::cartographer::transform::Rigid2d global_submap_pose_2d(
      data_a.global_submap->local_pose().translation().head<2>(),
      ::cartographer::transform::GetYaw(data_a.global_submap->local_pose()));

  LOG(INFO) << "global_static_submap local_pose: ("
            << global_submap_pose_2d.translation().x() << ", "
            << global_submap_pose_2d.translation().y()
            << " yaw=" << global_submap_pose_2d.rotation().angle() << ")";

  ::cartographer::mapping::scan_matching::proto::
      FastCorrelativeScanMatcherOptions2D fcsm_options;
  fcsm_options.set_linear_search_window(300.0);
  fcsm_options.set_angular_search_window(M_PI);
  fcsm_options.set_branch_and_bound_depth(7);
  ::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D fcsm(
      *global_grid, fcsm_options);

  LOG(INFO) << "Built FCSM from global_static_submap";

  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      ceres_options;
  ceres_options.set_occupied_space_weight(1.0);
  ceres_options.set_translation_weight(0.1);
  ceres_options.set_rotation_weight(0.1);
  ceres_options.set_line_weight(1.0);
  ceres_options.set_point_weight(1.0);
  ceres_options.mutable_ceres_solver_options()->set_max_num_iterations(20);
  ceres_options.mutable_ceres_solver_options()->set_num_threads(4);
  ::cartographer::mapping::scan_matching::CeresScanMatcher2D ceres_matcher(
      ceres_options);

  ::cartographer::mapping::scan_matching::proto::LineMatcherOptions
      line_matcher_options;
  line_matcher_options.set_line_delta_theta_threshold(0.2);
  line_matcher_options.set_line_delta_tra_elongation_threshold(0.5);
  line_matcher_options.set_min_delta_tra_parallel(0.5);
  line_matcher_options.set_max_match_distance(30.0);
  ::cartographer::mapping::scan_matching::LineMatcher2D line_matcher(
      line_matcher_options);

  const ::cartographer::mapping::LineMap2DList* global_line_map =
      data_a.global_submap->line_map_2d_list();
  ::cartographer::ikd_tree::iKdTree<pcl::PointXYZ>* global_cloud_ikdtree =
      data_a.global_submap->cloud_ikdtree();
  const bool has_line_map = global_line_map && global_line_map->size() > 0;
  const bool has_ikdtree = global_cloud_ikdtree != nullptr;
  LOG(INFO) << "Line map available: " << has_line_map
            << ", IKDTree available: " << has_ikdtree;

  struct ConsecutiveFrame {
    ::cartographer::transform::Rigid2d pose_in_submap;
    ::cartographer::transform::Rigid2d node_pose_in_B;
    ::cartographer::sensor::PointCloud point_cloud;
    std::vector<::cartographer::sensor::LineData> line_data_list;
  };
  std::vector<ConsecutiveFrame> consecutive;

  int node_count = 0;
  for (const auto& node_entry : node_clouds_b) {
    ++node_count;

    ::cartographer::sensor::PointCloud pc_2d;
    for (const auto& pt : node_entry.point_cloud) {
      pc_2d.push_back(
          {Eigen::Vector3f(pt.position.x(), pt.position.y(), 0.0f)});
    }
    if (pc_2d.empty()) {
      consecutive.clear();
      continue;
    }

    float score = 0.0f;
    ::cartographer::transform::Rigid2d pose_in_submap;
    const bool matched =
        fcsm.MatchFullSubmap(pc_2d, min_score, &score, &pose_in_submap);

    LOG(INFO) << "[node " << node_count << "/" << node_clouds_b.size()
              << "] FCSM score=" << score
              << (matched ? "" : " (below threshold)")
              << " consecutive=" << (matched ? consecutive.size() + 1 : 0)
              << "/" << consecutive_needed;

    if (!matched) {
      if (!consecutive.empty()) {
        LOG(INFO) << "  → FCSM failed, reset " << consecutive.size()
                  << " consecutive frames → 0";
        consecutive.clear();
      }
      continue;
    }

    const ::cartographer::transform::Rigid2d node_pose_in_B(
        node_entry.global_pose_b.translation().head<2>(),
        ::cartographer::transform::GetYaw(node_entry.global_pose_b));

    consecutive.push_back({pose_in_submap, node_pose_in_B, pc_2d,
                           node_entry.line_data_list});

    if (static_cast<int>(consecutive.size()) < consecutive_needed) continue;

    LOG(INFO) << "Found " << consecutive_needed
              << " consecutive FCSM matches. Running Ceres fine registration...";

    Eigen::Vector3d sum_trans = Eigen::Vector3d::Zero();
    double sum_yaw = 0.0;
    for (int i = 0; i < static_cast<int>(consecutive.size()); ++i) {
      const auto& frame = consecutive[i];
      ::cartographer::transform::Rigid2d ceres_pose;
      ceres::Solver::Summary ceres_summary;

      const ::cartographer::transform::Rigid2d pose_in_A =
          global_submap_pose_2d * frame.pose_in_submap;

      std::map<int, std::vector<::cartographer::sensor::LineData>>
          tracked_lane_data;
      if (has_line_map && !frame.line_data_list.empty()) {
        line_matcher.TrackLineFromMap(pose_in_A, frame.line_data_list,
                                      *global_line_map, tracked_lane_data);
      }

      if (has_ikdtree && global_line_map) {
        ceres_matcher.Match(frame.pose_in_submap.translation(),
                            frame.pose_in_submap, frame.point_cloud,
                            tracked_lane_data, *global_grid, *global_line_map,
                            *global_cloud_ikdtree, &ceres_pose,
                            &ceres_summary);
      } else {
        ceres_matcher.Match(frame.pose_in_submap.translation(),
                            frame.pose_in_submap, frame.point_cloud,
                            *global_grid, &ceres_pose, &ceres_summary);
      }

      const ::cartographer::transform::Rigid2d ceres_pose_in_A =
          global_submap_pose_2d * ceres_pose;

      const ::cartographer::transform::Rigid2d T_2d =
          ceres_pose_in_A * frame.node_pose_in_B.inverse();

      sum_trans.x() += T_2d.translation().x();
      sum_trans.y() += T_2d.translation().y();
      sum_yaw += T_2d.rotation().angle();

      LOG(INFO) << "  [Ceres " << (i + 1) << "/" << consecutive.size()
                << "] T=(" << T_2d.translation().x() << ", "
                << T_2d.translation().y()
                << " yaw=" << T_2d.rotation().angle() << ") "
                << ceres_summary.BriefReport();
    }

    const int n = static_cast<int>(consecutive.size());
    const Eigen::Vector3d avg_trans = sum_trans / n;
    const double avg_yaw = sum_yaw / n;

    const ::cartographer::transform::Rigid3d T_B_in_A(
        avg_trans, Eigen::AngleAxisd(avg_yaw, Eigen::Vector3d::UnitZ()));

    LOG(INFO) << "Final T_src_in_tgt (Ceres avg of " << n
              << " frames): t=(" << avg_trans.x() << ", " << avg_trans.y()
              << ") yaw=" << avg_yaw;

    return {true, T_B_in_A};
  }

  LOG(ERROR) << "Failed to find " << consecutive_needed
             << " consecutive FCSM matches. "
             << "Best run: " << consecutive.size() << " frames. "
             << "Try lowering --min_score or ensuring maps overlap more.";
  return {false, ::cartographer::transform::Rigid3d::Identity()};
}

// ──────────────────────────────────────────────────
// 变换辅助
// ──────────────────────────────────────────────────

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
    const ::cartographer::transform::Rigid3d& T_old_to_new) {
  const auto old_lp =
      ::cartographer::transform::ToRigid3(submap_2d->local_pose());
  *submap_2d->mutable_local_pose() =
      ::cartographer::transform::ToProto(T_old_to_new * old_lp);

  const auto transform_cloud =
      [&T_old_to_new](::cartographer::mapping::proto::PointCloudData* cloud) {
        for (auto& pt : *cloud->mutable_points()) {
          const Eigen::Vector3d old_p(pt.x(), pt.y(), pt.z());
          const Eigen::Vector3d new_p = T_old_to_new * old_p;
          pt.set_x(new_p.x());
          pt.set_y(new_p.y());
          pt.set_z(new_p.z());
        }
      };
  if (submap_2d->has_cloud()) transform_cloud(submap_2d->mutable_cloud());
  if (submap_2d->has_highlight_cloud()) {
    transform_cloud(submap_2d->mutable_highlight_cloud());
  }

  if (submap_2d->has_line_map_2d_list()) {
    const auto T_2d = ::cartographer::transform::Project2D(T_old_to_new);
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
                          const ::cartographer::transform::Rigid3d& T_old_to_new) {
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

  Eigen::AlignedBox2d new_bbox;
  for (double x : {old_min.x(), old_max.x()}) {
    for (double y : {old_min.y(), old_max.y()}) {
      const Eigen::Vector3d v = T_old_to_new * Eigen::Vector3d(x, y, 0.0);
      new_bbox.extend(Eigen::Vector2d(v.x(), v.y()));
    }
  }
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

  std::vector<float> buf(new_num_x * new_num_y, -1.0f);
  size_t copied = 0, dropped = 0, conflict = 0;
  for (int i = 0; i < num_y; ++i) {
    for (int j = 0; j < num_x; ++j) {
      const Eigen::Array2i old_idx(j, i);
      if (!old_grid.IsKnown(old_idx)) continue;
      const float prob = old_grid.GetProbability(old_idx);
      const Eigen::Vector2f old_c = old_lim.GetCellCenter(old_idx);
      const Eigen::Vector3d new_p =
          T_old_to_new * Eigen::Vector3d(old_c.x(), old_c.y(), 0.0);
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

// ──────────────────────────────────────────────────
// pcd 输出（可选）
// ──────────────────────────────────────────────────

namespace {

::cartographer::sensor::PointCloud ProtoCloudToPointCloud(
    const ::cartographer::mapping::proto::PointCloudData& proto_cloud) {
  ::cartographer::sensor::PointCloud cloud;
  for (const auto& pt : proto_cloud.points()) {
    cloud.push_back({Eigen::Vector3f(pt.x(), pt.y(), pt.z())});
  }
  return cloud;
}

void WriteSinglePcd(const ::cartographer::mapping::proto::PointCloudData& proto_cloud,
                    const std::string& filename, const double resolution) {
  auto cloud = ProtoCloudToPointCloud(proto_cloud);
  if (cloud.empty()) {
    LOG(INFO) << "Cloud empty, skipping " << filename;
    return;
  }
  const auto filtered = ::cartographer::sensor::VoxelFilter(
      cloud, static_cast<float>(resolution));
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.resize(filtered.size());
  for (size_t i = 0; i < filtered.size(); ++i) {
    const auto& p = filtered.points()[i].position;
    pcl_cloud.points[i].x = p.x();
    pcl_cloud.points[i].y = p.y();
    pcl_cloud.points[i].z = p.z();
  }
  pcl::io::savePCDFileBinary(filename, pcl_cloud);
  LOG(INFO) << "Saved " << filename << " (" << filtered.size() << " points)";
}

}  // namespace

// ──────────────────────────────────────────────────
// 变换 + 重写（高层封装）
// ──────────────────────────────────────────────────

void TransformAndRewriteMap(
    MapData data, const ::cartographer::transform::Rigid3d& T_old_to_new,
    const std::string& output_pbstream, const std::string& output_filestem,
    const RewriteOptions& opts) {
  auto& pose_graph = data.pose_graph;
  auto& traj_options = data.traj_options;
  auto& all_records = data.all_records;

  LOG(INFO) << "Applying T_old_to_new: translation=("
            << T_old_to_new.translation().x() << ", "
            << T_old_to_new.translation().y() << "), rotation="
            << T_old_to_new.rotation().coeffs().transpose();

  // Step 1: 应用变换到 pose_graph 内所有全局位姿。
  size_t n_submap = 0, n_node = 0, n_landmark = 0;
  for (auto& trajectory : *pose_graph.mutable_trajectory()) {
    for (auto& submap : *trajectory.mutable_submap()) {
      const auto old_pose =
          ::cartographer::transform::ToRigid3(submap.pose());
      *submap.mutable_pose() =
          ::cartographer::transform::ToProto(T_old_to_new * old_pose);
      ++n_submap;
    }
    for (auto& node : *trajectory.mutable_node()) {
      const auto old_pose = ::cartographer::transform::ToRigid3(node.pose());
      *node.mutable_pose() =
          ::cartographer::transform::ToProto(T_old_to_new * old_pose);
      ++n_node;
    }
  }
  for (auto& landmark : *pose_graph.mutable_landmark_poses()) {
    const auto old_pose =
        ::cartographer::transform::ToRigid3(landmark.global_pose());
    *landmark.mutable_global_pose() =
        ::cartographer::transform::ToProto(T_old_to_new * old_pose);
    ++n_landmark;
  }
  LOG(INFO) << "Pose graph transformed: " << n_submap << " submap poses, "
            << n_node << " node poses, " << n_landmark << " landmark poses";

  // Step 2: 应用变换到 SerializedData（global_static_submap 非栅格部分 +
  // trajectory_data 的 fixed_frame_origin）。
  ::cartographer::mapping::proto::Submap2D* static_submap_proto = nullptr;
  size_t n_static = 0, n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
      TransformStaticSubmapNonGrid(rec.mutable_global_static_submap(),
                                   T_old_to_new);
      static_submap_proto = rec.mutable_global_static_submap();
      ++n_static;
    } else if (rec.has_trajectory_data() &&
               rec.trajectory_data().has_fixed_frame_origin_in_map()) {
      auto* td = rec.mutable_trajectory_data();
      const auto old_pose = ::cartographer::transform::ToRigid3(
          td->fixed_frame_origin_in_map());
      *td->mutable_fixed_frame_origin_in_map() =
          ::cartographer::transform::ToProto(T_old_to_new * old_pose);
      ++n_traj_data;
    }
  }
  LOG(INFO) << "SerializedData transformed (non-grid): " << n_static
            << " global_static_submap, " << n_traj_data
            << " trajectory_data fixed_frame_origin";
  CHECK(static_submap_proto != nullptr)
      << "Map has no global_static_submap; cannot rewrite. "
         "Regenerate the map first.";

  // Step 3: 从节点重建全局栅格（无节点时回退到重栅格化）。
  bool has_nodes = false;
  for (const auto& traj : pose_graph.trajectory()) {
    if (traj.node_size() > 0) {
      has_nodes = true;
      break;
    }
  }
  if (has_nodes) {
    *static_submap_proto->mutable_grid() = BuildFreshGridFromNodes(
        pose_graph, all_records, traj_options, opts.resolution,
        /*lidar_range_pad=*/30.0);
  } else if (static_submap_proto->has_grid()) {
    LOG(WARNING) << "pose_graph has no nodes; falling back to rerasterize";
    RerasterizeGridProto(static_submap_proto->mutable_grid(), T_old_to_new);
  }

  // Step 4: 渲染。
  cv::Mat origin_mat;
  Eigen::Vector2d map_origin;
  double render_resolution = opts.resolution;
  RenderSingleStaticSubmap(*static_submap_proto, opts.occ_thresh,
                           opts.free_thresh, &origin_mat, &map_origin,
                           &render_resolution);

  // Step 5: 写 png / yaml（统一走 WriteNavigationMap）。
  std::string map_dir;
  {
    const size_t last_slash = output_filestem.find_last_of('/');
    map_dir =
        (last_slash != std::string::npos) ? output_filestem.substr(0, last_slash) : ".";
  }
  const std::string png_file = output_filestem + ".png";
  const std::string yaml_file = output_filestem + ".yaml";
  WriteNavigationMap(origin_mat, png_file, opts.navigation_map_filter_mode,
                     opts.save_debug_map, map_dir);
  ::cartographer::io::StreamFileWriter yaml_writer(yaml_file);
  WriteYaml(render_resolution, map_origin, output_filestem, &yaml_writer,
            opts.occ_thresh, opts.free_thresh);
  if (opts.save_origin_map) {
    cv::imwrite(output_filestem + "_origin.png", origin_mat);
    LOG(INFO) << "Saved origin map to: " << output_filestem << "_origin.png";
  }
  LOG(INFO) << "Saved map to: " << png_file << " / " << yaml_file;

  // Step 6（可选）: 写 pcd / highlight pcd。
  if (opts.save_map_pcd) {
    if (static_submap_proto->has_cloud()) {
      WriteSinglePcd(static_submap_proto->cloud(), output_filestem + ".pcd",
                     render_resolution);
    }
    if (static_submap_proto->has_highlight_cloud()) {
      WriteSinglePcd(static_submap_proto->highlight_cloud(),
                     output_filestem + "_highlight.pcd", render_resolution);
    }
  }

  // Step 7: 原子写回 pbstream。
  const std::string tmp_pbstream = output_pbstream + ".tmp";
  {
    ::cartographer::io::ProtoStreamWriter writer(tmp_pbstream);
    writer.WriteProto(data.header);

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
  CHECK_EQ(std::rename(tmp_pbstream.c_str(), output_pbstream.c_str()), 0)
      << "Failed to rename tmp pbstream to " << output_pbstream;
  LOG(INFO) << "Pbstream written to: " << output_pbstream;
}

// ──────────────────────────────────────────────────
// map.yaml 配置读取
// ──────────────────────────────────────────────────

namespace {

// 去掉字符串首尾空白。
std::string Trim(const std::string& s) {
  const size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return "";
  const size_t e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// 若行以 "key:" 开头，返回冒号后的值（已去空白）；否则返回 nullopt 语义（空 + false）。
bool ParseYamlLine(const std::string& line, const std::string& key,
                   std::string* value) {
  const std::string prefix = key + ":";
  if (line.compare(0, prefix.size(), prefix) != 0) return false;
  *value = Trim(line.substr(prefix.size()));
  return true;
}

}  // namespace

MapYamlConfig ReadMapYamlConfig(const std::string& yaml_path) {
  MapYamlConfig cfg;
  std::ifstream in(yaml_path);
  if (!in.is_open()) {
    LOG(INFO) << "map.yaml not found, skip inheriting render config: "
              << yaml_path;
    return cfg;
  }
  std::string line;
  std::string value;
  while (std::getline(in, line)) {
    line = Trim(line);
    if (ParseYamlLine(line, "resolution", &value)) {
      std::istringstream ss(value);
      if (ss >> cfg.resolution) cfg.has_resolution = true;
    } else if (ParseYamlLine(line, "occupied_thresh", &value)) {
      std::istringstream ss(value);
      if (ss >> cfg.occ_thresh) cfg.has_occ_thresh = true;
    } else if (ParseYamlLine(line, "free_thresh", &value)) {
      std::istringstream ss(value);
      if (ss >> cfg.free_thresh) cfg.has_free_thresh = true;
    }
  }
  LOG(INFO) << "Read map.yaml config from " << yaml_path << ": resolution="
            << (cfg.has_resolution ? std::to_string(cfg.resolution) : "n/a")
            << " occupied_thresh="
            << (cfg.has_occ_thresh ? std::to_string(cfg.occ_thresh) : "n/a")
            << " free_thresh="
            << (cfg.has_free_thresh ? std::to_string(cfg.free_thresh) : "n/a");
  return cfg;
}

// ──────────────────────────────────────────────────
// 路径解析
// ──────────────────────────────────────────────────

std::string ResolveMapDir(const std::string& name_or_path,
                          const std::string& maps_root) {
  std::string root = maps_root;
  if (root.empty()) {
    const char* home = std::getenv("HOME");
    CHECK(home != nullptr) << "HOME env not set";
    root = std::string(home) + "/.dexe/data/maps";
  }
  const std::string default_path = root + "/" + name_or_path;
  const std::string direct_path = name_or_path;

  const bool default_ok =
      (access((default_path + "/map.pbstream").c_str(), F_OK) == 0);
  const bool direct_ok =
      (access((direct_path + "/map.pbstream").c_str(), F_OK) == 0);

  if (default_ok) {
    LOG(INFO) << "Resolved to default maps path: " << default_path;
    return default_path;
  } else if (direct_ok) {
    LOG(INFO) << "Resolved to direct path: " << direct_path;
    return direct_path;
  }
  LOG(FATAL) << "Map not found for: " << name_or_path << "\n"
             << "  Tried: " << default_path << "\n"
             << "  Tried: " << direct_path;
  return "";  // unreachable
}

}  // namespace map_ops
}  // namespace cartographer_ros
