/*
 * merge_maps_main.cc
 *
 * 离线工具：将两个地图 pbstream 合并为一个。
 *
 * 用法：
 *   ros2 run cartographer_ros merge_maps_tool \
 *     --map_target <地图名称或路径> \
 *     --map_source <地图名称或路径> \
 *     --output_dir <输出目录（可选，默认为 map_target 目录）>
 *
 * 工作流程：
 *   1. 加载 target map（trajectory 0）和 source map（trajectory 1）的 pbstream
 *   2. 用 source map 各节点的点云，在 target map 所有已完成 submap 上做全局 scan match
 *   3. 从成功匹配中投票选出 source map 原点在 target map 坐标系中的位姿 T_src_in_tgt
 *   4. 将 T_src_in_tgt 应用到 source map 所有 submap/node 的全局 pose，写入合并后的 pbstream
 *   5. 重新渲染 map.png + map.yaml
 *   6. 保留 source map 原文件不变（需用户手动删除）
 *
 * 注意：
 *   - 两个地图必须有重叠区域
 *   - source map 可能不是第一帧就在 target map 中完成重定位，工具会自动找到最佳匹配帧
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <limits>
#include <unistd.h>

#include "cartographer/common/port.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/pbstream_version.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros/tools/map_transform_common.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_target, "", "target map 地图名称或目录路径（trajectory 0，坐标系以此为准，输出覆盖此目录）");
DEFINE_string(map_source, "", "source map 地图名称或目录路径（trajectory 1，将被重定位到 target map 坐标系中）");
DEFINE_string(output_dir, "", "合并结果输出目录（可选，默认覆盖写入 target map 目录）");
DEFINE_double(min_score, 0.45, "scan match 最低接受分数（0~1），低于此分数的帧不参与投票");
DEFINE_int32(min_votes, 10, "连续 scan match 成功帧数达到 N 后触发 Ceres 精配准取均值");
DEFINE_double(resolution, 0.05, "输出地图分辨率（米/格）");
DEFINE_double(occ_thresh, 0.65, "占用阈值");
DEFINE_double(free_thresh, 0.196, "空闲阈值");
DEFINE_string(navigation_map_filter_mode, "normal",
              "导航图噪点滤除模式（与建图节点同名参数一致）");
DEFINE_bool(save_debug_map, false, "是否保存噪声过滤的中间调试图");

namespace cartographer_ros {
namespace {

// ──────────────────────────────────────────────────
// 数据结构
// ──────────────────────────────────────────────────

struct MapData {
  // pose_graph 中的 trajectory 信息（submap/node 位姿）
  ::cartographer::mapping::proto::PoseGraph pose_graph;

  // 序列化头 + options（用于写回 pbstream）
  ::cartographer::mapping::proto::SerializationHeader header;
  ::cartographer::mapping::proto::AllTrajectoryBuilderOptions traj_options;

  // 所有 SerializedData 记录（submap纹理、node、imu 等）
  std::vector<::cartographer::mapping::proto::SerializedData> all_records;

  // submap 切片（用于渲染）
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;

  // 全局静态 submap（pbstream 中的 kGlobalStaticSubmap，整张地图合并成单一 grid）
  // 用于 FCSM 全局匹配。conv_tables 必须比 global_submap 先析构（所以先声明）
  std::unique_ptr<::cartographer::mapping::ValueConversionTables>
      global_submap_conv_tables;
  std::unique_ptr<::cartographer::mapping::Submap2D> global_submap;
};

// ──────────────────────────────────────────────────
// 加载 pbstream
// ──────────────────────────────────────────────────

MapData LoadMap(const std::string& pbstream_file) {
  MapData data;

  ::cartographer::io::ProtoStreamReader reader(pbstream_file);

  // 手动读取前三个固定字段，跳过 ProtoStreamDeserializer 的轨迹数量 CHECK
  // （合并后的地图可能有多条轨迹，但 AllTrajectoryBuilderOptions 只有一条）
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
  // reader.eof() 不检查（直接读到失败即结束）

  // 提取 global_static_submap（整张地图合并的单一 grid，用于 FCSM 全局匹配）
  // 必须使用成员 conv_tables，避免悬空指针（ProbabilityGrid 内部持有其指针）
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
    LOG(WARNING) << "No global_static_submap found in pbstream. "
                 << "FCSM matching will fall back to trajectory submaps.";
  }

  // 构建 submap_slices（用于 mapA 的 scan match 和最终渲染）
  for (const auto& rec : data.all_records) {
    if (rec.has_submap()) {
      const auto& submap_proto = rec.submap();
      const ::cartographer::mapping::SubmapId id{
          submap_proto.submap_id().trajectory_id(),
          submap_proto.submap_id().submap_index()};
      // 找对应的 global pose
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
      ::cartographer::io::FillSubmapSlice(
          global_pose, submap_proto,
          &data.submap_slices[id],
          &data.conversion_tables);
    }
  }

  return data;
}

// ──────────────────────────────────────────────────
// 从 mapB 的节点中提取点云用于 scan match
// node_data.local_pose: tracking→local frame
// pose_graph node.pose: tracking→global B frame
// 点云在 gravity_aligned frame，local_pose 将其转到 local
// 节点的 global pose 将 local 转到 global B
// ──────────────────────────────────────────────────

struct NodeCloudEntry {
  int trajectory_id;
  int node_index;
  ::cartographer::transform::Rigid3d global_pose_b;  // 在 mapB 坐标系下的 global pose
  ::cartographer::sensor::PointCloud point_cloud;    // gravity-aligned 点云
  std::vector<::cartographer::sensor::LineData> line_data_list;  // gravity-aligned 线特征
};

std::vector<NodeCloudEntry> ExtractNodeClouds(const MapData& data_b) {
  // 建立 nodeId -> global_pose 的映射（来自 pose_graph）
  std::map<std::pair<int,int>, ::cartographer::transform::Rigid3d> node_global_poses;
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
      entry.line_data_list.push_back(::cartographer::sensor::FromProto(line_proto));
    }
    result.push_back(std::move(entry));
  }

  LOG(INFO) << "Extracted " << result.size() << " node point clouds from mapB";
  return result;
}

// ──────────────────────────────────────────────────
// 在 target map 上做全局 scan match，找到 T_src_in_tgt
// 逻辑：
//   - 使用 global_static_submap（整张 target map 合并成单一 grid）构建单个 FCSM
//   - 找到连续 consecutive_needed 帧 FCSM 成功后，
//     对这些帧各跑一次 Ceres 精配准，取平均变换
// ──────────────────────────────────────────────────

std::pair<bool, ::cartographer::transform::Rigid3d> FindTransformBinA(
    const MapData& data_a,
    const std::vector<NodeCloudEntry>& node_clouds_b,
    const double min_score,
    const int consecutive_needed) {
  // ─── 必须有 global_static_submap ───
  CHECK(data_a.global_submap != nullptr)
      << "target map has no global_static_submap in pbstream. "
         "Cannot do global FCSM matching.";
  const ::cartographer::mapping::Grid2D* global_grid =
      data_a.global_submap->grid();
  CHECK(global_grid != nullptr) << "global_static_submap has no grid";

  // ⚠️ global_static_submap 的 local_pose **不是**「submap 局部系 → global A 系」
  // 的变换，不得参与位姿运算。该 submap 的栅格本身就建立在全局系上
  // （MapLimits.max 取首节点的全局位姿，插入的 range data 也是全局坐标），
  // 所以 FCSM / Ceres 的输入输出都已经在全局系；local_pose 只是与
  // limits().max() 相等的记账值（这是 Submap2D 构造约定，非不变式：可被
  // set_local_pose() 改写）。需要地图中心请用 Submap2D::GridCenterXY()，
  // 定位侧曾把它当 ±15m 窗口中心，导致点云被裁空、启动失败并 SIGSEGV。
  // 定位侧的全局重定位（localization/internal/2d/pose_initializer_2d.cc）
  // 同样不引用该字段。
  //
  // 此处原先假设它「通常为 Identity」并把它乘进了匹配结果 —— 该假设不成立：
  // 建图直出的图它等于首节点全局 XY（接近 0 但非 0），而从 node 重建过
  // global_static_submap 的图（align / replace 产物）它等于点云包围盒角点，
  // 可达几十米。乘进去会让 T_src_in_tgt 整体平移同样的量。
  // 这里只打日志用于排查，不参与计算。
  const auto& tgt_static_local_pose = data_a.global_submap->local_pose();
  LOG(INFO) << "global_static_submap local_pose (diagnostic only, not used in "
               "pose math): ("
            << tgt_static_local_pose.translation().x() << ", "
            << tgt_static_local_pose.translation().y() << " yaw="
            << ::cartographer::transform::GetYaw(tgt_static_local_pose) << ")";

  // ─── 构建 FCSM（单个，基于全局 grid）───
  ::cartographer::mapping::scan_matching::proto::FastCorrelativeScanMatcherOptions2D
      fcsm_options;
  fcsm_options.set_linear_search_window(300.0);   // MatchFullSubmap 内部会使用全图尺寸
  fcsm_options.set_angular_search_window(M_PI);
  fcsm_options.set_branch_and_bound_depth(7);
  ::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D fcsm(
      *global_grid, fcsm_options);

  LOG(INFO) << "Built FCSM from global_static_submap";

  // ─── Ceres 精配准器 ───
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

  // ─── 线特征匹配器 & global submap 附加数据 ───
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

  // ─── 连续帧追踪 ───
  struct ConsecutiveFrame {
    ::cartographer::transform::Rigid2d pose_in_A;  // FCSM 返回（已在 global A 坐标系）
    ::cartographer::transform::Rigid2d node_pose_in_B;
    ::cartographer::sensor::PointCloud point_cloud;
    std::vector<::cartographer::sensor::LineData> line_data_list;  // gravity-aligned 线特征
  };
  std::vector<ConsecutiveFrame> consecutive;

  int node_count = 0;
  for (const auto& node_entry : node_clouds_b) {
    ++node_count;

    // 构建 2D 点云（gravity-aligned → 仅保留 XY）
    ::cartographer::sensor::PointCloud pc_2d;
    for (const auto& pt : node_entry.point_cloud) {
      pc_2d.push_back(
          {Eigen::Vector3f(pt.position.x(), pt.position.y(), 0.0f)});
    }
    if (pc_2d.empty()) {
      consecutive.clear();
      continue;
    }

    // ─── FCSM 全图搜索 ───
    float score = 0.0f;
    // 匹配器输出直接就在 global A 坐标系（栅格即全局系），命名如实反映。
    ::cartographer::transform::Rigid2d pose_in_A;
    const bool matched =
        fcsm.MatchFullSubmap(pc_2d, min_score, &score, &pose_in_A);

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

    consecutive.push_back({pose_in_A, node_pose_in_B, pc_2d,
                           node_entry.line_data_list});

    if (static_cast<int>(consecutive.size()) < consecutive_needed) continue;

    // ─── 连续 consecutive_needed 帧 FCSM 成功，开始 Ceres 精配准 ───
    LOG(INFO) << "Found " << consecutive_needed
              << " consecutive FCSM matches. Running Ceres fine registration...";

    Eigen::Vector3d sum_trans = Eigen::Vector3d::Zero();
    double sum_yaw = 0.0;
    for (int i = 0; i < static_cast<int>(consecutive.size()); ++i) {
      const auto& frame = consecutive[i];
      ::cartographer::transform::Rigid2d ceres_pose;
      ceres::Solver::Summary ceres_summary;

      // 线特征追踪：将节点的 gravity-aligned 线特征对齐到 global_line_map
      // frame.pose_in_A 已在 global A 系，直接使用
      std::map<int, std::vector<::cartographer::sensor::LineData>> tracked_lane_data;
      if (has_line_map && !frame.line_data_list.empty()) {
        line_matcher.TrackLineFromMap(frame.pose_in_A, frame.line_data_list,
                                     *global_line_map, tracked_lane_data);
      }

      // Ceres 精配准：优先使用 grid + 线特征 + ikdtree，fallback 到仅 grid
      if (has_ikdtree && global_line_map) {
        ceres_matcher.Match(frame.pose_in_A.translation(),
                            frame.pose_in_A, frame.point_cloud,
                            tracked_lane_data,
                            *global_grid, *global_line_map,
                            *global_cloud_ikdtree,
                            &ceres_pose, &ceres_summary);
      } else {
        ceres_matcher.Match(frame.pose_in_A.translation(),
                            frame.pose_in_A, frame.point_cloud,
                            *global_grid, &ceres_pose, &ceres_summary);
      }

      // T_src_in_tgt（2D）= (node 在 A 系的位姿) * node_pose_in_B^-1
      // ceres_pose 已在 global A 系（栅格即全局系）
      const ::cartographer::transform::Rigid2d T_2d =
          ceres_pose * frame.node_pose_in_B.inverse();

      sum_trans.x() += T_2d.translation().x();
      sum_trans.y() += T_2d.translation().y();
      sum_yaw += T_2d.rotation().angle();

      LOG(INFO) << "  [Ceres " << (i + 1) << "/" << consecutive.size()
                << "] T_src_in_tgt=(" << T_2d.translation().x() << ", "
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
    LOG(INFO) << "Transform found at node " << node_count << "/"
              << node_clouds_b.size() << ". Skipping remaining "
              << (node_clouds_b.size() - node_count) << " nodes.";

    return {true, T_B_in_A};
  }

  // 未找到足够的连续帧
  LOG(ERROR) << "Failed to find " << consecutive_needed
             << " consecutive FCSM matches. "
             << "Best run: " << consecutive.size() << " frames. "
             << "Try lowering --min_score or ensuring maps overlap more.";
  return {false, ::cartographer::transform::Rigid3d::Identity()};
}

// ──────────────────────────────────────────────────
// 主流程
// ──────────────────────────────────────────────────

void Run(const std::string& pbstream_a,
         const std::string& pbstream_b,
         const std::string& filestem_out,
         const double min_score,
         const int min_votes,
         const double resolution,
         const double occ_thresh,
         const double free_thresh) {
  LOG(INFO) << "Loading target map: " << pbstream_a;
  MapData data_a = LoadMap(pbstream_a);
  LOG(INFO) << "Loading source map: " << pbstream_b;
  MapData data_b = LoadMap(pbstream_b);

  LOG(INFO) << "target map submaps: " << data_a.submap_slices.size()
            << ", source map submaps: " << data_b.submap_slices.size();

  // Step 1: 提取 source map 的节点点云
  const auto node_clouds_b = ExtractNodeClouds(data_b);
  CHECK(!node_clouds_b.empty()) << "source map has no node point clouds to match";

  // Step 2: 在 target map 上做全局 scan match，估计 T_src_in_tgt
  auto [success, T_B_in_A] = FindTransformBinA(data_a, node_clouds_b, min_score, min_votes);
  CHECK(success) << "Failed to locate source map in target map. "
                    "Try lowering --min_score or ensuring maps have overlapping regions.";

  LOG(INFO) << "T_src_in_tgt confirmed. Merging trajectories...";

  // Step 3: 构建合并后的 pose_graph
  // target map 保持原有 trajectory_id，source map 的所有内容改为 traj_b_id
  // 并对其 global pose 施加 T_src_in_tgt
  ::cartographer::mapping::proto::PoseGraph merged_pg = data_a.pose_graph;

  // 计算 mapB 的 trajectory_id：取 mapA 所有轨迹的最大 id + 1，避免冲突
  int traj_b_id = 0;
  for (const auto& traj : data_a.pose_graph.trajectory()) {
    traj_b_id = std::max(traj_b_id, traj.trajectory_id() + 1);
  }
  LOG(INFO) << "Assigned trajectory_id=" << traj_b_id << " to source map (mapB)";

  // 添加 mapB 的 trajectory（trajectory_id=traj_b_id），修改 pose
  {
    ::cartographer::mapping::proto::Trajectory new_traj;
    // source map 原始 trajectory（应只有一条 traj，取 id=0 的）
    for (const auto& b_traj : data_b.pose_graph.trajectory()) {
      new_traj.set_trajectory_id(traj_b_id);
      for (const auto& sm : b_traj.submap()) {
        auto* new_sm = new_traj.add_submap();
        new_sm->set_submap_index(sm.submap_index());
        const auto old_pose = ::cartographer::transform::ToRigid3(sm.pose());
        const auto new_pose = T_B_in_A * old_pose;
        *new_sm->mutable_pose() = ::cartographer::transform::ToProto(new_pose);
      }
      for (const auto& nd : b_traj.node()) {
        auto* new_nd = new_traj.add_node();
        new_nd->set_node_index(nd.node_index());
        new_nd->set_timestamp(nd.timestamp());
        const auto old_pose = ::cartographer::transform::ToRigid3(nd.pose());
        const auto new_pose = T_B_in_A * old_pose;
        *new_nd->mutable_pose() = ::cartographer::transform::ToProto(new_pose);
      }
      break;  // 只处理第一条 trajectory
    }
    *merged_pg.add_trajectory() = new_traj;
  }

  // 合并产物的数据版本取两者较低者：只要有一张是旧图，合并后就有一部分节点
  // 没有原始 range data，不能标成新版本（见 cartographer/io/pbstream_version.h）。
  ::cartographer::mapping::proto::SerializationHeader merged_header =
      data_a.header;
  merged_header.set_data_version(::cartographer::io::MinPbstreamDataVersion(
      data_a.header.data_version(), data_b.header.data_version()));
  LOG(INFO) << "data_version: target "
            << ::cartographer::io::PbstreamDataVersionLabel(
                   data_a.header.data_version())
            << ", source "
            << ::cartographer::io::PbstreamDataVersionLabel(
                   data_b.header.data_version())
            << " -> merged "
            << ::cartographer::io::PbstreamDataVersionLabel(
                   merged_header.data_version());

  // Step 4: 构建合并的 SerializedData 列表（target map 原样保留，source map 改 tid=1）
  std::vector<::cartographer::mapping::proto::SerializedData> merged_records;
  // 先写 target map 的所有记录
  for (const auto& rec : data_a.all_records) {
    merged_records.push_back(rec);
  }
  // 再写 source map 的记录，修改 trajectory_id
  for (const auto& rec : data_b.all_records) {
    ::cartographer::mapping::proto::SerializedData new_rec = rec;

    if (rec.has_submap()) {
      auto* sm = new_rec.mutable_submap();
      sm->mutable_submap_id()->set_trajectory_id(traj_b_id);
      // submap 内嵌的 local_pose 不变（是 local frame 内部坐标，与 global 无关）
    } else if (rec.has_node()) {
      auto* nd = new_rec.mutable_node();
      nd->mutable_node_id()->set_trajectory_id(traj_b_id);
    } else if (rec.has_trajectory_data()) {
      new_rec.mutable_trajectory_data()->set_trajectory_id(traj_b_id);
    } else if (rec.has_imu_data()) {
      new_rec.mutable_imu_data()->set_trajectory_id(traj_b_id);
    } else if (rec.has_odometry_data()) {
      new_rec.mutable_odometry_data()->set_trajectory_id(traj_b_id);
    } else if (rec.has_fixed_frame_pose_data()) {
      new_rec.mutable_fixed_frame_pose_data()->set_trajectory_id(traj_b_id);
    } else if (rec.has_landmark_data()) {
      new_rec.mutable_landmark_data()->set_trajectory_id(traj_b_id);
    }
    merged_records.push_back(std::move(new_rec));
  }

  // Step 4b: 重建合并后的 global_static_submap。
  // 修掉三个缺陷（此前 Step 4 把**两份** global_static_submap 原样写进输出，
  // 且 B 那份连 T_B_in_A 都没施加 —— 产物里躺着一份旧坐标系的全局栅格，而全局
  // 重定位正是靠它）：
  //   ① 丢弃两份旧的，只留一份从合并后 node 重建的；
  //   ② 栅格 / 点云 / 线特征全部在合并后的系里重建（与 align / replace 同管线，
  //      DEC-003），不再是「只渲染不重建」；
  //   ③ 渲染改走 RenderSingleStaticSubmap + WriteNavigationMap，面积口径从此
  //      与其余工具及建图侧一致。
  {
    std::vector<::cartographer::mapping::proto::SerializedData> cleaned;
    cleaned.reserve(merged_records.size());
    int dropped = 0;
    for (auto& rec : merged_records) {
      if (rec.has_global_static_submap()) { ++dropped; continue; }
      cleaned.push_back(std::move(rec));
    }
    merged_records = std::move(cleaned);
    LOG(INFO) << "Dropped " << dropped
              << " stale global_static_submap record(s); rebuilding one from "
                 "the merged trajectory nodes";
    ::cartographer::mapping::proto::SerializedData rebuilt;
    *rebuilt.mutable_global_static_submap() =
        ::cartographer_ros::map_transform::RebuildGlobalStaticSubmapFromNodes(
            merged_pg, merged_records, data_a.traj_options, resolution,
            /*lidar_range_pad=*/30.0, merged_header.data_version());
    merged_records.push_back(std::move(rebuilt));
  }

  // Step 5: 写入合并后的 pbstream
  const std::string out_pbstream = filestem_out + ".pbstream";
  const std::string tmp_pbstream = out_pbstream + ".tmp";
  {
    ::cartographer::io::ProtoStreamWriter writer(tmp_pbstream);
    writer.WriteProto(merged_header);

    ::cartographer::mapping::proto::SerializedData pg_data;
    *pg_data.mutable_pose_graph() = merged_pg;
    writer.WriteProto(pg_data);

    // 轨迹数与 options 条数必须一致：ProtoStreamDeserializer 对此有硬 CHECK，
    // 而定位侧 LoadState 走的就是它。此前直接照抄 target 的 traj_options
    // （只有 1 条）而 pose_graph 有 2 条轨迹 ⇒ **合并出来的地图定位加载不了**，
    // 连 cartographer_pbstream check 都会 CHECK 失败。
    // merge 自己的 LoadMap 手动读前三个字段绕过了该 CHECK，所以这个缺陷一直
    // 只在工具内部被掩盖着。
    ::cartographer::mapping::proto::AllTrajectoryBuilderOptions merged_opts =
        data_a.traj_options;
    const int num_trajectories = merged_pg.trajectory_size();
    CHECK_GT(merged_opts.options_with_sensor_ids_size(), 0)
        << "target map has no trajectory builder options";
    while (merged_opts.options_with_sensor_ids_size() < num_trajectories) {
      // 两张图实践中用同一套建图配置；options 只影响续建 / 重建，复制末条即可。
      *merged_opts.add_options_with_sensor_ids() =
          merged_opts.options_with_sensor_ids(
              merged_opts.options_with_sensor_ids_size() - 1);
    }
    LOG(INFO) << "trajectory_builder_options padded to "
              << merged_opts.options_with_sensor_ids_size() << " entries for "
              << num_trajectories << " trajectories";
    ::cartographer::mapping::proto::SerializedData opts_data;
    *opts_data.mutable_all_trajectory_builder_options() = merged_opts;
    writer.WriteProto(opts_data);

    for (const auto& rec : merged_records) {
      writer.WriteProto(rec);
    }
    CHECK(writer.Close()) << "Failed to close pbstream writer";
  }
  CHECK_EQ(std::rename(tmp_pbstream.c_str(), out_pbstream.c_str()), 0)
      << "Failed to rename tmp pbstream";
  LOG(INFO) << "Merged pbstream written to: " << out_pbstream;

  // Step 6: 渲染合并后的 map.png + map.yaml。
  // 走与 align / replace / 建图侧同一条管线：渲染重建出的单个
  // global_static_submap，再经 WriteNavigationMap 做导航噪点滤除。
  // 此前是把 mapA / mapB 两套 submap slice 画到一起（PaintSubmapSlices +
  // WritePng），既没重建、也没滤波，面积口径与其余工具不可比。
  const auto* rebuilt_submap = [&]() -> const ::cartographer::mapping::proto::Submap2D* {
    for (const auto& rec : merged_records) {
      if (rec.has_global_static_submap()) return &rec.global_static_submap();
    }
    return nullptr;
  }();
  CHECK(rebuilt_submap != nullptr)
      << "internal error: rebuilt global_static_submap missing";

  cv::Mat origin_mat;
  Eigen::Vector2d origin;
  double render_resolution = resolution;
  ::cartographer_ros::map_transform::RenderSingleStaticSubmap(
      *rebuilt_submap, occ_thresh, free_thresh, &origin_mat, &origin,
      &render_resolution);

  const std::string png_file = filestem_out + ".png";
  const std::string yaml_file = filestem_out + ".yaml";
  std::string map_dir;
  {
    const size_t last_slash = filestem_out.find_last_of('/');
    map_dir = (last_slash != std::string::npos)
                  ? filestem_out.substr(0, last_slash)
                  : ".";
  }
  // 与 align / replace / 建图侧同一条出图路径：导航噪点滤除后再写 png，
  // 于是 effective_area 数的是**滤波后最终图**的自由像素，口径终于可比。
  cv::Mat filtered_mat;
  WriteNavigationMap(origin_mat, png_file, FLAGS_navigation_map_filter_mode,
                     FLAGS_save_debug_map, map_dir, &filtered_mat);
  const GridMapStats map_stats = ComputeGridMapStats(
      origin_mat, filtered_mat, render_resolution, free_thresh);
  // 以 target 的 yaml 为底合并：target 是坐标系宿主，合并是在它上面追加
  // source 的轨迹，不是换掉它的内容 ⇒ created_at、display_yaw 等沿用 target。
  // pbstream_a 形如 <dir>/map.pbstream，同目录的 map.yaml 即 target 的。
  MapYamlMergeOptions yaml_opts;
  yaml_opts.base_yaml_path =
      pbstream_a.substr(0, pbstream_a.find_last_of('.')) + ".yaml";
  CHECK(WriteMapYamlMerged(yaml_file,
                           MakeMapYamlEntries(render_resolution, origin,
                                              filestem_out, occ_thresh, free_thresh,
                                              merged_header.data_version(),
                                              &map_stats),
                           yaml_opts))
      << "Failed to write " << yaml_file;
  LOG(INFO) << "Map area recomputed: total " << map_stats.TotalAreaM2()
            << " m2, effective (free) " << map_stats.EffectiveAreaM2() << " m2";

  LOG(INFO) << "Merged map image written to: " << filestem_out << ".png/.yaml";
  LOG(INFO) << "Map size: " << origin_mat.cols << "x" << origin_mat.rows
            << " pixels";
  LOG(INFO) << "Map origin: (" << origin.x() << ", " << origin.y() << ")";
  LOG(INFO) << "Done. source map files are preserved at original location.";
}

// ──────────────────────────────────────────────────
// 路径解析（同 reset_map_origin_main.cc 逻辑）
// ──────────────────────────────────────────────────

// 传入 name_or_path，先检查 ~/.dexe/data/maps/<name>，再直接检查路径
std::string ResolveMapDir(const std::string& name_or_path) {
  const char* home = std::getenv("HOME");
  CHECK(home != nullptr) << "HOME env not set";
  const std::string default_path =
      std::string(home) + "/.dexe/data/maps/" + name_or_path;
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

}  // namespace
}  // namespace cartographer_ros

// ──────────────────────────────────────────────────
// main
// ──────────────────────────────────────────────────

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_target.empty()) << "--map_target is required";
  CHECK(!FLAGS_map_source.empty()) << "--map_source is required";

  const std::string dir_a = ::cartographer_ros::ResolveMapDir(FLAGS_map_target);
  const std::string dir_b = ::cartographer_ros::ResolveMapDir(FLAGS_map_source);

  // 输出目录默认为 target map 目录
  const std::string out_dir = FLAGS_output_dir.empty() ? dir_a : FLAGS_output_dir;
  const std::string filestem_out = out_dir + "/map";

  const std::string pbstream_a = dir_a + "/map.pbstream";
  const std::string pbstream_b = dir_b + "/map.pbstream";

  ::cartographer_ros::Run(
      pbstream_a, pbstream_b, filestem_out,
      FLAGS_min_score, FLAGS_min_votes,
      FLAGS_resolution, FLAGS_occ_thresh, FLAGS_free_thresh);

  rclcpp::shutdown();
  return 0;
}
