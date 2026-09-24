/*
 * map_ops.cc
 *
 * 地图离线工具共享库实现。见 map_ops.h 头部注释。
 * 本文件中的加载逻辑抽取自 merge_maps_main.cc，变换 / 重写 / 渲染逻辑
 * 抽取自 reset_map_origin_main.cc；为避免改动现有工具，这里是独立副本（命名空间
 * cartographer_ros::map_ops）。
 * 匹配部分（FindTransformBinA）已按《replace_map功能实现.md》重构为三阶段管线：
 * 关键帧筛选 → BatchFCSM 全局初对齐 → 增量加帧迭代至收敛，含四层 outlier 拒绝；
 * merge_maps_main.cc 仍使用其匿名命名空间里的原实现，互不影响。
 */

#include "cartographer_ros/tools/map_ops/map_ops.h"

#include "cartographer_ros/tools/map_transform_common.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

#include "cartographer/common/math.h"
#include "cartographer/common/time.h"
#include "cartographer/io/image.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/structure_alignment.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/batch_fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
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
// 提取节点点云 / 关键帧筛选
// ──────────────────────────────────────────────────

namespace {

std::vector<NodeCloudEntry> ExtractNodeCloudsImpl(
    const MapData& data_b, const std::set<std::pair<int, int>>* keep) {
  std::map<std::pair<int, int>, ::cartographer::transform::Rigid3d>
      node_global_poses;
  std::map<std::pair<int, int>, int64_t> node_timestamps;
  for (const auto& traj : data_b.pose_graph.trajectory()) {
    for (const auto& node : traj.node()) {
      const auto key = std::make_pair(traj.trajectory_id(), node.node_index());
      node_global_poses[key] = ::cartographer::transform::ToRigid3(node.pose());
      node_timestamps[key] = node.timestamp();
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
    if (keep != nullptr && keep->count(key) == 0) continue;

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
    entry.timestamp = node_timestamps[key];
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

}  // namespace

std::vector<NodeCloudEntry> ExtractNodeClouds(const MapData& data) {
  return ExtractNodeCloudsImpl(data, /*keep=*/nullptr);
}

std::vector<NodeCloudEntry> ExtractNodeClouds(
    const MapData& data, const std::vector<std::pair<int, int>>& selected) {
  const std::set<std::pair<int, int>> keep(selected.begin(), selected.end());
  auto result = ExtractNodeCloudsImpl(data, &keep);
  std::stable_sort(result.begin(), result.end(),
                   [](const NodeCloudEntry& a, const NodeCloudEntry& b) {
                     return a.timestamp < b.timestamp;
                   });
  return result;
}

std::vector<std::pair<int, int>> SelectKeyframes(
    const MapData& data, const KeyframeSelectionOptions& opts) {
  // 带点云的节点集合（只查 proto 标志，不解压点云）。
  std::set<std::pair<int, int>> has_cloud;
  for (const auto& rec : data.all_records) {
    if (!rec.has_node()) continue;
    const auto& node_proto = rec.node();
    if (!node_proto.node_data().has_filtered_gravity_aligned_point_cloud()) {
      continue;
    }
    has_cloud.insert({node_proto.node_id().trajectory_id(),
                      node_proto.node_id().node_index()});
  }

  struct KeyCandidate {
    int trajectory_id;
    int node_index;
    int64_t timestamp;
  };

  // 每条轨迹独立跑纯距离/角度 MotionFilter（max_time_seconds 给极大值，
  // 时间条件永不触发），按时间戳顺序喂帧。
  const auto select_with = [&](const double max_distance,
                               const double max_angle) {
    std::vector<KeyCandidate> kept;
    for (const auto& traj : data.pose_graph.trajectory()) {
      ::cartographer::mapping::proto::MotionFilterOptions mf_options;
      mf_options.set_max_time_seconds(1e9);
      mf_options.set_max_distance_meters(max_distance);
      mf_options.set_max_angle_radians(max_angle);
      ::cartographer::mapping::MotionFilter filter(mf_options);

      // 轨迹节点按时间戳排序后再喂滤波器（proto 顺序通常已是时序，此处兜底）。
      std::vector<const ::cartographer::mapping::proto::Trajectory::Node*>
          nodes;
      for (const auto& node : traj.node()) nodes.push_back(&node);
      std::sort(nodes.begin(), nodes.end(),
                [](const auto* a, const auto* b) {
                  return a->timestamp() < b->timestamp();
                });

      for (const auto* node : nodes) {
        const auto key =
            std::make_pair(traj.trajectory_id(), node->node_index());
        if (has_cloud.count(key) == 0) continue;
        const auto time =
            ::cartographer::common::FromUniversal(node->timestamp());
        const auto pose = ::cartographer::transform::ToRigid3(node->pose());
        if (!filter.IsSimilar(time, pose)) {
          kept.push_back(
              {traj.trajectory_id(), node->node_index(), node->timestamp()});
        }
      }
    }
    return kept;
  };

  double distance = opts.max_distance_meters;
  double angle = opts.max_angle_radians;
  auto kept = select_with(distance, angle);
  while (static_cast<int>(kept.size()) < opts.min_frames &&
         distance >= 0.25) {
    LOG(INFO) << "SelectKeyframes: only " << kept.size()
              << " keyframes with spacing " << distance << " m / " << angle
              << " rad, tightening";
    distance *= 0.5;
    angle *= 0.5;
    kept = select_with(distance, angle);
  }
  if (static_cast<int>(kept.size()) < opts.min_frames) {
    // 兜底：取全部带点云节点（短轨迹场景）。
    kept.clear();
    for (const auto& traj : data.pose_graph.trajectory()) {
      for (const auto& node : traj.node()) {
        const auto key =
            std::make_pair(traj.trajectory_id(), node.node_index());
        if (has_cloud.count(key) == 0) continue;
        kept.push_back(
            {traj.trajectory_id(), node.node_index(), node.timestamp()});
      }
    }
    LOG(WARNING) << "SelectKeyframes: falling back to all " << kept.size()
                 << " cloud-bearing nodes (spacing fallback exhausted)";
  }

  std::stable_sort(kept.begin(), kept.end(),
                   [](const KeyCandidate& a, const KeyCandidate& b) {
                     return a.timestamp < b.timestamp;
                   });

  // 候选池上限：按时间序等间隔抽稀，保持时空分布。
  if (static_cast<int>(kept.size()) > opts.max_selected &&
      opts.max_selected > 0) {
    const size_t stride =
        (kept.size() + opts.max_selected - 1) / opts.max_selected;
    std::vector<KeyCandidate> decimated;
    for (size_t i = 0; i < kept.size(); i += stride) {
      decimated.push_back(kept[i]);
    }
    LOG(INFO) << "SelectKeyframes: decimated " << kept.size() << " -> "
              << decimated.size() << " (stride " << stride << ")";
    kept = std::move(decimated);
  }

  LOG(INFO) << "SelectKeyframes: kept " << kept.size()
            << " keyframes (spacing " << distance << " m / " << angle
            << " rad)";
  std::vector<std::pair<int, int>> result;
  result.reserve(kept.size());
  for (const auto& k : kept) {
    result.emplace_back(k.trajectory_id, k.node_index);
  }
  return result;
}

// ──────────────────────────────────────────────────
// 全局重定位（三阶段管线）
// ──────────────────────────────────────────────────

namespace {

::cartographer::sensor::PointCloud To2DCloud(
    const ::cartographer::sensor::PointCloud& cloud) {
  ::cartographer::sensor::PointCloud pc_2d;
  for (const auto& pt : cloud) {
    pc_2d.push_back(
        {Eigen::Vector3f(pt.position.x(), pt.position.y(), 0.0f)});
  }
  return pc_2d;
}

// 轻量位姿打分：复刻自
// cartographer/localization/internal/2d/pose_initializer_2d.cc 的
// PoseInitializer::ScorePoseAgainstGrid（仅 ProbabilityGrid 分支，该函数为
// 私有成员无法直接复用）。位姿下扫描点平均占据概率，O(点数)，不做搜索。
// first=false 表示栅格类型不支持打分（调用方不得据此拒绝）。
std::pair<bool, float> ScorePoseAgainstGrid(
    const ::cartographer::transform::Rigid2d& pose,
    const ::cartographer::sensor::PointCloud& point_cloud,
    const ::cartographer::mapping::Grid2D& grid) {
  if (point_cloud.empty()) return {true, 0.f};
  if (grid.GetGridType() !=
      ::cartographer::mapping::GridType::PROBABILITY_GRID) {
    return {false, 0.f};
  }
  const auto& limits = grid.limits();
  const auto& probability_grid =
      static_cast<const ::cartographer::mapping::ProbabilityGrid&>(grid);
  float score = 0.f;
  for (const auto& point : point_cloud) {
    const Eigen::Vector2d world =
        pose * Eigen::Vector2d(point.position.x(), point.position.y());
    const Eigen::Array2i cell = limits.GetCellIndex(world.cast<float>());
    score += probability_grid.GetProbability(cell);  // 越界返回 kMinProbability
  }
  return {true, score / static_cast<float>(point_cloud.size())};
}

double Median(std::vector<double> values) {
  const size_t n = values.size();
  CHECK_GT(n, 0u);
  std::sort(values.begin(), values.end());
  return values[n / 2];
}

// L3 鲁棒聚合：分量中位数 + yaw 圆中位数 → 一致性剔除 + 相对分数剔除 → 内点均值。
// refine_scores 与 Ts 等长；相对分数剔除以【T-内点中的最高分】为基准
// （不能用全池最高分：一个高分别名帧会抬高门槛、误杀全部好帧，再触发
// kept==0 兜底反而放行别名）。score_relative_margin<=0 关闭相对剔除。
// inlier_mask[i] 标记参与最终均值的帧。要求 Ts 非空。
::cartographer::transform::Rigid2d AggregateTransforms(
    const std::vector<::cartographer::transform::Rigid2d>& Ts,
    const std::vector<float>& refine_scores, const double outlier_translation,
    const double outlier_yaw, const double score_relative_margin,
    std::vector<bool>* inlier_mask) {
  const int n = static_cast<int>(Ts.size());
  CHECK_GT(n, 0);
  CHECK_EQ(refine_scores.size(), Ts.size());
  std::vector<double> xs(n), ys(n), yaws(n);
  for (int i = 0; i < n; ++i) {
    xs[i] = Ts[i].translation().x();
    ys[i] = Ts[i].translation().y();
    yaws[i] = Ts[i].rotation().angle();
  }

  const double med_x = Median(xs);
  const double med_y = Median(ys);
  // 圆中位数：取使圆距离和最小的样本（n<=200，O(n^2) 足够）。
  double med_yaw = yaws[0];
  double best_sum = std::numeric_limits<double>::infinity();
  for (const double cand : yaws) {
    double sum = 0.0;
    for (const double y : yaws) {
      sum += std::abs(
          ::cartographer::common::NormalizeAngleDifference(y - cand));
    }
    if (sum < best_sum) {
      best_sum = sum;
      med_yaw = cand;
    }
  }

  // 第一遍：标 T-内点，并记录 T-内点中的最高 refine 分数。
  inlier_mask->assign(n, false);
  float max_inlier_score = -std::numeric_limits<float>::infinity();
  for (int i = 0; i < n; ++i) {
    const double dt = std::hypot(xs[i] - med_x, ys[i] - med_y);
    const double dyaw = std::abs(
        ::cartographer::common::NormalizeAngleDifference(yaws[i] - med_yaw));
    if (dt > outlier_translation || dyaw > outlier_yaw) continue;
    (*inlier_mask)[i] = true;
    max_inlier_score = std::max(max_inlier_score, refine_scores[i]);
  }

  // 第二遍：T-内点中再按相对分数剔除，累加均值。
  double sum_x = 0.0, sum_y = 0.0, sum_sin = 0.0, sum_cos = 0.0;
  int kept = 0;
  for (int i = 0; i < n; ++i) {
    if (!(*inlier_mask)[i]) continue;
    if (score_relative_margin > 0.0 &&
        refine_scores[i] < max_inlier_score - score_relative_margin) {
      (*inlier_mask)[i] = false;
      continue;
    }
    sum_x += xs[i];
    sum_y += ys[i];
    sum_sin += std::sin(yaws[i]);
    sum_cos += std::cos(yaws[i]);
    ++kept;
  }
  // 中位数样本自身偏差为 0，kept 至少为 1；防御性兜底。
  if (kept == 0) {
    inlier_mask->assign(n, true);
    kept = n;
    for (int i = 0; i < n; ++i) {
      sum_x += xs[i];
      sum_y += ys[i];
      sum_sin += std::sin(yaws[i]);
      sum_cos += std::cos(yaws[i]);
    }
  }
  return ::cartographer::transform::Rigid2d(
      Eigen::Vector2d(sum_x / kept, sum_y / kept),
      std::atan2(sum_sin, sum_cos));
}

// 最远点贪心排序：种子取首帧，之后每次选"距已选集合最小距离最大"的帧。
// 返回 frames 下标的完整排列，供 Stage B 取前 K、Stage C 忽略。
std::vector<size_t> FarthestPointOrder(
    const std::vector<NodeCloudEntry>& frames) {
  const size_t n = frames.size();
  std::vector<size_t> order;
  order.reserve(n);
  if (n == 0) return order;
  std::vector<double> min_dist(n, std::numeric_limits<double>::infinity());
  std::vector<char> chosen(n, 0);
  size_t next = 0;
  for (size_t k = 0; k < n; ++k) {
    order.push_back(next);
    chosen[next] = 1;
    const Eigen::Vector2d p =
        frames[next].global_pose_b.translation().head<2>();
    size_t best_idx = n;
    double best_d = -1.0;
    for (size_t i = 0; i < n; ++i) {
      if (chosen[i]) continue;
      const double d =
          (frames[i].global_pose_b.translation().head<2>() - p).norm();
      if (d < min_dist[i]) min_dist[i] = d;
      if (min_dist[i] > best_d) {
        best_d = min_dist[i];
        best_idx = i;
      }
    }
    if (best_idx == n) break;
    next = best_idx;
  }
  return order;
}

}  // namespace

FindTransformResult FindTransformBinA(
    const MapData& data_a, const std::vector<NodeCloudEntry>& frames_b,
    const FindTransformOptions& opts) {
  FindTransformResult result;
  const auto t_start = std::chrono::steady_clock::now();
  const auto elapsed_s = [&t_start]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         t_start)
        .count();
  };

  CHECK(data_a.global_submap != nullptr)
      << "target map has no global_static_submap in pbstream. "
         "Cannot do global FCSM matching.";
  const ::cartographer::mapping::Grid2D* global_grid =
      data_a.global_submap->grid();
  CHECK(global_grid != nullptr) << "global_static_submap has no grid";

  // ⚠️ global_static_submap 的 local_pose **不是**「submap 系 → 全局系」的变换，
  // 不得参与任何位姿运算。理由：
  //   - 该 submap 的栅格本身就建立在全局系上（MapLimits.max 取首节点的全局位姿，
  //     插入的 range data 也是全局坐标），所以匹配器（FCSM / Ceres）的输入输出
  //     都已经在全局系；local_pose 只是与 limits().max() 相等的记账值。
  //   - 定位侧的全局重定位（localization/internal/2d/pose_initializer_2d.cc）
  //     同样不引用该字段，匹配器输出直接当作机器人的全局位姿。
  // 普通 submap 的 local_pose 语义不同（配合 submap 相对位姿使用），不要照搬。
  // 这里只打日志用于排查，不参与计算。
  const auto& tgt_static_local_pose = data_a.global_submap->local_pose();
  LOG(INFO) << "global_static_submap local_pose (diagnostic only, not used in "
               "pose math): ("
            << tgt_static_local_pose.translation().x() << ", "
            << tgt_static_local_pose.translation().y() << " yaw="
            << ::cartographer::transform::GetYaw(tgt_static_local_pose) << ")";

  if (frames_b.empty()) {
    LOG(ERROR) << "FindTransformBinA: no frames to match";
    result.stop_reason = "no_frames";
    result.elapsed_seconds = elapsed_s();
    return result;
  }

  // ── 匹配器准备 ──
  // 参数优先来自调用方从 lua 的 POSE_INITIALIZER_2D 读到的配置（见
  // FindTransformOptions 注释）。本函数与定位节点的 PoseInitializer 做同一件事，
  // 参数必须同源，否则两边会静默跑成两套算法。
  // 下面这组内置默认仅在调用方未提供 lua 时生效，是历史遗留值，**与生产配置
  // 差 1~2 个数量级**（例如 translation_weight 0.1 vs 20），且不含
  // use_improved_line_cost / huber 尺度，所以走这条路会打 WARNING。
  constexpr double kGlobalLinearSearchWindow = 300.0;  // m，等价全图搜索
  constexpr int kFallbackBranchAndBoundDepth = 7;

  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      ceres_options;
  ::cartographer::mapping::scan_matching::proto::LineMatcherOptions
      line_matcher_options;
  int branch_and_bound_depth = kFallbackBranchAndBoundDepth;
  if (opts.has_matcher_options) {
    ceres_options = opts.ceres_options;
    line_matcher_options = opts.line_matcher_options;
    branch_and_bound_depth = opts.branch_and_bound_depth;
    LOG(INFO) << "Matcher params source: lua (POSE_INITIALIZER_2D)";
  } else {
    ceres_options.set_occupied_space_weight(1.0);
    ceres_options.set_translation_weight(0.1);
    ceres_options.set_rotation_weight(0.1);
    ceres_options.set_line_weight(1.0);
    ceres_options.set_point_weight(1.0);
    ceres_options.mutable_ceres_solver_options()->set_max_num_iterations(20);
    ceres_options.mutable_ceres_solver_options()->set_num_threads(4);
    line_matcher_options.set_line_delta_theta_threshold(0.2);
    line_matcher_options.set_line_delta_tra_elongation_threshold(0.5);
    line_matcher_options.set_min_delta_tra_parallel(0.5);
    line_matcher_options.set_max_match_distance(30.0);
    LOG(WARNING) << "Matcher params source: BUILT-IN FALLBACK (no lua "
                    "configuration supplied). These differ from the production "
                    "localization config by 1-2 orders of magnitude and lack "
                    "improved line cost — pass --configuration_directory / "
                    "--configuration_basename to use the real config.";
  }
  ::cartographer::mapping::scan_matching::CeresScanMatcher2D ceres_matcher(
      ceres_options);
  ::cartographer::mapping::scan_matching::LineMatcher2D line_matcher(
      line_matcher_options);

  const int kBranchAndBoundDepth = branch_and_bound_depth;
  LOG(INFO) << "Matcher config | ceres: occ="
            << ceres_options.occupied_space_weight()
            << " trans=" << ceres_options.translation_weight()
            << " rot=" << ceres_options.rotation_weight()
            << " line=" << ceres_options.line_weight()
            << " point=" << ceres_options.point_weight()
            << " corr_dist=" << ceres_options.max_correspondence_distance()
            << " improved_line=" << ceres_options.use_improved_line_cost()
            << " iters="
            << ceres_options.ceres_solver_options().max_num_iterations()
            << " threads="
            << ceres_options.ceres_solver_options().num_threads()
            << " | line: dtheta="
            << line_matcher_options.line_delta_theta_threshold()
            << " delong="
            << line_matcher_options.line_delta_tra_elongation_threshold()
            << " dparallel=" << line_matcher_options.min_delta_tra_parallel()
            << " dmax=" << line_matcher_options.max_match_distance()
            << " | stageB: linear=" << kGlobalLinearSearchWindow
            << "m angular=pi depth=" << kBranchAndBoundDepth
            << " | stageC: linear=" << opts.local_linear_window
            << "m angular=" << opts.local_angular_window
            << "rad depth=" << kBranchAndBoundDepth;

  const ::cartographer::mapping::LineMap2DList* global_line_map =
      data_a.global_submap->line_map_2d_list();
  ::cartographer::ikd_tree::iKdTree<pcl::PointXYZ>* global_cloud_ikdtree =
      data_a.global_submap->cloud_ikdtree();
  const bool has_line_map = global_line_map && global_line_map->size() > 0;
  const bool has_ikdtree = global_cloud_ikdtree != nullptr;
  LOG(INFO) << "Line map available: " << has_line_map
            << ", IKDTree available: " << has_ikdtree;

  // Ceres 精化 + L2 后门（Ceres 收敛性 + 位姿打分）。
  struct RefineResult {
    bool ok = false;
    bool ceres_ok = false;  // Ceres 收敛（此时 ceres_pose 才有意义）
    ::cartographer::transform::Rigid2d ceres_pose;  // target 地图全局系
    float refine_score = 0.f;
    std::string reject_reason;
  };
  // 全部观测到的 refine 分数（过/不过门都记），供失败时数据化提示。
  std::vector<float> observed_refine;
  // 'pose_in_A'：待精化的节点位姿，**已在 target 地图全局系**（匹配器的直接输出，
  // 不需要也不允许再乘 global_static_submap 的 local_pose，理由见上方注释）。
  auto refine_frame =
      [&](const ::cartographer::transform::Rigid2d& pose_in_A,
          const ::cartographer::sensor::PointCloud& pc_2d,
          const std::vector<::cartographer::sensor::LineData>& lines) {
        RefineResult rr;

        std::map<int, std::vector<::cartographer::sensor::LineData>>
            tracked_lane_data;
        if (has_line_map && !lines.empty()) {
          line_matcher.TrackLineFromMap(pose_in_A, lines, *global_line_map,
                                        tracked_lane_data);
        }

        ceres::Solver::Summary summary;
        if (has_ikdtree && global_line_map) {
          ceres_matcher.Match(pose_in_A.translation(), pose_in_A,
                              pc_2d, tracked_lane_data, *global_grid,
                              *global_line_map, *global_cloud_ikdtree,
                              &rr.ceres_pose, &summary);
        } else {
          ceres_matcher.Match(pose_in_A.translation(), pose_in_A,
                              pc_2d, *global_grid, &rr.ceres_pose, &summary);
        }

        if (summary.termination_type != ceres::TerminationType::CONVERGENCE) {
          rr.reject_reason = std::string("ceres no convergence: ") +
                             ceres::TerminationTypeToString(
                                 summary.termination_type);
          return rr;
        }
        rr.ceres_ok = true;

        const auto [scored, refine_score] =
            ScorePoseAgainstGrid(rr.ceres_pose, pc_2d, *global_grid);
        if (scored) observed_refine.push_back(refine_score);
        rr.refine_score = scored ? refine_score : 1.f;
        if (scored && refine_score < opts.refine_min_score) {
          rr.reject_reason = "refine_score=" + std::to_string(refine_score) +
                             " < " + std::to_string(opts.refine_min_score);
          return rr;
        }
        rr.ok = true;
        return rr;
      };

  struct Candidate {
    ::cartographer::transform::Rigid2d T;  // src 原点 -> tgt 系
    float score = 0.f;
    float refine_score = 0.f;
  };
  std::vector<Candidate> pool;

  // ── Stage B：BatchFCSM 全局初对齐 ──
  ::cartographer::mapping::scan_matching::proto::
      FastCorrelativeScanMatcherOptions2D global_options;
  global_options.set_linear_search_window(kGlobalLinearSearchWindow);
  global_options.set_angular_search_window(M_PI);
  global_options.set_branch_and_bound_depth(kBranchAndBoundDepth);
  auto precomputation_stack =
      data_a.global_submap->GetOrBuildPrecomputationGridStack(global_options);
  ::cartographer::mapping::scan_matching::BatchFastCorrelativeScanMatcher2D
      batch_matcher(*global_grid, global_options, precomputation_stack);
  batch_matcher.set_num_threads(opts.bbs_num_threads);
  batch_matcher.set_timeout_ms(opts.bbs_timeout_ms);
  batch_matcher.set_use_angular_branching(opts.bbs_angular_branching);
  if (opts.bbs_angular_branching) {
    batch_matcher.set_angular_block_shift(2);
  }
  LOG(INFO) << "Built BatchFCSM from global_static_submap (precomputation "
            << batch_matcher.precomputation_ms() << " ms, threads="
            << opts.bbs_num_threads
            << ", angular_branching=" << opts.bbs_angular_branching << ")";

  const std::vector<size_t> fp_order = FarthestPointOrder(frames_b);
  std::vector<char> consumed(frames_b.size(), 0);

  // 鲁棒聚合（L3）：唯一调用点，Stage B/C 共用。
  std::vector<bool> inlier_mask;
  auto aggregate_pool = [&]() {
    std::vector<::cartographer::transform::Rigid2d> Ts;
    std::vector<float> scores;
    Ts.reserve(pool.size());
    scores.reserve(pool.size());
    for (const auto& c : pool) {
      Ts.push_back(c.T);
      scores.push_back(c.refine_score);
    }
    return AggregateTransforms(Ts, scores, opts.outlier_translation,
                               opts.outlier_yaw, opts.score_relative_margin,
                               &inlier_mask);
  };
  const auto count_inliers = [&]() {
    return static_cast<int>(
        std::count(inlier_mask.begin(), inlier_mask.end(), true));
  };

  // 数据化失败提示：打印观测 refine 分数分布与具体调参建议。
  auto log_score_hints = [&]() {
    if (!observed_refine.empty()) {
      std::vector<float> sorted = observed_refine;
      std::sort(sorted.begin(), sorted.end());
      char p25[16];
      snprintf(p25, sizeof(p25), "%.2f", sorted[sorted.size() / 4]);
      LOG(ERROR) << "observed refine_score: min=" << sorted.front()
                 << " median=" << sorted[sorted.size() / 2]
                 << " max=" << sorted.back() << " (n=" << sorted.size() << ")";
      if (result.rejected_l2 > result.global_matched) {
        LOG(ERROR) << "  hint: most frames died at the L2 score gate (absolute "
                      "score scale varies across map pairs); try "
                      "--refine_min_score=" << p25;
      }
    }
    if (pool.size() >= 2) {
      std::vector<bool> m;
      std::vector<::cartographer::transform::Rigid2d> Ts;
      std::vector<float> sc;
      for (const auto& c : pool) {
        Ts.push_back(c.T);
        sc.push_back(c.refine_score);
      }
      AggregateTransforms(Ts, sc, opts.outlier_translation, opts.outlier_yaw,
                          opts.score_relative_margin, &m);
      if (std::all_of(m.begin(), m.end(), [](bool b) { return b; })) {
        LOG(ERROR) << "  hint: all " << pool.size()
                   << " accepted frames agree tightly; try "
                      "--min_success_frames=" << pool.size();
      }
    }
  };

  // ── Stage B：BatchFCSM 全局初对齐（共识驱动停止）──
  // 每接受一帧重新聚合，仅当剔除后内点数 >= initial_frames 才停（或耗尽/超预算）：
  // 防止初始帧里别名多数把中位数锁到错误模式（Stage C 的 3m 预测窗口无法自救）。
  ::cartographer::transform::Rigid2d delta_T;
  for (size_t oi = 0; oi < fp_order.size(); ++oi) {
    if (elapsed_s() > opts.budget_seconds) {
      result.stop_reason = "budget";
      break;
    }
    const size_t fi = fp_order[oi];
    const auto& frame = frames_b[fi];
    const ::cartographer::sensor::PointCloud pc_2d =
        To2DCloud(frame.point_cloud);
    if (pc_2d.empty()) {
      consumed[fi] = 1;
      continue;
    }

    float score = 0.f;
    // 匹配器输出直接就在 target 地图全局系（栅格即全局系），命名如实反映。
    ::cartographer::transform::Rigid2d pose_in_A;
    ::cartographer::mapping::scan_matching::BatchMatchStats stats;
    const bool matched = batch_matcher.MatchFullSubmap(
        pc_2d, static_cast<float>(opts.min_score), &score, &pose_in_A,
        &stats);
    LOG(INFO) << "[StageB] frame " << frame.trajectory_id << ":"
              << frame.node_index << " BBS score=" << score << " t="
              << stats.t_total_ms << "ms"
              << (stats.timed_out ? " TIMEOUT" : "")
              << (matched ? "" : " (rejected L1)");
    if (!matched) {
      ++result.rejected_l1;
      continue;  // 失败帧不消耗资格，Stage C 可用 delta_T 预测重试
    }
    consumed[fi] = 1;

    const auto rr =
        refine_frame(pose_in_A, pc_2d, frame.line_data_list);
    if (!rr.ok) {
      ++result.rejected_l2;
      LOG(INFO) << "  frame " << frame.trajectory_id << ":" << frame.node_index
                << " rejected L2: " << rr.reject_reason;
      if (rr.ceres_ok) {  // 记录被拒帧的 T，便于核对"被拒帧是否其实正确"
        const ::cartographer::transform::Rigid2d node_pose_in_B(
            frame.global_pose_b.translation().head<2>(),
            ::cartographer::transform::GetYaw(frame.global_pose_b));
        const ::cartographer::transform::Rigid2d T_rej =
            rr.ceres_pose * node_pose_in_B.inverse();
        LOG(INFO) << "    rejected T=(" << T_rej.translation().x() << ", "
                  << T_rej.translation().y()
                  << " yaw=" << T_rej.rotation().angle() << ")";
      }
      continue;
    }
    const ::cartographer::transform::Rigid2d node_pose_in_B(
        frame.global_pose_b.translation().head<2>(),
        ::cartographer::transform::GetYaw(frame.global_pose_b));
    // T_src_in_tgt = (node 在 tgt 全局系的位姿) * (node 在 src 系位姿)⁻¹
    const ::cartographer::transform::Rigid2d T =
        rr.ceres_pose * node_pose_in_B.inverse();
    pool.push_back({T, score, rr.refine_score});
    ++result.global_matched;
    delta_T = aggregate_pool();
    result.inliers = count_inliers();
    LOG(INFO) << "  accepted: T=(" << T.translation().x() << ", "
              << T.translation().y()
              << " yaw=" << T.rotation().angle() << ") refine_score="
              << rr.refine_score << " inliers=" << result.inliers << "/"
              << opts.initial_frames;
    if (result.inliers >= opts.initial_frames) break;
  }

  if (pool.empty() || result.inliers < opts.min_success_frames) {
    LOG(ERROR) << "Only " << result.inliers << " inlier global matches (< "
               << opts.min_success_frames << "; pool=" << pool.size()
               << ", global_matched=" << result.global_matched
               << "). Try lowering --min_score, ensuring the maps overlap, "
                  "or use --transform_mode=manual with a known transform.";
    log_score_hints();
    result.stop_reason = "too_few_global_matches";
    result.elapsed_seconds = elapsed_s();
    return result;
  }

  // ── Stage C：增量加帧迭代直至收敛 ──
  ::cartographer::mapping::scan_matching::proto::
      FastCorrelativeScanMatcherOptions2D local_options;
  local_options.set_linear_search_window(opts.local_linear_window);
  local_options.set_angular_search_window(opts.local_angular_window);
  local_options.set_branch_and_bound_depth(kBranchAndBoundDepth);
  ::cartographer::mapping::scan_matching::FastCorrelativeScanMatcher2D
      local_fcsm(*global_grid, local_options);
  LOG(INFO) << "Built local FCSM (window " << opts.local_linear_window
            << " m / " << opts.local_angular_window << " rad)";
  LOG(INFO) << "StageB aggregate delta_T=(" << delta_T.translation().x()
            << ", " << delta_T.translation().y()
            << " yaw=" << delta_T.rotation().angle() << ") from "
            << pool.size() << " frames (" << result.inliers << " inliers)";

  // 剩余帧按时间序进入增量迭代（Stage B 未消费的全部帧，含全局匹配失败的）。
  std::vector<size_t> remaining;
  for (size_t i = 0; i < frames_b.size(); ++i) {
    if (!consumed[i]) remaining.push_back(i);
  }

  size_t cursor = 0;
  int iteration = 0;
  while (iteration < opts.max_iterations &&
         static_cast<int>(pool.size()) < opts.max_frames &&
         result.stop_reason.empty()) {
    if (elapsed_s() > opts.budget_seconds) {
      result.stop_reason = "budget";
      break;
    }
    if (cursor >= remaining.size()) {
      result.stop_reason = "frames_exhausted";
      break;
    }

    const size_t pool_before = pool.size();
    int attempts = 0;
    for (int s = 0; s < opts.frames_per_step && cursor < remaining.size();
         ++s, ++cursor) {
      const auto& frame = frames_b[remaining[cursor]];
      const ::cartographer::sensor::PointCloud pc_2d =
          To2DCloud(frame.point_cloud);
      if (pc_2d.empty()) continue;
      ++attempts;

      const ::cartographer::transform::Rigid2d node_pose_in_B(
          frame.global_pose_b.translation().head<2>(),
          ::cartographer::transform::GetYaw(frame.global_pose_b));
      const ::cartographer::transform::Rigid2d pose_pred =
          delta_T * node_pose_in_B;

      float score = 0.f;
      // 同 Stage B：local_fcsm 的初值与输出都在 target 地图全局系，
      // 因此 pose_pred = delta_T * node_pose_in_B 可以直接喂进来。
      ::cartographer::transform::Rigid2d pose_in_A;
      const bool matched = local_fcsm.Match(
          pose_pred, pc_2d, static_cast<float>(opts.local_min_score), &score,
          &pose_in_A);
      if (!matched) {
        ++result.rejected_l1;
        LOG(INFO) << "[StageC] frame " << frame.trajectory_id << ":"
                  << frame.node_index << " local score=" << score
                  << " (rejected L1)";
        continue;
      }

      const auto rr =
          refine_frame(pose_in_A, pc_2d, frame.line_data_list);
      if (!rr.ok) {
        ++result.rejected_l2;
        LOG(INFO) << "[StageC] frame " << frame.trajectory_id << ":"
                  << frame.node_index << " rejected L2: " << rr.reject_reason;
        continue;
      }
      const ::cartographer::transform::Rigid2d T =
          rr.ceres_pose * node_pose_in_B.inverse();
      pool.push_back({T, score, rr.refine_score});
      ++result.local_matched;
    }
    ++iteration;

    // L4：整批过半被拒时告警（delta_T 偏差大或该区域动态变化多），不中断。
    if (attempts >= 3 &&
        static_cast<int>(pool.size() - pool_before) <= attempts / 2) {
      LOG(WARNING) << "[StageC iter " << iteration << "] "
                   << (pool.size() - pool_before) << "/" << attempts
                   << " frames accepted — check delta_T or dynamic area";
    }
    if (pool.size() == pool_before) continue;  // 本批无新帧，聚合不变

    const ::cartographer::transform::Rigid2d new_delta_T = aggregate_pool();
    const double dt =
        (new_delta_T.translation() - delta_T.translation()).norm();
    const double dyaw = std::abs(
        ::cartographer::common::NormalizeAngleDifference(
            new_delta_T.rotation().angle() - delta_T.rotation().angle()));
    result.inliers = static_cast<int>(
        std::count(inlier_mask.begin(), inlier_mask.end(), true));
    delta_T = new_delta_T;
    LOG(INFO) << "[StageC iter " << iteration << "] frames=" << pool.size()
              << " inliers=" << result.inliers << " delta_T=("
              << delta_T.translation().x() << ", "
              << delta_T.translation().y()
              << " yaw=" << delta_T.rotation().angle() << ") dT=" << dt
              << " dyaw=" << dyaw;

    if (dt < opts.converge_translation && dyaw < opts.converge_yaw) {
      result.converged = true;
      result.stop_reason = "converged";
    }
  }

  if (result.stop_reason.empty()) {
    if (iteration >= opts.max_iterations) {
      result.stop_reason = "max_iterations";
    } else if (static_cast<int>(pool.size()) >= opts.max_frames) {
      result.stop_reason = "max_frames";
    } else {
      result.stop_reason = "budget";
    }
  }
  result.iterations = iteration;

  result.rejected_l3 =
      static_cast<int>(pool.size()) - result.inliers;
  result.T_src_in_tgt = ::cartographer::transform::Rigid3d(
      Eigen::Vector3d(delta_T.translation().x(), delta_T.translation().y(),
                      0.0),
      Eigen::AngleAxisd(delta_T.rotation().angle(), Eigen::Vector3d::UnitZ()));
  result.success = result.inliers >= opts.min_success_frames;
  // 内点相对最终 delta_T 的离散度：内点数只说明「有多少帧同意」，
  // 残差 RMS 才说明「它们同意得有多紧」。两者一起才够判断可信度。
  if (result.inliers > 0 && inlier_mask.size() == pool.size()) {
    double sum_sq_t = 0.0, sum_sq_yaw = 0.0;
    int n = 0;
    for (size_t i = 0; i < pool.size(); ++i) {
      if (!inlier_mask[i]) continue;
      const double dt =
          (pool[i].T.translation() - delta_T.translation()).norm();
      const double dyaw = std::abs(::cartographer::common::NormalizeAngleDifference(
          pool[i].T.rotation().angle() - delta_T.rotation().angle()));
      sum_sq_t += dt * dt;
      sum_sq_yaw += dyaw * dyaw;
      ++n;
    }
    if (n > 0) {
      result.rms_translation_m = std::sqrt(sum_sq_t / n);
      result.rms_yaw_rad = std::sqrt(sum_sq_yaw / n);
    }
  }
  // 「够了」与「稳了」分开表达：内点数达标即 success（T 可用），但未收敛时
  // delta_T 还在动，标 degraded 交给调用方决定是否接受。
  result.degraded = result.success && !result.converged;
  result.elapsed_seconds = elapsed_s();

  LOG(INFO) << "FindTransform summary: success=" << result.success
            << " T_src_in_tgt=(" << result.T_src_in_tgt.translation().x()
            << ", " << result.T_src_in_tgt.translation().y() << " yaw="
            << ::cartographer::transform::GetYaw(result.T_src_in_tgt) << ")"
            << " global=" << result.global_matched
            << " local=" << result.local_matched
            << " inliers=" << result.inliers << " rejected L1/L2/L3="
            << result.rejected_l1 << "/" << result.rejected_l2 << "/"
            << result.rejected_l3 << " iterations=" << result.iterations
            << " converged=" << result.converged
            << " stop_reason=" << result.stop_reason
            << " inlier_rms_xy=" << result.rms_translation_m << "m"
            << " inlier_rms_yaw=" << result.rms_yaw_rad << "rad"
            << " elapsed_ms="
            << static_cast<int>(result.elapsed_seconds * 1000);
  if (result.degraded) {
    LOG(WARNING) << "FindTransformBinA: enough inliers but NOT converged "
                    "(stop_reason=" << result.stop_reason
                 << "). T_src_in_tgt is usable but not settled — consider "
                    "raising --match_budget_seconds / --max_iterations, or "
                    "review the result before applying it.";
  }
  if (!result.success) {
    LOG(ERROR) << "FindTransformBinA: only " << result.inliers
               << " inlier frames (< " << opts.min_success_frames
               << "). Try lowering --min_score/--refine_min_score, ensuring "
                  "the maps overlap, or use --transform_mode=manual.";
    log_score_hints();
  }
  return result;
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
    const RewriteOptions& opts, RewriteStats* stats) {
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
  // 注意：global_static_submap 的非栅格部分只在「无 node 可重建」的退化路径
  // 才需要刚体变换（Step 3 有 node 时会整体重建并覆盖），故此处先只定位指针，
  // 变换推迟到 Step 3 的退化分支里做。
  ::cartographer::mapping::proto::Submap2D* static_submap_proto = nullptr;
  size_t n_static = 0, n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
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
  LOG(INFO) << "SerializedData: located " << n_static
            << " global_static_submap, transformed " << n_traj_data
            << " trajectory_data fixed_frame_origin";
  CHECK(static_submap_proto != nullptr)
      << "Map has no global_static_submap; cannot rewrite. "
         "Regenerate the map first.";

  // Step 3: 重建 global_static_submap（栅格 + 点云 + 线特征）。
  // ⭐ 与 align_map_tool 同款——刚体变换只搬位姿，栅格 cell / 点云
  // 体素格 / 线段拟合都锚在坐标系上，必须在新系从 node 重建。替换的变换是
  // 任意刚体（含平移），比对齐的纯旋转更容易把旧体素格与新栅格错开。
  // 直接复用 map_transform 的实现（同包同目录），不再在本文件里造第三份副本。
  bool has_nodes = false;
  for (const auto& traj : pose_graph.trajectory()) {
    if (traj.node_size() > 0) {
      has_nodes = true;
      break;
    }
  }
  if (has_nodes) {
    *static_submap_proto = ::cartographer_ros::map_transform::
        RebuildGlobalStaticSubmapFromNodes(pose_graph, all_records,
                                           traj_options, opts.resolution,
                                           /*lidar_range_pad=*/30.0,
                                           data.header.data_version());
  } else if (static_submap_proto->has_grid()) {
    // 无 node 可重建的退化路径：只能刚体变换 + 重栅格化，产物保留旧系量化。
    LOG(WARNING) << "pose_graph has no nodes; falling back to rigid transform "
                    "+ rerasterize (cloud/lines keep the OLD frame's voxel "
                    "quantization and culling decisions)";
    TransformStaticSubmapNonGrid(static_submap_proto, T_old_to_new);
    RerasterizeGridProto(static_submap_proto->mutable_grid(), T_old_to_new);
  }

  // Step 3b: 产物自检。变换本身没有内建的"对不对"判据，只能靠产物侧可观测量
  // 交叉验证；在写盘前就地采集，失败也不阻断（判断权交调用方）。
  if (stats != nullptr) {
    if (static_submap_proto->has_grid()) {
      const auto& cl = static_submap_proto->grid().limits().cell_limits();
      stats->grid_num_x_cells = cl.num_x_cells();
      stats->grid_num_y_cells = cl.num_y_cells();
    }
    stats->cloud_points = static_submap_proto->cloud().points_size();
    stats->line_count =
        static_submap_proto->line_map_2d_list().line_map_data_size();
    // DEC-013：替换后只做「对齐度检查」，不做对齐。残余大说明 target 坐标系
    // 本身未对齐（或替换引入的 yaw 误差把它推大了），由人决定是否单独补 align。
    if (stats->line_count > 0) {
      const auto line_map = ::cartographer_ros::map_transform::
          LineMap2DListFromProto(static_submap_proto->line_map_2d_list());
      const auto residual = ::cartographer::mapping::EstimateAlignmentYaw(
          line_map, opts.align_options);
      if (residual.has_value()) {
        stats->align_evaluated = true;
        stats->align_residual_yaw_rad = *residual;
      }
    }
    LOG(INFO) << "Rewrite self-check: grid " << stats->grid_num_x_cells << "x"
              << stats->grid_num_y_cells << ", cloud "
              << stats->cloud_points << " pts, lines " << stats->line_count
              << ", structure-alignment residual "
              << (stats->align_evaluated
                      ? std::to_string(stats->align_residual_yaw_rad * 180.0 /
                                       M_PI) + " deg"
                      : std::string("n/a"));
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
  // 地图重新生成了，地图信息也必须一起重算：面积是 map_manager get_maps 对外
  // 暴露的字段，沿用旧 yaml 或干脆不写都会让上层读到过期/0 值。
  // 口径与建图侧 SaveGridMap 一致 —— effective 必须数**滤波后最终图**的自由像素，
  // 所以这里要把 filtered_out 接出来。
  cv::Mat filtered_mat;
  WriteNavigationMap(origin_mat, png_file, opts.navigation_map_filter_mode,
                     opts.save_debug_map, map_dir, &filtered_mat);
  const GridMapStats map_stats = ComputeGridMapStats(
      origin_mat, filtered_mat, render_resolution, opts.free_thresh);
  // 全量重写 yaml 会丢掉建图侧写过、本工具不认识的键；宁可吵也别静默丢。
  for (const auto& key : FindDroppedYamlKeys(yaml_file, true)) {
    LOG(WARNING) << "map.yaml key dropped by rewrite: '" << key
                 << "' — this tool regenerates map.yaml from scratch and does "
                    "not know this key.";
  }
  ::cartographer::io::StreamFileWriter yaml_writer(yaml_file);
  WriteYaml(render_resolution, map_origin, output_filestem, &yaml_writer,
            opts.occ_thresh, opts.free_thresh, data.header.data_version(),
            &map_stats, opts.created_at);
  LOG(INFO) << "Map area recomputed: total " << map_stats.TotalAreaM2()
            << " m2, effective (free) " << map_stats.EffectiveAreaM2()
            << " m2 (" << map_stats.free_cells << "/" << map_stats.total_cells
            << " free pixels)";
  if (stats != nullptr) {
    stats->total_area_m2 = map_stats.TotalAreaM2();
    stats->effective_area_m2 = map_stats.EffectiveAreaM2();
  }
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
  // 分层布局（<map>/slam/ 放底图，assets/ 与 versions/ 与之平级）优先，
  // 再回退旧扁平布局。四种组合按此顺序尝试，与 align_map_tool 的探测口径一致。
  const std::vector<std::string> candidates = {
      root + "/" + name_or_path + "/slam",
      root + "/" + name_or_path,
      name_or_path + "/slam",
      name_or_path,
  };
  for (const auto& dir : candidates) {
    if (access((dir + "/map.pbstream").c_str(), F_OK) == 0) {
      LOG(INFO) << "Resolved map dir: " << dir;
      return dir;
    }
  }
  std::string tried;
  for (const auto& dir : candidates) tried += "\n  Tried: " + dir;
  LOG(FATAL) << "Map not found for: " << name_or_path << tried;
  return "";  // unreachable
}

double EstimateKnownAreaM2(const ::cartographer::mapping::Grid2D& grid) {
  Eigen::Array2i offset;
  ::cartographer::mapping::CellLimits cell_limits;
  grid.ComputeCroppedLimits(&offset, &cell_limits);
  int known = 0;
  for (int y = 0; y < cell_limits.num_y_cells; ++y) {
    for (int x = 0; x < cell_limits.num_x_cells; ++x) {
      if (grid.IsKnown(Eigen::Array2i(offset.y() + y, offset.x() + x))) {
        ++known;
      }
    }
  }
  const double res = grid.limits().resolution();
  return known * res * res;
}

}  // namespace map_ops
}  // namespace cartographer_ros
