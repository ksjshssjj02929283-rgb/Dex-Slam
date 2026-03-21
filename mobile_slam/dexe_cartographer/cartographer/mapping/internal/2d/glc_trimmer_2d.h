/*
 * GLCTrimmer2D — Unified GLC+Overlap Pose Graph Trimmer
 *
 * 数学基础：
 *   Carlevaris-Bianco et al., "Generic Factor-Based Node Marginalization and
 *   Edge Sparsification for Pose-Graph SLAM", ICRA 2014.
 *
 * 核心思想：
 *   1. 基于点云重叠率检测冗余节点（overlap trimming）
 *   2. 删除节点前用 Schur Complement + Chow-Liu Tree 计算 GLC 补偿约束
 *   3. 当 submap 的所有节点均被删除后，裁剪该 submap（带 GLC 补偿）
 *
 * 每次优化后被调用，无需积累触发——每轮直接扫描。
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_GLC_TRIMMER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_GLC_TRIMMER_2D_H_

#include <set>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/pose_graph_trimmer.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

class Submap2D;
class ProbabilityGrid;

// 统一的 GLC + Overlap Node Trimmer for 2D Pose Graph.
// 每次优化后调用 Trim()：
//   1. 遍历所有 finished submap，对其中节点计算点云重叠率
//   2. 高重叠节点执行 GLC 补偿后删除（TrimNode）
//   3. 若 submap 所有节点被删完，执行 GLC 补偿后删除 submap（TrimSubmap）
class GLCTrimmer2D : public PoseGraphTrimmer {
 public:
  struct Options {
    // ── Overlap 检测参数 ──
    // 重叠率高于此阈值视为冗余，删除节点
    double overlap_high_threshold = 0.8;

    // ── GLC 补偿参数 ──
    // true = GLC 信息补偿后再删；false = 直接删除（仅调试用）
    bool enable_glc = true;

    // ── 邻域搜索参数 ──
    // 在最近节点附近搜索的节点数量（含最近节点本身）。
    // n=1 表示只判断最近的一个节点；n>1 时还会检查其前后共 n 个节点。
    int neighbor_count = 3;

    // 邻域搜索的最大距离（米）。
    // 即使在 neighbor_count 内，距离超过此值的候选节点也会被跳过。
    double max_neighbor_dist = 2.0;

    // 每个 cur_node 处理时最多删除的节点数量。
    // 先删类型 A（其他轨迹，overlap < 0.5），再删类型 B（同轨迹，overlap 最大）。
    // ≤0 表示不限制。
    int max_trim_per_check = 1;
  };

  explicit GLCTrimmer2D(const Options& options);
  ~GLCTrimmer2D() override = default;

  void Trim(Trimmable* pose_graph) override;
  bool IsFinished() override { return false; }

  // ──────────────────────────────────────────────────────────────────
  // Markov Blanket 实体（可以是 submap 或 trajectory node）
  // ──────────────────────────────────────────────────────────────────
  struct MBEntity {
    enum Type { SUBMAP, TRAJ_NODE } type = TRAJ_NODE;
    SubmapId submap_id{0, 0};  // type == SUBMAP 时有效
    NodeId node_id{0, 0};      // type == TRAJ_NODE 时有效
    transform::Rigid2d pose = transform::Rigid2d::Identity();
    // 在局部信息矩阵中的变量起始列/行索引（每实体 3 DOF）
    int local_idx = -1;

    bool IsSame(const MBEntity& o) const {
      if (type != o.type) return false;
      return type == SUBMAP ? (submap_id == o.submap_id)
                            : (node_id == o.node_id);
    }
  };

  // 为单个节点计算 GLC+CLT 补偿约束（在删除节点前保留信息）。
  static std::vector<PoseGraphInterface::Constraint>
  ComputeGLCConstraintsForNode(const NodeId& node_id,
                               const Trimmable* pose_graph);

 private:
  // ── GLC 数学方法（全部静态，保持不变）──────────────────────────
  static void ComputeJacobians(const transform::Rigid2d& T_a,
                                const transform::Rigid2d& T_b,
                                Eigen::Matrix3d* J_a,
                                Eigen::Matrix3d* J_b);

  static Eigen::MatrixXd BuildLocalInfoMatrix(
      const std::vector<MBEntity>& del_entities,
      const std::vector<MBEntity>& mb_entities,
      const std::vector<PoseGraphInterface::Constraint>& constraints);

  static Eigen::MatrixXd SchurComplement(const Eigen::MatrixXd& Lambda_m,
                                          int num_del, int num_mb);

  static std::vector<std::pair<int, int>> ChowLiuTree(
      const Eigen::MatrixXd& Lambda_t, int num_mb);

  absl::optional<PoseGraphInterface::Constraint> MakeGLCConstraint(
      int mb_i, int mb_j,
      const std::vector<MBEntity>& mb_entities,
      const Eigen::MatrixXd& Lambda_t,
      const Trimmable* pose_graph) const;

  static absl::optional<NodeId> FindProxyNode(
      const SubmapId& submap_id,
      const Trimmable* pose_graph);

  // 为待裁剪 submap 计算 GLC 补偿约束列表
  std::vector<PoseGraphInterface::Constraint> ComputeGLCConstraints(
      const SubmapId& submap_id,
      const Trimmable* pose_graph) const;

  // ── Overlap 检测方法 ──────────────────────────────────────────
  // 计算节点点云与 submap ProbabilityGrid 的重叠率
  double CalculateOverlapWithSubmap(
      const TrajectoryNode& node,
      const SubmapId& submap_id,
      const Trimmable* pose_graph) const;

  // 在 optimized_submaps 中找到包含 node_id 的、且距离最近的 kFinished submap。
  // 若找不到则返回 {-1, -1}。
  static SubmapId FindNodeSubmap(
      const NodeId& node_id,
      const MapById<SubmapId, PoseGraphInterface::SubmapData>& optimized_submaps,
      const Trimmable* pose_graph);

  // 用 GLC 补偿后删除单个节点
  void TrimNodeWithGLC(const NodeId& node_id, Trimmable* pose_graph) const;

  // 检查并裁剪空 submap（所有节点已被删除的 submap）
  void TrimEmptySubmaps(Trimmable* pose_graph) const;

  // Trim() 的实际实现（由 Trim() 通过 Timer::Evaluate 包裹调用）
  void TrimImpl(Trimmable* pose_graph);

  const Options options_;

  // 上次已处理（trim 过）的 submap id，用于避免对同一 submap 重复处理。
  // 初始化为无效值 {-1, -1}。
  SubmapId last_trimmed_submap_{-1, -1};
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_GLC_TRIMMER_2D_H_
