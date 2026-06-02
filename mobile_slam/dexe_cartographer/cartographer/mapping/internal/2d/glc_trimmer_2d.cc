/*
 * GLCTrimmer2D — Unified GLC + Overlap Node Trimmer
 * 实现文件
 */

#include "cartographer/mapping/internal/2d/glc_trimmer_2d.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <unordered_map>

#include "Eigen/Cholesky"
#include "Eigen/Dense"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/transform/transform.h"
#include "cartographer/utility/include/timer.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

// ============================================================
// Union-Find（用于 Kruskal 最大权生成树）
// ============================================================
struct UnionFind {
  std::vector<int> parent, rank;
  explicit UnionFind(int n) : parent(n), rank(n, 0) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  int Find(int x) {
    return parent[x] == x ? x : (parent[x] = Find(parent[x]));
  }
  bool Union(int x, int y) {
    x = Find(x);
    y = Find(y);
    if (x == y) return false;
    if (rank[x] < rank[y]) std::swap(x, y);
    parent[y] = x;
    if (rank[x] == rank[y]) ++rank[x];
    return true;
  }
};

// 获取实体在 all_entities 列表中的索引，不存在返回 -1
int FindEntityIndex(const std::vector<GLCTrimmer2D::MBEntity>& entities,
                    const GLCTrimmer2D::MBEntity& target) {
  for (int i = 0; i < static_cast<int>(entities.size()); ++i) {
    if (entities[i].IsSame(target)) return i;
  }
  return -1;
}

// 从约束中提取 2D 相对位姿（Rigid3d → Rigid2d）
transform::Rigid2d ExtractPose2D(
    const PoseGraphInterface::Constraint::Pose& pose) {
  return transform::Project2D(pose.zbar_ij);
}

}  // namespace

// ============================================================
// GLCTrimmer2D 构造
// ============================================================
GLCTrimmer2D::GLCTrimmer2D(const Options& options) : options_(options) {}

// ============================================================
// SE2 Jacobian
//
// 约束残差：r = T_a^{-1} T_b ⊖ z_ab
// 在当前优化点线性化（r ≈ 0 处）
//
// T_rel = T_a^{-1} T_b：
//   rx = cos(θ_a)(x_b-x_a) + sin(θ_a)(y_b-y_a)
//   ry = -sin(θ_a)(x_b-x_a) + cos(θ_a)(y_b-y_a)
//
// J_a (∂r/∂T_a)：
//   [-cos(θ_a), -sin(θ_a),  ry]
//   [ sin(θ_a), -cos(θ_a), -rx]
//   [ 0,         0,         -1]
//
// J_b (∂r/∂T_b)：
//   [cos(θ_a),  sin(θ_a), 0]
//   [-sin(θ_a), cos(θ_a), 0]
//   [0,          0,        1]
// ============================================================
void GLCTrimmer2D::ComputeJacobians(const transform::Rigid2d& T_a,
                                     const transform::Rigid2d& T_b,
                                     Eigen::Matrix3d* J_a,
                                     Eigen::Matrix3d* J_b) {
  const double ca = std::cos(T_a.rotation().angle());
  const double sa = std::sin(T_a.rotation().angle());
  const double dx = T_b.translation().x() - T_a.translation().x();
  const double dy = T_b.translation().y() - T_a.translation().y();
  const double rx = ca * dx + sa * dy;
  const double ry = -sa * dx + ca * dy;

  // clang-format off
  *J_a << -ca, -sa,  ry,
           sa, -ca, -rx,
            0,   0,  -1;

  *J_b <<  ca,  sa,  0,
          -sa,  ca,  0,
            0,   0,  1;
  // clang-format on
}

// ============================================================
// BuildLocalInfoMatrix
//
// 变量排列：[del_0, del_1, ..., mb_0, mb_1, ...]
// 每个实体 3 DOF，总尺寸 = (|del| + |mb|) * 3
//
// 对每条涉及 del 或 mb 中任意实体的约束：
//   Ω_c = diag(w_t^2, w_t^2, w_r^2)
//   info_contribution = J^T Ω_c J，累加到对应 3×3 子块
// ============================================================
Eigen::MatrixXd GLCTrimmer2D::BuildLocalInfoMatrix(
    const std::vector<MBEntity>& del_entities,
    const std::vector<MBEntity>& mb_entities,
    const std::vector<PoseGraphInterface::Constraint>& constraints) {
  const int num_del = static_cast<int>(del_entities.size());
  const int num_mb = static_cast<int>(mb_entities.size());
  const int total = num_del + num_mb;
  Eigen::MatrixXd Lambda = Eigen::MatrixXd::Zero(total * 3, total * 3);

  // 合并所有实体以统一索引查找
  std::vector<MBEntity> all_entities;
  all_entities.insert(all_entities.end(), del_entities.begin(),
                      del_entities.end());
  all_entities.insert(all_entities.end(), mb_entities.begin(),
                      mb_entities.end());

  for (const auto& c : constraints) {
    // 确定约束两端实体
    MBEntity ent_a, ent_b;
    // 约束是 (submap, node) 类型
    ent_a.type = MBEntity::SUBMAP;
    ent_a.submap_id = c.submap_id;
    ent_b.type = MBEntity::TRAJ_NODE;
    ent_b.node_id = c.node_id;

    const int idx_a = FindEntityIndex(all_entities, ent_a);
    const int idx_b = FindEntityIndex(all_entities, ent_b);

    // 只处理至少有一端在 del 中的约束
    bool a_in_del = (idx_a >= 0 && idx_a < num_del);
    bool b_in_del = (idx_b >= 0 && idx_b < num_del);
    if (!a_in_del && !b_in_del) continue;
    // 两端都不在 del+mb 中则跳过
    if (idx_a < 0 || idx_b < 0) continue;

    // 信息矩阵 Ω = diag(w_t^2, w_t^2, w_r^2)
    const double wt = c.pose.translation_weight;
    const double wr = c.pose.rotation_weight;
    Eigen::Matrix3d Omega;
    Omega << wt * wt, 0, 0,
             0, wt * wt, 0,
             0, 0, wr * wr;

    // 获取两端位姿
    const transform::Rigid2d& T_a = all_entities[idx_a].pose;
    const transform::Rigid2d& T_b = all_entities[idx_b].pose;

    Eigen::Matrix3d Ja, Jb;
    ComputeJacobians(T_a, T_b, &Ja, &Jb);

    // 累加 J^T Ω J
    const int r_a = idx_a * 3;
    const int r_b = idx_b * 3;

    Lambda.block<3, 3>(r_a, r_a) += Ja.transpose() * Omega * Ja;
    Lambda.block<3, 3>(r_a, r_b) += Ja.transpose() * Omega * Jb;
    Lambda.block<3, 3>(r_b, r_a) += Jb.transpose() * Omega * Ja;
    Lambda.block<3, 3>(r_b, r_b) += Jb.transpose() * Omega * Jb;
  }

  return Lambda;
}

// ============================================================
// SchurComplement
//
// 输入：Lambda_m (num_del + num_mb)*3 × 同
// 消去前 num_del*3 个变量（β）
// 输出：Λ_t = Λ_αα - Λ_αβ Λ_ββ^{-1} Λ_βα
//        大小 num_mb*3 × num_mb*3
// ============================================================
Eigen::MatrixXd GLCTrimmer2D::SchurComplement(
    const Eigen::MatrixXd& Lambda_m, int num_del, int num_mb) {
  const int del_size = num_del * 3;
  const int mb_size = num_mb * 3;

  const Eigen::MatrixXd Lambda_bb =
      Lambda_m.block(0, 0, del_size, del_size);
  const Eigen::MatrixXd Lambda_ab =
      Lambda_m.block(del_size, 0, mb_size, del_size);
  const Eigen::MatrixXd Lambda_aa =
      Lambda_m.block(del_size, del_size, mb_size, mb_size);

  // 用 LDLT 分解求 Λ_ββ^{-1} Λ_βα
  const Eigen::LDLT<Eigen::MatrixXd> ldlt(Lambda_bb);
  if (ldlt.info() != Eigen::Success) {
    LOG(WARNING) << "GLC: Λ_ββ is singular; returning Λ_αα only.";
    return Lambda_aa;
  }
  const Eigen::MatrixXd Lambda_ba =
      Lambda_m.block(0, del_size, del_size, mb_size);
  return Lambda_aa - Lambda_ab * ldlt.solve(Lambda_ba);
}

// ============================================================
// ChowLiuTree
//
// 在 Λ_t（num_mb*3 × num_mb*3）上求最大权生成树。
// 树边权重 = 互信息 I(x_i, x_j)：
//   I = 0.5 * ln( det(Λ_t[ii]) * det(Λ_t[jj]) / det(Λ_t[ij_joint]) )
// 其中 Λ_t[ij_joint] 是 (x_i, x_j) 的 6×6 联合子块。
// 返回 MB 索引对 (i < j)。
// ============================================================
std::vector<std::pair<int, int>> GLCTrimmer2D::ChowLiuTree(
    const Eigen::MatrixXd& Lambda_t, int num_mb) {
  if (num_mb <= 1) return {};

  struct Edge {
    int i, j;
    double mi;  // mutual information
  };
  std::vector<Edge> edges;
  edges.reserve(num_mb * (num_mb - 1) / 2);

  for (int i = 0; i < num_mb; ++i) {
    for (int j = i + 1; j < num_mb; ++j) {
      const Eigen::Matrix3d Lii = Lambda_t.block<3, 3>(i * 3, i * 3);
      const Eigen::Matrix3d Ljj = Lambda_t.block<3, 3>(j * 3, j * 3);
      Eigen::Matrix<double, 6, 6> Lij_joint;
      Lij_joint.block<3, 3>(0, 0) = Lii;
      Lij_joint.block<3, 3>(0, 3) = Lambda_t.block<3, 3>(i * 3, j * 3);
      Lij_joint.block<3, 3>(3, 0) = Lambda_t.block<3, 3>(j * 3, i * 3);
      Lij_joint.block<3, 3>(3, 3) = Ljj;

      const double det_ii = Lii.determinant();
      const double det_jj = Ljj.determinant();
      const double det_joint = Lij_joint.determinant();

      // 互信息（若行列式非正则跳过，赋负值让 Kruskal 跳过）
      double mi = -1.0;
      if (det_ii > 0 && det_jj > 0 && det_joint > 0) {
        mi = 0.5 * std::log(det_ii * det_jj / det_joint);
      }
      edges.push_back({i, j, mi});
    }
  }

  // Kruskal：最大权生成树（按 mi 降序排列）
  std::sort(edges.begin(), edges.end(),
            [](const Edge& a, const Edge& b) { return a.mi > b.mi; });

  UnionFind uf(num_mb);
  std::vector<std::pair<int, int>> tree;
  for (const auto& e : edges) {
    if (e.mi <= 0) break;  // 负互信息，停止
    if (uf.Union(e.i, e.j)) {
      tree.emplace_back(e.i, e.j);
      if (static_cast<int>(tree.size()) == num_mb - 1) break;
    }
  }
  return tree;
}

// ============================================================
// FindProxyNode
//
// 找 submap 的代理节点：
// 在所有约束中找最后一个（node_index 最大的）INTRA_SUBMAP 节点。
// ============================================================
absl::optional<NodeId> GLCTrimmer2D::FindProxyNode(
    const SubmapId& submap_id, const Trimmable* pose_graph) {
  const auto& constraints = pose_graph->GetConstraints();
  absl::optional<NodeId> proxy;
  for (const auto& c : constraints) {
    if (c.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) continue;
    if (c.submap_id != submap_id) continue;
    if (!proxy.has_value() || c.node_id.node_index > proxy->node_index) {
      proxy = c.node_id;
    }
  }
  return proxy;
}

// ============================================================
// MakeGLCConstraint
//
// 为 CLT 边 (mb_i, mb_j) 生成 Cartographer INTER_SUBMAP 约束。
//
// Cartographer 约束格式：(submap, node, z, w_t, w_r)
// 需要将树边映射到 (submap, node) 形式：
//   - SUBMAP + TRAJ_NODE：直接 (S_i, n_j)
//   - TRAJ_NODE + SUBMAP：(S_j, n_i)
//   - TRAJ_NODE + TRAJ_NODE：(parent_submap_of_n_i, n_j)
//   - SUBMAP + SUBMAP：(S_i, proxy_node_of_S_j)
//
// 测量值 z = T_submap^{-1} * T_node（当前优化位姿）
// 权重由 Λ_t 对角块的 CLT 因子贡献导出
// ============================================================
absl::optional<PoseGraphInterface::Constraint> GLCTrimmer2D::MakeGLCConstraint(
    int mb_i, int mb_j,
    const std::vector<MBEntity>& mb_entities,
    const Eigen::MatrixXd& Lambda_t,
    const Trimmable* pose_graph) const {
  const MBEntity& ent_i = mb_entities[mb_i];
  const MBEntity& ent_j = mb_entities[mb_j];

  // 计算 CLT 度数（每个节点在树中的出现次数）
  // 在这里我们仅生成当前一条边，degree 按 1 估算（保守）
  // 完整实现应传入 degree 数组
  constexpr int kDegree = 1;

  // 确定 (submap_entity, node_entity) 映射
  SubmapId submap_id{0, 0};
  NodeId node_id{0, 0};
  transform::Rigid2d T_submap, T_node;
  const Eigen::Matrix3d info_block = Lambda_t.block<3, 3>(mb_j * 3, mb_j * 3);

  // 根据实体类型决定约束方向
  bool swapped = false;
  if (ent_i.type == MBEntity::SUBMAP && ent_j.type == MBEntity::TRAJ_NODE) {
    submap_id = ent_i.submap_id;
    node_id = ent_j.node_id;
    T_submap = ent_i.pose;
    T_node = ent_j.pose;
  } else if (ent_i.type == MBEntity::TRAJ_NODE &&
             ent_j.type == MBEntity::SUBMAP) {
    // 翻转：(S_j, n_i)
    submap_id = ent_j.submap_id;
    node_id = ent_i.node_id;
    T_submap = ent_j.pose;
    T_node = ent_i.pose;
    swapped = true;
  } else if (ent_i.type == MBEntity::TRAJ_NODE &&
             ent_j.type == MBEntity::TRAJ_NODE) {
    // 找 n_i 的父 submap 作为代理
    // 搜索约束，找 n_i 参与的 INTRA_SUBMAP 约束最大 submap
    const auto& all_constraints = pose_graph->GetConstraints();
    const auto& all_submaps = pose_graph->GetOptimizedSubmapData();
    absl::optional<SubmapId> parent_submap_id;
    for (const auto& c : all_constraints) {
      if (c.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) continue;
      if (c.node_id != ent_i.node_id) continue;
      if (!all_submaps.Contains(c.submap_id)) continue;
      if (!parent_submap_id.has_value() ||
          c.submap_id.submap_index > parent_submap_id->submap_index) {
        parent_submap_id = c.submap_id;
      }
    }
    if (!parent_submap_id.has_value()) {
      LOG(WARNING) << "GLC: cannot find parent submap for node "
                   << ent_i.node_id << "; skipping CLT edge.";
      return absl::nullopt;
    }
    submap_id = *parent_submap_id;
    node_id = ent_j.node_id;
    T_submap = transform::Project2D(
        all_submaps.at(submap_id).pose);
    T_node = ent_j.pose;
  } else {
    // 两端均为 SUBMAP：使用 S_j 的代理节点
    const auto proxy_node = FindProxyNode(ent_j.submap_id, pose_graph);
    if (!proxy_node.has_value()) {
      LOG(WARNING) << "GLC: no proxy node for submap " << ent_j.submap_id
                   << "; skipping CLT edge.";
      return absl::nullopt;
    }
    submap_id = ent_i.submap_id;
    node_id = *proxy_node;
    T_submap = ent_i.pose;
    const auto& nodes = pose_graph->GetTrajectoryNodes();
    if (!nodes.Contains(node_id)) return absl::nullopt;
    T_node = transform::Project2D(nodes.at(node_id).global_pose);
  }

  // GLC 测量值：当前优化位姿下的相对位姿
  const transform::Rigid2d z_glc = T_submap.inverse() * T_node;

  // 从 Λ_t 提取权重
  // Ω ≈ Λ_t[jj] / d_j（保守：取 Λ_t[jj]）
  // 信息矩阵对角元素 → translation_weight、rotation_weight
  const double lambda_t = 0.5 * (info_block(0, 0) + info_block(1, 1));
  const double lambda_r = info_block(2, 2);

  // 防止数值问题
  if (lambda_t <= 0 || lambda_r <= 0) {
    LOG(WARNING) << "GLC: non-positive information for CLT edge; skipping.";
    return absl::nullopt;
  }

  const double w_t = std::sqrt(lambda_t / kDegree);
  const double w_r = std::sqrt(lambda_r / kDegree);

  PoseGraphInterface::Constraint glc_constraint{
      submap_id,
      node_id,
      {transform::Embed3D(z_glc), w_t, w_r},
      PoseGraphInterface::Constraint::INTER_SUBMAP};
  return glc_constraint;
}

// ============================================================
// ComputeGLCConstraintsForNode
//
// 节点替换时调用：在删除 node_id 之前，用 GLC+CLT 保留其信息。
//
// del = { N_old }（被删节点）
// MB  = { 所有与 N_old 有约束（INTRA 或 INTER）的幸存子图 }
//
// 经过 Schur Complement 消去 N_old 后，MB 子图间出现密集信息。
// CLT 用最大权生成树稀疏化，每条边生成一条 INTER_SUBMAP 约束
//（通过 FindProxyNode 将子图-子图边转化为 (submap, proxy_node) 形式）。
// ============================================================
std::vector<PoseGraphInterface::Constraint>
GLCTrimmer2D::ComputeGLCConstraintsForNode(const NodeId& node_id,
                                            const Trimmable* pose_graph) {
  const auto& constraints = pose_graph->GetConstraints();
  const auto& nodes = pose_graph->GetTrajectoryNodes();
  const auto& submap_data = pose_graph->GetOptimizedSubmapData();

  if (!nodes.Contains(node_id)) return {};

  // ── del_entities = { N_old } ──────────────────────────────────
  MBEntity del_ent;
  del_ent.type = MBEntity::TRAJ_NODE;
  del_ent.node_id = node_id;
  del_ent.pose = transform::Project2D(nodes.at(node_id).global_pose);
  del_ent.local_idx = 0;
  const std::vector<MBEntity> del_entities = {del_ent};

  // ── MB = 所有与 N_old 有约束的幸存子图 ───────────────────────
  std::vector<MBEntity> mb_entities;
  auto AddSubmap = [&](const SubmapId& sid) {
    if (!submap_data.Contains(sid)) return;
    MBEntity e;
    e.type = MBEntity::SUBMAP;
    e.submap_id = sid;
    e.pose = transform::Project2D(submap_data.at(sid).pose);
    for (const auto& m : mb_entities) {
      if (m.IsSame(e)) return;
    }
    mb_entities.push_back(e);
  };

  for (const auto& c : constraints) {
    if (c.node_id == node_id) {
      AddSubmap(c.submap_id);
    }
  }

  if (mb_entities.size() < 2) {
    // MB 子图数量不足，无法构建有意义的树
    return {};
  }

  // ── 分配 local_idx ─────────────────────────────────────────────
  for (int i = 0; i < static_cast<int>(mb_entities.size()); ++i) {
    mb_entities[i].local_idx = i;
  }

  // ── 收集涉及 N_old 的局部约束 ──────────────────────────────────
  std::vector<PoseGraphInterface::Constraint> local_constraints;
  for (const auto& c : constraints) {
    if (c.node_id == node_id) {
      local_constraints.push_back(c);
    }
  }

  // ── 建立局部信息矩阵、Schur Complement、CLT ────────────────────
  const Eigen::MatrixXd Lambda_m =
      BuildLocalInfoMatrix(del_entities, mb_entities, local_constraints);
  const int num_mb = static_cast<int>(mb_entities.size());
  const Eigen::MatrixXd Lambda_t = SchurComplement(Lambda_m, 1, num_mb);
  const auto tree_edges = ChowLiuTree(Lambda_t, num_mb);

  // ── 生成 GLC 约束（子图-子图边 → INTER_SUBMAP via proxy node）──
  // 需要一个临时 GLCTrimmer2D 实例来调用 MakeGLCConstraint（非静态）
  GLCTrimmer2D tmp({});
  std::vector<PoseGraphInterface::Constraint> glc_constraints;
  for (const auto& edge : tree_edges) {
    auto maybe_c = tmp.MakeGLCConstraint(edge.first, edge.second,
                                         mb_entities, Lambda_t, pose_graph);
    if (maybe_c.has_value()) {
      // 确认 proxy node 不是将被删除的 N_old 本身
      if (maybe_c->node_id == node_id) continue;
      glc_constraints.push_back(*maybe_c);
    }
  }

  LOG(INFO) << "GLC node " << node_id << ": mb=" << num_mb
            << " tree_edges=" << tree_edges.size()
            << " glc=" << glc_constraints.size();
  return glc_constraints;
}

// ============================================================
// ComputeGLCConstraints
//
// 对 submap_id 计算 GLC 补偿约束：
//   1. 找出 exclusive 节点（只属于 submap_id 的节点）
//   2. 找出 Markov Blanket：
//      - 来自 INTER_SUBMAP(submap_id, n_surv)：n_surv 是 MB 节点
//      - 来自 INTER_SUBMAP(S_other, n_excl)：S_other 是 MB submap
//   3. 构建局部信息矩阵
//   4. Schur Complement → Λ_t
//   5. CLT → GLC 约束列表
// ============================================================
std::vector<PoseGraphInterface::Constraint>
GLCTrimmer2D::ComputeGLCConstraints(const SubmapId& submap_id,
                                     const Trimmable* pose_graph) const {
  const auto& constraints = pose_graph->GetConstraints();
  const auto& nodes = pose_graph->GetTrajectoryNodes();
  const auto& submap_data = pose_graph->GetOptimizedSubmapData();

  // ── 找出 exclusive 节点 ──────────────────────────────────────
  // exclusive = 在 submap_id 里，但不在任何其他 submap 里
  std::set<NodeId> nodes_in_target;
  std::set<NodeId> nodes_in_others;
  for (const auto& c : constraints) {
    if (c.tag != PoseGraphInterface::Constraint::INTRA_SUBMAP) continue;
    if (c.submap_id == submap_id) {
      nodes_in_target.insert(c.node_id);
    } else {
      nodes_in_others.insert(c.node_id);
    }
  }
  std::set<NodeId> excl_nodes;
  for (const NodeId& nid : nodes_in_target) {
    if (nodes_in_others.count(nid) == 0) {
      excl_nodes.insert(nid);
    }
  }

  // ── 构建 del_entities ─────────────────────────────────────────
  std::vector<MBEntity> del_entities;
  // 待删 submap S_k
  if (submap_data.Contains(submap_id)) {
    MBEntity e;
    e.type = MBEntity::SUBMAP;
    e.submap_id = submap_id;
    e.pose = transform::Project2D(submap_data.at(submap_id).pose);
    del_entities.push_back(e);
  }
  // 待删 exclusive 节点
  for (const NodeId& nid : excl_nodes) {
    if (!nodes.Contains(nid)) continue;
    MBEntity e;
    e.type = MBEntity::TRAJ_NODE;
    e.node_id = nid;
    e.pose = transform::Project2D(nodes.at(nid).global_pose);
    del_entities.push_back(e);
  }

  if (del_entities.empty()) return {};

  // ── 构建 mb_entities（MB = 与 del 直接相连的幸存实体）─────────
  // 辅助函数：添加 MBEntity（若尚未存在）
  auto AddMBEntity = [&](std::vector<MBEntity>& mb,
                         const MBEntity& candidate) {
    // 检查是否已在 del 或 mb 中
    for (const auto& d : del_entities) {
      if (d.IsSame(candidate)) return;
    }
    for (const auto& m : mb) {
      if (m.IsSame(candidate)) return;
    }
    mb.push_back(candidate);
  };

  std::vector<MBEntity> mb_entities;

  for (const auto& c : constraints) {
    // 约束涉及 S_k（submap 端）
    if (c.submap_id == submap_id) {
      if (c.tag == PoseGraphInterface::Constraint::INTER_SUBMAP) {
        // INTER(S_k, n_surv): n_surv 是 MB 节点
        if (!excl_nodes.count(c.node_id) && nodes.Contains(c.node_id)) {
          MBEntity e;
          e.type = MBEntity::TRAJ_NODE;
          e.node_id = c.node_id;
          e.pose = transform::Project2D(nodes.at(c.node_id).global_pose);
          AddMBEntity(mb_entities, e);
        }
      }
      continue;
    }
    // 约束涉及 exclusive 节点（节点端）
    if (excl_nodes.count(c.node_id)) {
      if (c.tag == PoseGraphInterface::Constraint::INTER_SUBMAP) {
        // INTER(S_other, n_excl): S_other 是 MB submap
        if (submap_data.Contains(c.submap_id)) {
          MBEntity e;
          e.type = MBEntity::SUBMAP;
          e.submap_id = c.submap_id;
          e.pose = transform::Project2D(submap_data.at(c.submap_id).pose);
          AddMBEntity(mb_entities, e);
        }
      }
    }
  }

  if (mb_entities.size() < 2) {
    // MB 太小，不足以构建树，无需补偿
    return {};
  }

  // ── 分配 local_idx ─────────────────────────────────────────────
  for (int i = 0; i < static_cast<int>(del_entities.size()); ++i) {
    del_entities[i].local_idx = i;
  }
  for (int i = 0; i < static_cast<int>(mb_entities.size()); ++i) {
    mb_entities[i].local_idx = i;
  }

  // ── 收集局部约束（至少一端在 del+mb 中）──────────────────────
  // 合并实体以便查找
  std::vector<MBEntity> all_entities = del_entities;
  all_entities.insert(all_entities.end(), mb_entities.begin(),
                      mb_entities.end());

  std::vector<PoseGraphInterface::Constraint> local_constraints;
  for (const auto& c : constraints) {
    MBEntity ea, eb;
    ea.type = MBEntity::SUBMAP;
    ea.submap_id = c.submap_id;
    eb.type = MBEntity::TRAJ_NODE;
    eb.node_id = c.node_id;
    bool a_in = (FindEntityIndex(all_entities, ea) >= 0);
    bool b_in = (FindEntityIndex(all_entities, eb) >= 0);
    if (a_in || b_in) {
      // 需要 pose 字段，补充
      if (a_in && submap_data.Contains(c.submap_id)) {
        // pose 已经在 MBEntity 中，不需要额外处理
      }
      local_constraints.push_back(c);
    }
  }

  // ── 构建局部信息矩阵 ──────────────────────────────────────────
  // 先将 pose 填入 all_entities（用于 Jacobian 计算）
  // del_entities 和 mb_entities 中 pose 已经填好
  const Eigen::MatrixXd Lambda_m =
      BuildLocalInfoMatrix(del_entities, mb_entities, local_constraints);

  const int num_del = static_cast<int>(del_entities.size());
  const int num_mb = static_cast<int>(mb_entities.size());

  // ── Schur Complement ─────────────────────────────────────────────
  const Eigen::MatrixXd Lambda_t = SchurComplement(Lambda_m, num_del, num_mb);

  // ── Chow-Liu Tree ─────────────────────────────────────────────────
  const auto tree_edges = ChowLiuTree(Lambda_t, num_mb);

  // ── 生成 GLC 约束 ─────────────────────────────────────────────
  std::vector<PoseGraphInterface::Constraint> glc_constraints;
  for (const auto& edge : tree_edges) {
    auto maybe_constraint = MakeGLCConstraint(
        edge.first, edge.second, mb_entities, Lambda_t, pose_graph);
    if (maybe_constraint.has_value()) {
      glc_constraints.push_back(*maybe_constraint);
    }
  }

  LOG(INFO) << "GLC: submap " << submap_id << " | del=" << num_del
            << " mb=" << num_mb << " tree_edges=" << tree_edges.size()
            << " glc_constraints=" << glc_constraints.size();

  return glc_constraints;
}

// ============================================================
// FindNodeSubmap
//
// 在 optimized_submaps（kFinished）中，找到包含 node_id 的
// submap，若存在多个则选取与该节点位置距离最近的那个。
// 找不到时返回 {-1, -1}。
// ============================================================
/* static */
SubmapId GLCTrimmer2D::FindNodeSubmap(
    const NodeId& node_id,
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& optimized_submaps,
    const Trimmable* pose_graph) {
  SubmapId best{-1, -1};
  double best_dist = std::numeric_limits<double>::max();

  for (const auto& submap_entry : optimized_submaps) {
    const SubmapId& sid = submap_entry.id;
    const auto& node_ids = pose_graph->GetSubmapNodeIds(sid);
    if (node_ids.count(node_id) == 0) continue;

    // 以 submap 位姿原点与节点的距离作为优先度
    const double dist =
        (submap_entry.data.pose.translation().head<2>() -
         pose_graph->GetTrajectoryNodes().at(node_id)
             .global_pose.translation().head<2>())
            .norm();
    if (dist < best_dist) {
      best_dist = dist;
      best = sid;
    }
  }
  return best;
}

// ============================================================
// CalculateOverlapWithSubmap
//
// 将节点的点云投影到 submap 的 ProbabilityGrid 上，
// 计算落在占据单元（prob > 0.5）的点占总有效点的比例。
// ============================================================
double GLCTrimmer2D::CalculateOverlapWithSubmap(
    const TrajectoryNode& node,
    const SubmapId& submap_id,
    const Trimmable* pose_graph) const {
  const Submap2D* submap = pose_graph->GetSubmap2D(submap_id);
  if (submap == nullptr || submap->grid() == nullptr) {
    return 0.0;
  }
  const auto* grid = static_cast<const ProbabilityGrid*>(submap->grid());

  // ProbabilityGrid 是在 local SLAM 坐标系中构建的（InsertIntoSubmap 传入 local 坐标系点云）。
  // GetCellIndex 期望的输入也是 local 坐标系中的点。
  // 因此变换应为：T_submap_cloud = T_local_submap^{-1} * T_local_node_2d
  //   - submap.local_pose()      = submap 原点在 local SLAM 坐标系下（纯平移，无旋转）
  //   - node.local_pose          = 节点在 local SLAM 坐标系下（含 gravity_alignment 旋转）
  //   - Project2D(local_pose * gravity_alignment^{-1}) 才是正确的 2D node 位姿
  //     （与 pose_graph_2d.cc 中 local_pose_2d 的计算方式保持一致）
  const transform::Rigid2d node_local_2d = transform::Project2D(
      node.constant_data->local_pose *
      transform::Rigid3d::Rotation(
          node.constant_data->gravity_alignment.inverse()));
  const transform::Rigid2d T_submap_cloud_2d = node_local_2d;
  const transform::Rigid2f T_submap_cloud_2f = T_submap_cloud_2d.cast<float>();

  const auto& point_cloud =
      node.constant_data->filtered_gravity_aligned_point_cloud;
  if (point_cloud.empty()) {
    return 0.0;
  }

  int hit_count = 0;
  int total_count = 0;
  for (const auto& point : point_cloud) {
    // 只取 XY 做 2D 变换，与 bak 一致
    const Eigen::Vector2f p_submap =
        T_submap_cloud_2f * point.position.head<2>();
    // LOG(INFO) << "GLC: point " << p_submap.transpose()
    //           << " transformed to submap frame " << p_submap.transpose();
    const Eigen::Array2i cell_index = grid->limits().GetCellIndex(p_submap);
    if (grid->limits().Contains(cell_index)) {
      ++total_count;
      if (grid->GetProbability(cell_index) > 0.5f) {
        ++hit_count;
      }
    }
  }
  // LOG(INFO) << "______";
  if (total_count == 0) return 0.0;
  return static_cast<double>(hit_count) / static_cast<double>(total_count);
}

// ============================================================
// TrimNodeWithGLC
//
// 在删除节点前：
//   1. 计算 GLC 补偿约束（Schur Complement + CLT）并插入 pose graph
//   2. 调用 TrimNode 从 pose graph 中删除节点、所有关联约束
//      以及从 trajectory_nodes、optimization_problem 中清除
// ============================================================
void GLCTrimmer2D::TrimNodeWithGLC(const NodeId& node_id,
                                    Trimmable* pose_graph) const {
  // Step 1: 生成并插入 GLC 补偿约束（必须在删除节点之前）
  if (options_.enable_glc) {
    const auto glc_constraints =
        ComputeGLCConstraintsForNode(node_id, pose_graph);
    int added = 0;
    for (const auto& c : glc_constraints) {
      // 过滤：proxy node 必须仍然存在于图中
      if (!pose_graph->GetTrajectoryNodes().Contains(c.node_id)) {
        LOG(WARNING) << "[GLCTrimmer2D] GLC: proxy node ("
                     << c.node_id.trajectory_id << ","
                     << c.node_id.node_index << ") not found, skip";
        continue;
      }
      // 过滤：目标 submap 必须在 optimized_submaps 中（即已 kFinished）
      if (!pose_graph->GetOptimizedSubmapData().Contains(c.submap_id)) {
        LOG(WARNING) << "[GLCTrimmer2D] GLC: submap " << c.submap_id
                     << " not in optimized_submaps, skip";
        continue;
      }
      pose_graph->AddConstraint(c);
      ++added;
    }
    LOG(INFO) << "[GLCTrimmer2D] node (" << node_id.trajectory_id << ","
              << node_id.node_index << "): GLC generated="
              << glc_constraints.size() << " added=" << added;
  }

  // Step 2: 从 pose graph 中删除节点
  //   - 从所有 submap 的 node_ids 集合中移除
  //   - 删除所有与该节点相关的约束（INTRA + INTER）
  //   - 清理失去约束的 submap 的 scan matcher
  //   - 从 trajectory_nodes 和 optimization_problem 中删除
  pose_graph->TrimNode(node_id);
  LOG(INFO) << "[GLCTrimmer2D] trimmed node ("
            << node_id.trajectory_id << "," << node_id.node_index << ")";
}

// ============================================================
// TrimEmptySubmaps
//
// 遍历所有 finished submap，若其 node_ids 集合为空，
// 则执行 GLC 补偿后调用 TrimSubmap 删除。
// ============================================================
void GLCTrimmer2D::TrimEmptySubmaps(Trimmable* pose_graph) const {
  const auto optimized_submaps = pose_graph->GetOptimizedSubmapData();
  std::vector<SubmapId> empty_submaps;

  // GetOptimizedSubmapData() 已保证只返回 kFinished 状态的 submap
  // 无需再检查轨迹级别的 IsFinished/IsTrajectoryFrozen
  for (const auto& submap_entry : optimized_submaps) {
    const SubmapId& submap_id = submap_entry.id;
    const auto& node_ids = pose_graph->GetSubmapNodeIds(submap_id);
    if (node_ids.empty()) {
      empty_submaps.push_back(submap_id);
    }
  }

  for (const SubmapId& sid : empty_submaps) {
    // Step 1: GLC 补偿（submap 级别）——在 TrimSubmap 前插入
    if (options_.enable_glc) {
      const auto glc_constraints = ComputeGLCConstraints(sid, pose_graph);
      int added = 0;
      for (const auto& c : glc_constraints) {
        if (!pose_graph->GetTrajectoryNodes().Contains(c.node_id)) continue;
        if (!pose_graph->GetOptimizedSubmapData().Contains(c.submap_id)) continue;
        if (c.submap_id == sid) continue;  // 不能指向即将被删的 submap
        pose_graph->AddConstraint(c);
        ++added;
      }
      LOG(INFO) << "[GLCTrimmer2D] submap " << sid << " GLC generated="
                << glc_constraints.size() << " added=" << added;
    }
    // Step 2: 删除空 submap（TrimSubmap 会同时删除约束和 exclusive 节点）
    pose_graph->TrimSubmap(sid);
    LOG(INFO) << "[GLCTrimmer2D] trimmed empty submap " << sid;
  }
}

// ============================================================
// Trim（主入口，每次优化后被调用）
//
// 新算法（以最后一个节点为锚点）：
//
//  【永远保留】当前轨迹最新节点始终被保留，确保轨迹可追加。
//
//  【删除逻辑】
//   1. 取当前轨迹最新节点 cur_node（最大 node_index）。
//   2. 找 cur_node 所在的已 finished submap（cur_submap）。
//      若 cur_node 尚未归属任何 kFinished submap，本轮跳过。
//   3. 在 cur_submap 中，按与 cur_node 的空间距离排序，
//      取最近的节点 nearest_node 以及其附近共 neighbor_count 个节点。
//   4. 对每个候选节点 cand，用其点云计算与 cur_submap 的重叠率：
//      a. 若 cand 属于**其他轨迹**（非当前轨迹），且重叠率 < 50%
//         → 信息已过时，直接删除。
//      b. 若 cand 属于**当前轨迹**，且重叠率 >= overlap_high_threshold
//         → 信息冗余，删除重叠率最大的那个。
//   5. 清理空 submap。
// ============================================================
void GLCTrimmer2D::Trim(Trimmable* pose_graph) {
  cartographer::Timer::Evaluate([&]() { TrimImpl(pose_graph); },
                              "GLCTrimmer2D::Trim", /*print=*/false);
}

void GLCTrimmer2D::TrimImpl(Trimmable* pose_graph) {
  const auto& all_nodes = pose_graph->GetTrajectoryNodes();
  if (all_nodes.empty()) return;

  const auto optimized_submaps = pose_graph->GetOptimizedSubmapData();
  if (optimized_submaps.empty()) return;

  // ── Step 1: 找"最新完成的 submap" ────────────────────────────────
  // 在所有 kFinished（optimized_submaps 中）的 submap 里，
  // 选取 (trajectory_id, submap_index) 字典序最大的那个，
  // 即最近刚完成的 submap。
  // 同时跳过已经被本 trimmer 处理过的 submap（用 last_trimmed_submap_ 记录）。
  SubmapId newest_submap_id{-1, -1};
  for (const auto& submap_entry : optimized_submaps) {
    const SubmapId& sid = submap_entry.id;
    if (sid == last_trimmed_submap_) continue;  // 跳过上次已处理的
    if (newest_submap_id.trajectory_id < 0 ||
        sid.trajectory_id > newest_submap_id.trajectory_id ||
        (sid.trajectory_id == newest_submap_id.trajectory_id &&
         sid.submap_index > newest_submap_id.submap_index)) {
      newest_submap_id = sid;
    }
  }

  if (newest_submap_id.trajectory_id < 0) {
    // 没有新的 submap 需要处理
    TrimEmptySubmaps(pose_graph);
    return;
  }

  // ── Step 2: 遍历 newest_submap 中的每个节点 ──────────────────────
  // 拷贝一份，防止迭代中被修改
  const std::set<NodeId> nodes_in_newest =
      pose_graph->GetSubmapNodeIds(newest_submap_id);

  if (nodes_in_newest.empty()) {
    last_trimmed_submap_ = newest_submap_id;
    TrimEmptySubmaps(pose_graph);
    return;
  }

  LOG(INFO) << "[GLCTrimmer2D] processing newest submap " << newest_submap_id
            << " with " << nodes_in_newest.size() << " nodes";

  // 收集 newest_submap 以外的所有节点（含其位置）供最近邻搜索
  // 结构：(NodeId, global_pos_2d)
  struct NodePos {
    NodeId id;
    Eigen::Vector2d pos;
  };
  std::vector<NodePos> other_nodes;
  other_nodes.reserve(all_nodes.size());
  for (const auto& id_data : all_nodes) {
    const NodeId& nid = id_data.id;
    // 排除 newest_submap 里的节点
    if (nodes_in_newest.count(nid)) continue;
    other_nodes.push_back(
        {nid, id_data.data.global_pose.translation().head<2>()});
  }

  // ── 批量收集阶段 ──────────────────────────────────────────────────
  // 遍历 newest_submap 中所有 cur_node，对每个节点按 max_trim_per_check
  // 的配额标记候选节点（类型 A 优先，类型 B 其次）。
  // 已被标记删除的节点不再参与后续 cur_node 的近邻搜索（用 marked_for_trim
  // 集合排除），从而避免对同一节点重复计算 GLC marginalization。
  // 批量收集完成后，对所有标记节点执行一次性 TrimNodeWithGLC。
  //
  // CandInfo：候选节点的 trim 信息
  struct CandInfo {
    NodeId id;
    double overlap;
    bool is_other_traj;  // true = 类型 A，false = 类型 B
  };

  // 全局 trim 列表（保持插入顺序：A 类在前，B 类在后；同类内先被标记的先 trim）
  std::vector<CandInfo> global_trim_list;
  // 已标记删除的节点 ID 集合（排除重复，也用于近邻搜索时跳过）
  std::set<NodeId> marked_for_trim;

  for (const NodeId& cur_node_id : nodes_in_newest) {
    if (!all_nodes.Contains(cur_node_id)) continue;

    const Eigen::Vector2d cur_pos =
        all_nodes.at(cur_node_id).global_pose.translation().head<2>();

    // ── Step 3: 在 newest_submap 以外找最近的 neighbor_count 个节点 ──
    // 跳过已被标记删除的节点
    if (other_nodes.empty()) continue;

    std::vector<std::pair<double, int>> dist_idx;
    dist_idx.reserve(other_nodes.size());
    for (int i = 0; i < static_cast<int>(other_nodes.size()); ++i) {
      // 已标记删除的节点不参与近邻搜索
      if (marked_for_trim.count(other_nodes[i].id)) continue;
      const double d = (other_nodes[i].pos - cur_pos).norm();
      dist_idx.emplace_back(d, i);
    }
    if (dist_idx.empty()) continue;

    const int n = std::min(options_.neighbor_count,
                           static_cast<int>(dist_idx.size()));
    std::partial_sort(dist_idx.begin(), dist_idx.begin() + n,
                      dist_idx.end());

    // ── Step 4: 对每个候选节点计算重叠率并按优先级分类 ───────────────
    const int cur_traj_id = cur_node_id.trajectory_id;

    std::vector<CandInfo> type_a_cands;
    std::vector<CandInfo> type_b_cands;

    for (int i = 0; i < n; ++i) {
      const double dist = dist_idx[i].first;
      if (dist > options_.max_neighbor_dist) continue;

      const NodeId& cand_id = other_nodes[dist_idx[i].second].id;
      if (!all_nodes.Contains(cand_id)) continue;
      if (marked_for_trim.count(cand_id)) continue;  // 已标记，跳过

      SubmapId cand_submap_id{-1, -1};
      double overlap = 0.0;
      cartographer::Timer::Evaluate(
          [&]() {
            cand_submap_id = FindNodeSubmap(cand_id, optimized_submaps, pose_graph);
          },
          "GLCTrimmer2D::FindNodeSubmap");
      if (cand_submap_id.trajectory_id >= 0) {
        cartographer::Timer::Evaluate(
            [&]() {
              overlap = CalculateOverlapWithSubmap(
                  all_nodes.at(cur_node_id), cand_submap_id, pose_graph);
            },
            "GLCTrimmer2D::CalculateOverlap");
      }

      LOG(INFO) << "[GLCTrimmer2D] cur=(" << cur_node_id.trajectory_id
                << "," << cur_node_id.node_index << ") cand=("
                << cand_id.trajectory_id << "," << cand_id.node_index
                << ") cand_submap=" << cand_submap_id
                << " overlap=" << overlap
                << " dist=" << dist;

      if (cand_id.trajectory_id != cur_traj_id) {
        if (overlap > 0.0) {
          type_a_cands.push_back({cand_id, overlap, true});
        }
      } else {
        if (overlap >= options_.overlap_high_threshold) {
          type_b_cands.push_back({cand_id, overlap, false});
        }
      }
    }

    // 类型 B 按 overlap 降序排列，优先标记冗余度最高的
    std::sort(type_b_cands.begin(), type_b_cands.end(),
              [](const CandInfo& a, const CandInfo& b) {
                return a.overlap > b.overlap;
              });

    // 按 max_trim_per_check 配额，将候选节点加入全局 trim 列表
    // 类型 A 优先，类型 B 其次
    const int max_trim = options_.max_trim_per_check;
    int local_quota = 0;
    auto try_mark = [&](const CandInfo& cand) {
      if (max_trim > 0 && local_quota >= max_trim) return;
      if (marked_for_trim.count(cand.id)) return;  // 已被其他 cur_node 标记
      global_trim_list.push_back(cand);
      marked_for_trim.insert(cand.id);
      ++local_quota;
    };
    for (const auto& c : type_a_cands) try_mark(c);
    for (const auto& c : type_b_cands) try_mark(c);
  }

  // ── 批量 Trim 阶段 ────────────────────────────────────────────────
  // 对所有标记节点执行一次性 TrimNodeWithGLC（每个节点只 marginalize 一次）
  LOG(INFO) << "[GLCTrimmer2D] batch trim " << global_trim_list.size()
            << " nodes from submap " << newest_submap_id;
  for (const auto& cand : global_trim_list) {
    if (!all_nodes.Contains(cand.id)) continue;  // 可能已被前序 trim 间接移除
    if (cand.is_other_traj) {
      LOG(INFO) << "[GLCTrimmer2D] trim stale other-traj node ("
                << cand.id.trajectory_id << "," << cand.id.node_index
                << ") overlap=" << cand.overlap;
    } else {
      LOG(INFO) << "[GLCTrimmer2D] trim redundant same-traj node ("
                << cand.id.trajectory_id << "," << cand.id.node_index
                << ") overlap=" << cand.overlap;
    }
    cartographer::Timer::Evaluate(
        [&]() { TrimNodeWithGLC(cand.id, pose_graph); },
        "GLCTrimmer2D::TrimNodeWithGLC");
  }

  // ── Step 6: 记录已处理的 submap，清理空 submap ───────────────────
  last_trimmed_submap_ = newest_submap_id;
  TrimEmptySubmaps(pose_graph);
}

}  // namespace mapping
}  // namespace cartographer
