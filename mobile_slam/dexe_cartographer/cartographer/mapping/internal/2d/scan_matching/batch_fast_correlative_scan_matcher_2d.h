/*
 * batch_fast_correlative_scan_matcher_2d.h
 *
 * 全局初始定位（MatchFullSubmap）的加速版分支定界匹配器。
 *
 * 与 FastCorrelativeScanMatcher2D 共享 PrecomputationGridStack2D 与打分定义，
 * 搜索语义等价（admissible 上界 + 全局最优解不变），仅执行策略不同：
 *   1. 递归 DFS → best-first 优先队列：更早收紧 best_score，砍长尾；
 *      当队列最大上界 <= best_score 且攒批为空时即全局最优，可提前终止。
 *   2. 子候选攒批（branch stock），批内 OpenMP 并行打分——打分占基线
 *      总耗时 ~99%，是并行化的主收益来源。
 *   3. 可选 timeout：超时返回当前最优（全局定位为粗定位，后级有
 *      ceres 精化与一致性验证兜底）。
 *
 * 原版 matcher 保留不动（回环检测仍在使用）；本类仅供全局初始定位链路
 * 与 benchmark 工具使用。
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 单次匹配的执行统计（字段口径与 benchmark 工具的 BbsProfile 对应）。
struct BatchMatchStats {
  double t_rotate_ms = 0.;       // 旋转 scan 展开
  double t_discretize_ms = 0.;   // 离散化 + 窗口收紧
  double t_gen_lowres_ms = 0.;   // 初始候选生成
  double t_score_lowres_ms = 0.; // 初始候选打分（并行）
  double t_search_ms = 0.;       // best-first 主循环（含批打分）
  double t_total_ms = 0.;
  int num_rotated_scans = 0;
  std::size_t points_per_scan = 0;
  std::size_t num_initial_candidates = 0;
  std::size_t num_expanded_nodes = 0;
  std::size_t num_scored_candidates = 0;
  std::vector<std::size_t> scored_per_depth;
  float best_score = 0.f;
  bool matched = false;
  bool timed_out = false;
};

class BatchFastCorrelativeScanMatcher2D {
 public:
  BatchFastCorrelativeScanMatcher2D(
      const Grid2D& grid,
      const proto::FastCorrelativeScanMatcherOptions2D& options);
  ~BatchFastCorrelativeScanMatcher2D();

  BatchFastCorrelativeScanMatcher2D(
      const BatchFastCorrelativeScanMatcher2D&) = delete;
  BatchFastCorrelativeScanMatcher2D& operator=(
      const BatchFastCorrelativeScanMatcher2D&) = delete;

  // 与原版 MatchFullSubmap 语义一致的全图搜索；stats 可为 nullptr。
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate,
                       BatchMatchStats* stats = nullptr) const;

  void set_num_threads(int n) { num_threads_ = n; }
  void set_batch_size(std::size_t n) { batch_size_ = n; }
  // <=0 表示不限时。超时返回当前最优（matched 取决于 best_score>min_score）。
  void set_timeout_ms(double ms) { timeout_ms_ = ms; }

  double precomputation_ms() const { return precomputation_ms_; }

 private:
  // 队列/攒批中的候选：比 Candidate2D 多携带 depth（best-first 跨层混排）。
  struct BatchCandidate {
    float score = 0.f;
    int depth = 0;
    int scan_index = 0;
    int x_index_offset = 0;
    int y_index_offset = 0;
    bool operator<(const BatchCandidate& o) const { return score < o.score; }
  };

  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate, BatchMatchStats* stats) const;

  // 批内 OpenMP 并行打分；候选各自用 depth 对应层的 precomputation grid。
  void ScoreBatch(const std::vector<DiscreteScan2D>& discrete_scans,
                  std::vector<BatchCandidate>* batch,
                  BatchMatchStats* stats) const;

  const proto::FastCorrelativeScanMatcherOptions2D options_;
  MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
  double precomputation_ms_ = 0.;

  int num_threads_ = 4;
  std::size_t batch_size_ = 8192;
  double timeout_ms_ = 0.;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_
