/*
 * instrumented_fast_correlative_scan_matcher_2d.h
 *
 * FastCorrelativeScanMatcher2D 的离线基准测量副本。
 *
 * 搜索逻辑与生产实现（cartographer/mapping/internal/2d/scan_matching/
 * fast_correlative_scan_matcher_2d.cc）逐行对应，仅额外记录各阶段耗时与
 * 搜索树规模统计。只链接进 bbs_benchmark_tool，生产库代码零改动——
 * 保证基线测量不受埋点污染，后续优化版本可接入同一工具做 A/B 对照。
 *
 * 注意：统计计数采用批量累加（每次打分调用累加 candidates.size()），
 * 不在逐点内循环里插桩，对耗时测量的扰动可忽略。
 */

#ifndef CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_INSTRUMENTED_FCSM_2D_H_
#define CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_INSTRUMENTED_FCSM_2D_H_

#include <memory>
#include <vector>

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer_ros {
namespace bbs_benchmark {

// 单次 MatchFullSubmap 的分阶段耗时与搜索规模统计。
struct BbsProfile {
  // --- 各阶段耗时（ms）---
  double t_rotate_ms = 0.;       // GenerateRotatedScans（角度枚举展开）
  double t_discretize_ms = 0.;   // DiscretizeScans + ShrinkToFit
  double t_gen_lowres_ms = 0.;   // 生成最低分辨率初始候选
  double t_score_lowres_ms = 0.; // 初始候选打分 + 排序
  double t_bnb_ms = 0.;          // BranchAndBound 递归（含内部打分）
  double t_total_ms = 0.;        // MatchFullSubmap 总耗时

  // --- 搜索规模 ---
  int num_rotated_scans = 0;           // 角度假设数（num_scans）
  std::size_t points_per_scan = 0;     // 单帧点数
  std::size_t num_initial_candidates = 0;  // 最低分辨率初始候选数
  std::size_t num_expanded_nodes = 0;      // BnB 中被展开（分支）的节点数
  std::size_t num_scored_candidates = 0;   // 全部被打分的候选总数（含初始层）
  std::size_t num_point_lookups = 0;       // 打分逐点查表总次数 = Σ(候选数×点数)
  std::vector<std::size_t> scored_per_depth;  // 各深度层被打分的候选数，index=depth

  // --- 结果 ---
  float best_score = 0.f;
  bool matched = false;
};

class InstrumentedFastCorrelativeScanMatcher2D {
 public:
  // 构造时完成 precomputation grid 金字塔构建（与生产实现一致），
  // 构建耗时通过 precomputation_ms() 读取。
  InstrumentedFastCorrelativeScanMatcher2D(
      const ::cartographer::mapping::Grid2D& grid,
      const ::cartographer::mapping::scan_matching::proto::
          FastCorrelativeScanMatcherOptions2D& options);

  // 与生产 MatchFullSubmap 行为一致：全图平移窗口 + 全 2π 角度窗口。
  bool MatchFullSubmap(const ::cartographer::sensor::PointCloud& point_cloud,
                       float min_score, float* score,
                       ::cartographer::transform::Rigid2d* pose_estimate,
                       BbsProfile* profile) const;

  double precomputation_ms() const { return precomputation_ms_; }
  int max_depth() const { return precomputation_grid_stack_->max_depth(); }

 private:
  using Candidate2D = ::cartographer::mapping::scan_matching::Candidate2D;
  using DiscreteScan2D = ::cartographer::mapping::scan_matching::DiscreteScan2D;
  using SearchParameters =
      ::cartographer::mapping::scan_matching::SearchParameters;
  using PrecomputationGrid2D =
      ::cartographer::mapping::scan_matching::PrecomputationGrid2D;

  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const ::cartographer::transform::Rigid2d& initial_pose_estimate,
      const ::cartographer::sensor::PointCloud& point_cloud, float min_score,
      float* score, ::cartographer::transform::Rigid2d* pose_estimate,
      BbsProfile* profile) const;

  std::vector<Candidate2D> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan2D>& discrete_scans,
      const SearchParameters& search_parameters, BbsProfile* profile) const;

  std::vector<Candidate2D> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;

  void ScoreCandidates(const PrecomputationGrid2D& precomputation_grid,
                       const std::vector<DiscreteScan2D>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate2D>* candidates,
                       int depth, BbsProfile* profile) const;

  Candidate2D BranchAndBound(const std::vector<DiscreteScan2D>& discrete_scans,
                             const SearchParameters& search_parameters,
                             const std::vector<Candidate2D>& candidates,
                             int candidate_depth, float min_score,
                             BbsProfile* profile) const;

  const ::cartographer::mapping::scan_matching::proto::
      FastCorrelativeScanMatcherOptions2D options_;
  ::cartographer::mapping::MapLimits limits_;
  std::unique_ptr<
      ::cartographer::mapping::scan_matching::PrecomputationGridStack2D>
      precomputation_grid_stack_;
  double precomputation_ms_ = 0.;
};

}  // namespace bbs_benchmark
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_INSTRUMENTED_FCSM_2D_H_
