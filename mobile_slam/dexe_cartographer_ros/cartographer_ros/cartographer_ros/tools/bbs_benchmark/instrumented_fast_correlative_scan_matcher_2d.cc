#include "cartographer_ros/tools/bbs_benchmark/instrumented_fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer_ros {
namespace bbs_benchmark {

namespace {

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point& start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                              start)
             .count() /
         1e6;
}

}  // namespace

InstrumentedFastCorrelativeScanMatcher2D::
    InstrumentedFastCorrelativeScanMatcher2D(
        const ::cartographer::mapping::Grid2D& grid,
        const ::cartographer::mapping::scan_matching::proto::
            FastCorrelativeScanMatcherOptions2D& options)
    : options_(options), limits_(grid.limits()) {
  const auto start = Clock::now();
  precomputation_grid_stack_ = absl::make_unique<
      ::cartographer::mapping::scan_matching::PrecomputationGridStack2D>(
      grid, options);
  precomputation_ms_ = ElapsedMs(start);
}

bool InstrumentedFastCorrelativeScanMatcher2D::MatchFullSubmap(
    const ::cartographer::sensor::PointCloud& point_cloud, float min_score,
    float* score, ::cartographer::transform::Rigid2d* pose_estimate,
    BbsProfile* profile) const {
  // 与生产 MatchFullSubmap 完全一致的搜索窗口设置。
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(),  // 平移窗口：1e6 格/方向（覆盖全图）
      M_PI,                        // 角度窗口：双向 180°
      point_cloud, limits_.resolution());
  const ::cartographer::transform::Rigid2d center =
      ::cartographer::transform::Rigid2d::Translation(
          limits_.max() -
          0.5 * limits_.resolution() *
              Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                              limits_.cell_limits().num_x_cells));
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate, profile);
}

bool InstrumentedFastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const ::cartographer::transform::Rigid2d& initial_pose_estimate,
    const ::cartographer::sensor::PointCloud& point_cloud, float min_score,
    float* score, ::cartographer::transform::Rigid2d* pose_estimate,
    BbsProfile* profile) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);
  CHECK(profile != nullptr);

  profile->points_per_scan = point_cloud.size();
  profile->scored_per_depth.assign(
      precomputation_grid_stack_->max_depth() + 1, 0);

  const auto t_total = Clock::now();

  // 阶段 1：角度枚举展开（生成全部旋转后的 scan）。
  const auto t_rotate = Clock::now();
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const ::cartographer::sensor::PointCloud rotated_point_cloud =
      ::cartographer::sensor::TransformPointCloud(
          point_cloud,
          ::cartographer::transform::Rigid3f::Rotation(Eigen::AngleAxisf(
              initial_rotation.cast<float>().angle(),
              Eigen::Vector3f::UnitZ())));
  const std::vector<::cartographer::sensor::PointCloud> rotated_scans =
      ::cartographer::mapping::scan_matching::GenerateRotatedScans(
          rotated_point_cloud, search_parameters);
  profile->t_rotate_ms = ElapsedMs(t_rotate);
  profile->num_rotated_scans = search_parameters.num_scans;

  // 阶段 2：离散化到栅格索引 + 收紧搜索窗口。
  const auto t_discretize = Clock::now();
  const std::vector<DiscreteScan2D> discrete_scans =
      ::cartographer::mapping::scan_matching::DiscretizeScans(
          limits_, rotated_scans,
          Eigen::Translation2f(initial_pose_estimate.translation().x(),
                               initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());
  profile->t_discretize_ms = ElapsedMs(t_discretize);

  // 阶段 3+4：最低分辨率初始候选生成与打分（内部分别计时）。
  const std::vector<Candidate2D> lowest_resolution_candidates =
      ComputeLowestResolutionCandidates(discrete_scans, search_parameters,
                                        profile);

  // 阶段 5：分支定界。
  const auto t_bnb = Clock::now();
  const Candidate2D best_candidate = BranchAndBound(
      discrete_scans, search_parameters, lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), min_score, profile);
  profile->t_bnb_ms = ElapsedMs(t_bnb);

  profile->t_total_ms = ElapsedMs(t_total);
  profile->best_score = best_candidate.score;

  if (best_candidate.score > min_score) {
    profile->matched = true;
    *score = best_candidate.score;
    const Eigen::Rotation2Dd final_rotation =
        initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation);
    *pose_estimate = ::cartographer::transform::Rigid2d(
        {initial_pose_estimate.translation().x() + best_candidate.x,
         initial_pose_estimate.translation().y() + best_candidate.y},
        ::cartographer::common::NormalizeAngleDifference(
            final_rotation.angle()));
    return true;
  }
  profile->matched = false;
  return false;
}

std::vector<InstrumentedFastCorrelativeScanMatcher2D::Candidate2D>
InstrumentedFastCorrelativeScanMatcher2D::ComputeLowestResolutionCandidates(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters, BbsProfile* profile) const {
  const auto t_gen = Clock::now();
  std::vector<Candidate2D> lowest_resolution_candidates =
      GenerateLowestResolutionCandidates(search_parameters);
  profile->t_gen_lowres_ms = ElapsedMs(t_gen);
  profile->num_initial_candidates = lowest_resolution_candidates.size();

  const auto t_score = Clock::now();
  ScoreCandidates(
      precomputation_grid_stack_->Get(precomputation_grid_stack_->max_depth()),
      discrete_scans, search_parameters, &lowest_resolution_candidates,
      precomputation_grid_stack_->max_depth(), profile);
  profile->t_score_lowres_ms = ElapsedMs(t_score);
  return lowest_resolution_candidates;
}

std::vector<InstrumentedFastCorrelativeScanMatcher2D::Candidate2D>
InstrumentedFastCorrelativeScanMatcher2D::GenerateLowestResolutionCandidates(
    const SearchParameters& search_parameters) const {
  const int linear_step_size = 1 << precomputation_grid_stack_->max_depth();
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_lowest_resolution_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + linear_step_size) /
        linear_step_size;
    const int num_lowest_resolution_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + linear_step_size) /
        linear_step_size;
    num_candidates += num_lowest_resolution_linear_x_candidates *
                      num_lowest_resolution_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         x_index_offset += linear_step_size) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           y_index_offset += linear_step_size) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

void InstrumentedFastCorrelativeScanMatcher2D::ScoreCandidates(
    const PrecomputationGrid2D& precomputation_grid,
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates, const int depth,
    BbsProfile* profile) const {
  for (Candidate2D& candidate : *candidates) {
    int sum = 0;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      sum += precomputation_grid.GetValue(proposed_xy_index);
    }
    candidate.score = precomputation_grid.ToScore(
        sum / static_cast<float>(discrete_scans[candidate.scan_index].size()));
  }
  std::sort(candidates->begin(), candidates->end(),
            std::greater<Candidate2D>());

  // 统计批量累加，不进逐点内循环。
  profile->num_scored_candidates += candidates->size();
  profile->num_point_lookups +=
      candidates->size() * profile->points_per_scan;
  profile->scored_per_depth[depth] += candidates->size();
}

InstrumentedFastCorrelativeScanMatcher2D::Candidate2D
InstrumentedFastCorrelativeScanMatcher2D::BranchAndBound(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    const std::vector<Candidate2D>& candidates, const int candidate_depth,
    const float min_score, BbsProfile* profile) const {
  if (candidate_depth == 0) {
    return *candidates.begin();
  }

  Candidate2D best_high_resolution_candidate(0, 0, 0, search_parameters);
  best_high_resolution_candidate.score = min_score;
  for (const Candidate2D& candidate : candidates) {
    if (candidate.score <= min_score) {
      break;
    }
    ++profile->num_expanded_nodes;
    std::vector<Candidate2D> higher_resolution_candidates;
    const int half_width = 1 << (candidate_depth - 1);
    for (int x_offset : {0, half_width}) {
      if (candidate.x_index_offset + x_offset >
          search_parameters.linear_bounds[candidate.scan_index].max_x) {
        break;
      }
      for (int y_offset : {0, half_width}) {
        if (candidate.y_index_offset + y_offset >
            search_parameters.linear_bounds[candidate.scan_index].max_y) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            candidate.scan_index, candidate.x_index_offset + x_offset,
            candidate.y_index_offset + y_offset, search_parameters);
      }
    }
    ScoreCandidates(precomputation_grid_stack_->Get(candidate_depth - 1),
                    discrete_scans, search_parameters,
                    &higher_resolution_candidates, candidate_depth - 1,
                    profile);
    best_high_resolution_candidate = std::max(
        best_high_resolution_candidate,
        BranchAndBound(discrete_scans, search_parameters,
                       higher_resolution_candidates, candidate_depth - 1,
                       best_high_resolution_candidate.score, profile));
  }
  return best_high_resolution_candidate;
}

}  // namespace bbs_benchmark
}  // namespace cartographer_ros
