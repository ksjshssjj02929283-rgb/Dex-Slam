#include "cartographer/mapping/internal/2d/scan_matching/batch_fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

using Clock = std::chrono::steady_clock;

double ElapsedMs(const Clock::time_point& start) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() -
                                                              start)
             .count() /
         1e6;
}

}  // namespace

BatchFastCorrelativeScanMatcher2D::BatchFastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options), limits_(grid.limits()) {
  const auto start = Clock::now();
  precomputation_grid_stack_ =
      absl::make_unique<PrecomputationGridStack2D>(grid, options);
  precomputation_ms_ = ElapsedMs(start);
}

BatchFastCorrelativeScanMatcher2D::~BatchFastCorrelativeScanMatcher2D() {}

bool BatchFastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate, BatchMatchStats* stats) const {
  // 与原版 MatchFullSubmap 完全一致的搜索窗口设置。
  const SearchParameters search_parameters(
      1e6 * limits_.resolution(), M_PI, point_cloud, limits_.resolution());
  const transform::Rigid2d center = transform::Rigid2d::Translation(
      limits_.max() - 0.5 * limits_.resolution() *
                          Eigen::Vector2d(limits_.cell_limits().num_y_cells,
                                          limits_.cell_limits().num_x_cells));
  BatchMatchStats local_stats;
  return MatchWithSearchParameters(search_parameters, center, point_cloud,
                                   min_score, score, pose_estimate,
                                   stats != nullptr ? stats : &local_stats);
}

bool BatchFastCorrelativeScanMatcher2D::MatchWithSearchParameters(
    SearchParameters search_parameters,
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const float min_score, float* score,
    transform::Rigid2d* pose_estimate, BatchMatchStats* stats) const {
  CHECK(score != nullptr);
  CHECK(pose_estimate != nullptr);

  const auto t_total = Clock::now();
  const int max_depth = precomputation_grid_stack_->max_depth();
  stats->points_per_scan = point_cloud.size();
  stats->scored_per_depth.assign(max_depth + 1, 0);

  // V1.1 角度块表：block[d] = 该层一个角度节点覆盖的底层角度数（2 的幂）。
  // 小角度近似下该层允许的角度步长 ∝ 层等效格尺寸（2^d），取 2x 安全系数
  // → block[d] = 2^(d-1)；L0 恒为 1（最终角度精度与 V1/V0 完全一致）。
  std::vector<int> angular_blocks(max_depth + 1, 1);
  if (use_angular_branching_) {
    for (int d = 1; d <= max_depth; ++d) {
      angular_blocks[d] = std::max(1, 1 << (d - angular_block_shift_));
    }
  }

  // 旋转展开与离散化：与原版一致。
  const auto t_rotate = Clock::now();
  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  stats->t_rotate_ms = ElapsedMs(t_rotate);
  stats->num_rotated_scans = search_parameters.num_scans;

  const auto t_discretize = Clock::now();
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      limits_, rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  search_parameters.ShrinkToFit(discrete_scans, limits_.cell_limits());
  stats->t_discretize_ms = ElapsedMs(t_discretize);

  // 初始层（最粗分辨率）候选枚举。角度分支开启时按顶层角度块步进
  // （scan_index 存块起始，bounds 用块中心代表角度的）。
  const auto t_gen = Clock::now();
  const int linear_step_size = 1 << max_depth;
  const int top_block = angular_blocks[max_depth];
  std::vector<BatchCandidate> initial;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       scan_index += top_block) {
    const int rep = std::min(scan_index + top_block / 2,
                             search_parameters.num_scans - 1);
    const auto& bounds = search_parameters.linear_bounds[rep];
    for (int x = bounds.min_x; x <= bounds.max_x; x += linear_step_size) {
      for (int y = bounds.min_y; y <= bounds.max_y; y += linear_step_size) {
        initial.push_back(BatchCandidate{0.f, max_depth, scan_index, x, y});
      }
    }
  }
  stats->t_gen_lowres_ms = ElapsedMs(t_gen);
  stats->num_initial_candidates = initial.size();

  const auto t_init_score = Clock::now();
  ScoreBatch(discrete_scans, angular_blocks, &initial, stats);
  stats->t_score_lowres_ms = ElapsedMs(t_init_score);

  // best-first 主循环 + 攒批并行打分。
  const auto t_search = Clock::now();
  const auto deadline =
      timeout_ms_ > 0.
          ? Clock::now() + std::chrono::microseconds(
                               static_cast<long>(timeout_ms_ * 1000))
          : Clock::time_point::max();

  std::priority_queue<BatchCandidate> queue(initial.begin(), initial.end());
  std::vector<BatchCandidate> stock;
  stock.reserve(batch_size_);

  float best_score = min_score;
  BatchCandidate best;
  bool has_best = false;

  // flush：批打分后只把仍可能超过 best_score 的候选入队（剪枝）。
  const auto flush_stock = [&]() {
    ScoreBatch(discrete_scans, angular_blocks, &stock, stats);
    for (const auto& c : stock) {
      if (c.score > best_score) queue.push(c);
    }
    stock.clear();
  };

  while (true) {
    if (queue.empty() || queue.top().score <= best_score) {
      // 队列已无希望候选。攒批里可能还有未打分的——补一批后再判。
      if (stock.empty()) break;
      if (Clock::now() > deadline) {
        stats->timed_out = true;
        break;
      }
      flush_stock();
      continue;
    }

    const BatchCandidate candidate = queue.top();
    queue.pop();

    if (candidate.depth == 0) {
      // 上界排序保证：pop 出的 leaf 即当前可达最优；
      // 继续循环以确认队列/攒批中无更高上界（可能存在未打分批）。
      best = candidate;
      best_score = candidate.score;
      has_best = true;
      continue;
    }

    // 分支：平移 2x2 细分；角度分支开启时角度块同步二分（最多 2x2x2 子候选）。
    ++stats->num_expanded_nodes;
    const int half_width = 1 << (candidate.depth - 1);
    const int parent_block = angular_blocks[candidate.depth];
    const int child_block = angular_blocks[candidate.depth - 1];
    int angle_children[2] = {candidate.scan_index, 0};
    int num_angle_children = 1;
    if (child_block < parent_block &&
        candidate.scan_index + child_block < search_parameters.num_scans) {
      angle_children[1] = candidate.scan_index + child_block;
      num_angle_children = 2;
    }
    for (int a = 0; a < num_angle_children; ++a) {
      const int angle_start = angle_children[a];
      const int rep = std::min(angle_start + child_block / 2,
                               search_parameters.num_scans - 1);
      const auto& bounds = search_parameters.linear_bounds[rep];
      for (int x_offset : {0, half_width}) {
        if (candidate.x_index_offset + x_offset > bounds.max_x) break;
        for (int y_offset : {0, half_width}) {
          if (candidate.y_index_offset + y_offset > bounds.max_y) break;
          stock.push_back(BatchCandidate{
              0.f, candidate.depth - 1, angle_start,
              candidate.x_index_offset + x_offset,
              candidate.y_index_offset + y_offset});
        }
      }
    }

    if (stock.size() >= batch_size_) {
      if (Clock::now() > deadline) {
        stats->timed_out = true;
        break;
      }
      flush_stock();
    }
  }
  stats->t_search_ms = ElapsedMs(t_search);
  stats->t_total_ms = ElapsedMs(t_total);

  if (has_best && best_score > min_score) {
    stats->best_score = best_score;
    stats->matched = true;
    *score = best_score;
    const double resolution = search_parameters.resolution;
    const double x = -best.y_index_offset * resolution;
    const double y = -best.x_index_offset * resolution;
    const double orientation =
        (best.scan_index - search_parameters.num_angular_perturbations) *
        search_parameters.angular_perturbation_step_size;
    const Eigen::Rotation2Dd final_rotation =
        initial_rotation * Eigen::Rotation2Dd(orientation);
    *pose_estimate = transform::Rigid2d(
        {initial_pose_estimate.translation().x() + x,
         initial_pose_estimate.translation().y() + y},
        common::NormalizeAngleDifference(final_rotation.angle()));
    return true;
  }
  stats->best_score = has_best ? best_score : 0.f;
  stats->matched = false;
  return false;
}

void BatchFastCorrelativeScanMatcher2D::ScoreBatch(
    const std::vector<DiscreteScan2D>& discrete_scans,
    const std::vector<int>& angular_blocks,
    std::vector<BatchCandidate>* batch, BatchMatchStats* stats) const {
  const int n = static_cast<int>(batch->size());
  const int num_scans = static_cast<int>(discrete_scans.size());
#pragma omp parallel for schedule(dynamic, 256) num_threads(num_threads_)
  for (int i = 0; i < n; ++i) {
    BatchCandidate& c = (*batch)[i];
    const PrecomputationGrid2D& grid =
        precomputation_grid_stack_->Get(c.depth);
    // 角度块中心为代表角度打分（块大小 1 时即自身，V1 行为）。
    const int rep =
        std::min(c.scan_index + angular_blocks[c.depth] / 2, num_scans - 1);
    const DiscreteScan2D& scan = discrete_scans[rep];
    int sum = 0;
    for (const Eigen::Array2i& xy_index : scan) {
      sum += grid.GetValue(Eigen::Array2i(xy_index.x() + c.x_index_offset,
                                          xy_index.y() + c.y_index_offset));
    }
    c.score = grid.ToScore(sum / static_cast<float>(scan.size()));
  }
  stats->num_scored_candidates += batch->size();
  for (const auto& c : *batch) {
    stats->scored_per_depth[c.depth] += 1;
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
