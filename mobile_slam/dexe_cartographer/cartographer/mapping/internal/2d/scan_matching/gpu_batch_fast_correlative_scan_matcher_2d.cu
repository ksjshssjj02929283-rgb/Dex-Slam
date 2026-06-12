#include "cartographer/mapping/internal/2d/scan_matching/gpu_batch_fast_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <vector>

#include <cuda_runtime.h>

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

#define CUDA_CHECK(expr)                                            \
  do {                                                              \
    const cudaError_t err = (expr);                                 \
    CHECK_EQ(err, cudaSuccess) << "CUDA: " << cudaGetErrorString(err); \
  } while (0)

// 每层金字塔栅格的 device 端元数据（与 PrecomputationGrid2D::GetValue 对应）。
struct GpuGridMeta {
  int offset_x, offset_y;
  int num_x, num_y;  // wide_limits
  float min_score, max_score;
  const unsigned char* cells;
};

// host/device 共用候选 POD（语义同 V1 的 BatchCandidate）。
struct GpuCandidate {
  float score;
  int depth;
  int scan_index;
  int x_index_offset;
  int y_index_offset;
  bool operator<(const GpuCandidate& o) const { return score < o.score; }
};

// 每线程一个候选：按候选 depth 取层元数据，遍历对应离散 scan 查表求和。
// sum 为整数累加，换算与 CPU 同一公式 → 结果与 CPU 逐位一致。
__global__ void ScoreKernel(const GpuGridMeta* metas, const int2* scan_points,
                            const int* scan_offsets,
                            const int* angular_blocks, const int num_scans,
                            GpuCandidate* candidates,
                            const int num_candidates) {
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= num_candidates) return;
  GpuCandidate c = candidates[i];
  const GpuGridMeta m = metas[c.depth];
  // 角度块中心为代表角度（块大小 1 时即自身，V2 行为）。
  const int rep = min(c.scan_index + angular_blocks[c.depth] / 2,
                      num_scans - 1);
  const int begin = scan_offsets[rep];
  const int end = scan_offsets[rep + 1];
  int sum = 0;
  for (int j = begin; j < end; ++j) {
    const int x = scan_points[j].x + c.x_index_offset - m.offset_x;
    const int y = scan_points[j].y + c.y_index_offset - m.offset_y;
    if (static_cast<unsigned>(x) < static_cast<unsigned>(m.num_x) &&
        static_cast<unsigned>(y) < static_cast<unsigned>(m.num_y)) {
      sum += m.cells[x + y * m.num_x];
    }
  }
  candidates[i].score =
      m.min_score + (sum / static_cast<float>(end - begin)) *
                        ((m.max_score - m.min_score) / 255.f);
}

}  // namespace

struct GpuBatchFastCorrelativeScanMatcher2D::GpuData {
  std::vector<unsigned char*> layer_cells;  // 每层 device 栅格
  GpuGridMeta* metas = nullptr;             // device 层元数据数组

  // 离散 scan（每次匹配重传，容量复用）
  int2* scan_points = nullptr;
  int* scan_offsets = nullptr;
  std::size_t scan_points_capacity = 0;
  std::size_t scan_offsets_capacity = 0;

  // 每层角度块表（每次匹配上传，max_depth+1 个 int）
  int* angular_blocks = nullptr;
  std::size_t angular_blocks_capacity = 0;

  // 候选批（pinned host + device，容量复用）
  GpuCandidate* h_candidates = nullptr;  // pinned
  GpuCandidate* d_candidates = nullptr;
  std::size_t candidates_capacity = 0;

  cudaStream_t stream = nullptr;

  ~GpuData() {
    for (auto* p : layer_cells) cudaFree(p);
    if (metas) cudaFree(metas);
    if (scan_points) cudaFree(scan_points);
    if (scan_offsets) cudaFree(scan_offsets);
    if (angular_blocks) cudaFree(angular_blocks);
    if (h_candidates) cudaFreeHost(h_candidates);
    if (d_candidates) cudaFree(d_candidates);
    if (stream) cudaStreamDestroy(stream);
  }

  void EnsureCandidateCapacity(const std::size_t n) {
    if (n <= candidates_capacity) return;
    if (h_candidates) cudaFreeHost(h_candidates);
    if (d_candidates) cudaFree(d_candidates);
    CUDA_CHECK(cudaMallocHost(&h_candidates, n * sizeof(GpuCandidate)));
    CUDA_CHECK(cudaMalloc(&d_candidates, n * sizeof(GpuCandidate)));
    candidates_capacity = n;
  }
};

GpuBatchFastCorrelativeScanMatcher2D::GpuBatchFastCorrelativeScanMatcher2D(
    const Grid2D& grid,
    const proto::FastCorrelativeScanMatcherOptions2D& options)
    : options_(options), limits_(grid.limits()), gpu_(new GpuData) {
  const auto start = Clock::now();
  precomputation_grid_stack_ =
      absl::make_unique<PrecomputationGridStack2D>(grid, options);

  // 金字塔各层一次性常驻显存。
  CUDA_CHECK(cudaStreamCreate(&gpu_->stream));
  const int num_layers = precomputation_grid_stack_->max_depth() + 1;
  std::vector<GpuGridMeta> metas(num_layers);
  gpu_->layer_cells.resize(num_layers, nullptr);
  for (int d = 0; d < num_layers; ++d) {
    const PrecomputationGrid2D& g = precomputation_grid_stack_->Get(d);
    const auto& cells = g.cells();
    CUDA_CHECK(cudaMalloc(&gpu_->layer_cells[d], cells.size()));
    CUDA_CHECK(cudaMemcpy(gpu_->layer_cells[d], cells.data(), cells.size(),
                          cudaMemcpyHostToDevice));
    metas[d] = GpuGridMeta{g.offset().x(),          g.offset().y(),
                           g.wide_limits().num_x_cells,
                           g.wide_limits().num_y_cells,
                           g.min_score(),           g.max_score(),
                           gpu_->layer_cells[d]};
  }
  CUDA_CHECK(cudaMalloc(&gpu_->metas, num_layers * sizeof(GpuGridMeta)));
  CUDA_CHECK(cudaMemcpy(gpu_->metas, metas.data(),
                        num_layers * sizeof(GpuGridMeta),
                        cudaMemcpyHostToDevice));
  precomputation_ms_ = ElapsedMs(start);
}

GpuBatchFastCorrelativeScanMatcher2D::~GpuBatchFastCorrelativeScanMatcher2D() {}

bool GpuBatchFastCorrelativeScanMatcher2D::MatchFullSubmap(
    const sensor::PointCloud& point_cloud, float min_score, float* score,
    transform::Rigid2d* pose_estimate, BatchMatchStats* stats) const {
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

bool GpuBatchFastCorrelativeScanMatcher2D::MatchWithSearchParameters(
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

  // V2.1 角度块表（与 V1.1 同一公式与安全系数约定）。
  std::vector<int> angular_blocks(max_depth + 1, 1);
  if (use_angular_branching_) {
    for (int d = 1; d <= max_depth; ++d) {
      angular_blocks[d] = std::max(1, 1 << (d - angular_block_shift_));
    }
  }
  if (angular_blocks.size() > gpu_->angular_blocks_capacity) {
    if (gpu_->angular_blocks) cudaFree(gpu_->angular_blocks);
    CUDA_CHECK(cudaMalloc(&gpu_->angular_blocks,
                          angular_blocks.size() * sizeof(int)));
    gpu_->angular_blocks_capacity = angular_blocks.size();
  }
  CUDA_CHECK(cudaMemcpyAsync(gpu_->angular_blocks, angular_blocks.data(),
                             angular_blocks.size() * sizeof(int),
                             cudaMemcpyHostToDevice, gpu_->stream));

  // 旋转展开与离散化：与原版/V1 一致（CPU）。
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

  // 离散 scan 扁平化上传（int2 点数组 + 每 scan 偏移表）。
  {
    std::vector<int> offsets(discrete_scans.size() + 1, 0);
    std::size_t total = 0;
    for (std::size_t s = 0; s < discrete_scans.size(); ++s) {
      total += discrete_scans[s].size();
      offsets[s + 1] = static_cast<int>(total);
    }
    std::vector<int2> flat(total);
    std::size_t k = 0;
    for (const auto& scan : discrete_scans) {
      for (const auto& xy : scan) {
        flat[k].x = xy.x();
        flat[k].y = xy.y();
        ++k;
      }
    }
    if (total > gpu_->scan_points_capacity) {
      if (gpu_->scan_points) cudaFree(gpu_->scan_points);
      CUDA_CHECK(cudaMalloc(&gpu_->scan_points, total * sizeof(int2)));
      gpu_->scan_points_capacity = total;
    }
    if (offsets.size() > gpu_->scan_offsets_capacity) {
      if (gpu_->scan_offsets) cudaFree(gpu_->scan_offsets);
      CUDA_CHECK(
          cudaMalloc(&gpu_->scan_offsets, offsets.size() * sizeof(int)));
      gpu_->scan_offsets_capacity = offsets.size();
    }
    CUDA_CHECK(cudaMemcpyAsync(gpu_->scan_points, flat.data(),
                               total * sizeof(int2), cudaMemcpyHostToDevice,
                               gpu_->stream));
    CUDA_CHECK(cudaMemcpyAsync(gpu_->scan_offsets, offsets.data(),
                               offsets.size() * sizeof(int),
                               cudaMemcpyHostToDevice, gpu_->stream));
  }
  stats->t_discretize_ms = ElapsedMs(t_discretize);

  // GPU 批打分：pinned 缓冲 H2D → kernel → D2H 回写同一向量。
  const auto score_batch_gpu = [&](std::vector<GpuCandidate>* batch) {
    const std::size_t n = batch->size();
    if (n == 0) return;
    gpu_->EnsureCandidateCapacity(std::max(n, batch_size_));
    std::copy(batch->begin(), batch->end(), gpu_->h_candidates);
    CUDA_CHECK(cudaMemcpyAsync(gpu_->d_candidates, gpu_->h_candidates,
                               n * sizeof(GpuCandidate),
                               cudaMemcpyHostToDevice, gpu_->stream));
    const int block = 128;
    const int grid_dim = static_cast<int>((n + block - 1) / block);
    ScoreKernel<<<grid_dim, block, 0, gpu_->stream>>>(
        gpu_->metas, gpu_->scan_points, gpu_->scan_offsets,
        gpu_->angular_blocks, search_parameters.num_scans, gpu_->d_candidates,
        static_cast<int>(n));
    CUDA_CHECK(cudaGetLastError());
    CUDA_CHECK(cudaMemcpyAsync(gpu_->h_candidates, gpu_->d_candidates,
                               n * sizeof(GpuCandidate),
                               cudaMemcpyDeviceToHost, gpu_->stream));
    CUDA_CHECK(cudaStreamSynchronize(gpu_->stream));
    std::copy(gpu_->h_candidates, gpu_->h_candidates + n, batch->begin());
    stats->num_scored_candidates += n;
    for (const auto& c : *batch) stats->scored_per_depth[c.depth] += 1;
  };

  // 初始层枚举（角度分支开启时按顶层角度块步进）→ GPU 打分。
  const auto t_gen = Clock::now();
  const int linear_step_size = 1 << max_depth;
  const int top_block = angular_blocks[max_depth];
  std::vector<GpuCandidate> initial;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       scan_index += top_block) {
    const int rep = std::min(scan_index + top_block / 2,
                             search_parameters.num_scans - 1);
    const auto& bounds = search_parameters.linear_bounds[rep];
    for (int x = bounds.min_x; x <= bounds.max_x; x += linear_step_size) {
      for (int y = bounds.min_y; y <= bounds.max_y; y += linear_step_size) {
        initial.push_back(GpuCandidate{0.f, max_depth, scan_index, x, y});
      }
    }
  }
  stats->t_gen_lowres_ms = ElapsedMs(t_gen);
  stats->num_initial_candidates = initial.size();

  const auto t_init_score = Clock::now();
  score_batch_gpu(&initial);
  stats->t_score_lowres_ms = ElapsedMs(t_init_score);

  // best-first 主循环（与 V1 相同语义）。
  const auto t_search = Clock::now();
  const auto deadline =
      timeout_ms_ > 0.
          ? Clock::now() + std::chrono::microseconds(
                               static_cast<long>(timeout_ms_ * 1000))
          : Clock::time_point::max();

  std::priority_queue<GpuCandidate> queue(initial.begin(), initial.end());
  std::vector<GpuCandidate> stock;
  stock.reserve(batch_size_);

  float best_score = min_score;
  GpuCandidate best;
  bool has_best = false;

  const auto flush_stock = [&]() {
    score_batch_gpu(&stock);
    for (const auto& c : stock) {
      if (c.score > best_score) queue.push(c);
    }
    stock.clear();
  };

  while (true) {
    if (queue.empty() || queue.top().score <= best_score) {
      if (stock.empty()) break;
      if (Clock::now() > deadline) {
        stats->timed_out = true;
        break;
      }
      flush_stock();
      continue;
    }
    const GpuCandidate candidate = queue.top();
    queue.pop();
    if (candidate.depth == 0) {
      best = candidate;
      best_score = candidate.score;
      has_best = true;
      continue;
    }
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
          stock.push_back(GpuCandidate{
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

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
