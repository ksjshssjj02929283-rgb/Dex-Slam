/*
 * gpu_batch_fast_correlative_scan_matcher_2d.h
 *
 * V2：全局初始定位的 GPU（CUDA）加速版分支定界匹配器。
 *
 * 树管理（best-first 优先队列 + 攒批）与 V1（BatchFastCorrelativeScanMatcher2D）
 * 相同；区别仅在"批打分"的执行后端换为 CUDA kernel（3D-BBS 攒批模式）：
 *   - 金字塔各层栅格构造时一次性常驻显存
 *   - 每次匹配开始时上传 721 个离散化 scan（~1.2MB）
 *   - 每批仅传输候选数组与回传分数
 * 打分为整数累加 + 单次浮点换算，与 CPU 结果逐位一致（无浮点求和顺序差异）。
 *
 * 本头文件不含 CUDA 类型（PIMPL），实现全部在 .cu 中；
 * 需要 cartographer 以 BUILD_GPU_MATCHER=ON 构建。
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GPU_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GPU_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_

#include <memory>

#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/batch_fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

class GpuBatchFastCorrelativeScanMatcher2D {
 public:
  // prebuilt 非空时复用预构建金字塔（pbstream 加载，A4），跳过 CPU 构建；
  // 无论来源都会上传显存。为空则从 grid 构建（benchmark 原行为不变）。
  GpuBatchFastCorrelativeScanMatcher2D(
      const Grid2D& grid,
      const proto::FastCorrelativeScanMatcherOptions2D& options,
      std::shared_ptr<PrecomputationGridStack2D> prebuilt = nullptr);
  ~GpuBatchFastCorrelativeScanMatcher2D();

  GpuBatchFastCorrelativeScanMatcher2D(
      const GpuBatchFastCorrelativeScanMatcher2D&) = delete;
  GpuBatchFastCorrelativeScanMatcher2D& operator=(
      const GpuBatchFastCorrelativeScanMatcher2D&) = delete;

  // 与原版/V1 语义一致的全图搜索；stats 复用 V1 的 BatchMatchStats。
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate,
                       BatchMatchStats* stats = nullptr) const;

  void set_batch_size(std::size_t n) { batch_size_ = n; }
  void set_timeout_ms(double ms) { timeout_ms_ = ms; }
  // V2.1 角度金字塔分支：与 V1.1 同一机制与同一近似上界约定
  //（shift 默认 2，依据 V1.1 的 200 位姿压测定参，见 DEC-005）。
  void set_use_angular_branching(bool enable) {
    use_angular_branching_ = enable;
  }
  void set_angular_block_shift(int shift) { angular_block_shift_ = shift; }

  // 含金字塔 CPU 构建 + 显存上传两段。
  double precomputation_ms() const { return precomputation_ms_; }

 private:
  struct GpuData;  // CUDA 资源（device 指针等），定义在 .cu 中

  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate, BatchMatchStats* stats) const;

  const proto::FastCorrelativeScanMatcherOptions2D options_;
  MapLimits limits_;
  std::shared_ptr<PrecomputationGridStack2D> precomputation_grid_stack_;
  std::unique_ptr<GpuData> gpu_;
  double precomputation_ms_ = 0.;

  std::size_t batch_size_ = 32768;
  double timeout_ms_ = 0.;
  bool use_angular_branching_ = false;
  int angular_block_shift_ = 2;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_GPU_BATCH_FAST_CORRELATIVE_SCAN_MATCHER_2D_H_
