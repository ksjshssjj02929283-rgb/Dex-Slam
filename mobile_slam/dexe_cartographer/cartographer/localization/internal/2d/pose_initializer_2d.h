/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/batch_fast_correlative_scan_matcher_2d.h"
#ifdef BUILD_GPU_MATCHER
#include "cartographer/mapping/internal/2d/scan_matching/gpu_batch_fast_correlative_scan_matcher_2d.h"
#endif
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/laser_perception.h"
#include "cartographer/localization/proto/pose_initializer_options_2d.pb.h"
#include "cartographer/mapping/2d/line_map_2d.h"

namespace cartographer {
namespace localization {

inline double normalized_angle(const double angle)
{
  double res = angle;
  while (res > M_PI) {
    res -= 2.0 * M_PI;
  }
  while (res <= -M_PI) {
    res += 2.0 * M_PI;
  }
  return res;
}

enum class PoseInitializerStatus {
  NotInitialized = -2,
  Failed = -1,
  Idle = 0,
  LocalEstimating = 1,     // 局部重定位中
  GlobalEstimating = 2,    // 全局重定位中
  Success = 3
};

// Info about the most recent global relocalization run: per-stage timing
// (milliseconds) plus the result. Stages: BBS full-submap match -> real-time
// correlative refine -> line tracking -> ceres refine -> validation.
// Always recorded internally; read back via GetLastGlobalRelocInfo().
struct GlobalRelocInfo {
  double t_bbs_ms = 0.0;
  double t_rtcsm_ms = 0.0;
  double t_line_ms = 0.0;
  double t_ceres_ms = 0.0;
  double t_validate_ms = 0.0;
  double t_total_ms = 0.0;
  // --- profiling breakdown (M1, 2026-06-25): localize the untimed gap ---
  // 5 段计时之和远小于 t_total，缺口落在这些未计时区，逐一拆开定位真瓶颈。
  double t_setup_ms = 0.0;           // t_total_start -> t_bbs：BBS 前的准备区（检查 + 匹配器构造），缺口主嫌疑
  double t_fcsm_build_ms = 0.0;      // 原版 FastCorrelativeScanMatcher2D 构造（建金字塔，不读 pbstream 固化）
  double t_validate_fcsm_ms = 0.0;   // 验证里的原版 FCSM Match（重复全局匹配）
  double t_validate_rtcsm_ms = 0.0;  // 验证里的 RTCSM Match
  int call_index = 0;                // 1 = 进程启动后首次（冷启动）重定位，用于区分一次性开销
  bool matched = false;
  float bbs_score = 0.0f;
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();
};

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class PoseInitializer {
 public:
  explicit PoseInitializer(
	  const proto::PoseInitializerOptions2D& options);
  ~PoseInitializer();

  PoseInitializer(const PoseInitializer&) = delete;
  PoseInitializer& operator=(const PoseInitializer&) = delete;

  void Initialize();
  void ResetStatus();

  // Matching may use a dynamic local submap; validation always uses the static
  // map.
  void InitGlobalSubmap(
      const std::shared_ptr<mapping::Submap2D>& global_submap,
      const std::shared_ptr<const mapping::Submap2D>& static_submap);

  bool SetInitialPose(const transform::Rigid2d& initial_pose);

  // 触发全局重定位，不需要初始位姿
  void TriggerGlobalRelocalization();

  // 加入点云数据和线特征数据
  void UpdateSensorData(const sensor::PointCloud& scan_cloud,
                     const std::vector<sensor::LineData>& line_data);
  const PoseInitializerStatus GetInitializerStatus() const { return initializer_status_; }

  std::shared_ptr<transform::Rigid2d> GetEstimatedPose();

  void start();
  void stop();

  // Runs one global relocalization synchronously on the calling thread (no
  // worker thread). The caller prepares state first via Initialize /
  // InitGlobalSubmap / UpdateSensorData / TriggerGlobalRelocalization. The async
  // worker (Run) and offline benchmark drivers call this same method, so the two
  // paths stay unified. This run's per-stage timing + result are recorded
  // internally; read them with GetLastGlobalRelocInfo(), and check
  // GetInitializerStatus() == Success for the outcome.
  void GlobalRelocalzationOnce();

  // Timing + result of the most recent GlobalRelocalzationOnce() call.
  const GlobalRelocInfo& GetLastGlobalRelocInfo() const {
    return last_global_reloc_info_;
  }

 private:
  friend class PoseInitializerTest;

  void Run();
  void RelocalzationOnce();
  void CreateRelocalzationThread();

  bool IsRelocalizationSuccess();  bool IsPoseValid(const transform::Rigid2d& pose,
                   const transform::Rigid2d& target,
                   const double dist_th = 0.5,
                   const double yaw_th = 0.765 /*45 degrees*/);

  bool MatchFullSubmapDispatch(const mapping::Grid2D& grid,
                               const sensor::PointCloud& cloud, float min_score,
                               float* score, transform::Rigid2d* pose);

  // 轻量位姿打分（M2, DEC-002）：对单个位姿（不做搜索）计算扫描点的平均占据概率，
  // 口径同 RTCSM ComputeCandidateScore，用于替代 validate 里的全局重匹配。
  float ScorePoseAgainstGrid(const transform::Rigid2d& pose,
                             const sensor::PointCloud& point_cloud,
                             const mapping::Grid2D& grid) const;

private:
  const proto::PoseInitializerOptions2D options_;

  std::shared_ptr<mapping::Submap2D> global_submap_ = nullptr;
  std::shared_ptr<const mapping::Submap2D> static_validation_submap_;

  std::shared_ptr<transform::Rigid2d> initial_pose_;
  std::shared_ptr<transform::Rigid2d> global_pose_estimate_;
  // 主链路 BBS 全局解（校验门②复用：加速 BBS 确定性，重跑会返回同一位姿，故存下来直接比 delta，
  // 不重复 ~490ms 全局搜索；DEC-003）。
  transform::Rigid2d last_global_bbs_pose_ = transform::Rigid2d::Identity();
  bool need_apply_initial_pose_ = false;
  bool need_global_relocalization_ = false;  // 标识需要进行全局重定位

  // 最近一次 GlobalRelocalzationOnce 的耗时与结果；经 getter 读取。
  GlobalRelocInfo last_global_reloc_info_;
  int global_reloc_call_count_ = 0;  // M1 profiling：统计进程内全局重定位调用次数（区分冷/稳态）

  bool working_ = false;

  std::unique_ptr<std::thread> relocalization_thread_;
  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  std::shared_ptr<sensor::PointCloud> curr_cloud_;
  std::shared_ptr<std::vector<sensor::LineData>> curr_line_data_;

  std::unique_ptr<mapping::scan_matching::FastCorrelativeScanMatcher2D>
  fast_correlative_scan_matcher_;

  // 加速版全局匹配器（仅用于 MatchFullSubmap；局部 Match 与验证仍用原版）。
  std::unique_ptr<mapping::scan_matching::BatchFastCorrelativeScanMatcher2D>
  batch_global_matcher_;
#ifdef BUILD_GPU_MATCHER
  std::unique_ptr<mapping::scan_matching::GpuBatchFastCorrelativeScanMatcher2D>
  gpu_global_matcher_;
#endif

  std::unique_ptr<mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>
  real_time_correlative_scan_matcher_;

  std::unique_ptr<mapping::scan_matching::CeresScanMatcher2D>
  ceres_scan_matcher_;

  std::unique_ptr<mapping::scan_matching::LineMatcher2D> line_matcher_;

  PoseInitializerStatus initializer_status_ = PoseInitializerStatus::NotInitialized;
};

}  // namespace localization
}  // namespace localization

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_
