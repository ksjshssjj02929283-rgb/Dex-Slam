/*
 * Copyright 2025 The Cartographer Authors
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

#ifndef CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_CHECKER_H_
#define CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_CHECKER_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/localization/proto/pose_estimator_options_2d.pb.h"

namespace cartographer {
namespace localization {

// 定位一致性检查结果
struct ConsistencyCheckResult {
  bool is_consistent;               // 是否通过一致性检查
  double translation_error;         // 平移误差 (m)
  double rotation_error;            // 旋转误差 (rad)
  double confidence;                // 置信度 [0, 1]
  double linear_velocity;           // 线速度 (m/s)
  double angular_velocity;          // 角速度 (rad/s)
  bool velocity_valid;              // 速度是否有效
  std::string failure_reason;       // 失败原因描述
  
  // 打印检查结果
  std::string ToString() const;
};

// 存储的历史位姿信息
struct PoseRecord {
  common::Time time;
  transform::Rigid2d pose;
};

class ConsistencyChecker {
 public:
  explicit ConsistencyChecker(
      const proto::PoseEstimatorOptions2D& options);
  
  ConsistencyChecker(const ConsistencyChecker&) = delete;
  ConsistencyChecker& operator=(const ConsistencyChecker&) = delete;

  // 设置 scan matcher（在 PoseEstimator2D 初始化后调用）
  void SetRealTimeCorrelativeScanMatcher(
      const mapping::scan_matching::RealTimeCorrelativeScanMatcher2D* matcher);
  
  void SetCeresScanMatcher(
      const mapping::scan_matching::CeresScanMatcher2D* matcher);

  // 检查预测位姿与估计位姿之间的一致性
  ConsistencyCheckResult CheckPoseConsistency(
      const common::Time& time,
      const transform::Rigid2d& predicted_pose,
      const transform::Rigid2d& estimated_pose,
      const sensor::PointCloud& point_cloud,
      const mapping::Submap2D& local_submap);

  // 获取当前置信度
  double GetConfidence() const { return current_confidence_; }

  // 验证位姿估计质量（仅返回是否通过验证）
  bool ValidatePoseEstimate(
      const transform::Rigid2d& estimated_pose,
      const sensor::PointCloud& point_cloud,
      const mapping::Submap2D& local_submap);

  // 从局部地图恢复定位
  // 如果恢复成功，会自动进行一次一致性检查
  // 返回值: 恢复成功返回恢复的位姿，失败返回nullptr
  std::unique_ptr<transform::Rigid2d> RecoverFromLocalMap(
      const common::Time& time,
      const transform::Rigid2d& validation_pose,
      const sensor::PointCloud& point_cloud,
      const sensor::PointCloud& highlight_cloud,
      const std::shared_ptr<mapping::Submap2D>& local_submap);

  // 重置检查器状态
  void Reset();

 private:
  // 计算点云与地图的重合度（置信度）
  double CalculateMapOverlapConfidence(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose,
      const mapping::Submap2D& local_submap);

  // 计算速度（基于位姿历史和当前位姿）
  // 如果速度检查通过，将当前位姿加入历史记录
  // 返回: 是否成功计算速度（需要至少3帧）
  bool CalculateVelocity(const common::Time& current_time,
                         const transform::Rigid2d& current_pose,
                         double* linear_velocity, 
                         double* angular_velocity);

  // 添加位姿记录
  void AddPoseRecord(const common::Time& time, const transform::Rigid2d& pose);

  // 配置选项
  const proto::PoseEstimatorOptions2D& options_;

  // Scan matchers（由 PoseEstimator2D 设置）
  const mapping::scan_matching::RealTimeCorrelativeScanMatcher2D* 
      real_time_correlative_scan_matcher_ = nullptr;
  const mapping::scan_matching::CeresScanMatcher2D* 
      ceres_scan_matcher_ = nullptr;

  // 历史位姿记录（保存最近N帧用于速度计算）
  std::deque<PoseRecord> pose_history_;
  static constexpr size_t kPoseHistorySize = 10;  // 保存10帧
  static constexpr size_t kMinPosesForVelocity = 3;  // 至少需要3帧计算速度

  // 当前置信度
  double current_confidence_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_CONSISTENCY_CHECKER_H_
