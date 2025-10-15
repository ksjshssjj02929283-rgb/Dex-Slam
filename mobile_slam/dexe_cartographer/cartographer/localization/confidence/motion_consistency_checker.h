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

#ifndef CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MOTION_CONSISTENCY_CHECKER_H_
#define CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MOTION_CONSISTENCY_CHECKER_H_

#include <deque>
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace localization {
namespace confidence {

// L2层：运动一致性检查器
class MotionConsistencyChecker {
 public:
  struct Result {
    double prediction_observation_score;  // 预测-观测一致性 [0,1]
    double velocity_smoothness_score;     // 速度平滑度 [0,1]
    double slippage_score;                // 打滑检测得分 [0,1]，0表示打滑
    double total_confidence;              // 综合置信度 [0,1]
    
    double delta_distance;  // 预测与观测的距离偏差(m)
    double delta_angle;     // 预测与观测的角度偏差(rad)
    bool is_slipping;       // 是否检测到打滑
    
    // 打印评估项
    std::string ToString() const;
    void Print() const;
  };

  explicit MotionConsistencyChecker(int velocity_window_size = 10);

  // 计算预测-观测偏差得分
  double ComputePredictionObservationScore(
      const transform::Rigid2d& predicted,
      const transform::Rigid2d& observed) const;

  // 更新速度历史并计算平滑度
  double UpdateAndComputeVelocitySmoothness(
      const transform::Rigid2d& current_pose,
      common::Time current_time);

  // 检测打滑（会更新历史位姿）
  // 原理：计算frame-to-frame的增量，比较预测增量与观测增量
  // predicted, observed: 当前帧的绝对位姿
  // time: 当前帧时间
  double DetectSlippage(
      const transform::Rigid2d& predicted,
      const transform::Rigid2d& observed,
      common::Time time);

  // 综合评估
  Result Evaluate(
      const transform::Rigid2d& predicted,
      const transform::Rigid2d& observed,
      common::Time time);

  // 重置状态
  void Reset();

 private:
  struct VelocityRecord {
    common::Time time;
    double linear_velocity;   // m/s
    double angular_velocity;  // rad/s
  };

  // 参数
  static constexpr double kSigmaDistance = 0.3;      // 距离偏差阈值(m)
  static constexpr double kSigmaAngle = 0.262;       // 角度偏差阈值(15度)
  static constexpr double kSmoothGamma = 2.0;        // 平滑度系数
  
  // 打滑检测参数（基于增量）
  static constexpr double kMinDeltaThreshold = 0.05;           // 最小增量阈值(m)，低于此值认为静止
  static constexpr double kDisplacementRatioLow = 0.3;         // 位移比率下限（打滑）
  static constexpr double kDisplacementRatioHigh = 2.5;        // 位移比率上限（跳变）
  static constexpr double kRotationRatioLow = 0.4;             // 旋转比率下限
  static constexpr double kRotationRatioHigh = 2.5;            // 旋转比率上限
  static constexpr double kVelocityConsistencyThreshold = 1.5; // 速度一致性阈值
  
  int velocity_window_size_;
  std::deque<VelocityRecord> velocity_history_;
  
  // 历史位姿（用于计算增量）
  transform::Rigid2d last_observed_pose_;   // 上一帧的观测位姿（用于速度计算）
  transform::Rigid2d last_predicted_pose_;  // 上一帧的预测位姿（用于打滑检测）
  common::Time last_time_;
  bool is_initialized_ = false;
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MOTION_CONSISTENCY_CHECKER_H_
