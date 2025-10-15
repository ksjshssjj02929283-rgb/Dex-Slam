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
    double total_confidence;              // 综合置信度 [0,1]
    
    double delta_distance;  // 预测与观测的距离偏差(m)
    double delta_angle;     // 预测与观测的角度偏差(rad)
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
  
  int velocity_window_size_;
  std::deque<VelocityRecord> velocity_history_;
  
  transform::Rigid2d last_pose_;
  common::Time last_time_;
  bool is_initialized_ = false;
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MOTION_CONSISTENCY_CHECKER_H_
