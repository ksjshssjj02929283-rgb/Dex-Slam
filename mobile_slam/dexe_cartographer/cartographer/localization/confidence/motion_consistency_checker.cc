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

#include "cartographer/localization/confidence/motion_consistency_checker.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

// Result打印方法实现
std::string MotionConsistencyChecker::Result::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  
  oss << "MotionConsistencyChecker Result:\n";
  oss << "  Total Confidence:    " << total_confidence << "\n";
  oss << "  ├─ Pred-Obs Score:   " << prediction_observation_score << "\n";
  oss << "  │  ├─ Delta Dist:    " << delta_distance << " m\n";
  oss << "  │  └─ Delta Angle:   " << (delta_angle * 180.0 / M_PI) << " deg\n";
  oss << "  └─ Velocity Smooth:  " << velocity_smoothness_score << "\n";
  
  return oss.str();
}

void MotionConsistencyChecker::Result::Print() const {
  LOG(INFO) << ToString();
}

MotionConsistencyChecker::MotionConsistencyChecker(int velocity_window_size)
    : velocity_window_size_(velocity_window_size) {}

double MotionConsistencyChecker::ComputePredictionObservationScore(
    const transform::Rigid2d& predicted,
    const transform::Rigid2d& observed) const {
  
  // 计算位移偏差
  Eigen::Vector2d delta_translation = 
      observed.translation() - predicted.translation();
  double delta_distance = delta_translation.norm();
  
  // 计算角度偏差
  double delta_angle = std::abs(
      (predicted.inverse() * observed).rotation().angle());
  
  // 使用高斯衰减函数
  double distance_score = std::exp(-delta_distance / kSigmaDistance);
  double angle_score = std::exp(-delta_angle / kSigmaAngle);
  
  return distance_score * angle_score;
}

double MotionConsistencyChecker::UpdateAndComputeVelocitySmoothness(
    const transform::Rigid2d& current_pose,
    common::Time current_time) {
  
  if (!is_initialized_) {
    last_pose_ = current_pose;
    last_time_ = current_time;
    is_initialized_ = true;
    return 1.0;  // 第一帧默认高置信度
  }
  
  // 计算时间间隔
  double dt = common::ToSeconds(current_time - last_time_);
  if (dt < 1e-6) {
    return 1.0;  // 时间间隔太小，跳过
  }
  
  // 计算速度
  Eigen::Vector2d delta_translation = 
      current_pose.translation() - last_pose_.translation();
  double linear_velocity = delta_translation.norm() / dt;
  
  double delta_angle = std::abs(
      (last_pose_.inverse() * current_pose).rotation().angle());
  double angular_velocity = delta_angle / dt;
  
  // 添加到历史记录
  VelocityRecord record;
  record.time = current_time;
  record.linear_velocity = linear_velocity;
  record.angular_velocity = angular_velocity;
  
  velocity_history_.push_back(record);
  
  // 保持窗口大小
  while (velocity_history_.size() > static_cast<size_t>(velocity_window_size_)) {
    velocity_history_.pop_front();
  }
  
  // 计算平滑度
  if (velocity_history_.size() < 3) {
    last_pose_ = current_pose;
    last_time_ = current_time;
    return 1.0;  // 数据不足
  }
  
  // 计算加速度变化率
  double sum_accel_change = 0.0;
  double mean_accel = 0.0;
  
  for (size_t i = 1; i < velocity_history_.size(); ++i) {
    double dt_i = common::ToSeconds(
        velocity_history_[i].time - velocity_history_[i-1].time);
    if (dt_i > 1e-6) {
      double accel = (velocity_history_[i].linear_velocity - 
                     velocity_history_[i-1].linear_velocity) / dt_i;
      mean_accel += accel;
    }
  }
  mean_accel /= (velocity_history_.size() - 1);
  
  // 计算与平均加速度的偏差
  for (size_t i = 1; i < velocity_history_.size(); ++i) {
    double dt_i = common::ToSeconds(
        velocity_history_[i].time - velocity_history_[i-1].time);
    if (dt_i > 1e-6) {
      double accel = (velocity_history_[i].linear_velocity - 
                     velocity_history_[i-1].linear_velocity) / dt_i;
      sum_accel_change += std::abs(accel - mean_accel);
    }
  }
  
  double avg_accel_change = sum_accel_change / (velocity_history_.size() - 1);
  
  // 平滑度得分
  double smoothness = std::exp(-kSmoothGamma * avg_accel_change);
  
  last_pose_ = current_pose;
  last_time_ = current_time;
  
  return smoothness;
}

MotionConsistencyChecker::Result MotionConsistencyChecker::Evaluate(
    const transform::Rigid2d& predicted,
    const transform::Rigid2d& observed,
    common::Time time) {
  
  Result result;
  
  // 计算预测-观测一致性
  result.prediction_observation_score = 
      ComputePredictionObservationScore(predicted, observed);
  
  // 计算位移和角度偏差（用于调试）
  Eigen::Vector2d delta_translation = 
      observed.translation() - predicted.translation();
  result.delta_distance = delta_translation.norm();
  result.delta_angle = std::abs(
      (predicted.inverse() * observed).rotation().angle());
  
  // 更新并计算速度平滑度
  result.velocity_smoothness_score = 
      UpdateAndComputeVelocitySmoothness(observed, time);
  
  // 综合置信度（均等权重）
  result.total_confidence = 
      0.5 * result.prediction_observation_score +
      0.5 * result.velocity_smoothness_score;
  
  return result;
}

void MotionConsistencyChecker::Reset() {
  velocity_history_.clear();
  is_initialized_ = false;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
