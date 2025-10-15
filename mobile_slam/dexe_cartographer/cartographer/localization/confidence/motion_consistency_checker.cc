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
  oss << "  ├─ Velocity Smooth:  " << velocity_smoothness_score << "\n";
  oss << "  └─ Slippage Score:   " << slippage_score 
      << (is_slipping ? " (SLIPPING DETECTED!)" : "") << "\n";
  
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
  
  // 使用已有的初始化状态（由Evaluate统一管理）
  if (!is_initialized_) {
    return 1.0;
  }
  
  // 计算时间间隔
  double dt = common::ToSeconds(current_time - last_time_);
  if (dt < 1e-6) {
    return 1.0;  // 时间间隔太小，跳过
  }
  
  // 计算速度（使用历史位姿）
  Eigen::Vector2d delta_translation = 
      current_pose.translation() - last_observed_pose_.translation();
  double linear_velocity = delta_translation.norm() / dt;
  
  double delta_angle = std::abs(
      (last_observed_pose_.inverse() * current_pose).rotation().angle());
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
  
  // 计算平滑度（需要至少3个数据点）
  if (velocity_history_.size() < 3) {
    return 1.0;
  }
  
  // 计算加速度变化率（平滑度指标）
  double sum_accel_change = 0.0;
  double mean_accel = 0.0;
  
  // 计算平均加速度
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
  
  // 平滑度得分（加速度变化越小，得分越高）
  double smoothness = std::exp(-kSmoothGamma * avg_accel_change);
  
  return smoothness;
}

double MotionConsistencyChecker::DetectSlippage(
    const transform::Rigid2d& predicted,
    const transform::Rigid2d& observed,
    common::Time time) {
  
  // 第一帧：无法计算增量
  if (!is_initialized_) {
    return 1.0;
  }
  
  // ========== 计算本帧相对于上一帧的增量 ==========
  
  // 预测增量（里程计认为的运动）
  transform::Rigid2d predicted_delta = last_predicted_pose_.inverse() * predicted;
  double predicted_linear = predicted_delta.translation().norm();
  double predicted_angular = std::abs(predicted_delta.rotation().angle());
  
  // 观测增量（扫描匹配观测到的实际运动）
  transform::Rigid2d observed_delta = last_observed_pose_.inverse() * observed;
  double observed_linear = observed_delta.translation().norm();
  double observed_angular = std::abs(observed_delta.rotation().angle());
  // 如果运动量太小，无法可靠判断（可能是静止）
  if (predicted_linear < kMinDeltaThreshold && predicted_angular < 0.01) {
    return 1.0;  // 静止状态，不判断打滑
  }
  
  // 计算比率
  double displacement_ratio = (predicted_linear > 1e-6) 
      ? (observed_linear / predicted_linear) : 1.0;
  double rotation_ratio = (predicted_angular > 0.01) 
      ? (observed_angular / predicted_angular) : 1.0;
  
  // 计算速度（用于一致性检查）
  double dt = common::ToSeconds(time - last_time_);
  if (dt < 1e-6) dt = 0.1;  // 默认100ms
  
  double predicted_linear_vel = predicted_linear / dt;
  double observed_linear_vel = observed_linear / dt;
  
  // ========== 核心判断指标日志 ==========
  // LOG(INFO) << "[Slippage Detector] Core Metrics:";
  // LOG(INFO) << "  Delta:  pred_linear=" << predicted_linear << "m, obs_linear=" << observed_linear << "m";
  // LOG(INFO) << "  Delta:  pred_angular=" << (predicted_angular * 180.0 / M_PI) << "°, obs_angular=" << (observed_angular * 180.0 / M_PI) << "°";
  // LOG(INFO) << "  Ratio:  displacement=" << displacement_ratio << ", rotation=" << rotation_ratio;
  // LOG(INFO) << "  Velocity: pred=" << predicted_linear_vel << "m/s, obs=" << observed_linear_vel << "m/s";
  // LOG(INFO) << "  Time delta: " << dt << "s";

  // ========== 多指标打滑检测 ==========
  
  bool is_slipping = false;
  std::string slippage_reason;
  
  // 【指标1】位移比率异常检测
  if (predicted_linear > kMinDeltaThreshold) {
    if (displacement_ratio < kDisplacementRatioLow) {
      is_slipping = true;
      slippage_reason = "Low displacement ratio (translation slippage)";
    } else if (displacement_ratio > kDisplacementRatioHigh) {
      is_slipping = true;
      slippage_reason = "High displacement ratio (unexpected jump)";
    }
  }
  
  // 【指标2】旋转比率异常检测
  if (!is_slipping && predicted_angular > 0.05) {  // 约3度
    if (rotation_ratio < kRotationRatioLow) {
      is_slipping = true;
      slippage_reason = "Low rotation ratio (rotation slippage)";
    } else if (rotation_ratio > kRotationRatioHigh) {
      is_slipping = true;
      slippage_reason = "High rotation ratio (unexpected rotation)";
    }
  }
  
  // 【指标3】速度一致性检测
  if (!is_slipping && predicted_linear_vel > 0.1) {  // 速度>0.1m/s时检查
    double velocity_diff = std::abs(observed_linear_vel - predicted_linear_vel);
    double velocity_ratio = velocity_diff / std::max(predicted_linear_vel, 0.1);
    
    if (velocity_ratio > kVelocityConsistencyThreshold) {
      is_slipping = true;
      slippage_reason = "Velocity inconsistency detected";
    }
  }
  
  // 【指标4】异常旋转检测（里程计认为直行，实际旋转）
  if (!is_slipping && predicted_angular < 0.087) {  // <5度，认为应该直行
    if (observed_angular > 0.524) {  // >30度
      is_slipping = true;
      slippage_reason = "Unexpected rotation while going straight";
    }
  }
  
  // 【指标5】原地旋转异常（里程计显示平移，实际只旋转）
  if (!is_slipping && predicted_linear > kMinDeltaThreshold) {
    if (observed_linear < kMinDeltaThreshold * 0.5 && observed_angular > 0.3) {
      is_slipping = true;
      slippage_reason = "In-place rotation instead of translation";
    }
  }
  
  // 硬打滑检测：直接返回0
  if (is_slipping) {
    LOG(WARNING) << "========== SLIPPAGE DETECTED ==========";
    LOG(WARNING) << "Reason: " << slippage_reason;
    LOG(WARNING) << "  Predicted delta: linear=" << predicted_linear << "m, angular=" 
                 << (predicted_angular * 180.0 / M_PI) << "°";
    LOG(WARNING) << "  Observed delta:  linear=" << observed_linear << "m, angular=" 
                 << (observed_angular * 180.0 / M_PI) << "°";
    LOG(WARNING) << "  Ratios: displacement=" << displacement_ratio 
                 << " (threshold: " << kDisplacementRatioLow << "-" << kDisplacementRatioHigh << ")"
                 << ", rotation=" << rotation_ratio
                 << " (threshold: " << kRotationRatioLow << "-" << kRotationRatioHigh << ")";
    LOG(WARNING) << "======================================";
    return 0.0;
  }
  
  // 正常情况：输出软得分计算信息
  // LOG(INFO) << "[Slippage Detector] Normal motion - soft score calculation";
  // LOG(INFO) << "  Translation score: " << (0.6 * (displacement_ratio / kDisplacementRatioLow));
  // LOG(INFO) << "  Rotation score: " << (0.4 * (rotation_ratio / kRotationRatioLow));
  
  // ========== 软打滑得分（基于比率的连续评分） ==========
  
  // 位移得分（比率越接近1.0，得分越高）
  double translation_score = 1.0;
  if (predicted_linear > kMinDeltaThreshold) {
    if (displacement_ratio < 1.0) {
      // 比率在[0, 1]区间，线性映射到[0, 1/kLow]
      translation_score = displacement_ratio / kDisplacementRatioLow;
    } else {
      // 比率在[1, inf]区间，映射到[1, 0]
      translation_score = 2.0 - (displacement_ratio / kDisplacementRatioHigh);
    }
    translation_score = std::max(0.0, std::min(1.0, translation_score));
  }
  
  // 旋转得分
  double rotation_score = 1.0;
  if (predicted_angular > 0.05) {
    if (rotation_ratio < 1.0) {
      rotation_score = rotation_ratio / kRotationRatioLow;
    } else {
      rotation_score = 2.0 - (rotation_ratio / kRotationRatioHigh);
    }
    rotation_score = std::max(0.0, std::min(1.0, rotation_score));
  }
  
  // 综合得分（位移为主，旋转为辅）
  return 0.6 * translation_score + 0.4 * rotation_score;
}

MotionConsistencyChecker::Result MotionConsistencyChecker::Evaluate(
    const transform::Rigid2d& predicted,
    const transform::Rigid2d& observed,
    common::Time time) {
  
  Result result;
  
  // 第一帧：统一初始化
  if (!is_initialized_) {
    last_predicted_pose_ = predicted;
    last_observed_pose_ = observed;
    last_time_ = time;
    is_initialized_ = true;
    
    // 第一帧返回默认高置信度
    result.prediction_observation_score = 1.0;
    result.velocity_smoothness_score = 1.0;
    result.slippage_score = 1.0;
    result.total_confidence = 1.0;
    result.delta_distance = 0.0;
    result.delta_angle = 0.0;
    result.is_slipping = false;
    
    LOG(INFO) << "[MotionConsistencyChecker] First frame initialized";
    return result;
  }
  
  // 1. 计算预测-观测一致性得分
  result.prediction_observation_score = 
      ComputePredictionObservationScore(predicted, observed);
  
  // 计算偏差（用于调试输出）
  Eigen::Vector2d delta_translation = 
      observed.translation() - predicted.translation();
  result.delta_distance = delta_translation.norm();
  result.delta_angle = std::abs(
      (predicted.inverse() * observed).rotation().angle());
  
  // 2. 打滑检测（使用当前的历史位姿计算增量）
  result.slippage_score = DetectSlippage(predicted, observed, time);
  result.is_slipping = (result.slippage_score < 0.01);
  
  // 3. 速度平滑度评估（使用当前的历史位姿）
  result.velocity_smoothness_score = 
      UpdateAndComputeVelocitySmoothness(observed, time);
  
  // 4. 更新历史位姿（在所有计算完成后统一更新，用于下一帧）
  last_predicted_pose_ = predicted;
  last_observed_pose_ = observed;
  last_time_ = time;
  
  // 5. 综合置信度计算
  if (result.is_slipping) {
    result.total_confidence = 0.0;  // 检测到打滑，置信度归零
  } else {
    result.total_confidence = 
        0.35 * result.prediction_observation_score +
        0.30 * result.velocity_smoothness_score +
        0.35 * result.slippage_score;
  }
  
  return result;
}

void MotionConsistencyChecker::Reset() {
  velocity_history_.clear();
  is_initialized_ = false;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
