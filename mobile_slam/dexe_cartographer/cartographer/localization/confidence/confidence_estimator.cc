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

#include "cartographer/localization/confidence/confidence_estimator.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

// 辅助函数：置信度等级转字符串
std::string ConfidenceLevelToString(ConfidenceLevel level) {
  switch (level) {
    case ConfidenceLevel::EXCELLENT: return "EXCELLENT";
    case ConfidenceLevel::GOOD: return "GOOD";
    case ConfidenceLevel::MEDIUM: return "MEDIUM";
    case ConfidenceLevel::POOR: return "POOR";
    case ConfidenceLevel::FAILED: return "FAILED";
    default: return "UNKNOWN";
  }
}

// Metrics打印方法实现
std::string ConfidenceEstimator::Metrics::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  
  oss << "\n========== Confidence Evaluation Metrics ==========\n";
  oss << "Overall:\n";
  oss << "  Total Confidence:     " << total_confidence << "\n";
  oss << "  Confidence Level:     " << ConfidenceLevelToString(level) << "\n";
  oss << "\n";
  
  oss << "Layer 1 - Scan Match (Weight: varies):\n";
  oss << "  Scan Match Score:     " << scan_match_score << "\n";
  oss << "    ├─ Ceres Residual:  " << ceres_residual_score << "\n";
  oss << "    ├─ Match Ratio:     " << match_ratio << "\n";
  oss << "    └─ Covariance:      " << covariance_score << "\n";
  oss << "\n";
  
  oss << "Layer 2 - Motion Consistency (Weight: varies):\n";
  oss << "  Motion Consistency:   " << motion_consistency << "\n";
  oss << "    ├─ Motion Delta:    " << motion_delta_score << "\n";
  oss << "    └─ Velocity Smooth: " << velocity_smooth_score << "\n";
  oss << "\n";
  
  oss << "Layer 3 - Map Coherence (Weight: varies):\n";
  oss << "  Map Coherence:        " << map_coherence << "\n";
  oss << "    ├─ Map Entropy:     " << map_entropy << "\n";
  oss << "    └─ Submap Match:    " << submap_match_score << "\n";
  oss << "\n";
  
  oss << "Layer 4 - Degeneracy Detection (Weight: varies):\n";
  oss << "  Degeneracy Factor:    " << degeneracy_factor << "\n";
  oss << "    ├─ Corridor:        " << (is_corridor_detected ? "YES" : "NO") << "\n";
  oss << "    └─ Feature Dist:    " << feature_distribution_score << "\n";
  oss << "===================================================\n";
  
  return oss.str();
}

void ConfidenceEstimator::Metrics::Print() const {
  LOG(INFO) << ToString();
}

ConfidenceEstimator::ConfidenceEstimator() : ConfidenceEstimator(Config()) {}

ConfidenceEstimator::ConfidenceEstimator(const Config& config)
    : config_(config),
      scan_match_confidence_(new ScanMatchConfidence()),
      motion_checker_(new MotionConsistencyChecker()),
      map_evaluator_(new MapCoherenceEvaluator()),
      degeneracy_detector_(new DegeneracyDetector()) {}

void ConfidenceEstimator::AdjustWeightsByRuntime(double runtime_seconds) {
  // 根据运行时间动态调整权重
  
  if (runtime_seconds < config_.initialization_duration_sec) {
    // 初始化阶段：更依赖扫描匹配
    config_.w_scan_match = 0.6;
    config_.w_motion = 0.3;
    config_.w_map = 0.1;
    config_.w_degeneracy = 0.0;
  } else if (is_post_loop_closure_) {
    // 闭环后阶段：提高地图一致性权重
    config_.w_scan_match = 0.25;
    config_.w_motion = 0.2;
    config_.w_map = 0.5;
    config_.w_degeneracy = 0.05;
  } else {
    // 正常运行阶段：均衡权重
    config_.w_scan_match = 0.3;
    config_.w_motion = 0.25;
    config_.w_map = 0.35;
    config_.w_degeneracy = 0.1;
  }
  
  last_runtime_seconds_ = runtime_seconds;
}

ConfidenceLevel ConfidenceEstimator::DetermineLevel(double total_confidence) const {
  if (total_confidence >= config_.excellent_threshold) {
    return ConfidenceLevel::EXCELLENT;
  } else if (total_confidence >= config_.good_threshold) {
    return ConfidenceLevel::GOOD;
  } else if (total_confidence >= config_.medium_threshold) {
    return ConfidenceLevel::MEDIUM;
  } else if (total_confidence >= config_.poor_threshold) {
    return ConfidenceLevel::POOR;
  } else {
    return ConfidenceLevel::FAILED;
  }
}

ConfidenceEstimator::Metrics ConfidenceEstimator::Evaluate(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose_estimate,
    const transform::Rigid2d& pose_prediction,
    const Eigen::Matrix3d& covariance,
    double ceres_final_cost,
    int num_ceres_residuals,
    const mapping::Grid2D* matching_grid,
    double runtime_seconds,
    bool compute_uncertainty) {
  
  Metrics metrics;
  
  // 动态调整权重
  if (runtime_seconds > 0.0) {
    AdjustWeightsByRuntime(runtime_seconds);
  }
  
  // ========== L1: 扫描匹配置信度 ==========
  auto scan_match_result = scan_match_confidence_->Evaluate(
      point_cloud, pose_estimate, covariance,
      ceres_final_cost, num_ceres_residuals,
      matching_grid, compute_uncertainty);
  
  metrics.scan_match_score = scan_match_result.total_confidence;
  metrics.ceres_residual_score = scan_match_result.ceres_confidence;
  metrics.match_ratio = scan_match_result.match_ratio;
  metrics.covariance_score = scan_match_result.covariance_confidence;
  
  // ========== L2: 运动一致性 ==========
  if (config_.enable_motion_check) {
    auto motion_result = motion_checker_->Evaluate(
        pose_prediction, pose_estimate, common::FromUniversal(0));
    metrics.motion_consistency = motion_result.total_confidence;
    metrics.motion_delta_score = motion_result.prediction_observation_score;
    metrics.velocity_smooth_score = motion_result.velocity_smoothness_score;
  } else {
    metrics.motion_consistency = 1.0;
    metrics.motion_delta_score = 1.0;
    metrics.velocity_smooth_score = 1.0;
  }
  
  // ========== L3: 地图一致性 ==========
  if (config_.enable_map_check && matching_grid != nullptr) {
    auto map_result = map_evaluator_->Evaluate(
        point_cloud, pose_estimate, matching_grid);
    metrics.map_coherence = map_result.total_confidence;
    metrics.map_entropy = map_result.map_entropy;
    metrics.submap_match_score = map_result.submap_match_score;
  } else {
    metrics.map_coherence = 0.8;  // 默认较高值
    metrics.map_entropy = 0.8;
    metrics.submap_match_score = 0.8;
  }
  
  // ========== L4: 几何退化检测 ==========
  if (config_.enable_degeneracy_check) {
    auto degeneracy_result = degeneracy_detector_->Evaluate(
        point_cloud, covariance);
    metrics.degeneracy_factor = degeneracy_result.degeneracy_factor;
    metrics.is_corridor_detected = degeneracy_result.is_corridor;
    metrics.feature_distribution_score = degeneracy_result.feature_distribution;
  } else {
    metrics.degeneracy_factor = 1.0;
    metrics.is_corridor_detected = false;
    metrics.feature_distribution_score = 1.0;
  }
  
  // ========== 融合计算总置信度 ==========
  double weight_sum = config_.w_scan_match + config_.w_motion + 
                      config_.w_map + config_.w_degeneracy;
  
  metrics.total_confidence = (
      config_.w_scan_match * metrics.scan_match_score +
      config_.w_motion * metrics.motion_consistency +
      config_.w_map * metrics.map_coherence +
      config_.w_degeneracy * metrics.degeneracy_factor
  ) / weight_sum;
  
  // 确保在[0,1]范围内
  metrics.total_confidence = std::min(1.0, std::max(0.0, metrics.total_confidence));
  
  // 确定置信度等级
  metrics.level = DetermineLevel(metrics.total_confidence);
  
  // 日志输出（VLOG级别，可通过glog配置控制）
  VLOG(2) << "Confidence Evaluation:"
          << " Total=" << metrics.total_confidence
          << " Level=" << static_cast<int>(metrics.level)
          << " | L1=" << metrics.scan_match_score
          << " L2=" << metrics.motion_consistency
          << " L3=" << metrics.map_coherence
          << " L4=" << metrics.degeneracy_factor;
  
  if (metrics.is_corridor_detected) {
    LOG(WARNING) << "Corridor environment detected! Confidence may be degraded.";
  }
  
  if (metrics.level <= ConfidenceLevel::POOR) {
    LOG(WARNING) << "Low localization confidence: " << metrics.total_confidence
                 << " - Consider re-localization or reducing speed.";
  }
  
  return metrics;
}

void ConfidenceEstimator::Reset() {
  motion_checker_->Reset();
  is_post_loop_closure_ = false;
  last_runtime_seconds_ = 0.0;
  LOG(INFO) << "Confidence estimator reset.";
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
