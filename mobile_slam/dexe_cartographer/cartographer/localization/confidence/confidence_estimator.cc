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
  
  if (scan_match_score.has_value()) {
    oss << "Layer 1 - Scan Match (Weight: varies):\n";
    oss << "  Scan Match Score:     " << *scan_match_score << "\n";
    if (ceres_residual_score.has_value()) {
      oss << "    ├─ Ceres Residual:  " << *ceres_residual_score << "\n";
    }
    if (match_ratio.has_value()) {
      oss << "    ├─ Match Ratio:     " << *match_ratio << "\n";
    }
    if (covariance_score.has_value()) {
      oss << "    └─ Covariance:      " << *covariance_score << "\n";
    }
    oss << "\n";
  }
  
  if (motion_consistency.has_value()) {
    oss << "Layer 2 - Motion Consistency (Weight: varies):\n";
    oss << "  Motion Consistency:   " << *motion_consistency << "\n";
    if (motion_delta_score.has_value()) {
      oss << "    ├─ Motion Delta:    " << *motion_delta_score << "\n";
    }
    if (velocity_smooth_score.has_value()) {
      oss << "    └─ Velocity Smooth: " << *velocity_smooth_score << "\n";
    }
    oss << "\n";
  }
  
  if (map_coherence.has_value()) {
    oss << "Layer 3 - Map Coherence (Weight: varies):\n";
    oss << "  Map Coherence:        " << *map_coherence << "\n";
    if (map_entropy.has_value()) {
      oss << "    ├─ Map Entropy:     " << *map_entropy << "\n";
    }
    if (submap_match_score.has_value()) {
      oss << "    └─ Submap Match:    " << *submap_match_score << "\n";
    }
    oss << "\n";
  }
  
  if (degeneracy_factor.has_value()) {
    oss << "Layer 4 - Degeneracy Detection (Weight: varies):\n";
    oss << "  Degeneracy Factor:    " << *degeneracy_factor << "\n";
    if (is_corridor_detected.has_value()) {
      oss << "    ├─ Corridor:        " << (*is_corridor_detected ? "YES" : "NO") << "\n";
    }
    if (feature_distribution_score.has_value()) {
      oss << "    └─ Feature Dist:    " << *feature_distribution_score << "\n";
    }
  }
  
  oss << "===================================================\n";
  
  return oss.str();
}

void ConfidenceEstimator::Metrics::Print() const {
  LOG(INFO) << ToString();
}

ConfidenceEstimator::ConfidenceEstimator(const proto::ConfidenceEstimatorOptions& options)
    : options_(options),
      scan_match_confidence_(new ScanMatchConfidence()),
      motion_checker_(new MotionConsistencyChecker()),
      map_evaluator_(new MapCoherenceEvaluator()),
      degeneracy_detector_(new DegeneracyDetector()) {}


ConfidenceLevel ConfidenceEstimator::DetermineLevel(double total_confidence) const {
  if (total_confidence >= options_.excellent_threshold()) {
    return ConfidenceLevel::EXCELLENT;
  } else if (total_confidence >= options_.good_threshold()) {
    return ConfidenceLevel::GOOD;
  } else if (total_confidence >= options_.medium_threshold()) {
    return ConfidenceLevel::MEDIUM;
  } else if (total_confidence >= options_.poor_threshold()) {
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
    const std::vector<sensor::LineData>& line_data,
    const mapping::Grid2D* matching_grid,
    double runtime_seconds,
    bool compute_uncertainty) {
  
  Metrics metrics;
  
  // 根据运行时间选择固定参数（初始化阶段 vs 正常阶段）
  const bool is_initialization = runtime_seconds > 0.0 && 
                                  runtime_seconds < options_.initialization_duration_sec();
  
  // 选择当前阶段的权重
  const double w_scan_match = is_initialization ? options_.init_w_scan_match() : options_.normal_w_scan_match();
  const double w_motion = is_initialization ? options_.init_w_motion() : options_.normal_w_motion();
  const double w_map = is_initialization ? options_.init_w_map() : options_.normal_w_map();
  const double w_degeneracy = is_initialization ? options_.init_w_degeneracy() : options_.normal_w_degeneracy();
  
  last_runtime_seconds_ = runtime_seconds;
  
  // ========== L1: 扫描匹配置信度（基础层，始终执行）==========
  auto scan_match_result = scan_match_confidence_->Evaluate(
      point_cloud, pose_estimate, covariance,
      ceres_final_cost, num_ceres_residuals,
      matching_grid, compute_uncertainty);
  
  metrics.scan_match_score = scan_match_result.total_confidence;
  metrics.ceres_residual_score = scan_match_result.ceres_confidence;
  metrics.match_ratio = scan_match_result.match_ratio;
  metrics.covariance_score = scan_match_result.covariance_confidence;
  
  // ========== L2: 运动一致性 ==========
  if (options_.enable_motion_check()) {
    auto motion_result = motion_checker_->Evaluate(
        pose_prediction, pose_estimate, common::FromUniversal(0));
    metrics.motion_consistency = motion_result.total_confidence;
    metrics.motion_delta_score = motion_result.prediction_observation_score;
    metrics.velocity_smooth_score = motion_result.velocity_smoothness_score;
  }
  
  // ========== L3: 地图一致性 ==========
  if (options_.enable_map_check() && matching_grid != nullptr) {
    auto map_result = map_evaluator_->Evaluate(
        point_cloud, pose_estimate, matching_grid);
    metrics.map_coherence = map_result.total_confidence;
    metrics.map_entropy = map_result.map_entropy;
    metrics.submap_match_score = map_result.submap_match_score;
  }
  
  // ========== L4: 几何退化检测 ==========
  if (options_.enable_degeneracy_check()) {
    auto degeneracy_result = line_data.empty() 
      ? degeneracy_detector_->Evaluate(point_cloud, covariance)
      : degeneracy_detector_->EvaluateWithLines(point_cloud, line_data, covariance);
    
    metrics.degeneracy_factor = degeneracy_result.degeneracy_factor;
    metrics.is_corridor_detected = degeneracy_result.is_corridor;
    metrics.feature_distribution_score = degeneracy_result.feature_distribution;
  }
  
  // ========== 融合计算总置信度（只考虑有效的评估项）==========
  double weighted_sum = 0.0;
  double weight_sum = 0.0;
  
  if (metrics.scan_match_score.has_value()) {
    weighted_sum += w_scan_match * (*metrics.scan_match_score);
    weight_sum += w_scan_match;
  }
  
  if (metrics.motion_consistency.has_value()) {
    weighted_sum += w_motion * (*metrics.motion_consistency);
    weight_sum += w_motion;
  }
  
  if (metrics.map_coherence.has_value()) {
    weighted_sum += w_map * (*metrics.map_coherence);
    weight_sum += w_map;
  }
  
  if (metrics.degeneracy_factor.has_value()) {
    weighted_sum += w_degeneracy * (*metrics.degeneracy_factor);
    weight_sum += w_degeneracy;
  }
  
  // 计算加权平均值
  if (weight_sum > 0.0) {
    metrics.total_confidence = weighted_sum / weight_sum;
  } else {
    // 如果没有任何有效评估项，置信度为0
    metrics.total_confidence = 0.0;
    LOG(WARNING) << "No valid evaluation layer enabled! Total confidence = 0.0";
  }
  
  // 确保在[0,1]范围内
  metrics.total_confidence = std::min(1.0, std::max(0.0, metrics.total_confidence));
  
  // 确定置信度等级
  metrics.level = DetermineLevel(metrics.total_confidence);
  
  // 日志输出（VLOG级别，可通过glog配置控制）
  VLOG(2) << "Confidence Evaluation (" << (is_initialization ? "INIT" : "NORMAL") << " phase):"
          << " Total=" << metrics.total_confidence
          << " Level=" << static_cast<int>(metrics.level)
          << " | L1=" << (metrics.scan_match_score.has_value() ? 
                          std::to_string(*metrics.scan_match_score) : "N/A")
          << " (w=" << w_scan_match << ")"
          << " L2=" << (metrics.motion_consistency.has_value() ? 
                        std::to_string(*metrics.motion_consistency) : "N/A")
          << " (w=" << w_motion << ")"
          << " L3=" << (metrics.map_coherence.has_value() ? 
                        std::to_string(*metrics.map_coherence) : "N/A")
          << " (w=" << w_map << ")"
          << " L4=" << (metrics.degeneracy_factor.has_value() ? 
                        std::to_string(*metrics.degeneracy_factor) : "N/A")
          << " (w=" << w_degeneracy << ")";
  
  if (metrics.is_corridor_detected.has_value() && *metrics.is_corridor_detected) {
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
  last_runtime_seconds_ = 0.0;
  LOG(INFO) << "Confidence estimator reset.";
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
