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

#ifndef CARTOGRAPHER_LOCALIZATION_CONFIDENCE_CONFIDENCE_ESTIMATOR_H_
#define CARTOGRAPHER_LOCALIZATION_CONFIDENCE_CONFIDENCE_ESTIMATOR_H_

#include <memory>
#include "cartographer/localization/confidence/scan_match_confidence.h"
#include "cartographer/localization/confidence/motion_consistency_checker.h"
#include "cartographer/localization/confidence/map_coherence_evaluator.h"
#include "cartographer/localization/confidence/degeneracy_detector.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "Eigen/Core"

namespace cartographer {
namespace localization {
namespace confidence {

// 置信度等级枚举
enum class ConfidenceLevel {
  EXCELLENT = 4,  // >= 0.8
  GOOD = 3,       // >= 0.6
  MEDIUM = 2,     // >= 0.4
  POOR = 1,       // >= 0.2
  FAILED = 0      // < 0.2
};

// 主置信度评估器接口
class ConfidenceEstimator {
 public:
  // 置信度评估结果结构
  struct Metrics {
    double scan_match_score;      // L1: 扫描匹配置信度
    double motion_consistency;    // L2: 运动一致性
    double map_coherence;         // L3: 地图一致性
    double degeneracy_factor;     // L4: 几何退化因子
    double total_confidence;      // 总置信度
    ConfidenceLevel level;        // 置信度等级
    
    // 详细指标
    double csm_score;             // CSM匹配得分
    double ceres_residual_score;  // Ceres残差得分
    double match_ratio;           // 匹配点比例
    double covariance_score;      // 协方差得分
    
    double motion_delta_score;    // 运动偏差得分
    double velocity_smooth_score; // 速度平滑度得分
    
    double map_entropy;           // 地图熵
    double submap_match_score;    // 子图匹配得分
    
    bool is_corridor_detected;    // 是否检测到走廊
    double feature_distribution_score; // 特征分布得分
  };

  struct Config {
    // L1 权重
    double w_csm = 0.3;
    double w_ceres = 0.3;
    double w_match_ratio = 0.2;
    double w_covariance = 0.2;
    
    // 融合权重（动态调整）
    double w_scan_match = 0.3;
    double w_motion = 0.25;
    double w_map = 0.35;
    double w_degeneracy = 0.1;
    
    // 阈值
    double excellent_threshold = 0.8;
    double good_threshold = 0.6;
    double medium_threshold = 0.4;
    double poor_threshold = 0.2;
    
    // 运行时间（用于动态权重调整）
    double initialization_duration_sec = 5.0;
    double post_loop_duration_sec = 10.0;
    
    // 是否启用各层
    bool enable_motion_check = true;
    bool enable_map_check = true;
    bool enable_degeneracy_check = true;
  };

  ConfidenceEstimator();
  explicit ConfidenceEstimator(const Config& config);
  
  // 主评估接口
  Metrics Evaluate(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose_estimate,
      const transform::Rigid2d& pose_prediction,
      const Eigen::Matrix3d& covariance,
      double csm_score,
      double ceres_final_cost,
      int num_ceres_residuals,
      const mapping::Grid2D* matching_grid = nullptr,
      double runtime_seconds = 0.0);

  // 更新配置
  void UpdateConfig(const Config& config) { config_ = config; }
  Config GetConfig() const { return config_; }
  
  // 重置状态（用于重定位）
  void Reset();

 private:
  ConfidenceLevel DetermineLevel(double total_confidence) const;
  void AdjustWeightsByRuntime(double runtime_seconds);
  
  Config config_;
  
  // 各层评估器
  std::unique_ptr<ScanMatchConfidence> scan_match_confidence_;
  std::unique_ptr<MotionConsistencyChecker> motion_checker_;
  std::unique_ptr<MapCoherenceEvaluator> map_evaluator_;
  std::unique_ptr<DegeneracyDetector> degeneracy_detector_;
  
  // 运行时状态
  double last_runtime_seconds_ = 0.0;
  bool is_post_loop_closure_ = false;
  common::Time last_loop_closure_time_;
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_CONFIDENCE_ESTIMATOR_H_
