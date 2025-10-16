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
#include <optional>
#include "cartographer/localization/confidence/scan_match_confidence.h"
#include "cartographer/localization/confidence/motion_consistency_checker.h"
#include "cartographer/localization/confidence/map_coherence_evaluator.h"
#include "cartographer/localization/confidence/degeneracy_detector.h"
#include "cartographer/localization/proto/confidence_estimator_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "Eigen/Core"

namespace cartographer {
namespace localization {
namespace confidence {

// 置信度等级枚举
enum class ConfidenceLevel {
  EXCELLENT = 4,
  GOOD = 3,
  MEDIUM = 2,
  POOR = 1,
  FAILED = 0
};

// 主置信度评估器接口
class ConfidenceEstimator {
 public:
  // 置信度评估结果结构
  struct Metrics {
    std::optional<double> scan_match_score;      // L1: 扫描匹配置信度
    std::optional<double> motion_consistency;    // L2: 运动一致性
    std::optional<double> map_coherence;         // L3: 地图一致性
    std::optional<double> degeneracy_factor;     // L4: 几何退化因子
    double total_confidence;      // 总置信度
    ConfidenceLevel level;        // 置信度等级
    
    // 详细指标
    std::optional<double> ceres_residual_score;  // Ceres残差得分
    std::optional<double> match_ratio;           // 匹配点比例
    std::optional<double> covariance_score;      // 协方差得分
    
    std::optional<double> motion_delta_score;    // 运动偏差得分
    std::optional<double> velocity_smooth_score; // 速度平滑度得分
    
    std::optional<double> map_entropy;           // 地图熵
    std::optional<double> submap_match_score;    // 子图匹配得分
    
    std::optional<bool> is_corridor_detected;    // 是否检测到走廊
    std::optional<double> feature_distribution_score; // 特征分布得分
    
    // 打印所有评估项
    std::string ToString() const;
    void Print() const;
  };

  explicit ConfidenceEstimator(const proto::ConfidenceEstimatorOptions& options);
  
  // 主评估接口
  Metrics Evaluate(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose_estimate,
      const transform::Rigid2d& pose_prediction,
      const Eigen::Matrix3d& covariance,
      double ceres_final_cost,
      int num_ceres_residuals,
      const std::vector<sensor::LineData>& line_data = std::vector<sensor::LineData>(),
      const mapping::Grid2D* matching_grid = nullptr,
      double runtime_seconds = 0.0,
      bool compute_uncertainty = false);

  
  // 重置状态（用于重定位）
  void Reset();
  
  // 关键失败检测结构
  struct CriticalFailureInfo {
    bool is_critical_failure = false;    // 是否为关键失败
    std::string failure_reason;          // 失败原因描述
    std::vector<std::string> triggered_conditions;  // 触发的所有条件
  };
  
  // 检查是否存在导致定位直接失败的关键条件
  // 返回关键失败信息，如果is_critical_failure=true则定位应直接判定为FAILED
  CriticalFailureInfo CheckCriticalFailure(
      const Metrics& metrics,
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose_prediction,
      const transform::Rigid2d& pose_estimate,
      double ceres_final_cost,
      int num_ceres_residuals) const;

 private:
  ConfidenceLevel DetermineLevel(double total_confidence) const;
  
  // 配置选项
  proto::ConfidenceEstimatorOptions options_;
  
  // 各层评估器
  std::unique_ptr<ScanMatchConfidence> scan_match_confidence_;
  std::unique_ptr<MotionConsistencyChecker> motion_checker_;
  std::unique_ptr<MapCoherenceEvaluator> map_evaluator_;
  std::unique_ptr<DegeneracyDetector> degeneracy_detector_;
  
  // 运行时状态
  double last_runtime_seconds_ = 0.0;
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_CONFIDENCE_ESTIMATOR_H_
