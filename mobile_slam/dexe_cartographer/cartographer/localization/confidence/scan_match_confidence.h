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

#ifndef CARTOGRAPHER_LOCALIZATION_CONFIDENCE_SCAN_MATCH_CONFIDENCE_H_
#define CARTOGRAPHER_LOCALIZATION_CONFIDENCE_SCAN_MATCH_CONFIDENCE_H_

#include "cartographer/sensor/point_cloud.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "Eigen/Core"

namespace cartographer {
namespace localization {
namespace confidence {

// L1层：扫描匹配置信度评估
class ScanMatchConfidence {
 public:
  struct Result {
    double csm_confidence;        // CSM匹配置信度 [0,1]
    double ceres_confidence;      // Ceres优化置信度 [0,1]
    double match_ratio;           // 匹配点比例 [0,1]
    double covariance_confidence; // 协方差置信度 [0,1]
    double angle_coverage_entropy;// 角度覆盖熵
    double total_confidence;      // 综合置信度 [0,1]
  };

  ScanMatchConfidence() = default;

  // 计算CSM置信度
  // csm_score: CSM匹配得分（通常0-1范围）
  // 返回归一化的置信度
  double ComputeCSMConfidence(double csm_score) const;

  // 计算Ceres优化置信度
  // final_cost: Ceres最终cost值
  // num_residuals: 残差数量
  double ComputeCeresConfidence(double final_cost, int num_residuals) const;

  // 计算匹配点比例和覆盖率
  // point_cloud: 激光点云
  // grid: 匹配的概率栅格
  // pose: 匹配后的位姿
  double ComputeMatchRatio(
      const sensor::PointCloud& point_cloud,
      const mapping::Grid2D& grid,
      const transform::Rigid2d& pose) const;

  // 计算角度覆盖熵
  double ComputeAngleCoverageEntropy(
      const sensor::PointCloud& point_cloud,
      const mapping::Grid2D& grid,
      const transform::Rigid2d& pose,
      int num_sectors = 36) const;

  // 计算协方差置信度
  // covariance: 3x3协方差矩阵（x, y, theta）
  double ComputeCovarianceConfidence(const Eigen::Matrix3d& covariance) const;

  // 综合评估
  Result Evaluate(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose,
      const Eigen::Matrix3d& covariance,
      double csm_score,
      double ceres_final_cost,
      int num_residuals,
      const mapping::Grid2D* grid = nullptr) const;

 private:
  // 参数配置
  static constexpr double kCeresResidualSigma = 0.05;  // Ceres残差标准差阈值
  static constexpr double kMinOccupancyProbability = 0.55;  // 最小占用概率
  static constexpr double kCovarianceAlpha = 10.0;  // 椭圆面积系数
  static constexpr double kCovarianceBeta = 0.1;    // 条件数系数
  static constexpr double kMaxConditionNumber = 100.0;  // 最大条件数
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_SCAN_MATCH_CONFIDENCE_H_
