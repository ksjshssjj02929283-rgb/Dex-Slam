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
    double ceres_confidence;      // Ceres优化置信度 [0,1]
    double match_ratio;           // 匹配点比例 [0,1]
    double covariance_confidence; // 协方差置信度 [0,1]
    double angle_coverage_entropy;// 角度覆盖熵
    double total_confidence;      // 综合置信度 [0,1]
    
    // 打印评估项
    std::string ToString() const;
    void Print() const;
  };

  ScanMatchConfidence() = default;

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
  double ComputeCovarianceConfidence(const Eigen::Matrix3d& covariance, 
                                     bool compute_uncertainty = false) const;

  // 综合评估
  Result Evaluate(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose,
      const Eigen::Matrix3d& covariance,
      double ceres_final_cost,
      int num_residuals,
      const mapping::Grid2D* grid = nullptr,
      bool compute_uncertainty = false) const;

 private:
  // 参数配置
  // 严格的10cm误差阈值配置：
  // - 当位置不确定性（1-sigma）= 10cm时，协方差置信度 ≈ 0.37
  // - 当位置不确定性（1-sigma）> 15cm时，协方差置信度 < 0.1（定位失败）
  
  static constexpr double kCeresResidualSigma = 1.0;  // Ceres残差标准差阈值
  static constexpr double kMinOccupancyProbability = 0.55;  // 最小占用概率
  
  // 椭圆面积系数：α = 32.0
  // 面积-置信度映射：
  //   A=0.0314m² (r=10cm圆) → exp(-32×0.0314) ≈ 0.368 (阈值)
  //   A=0.0707m² (r=15cm圆) → exp(-32×0.0707) ≈ 0.100 (定位失败)
  //   A=0.1257m² (r=20cm圆) → exp(-32×0.1257) ≈ 0.020 (严重失败)
  static constexpr double kCovarianceAlpha = 32.0;  // 椭圆面积系数（严格10cm阈值）
  
  // 条件数系数：β = 0.2（加强退化检测）
  // 条件数-置信度映射：
  //   κ=10  (轻度扁) → 1/(1+0.2×ln(10)) ≈ 0.685
  //   κ=50  (中度扁) → 1/(1+0.2×ln(50)) ≈ 0.440
  //   κ=100 (严重扁) → 1/(1+0.2×ln(100)) ≈ 0.352
  static constexpr double kCovarianceBeta = 0.2;    // 条件数系数（加强退化检测）
  
  static constexpr double kMaxConditionNumber = 100.0;  // 最大条件数（椭圆扁平程度上限）
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_SCAN_MATCH_CONFIDENCE_H_
