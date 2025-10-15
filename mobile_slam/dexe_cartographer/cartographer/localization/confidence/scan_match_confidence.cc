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

#include "cartographer/localization/confidence/scan_match_confidence.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

// Result打印方法实现
std::string ScanMatchConfidence::Result::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  
  oss << "ScanMatchConfidence Result:\n";
  oss << "  Total Confidence:    " << total_confidence << "\n";
  oss << "  ├─ CSM:              " << csm_confidence << "\n";
  oss << "  ├─ Ceres:            " << ceres_confidence << "\n";
  oss << "  ├─ Match Ratio:      " << match_ratio << "\n";
  oss << "  ├─ Covariance:       " << covariance_confidence << "\n";
  oss << "  └─ Angle Coverage:   " << angle_coverage_entropy << "\n";
  
  return oss.str();
}

void ScanMatchConfidence::Result::Print() const {
  LOG(INFO) << ToString();
}

double ScanMatchConfidence::ComputeCSMConfidence(double csm_score) const {
  // CSM得分通常在0-1范围，直接作为置信度
  // 可以应用非线性变换增强区分度
  return std::min(1.0, std::max(0.0, csm_score));
}

double ScanMatchConfidence::ComputeCeresConfidence(
    double final_cost, int num_residuals) const {
  if (num_residuals <= 0) {
    return 0.0;
  }
  
  // 计算平均残差
  double mean_residual = std::sqrt(final_cost / num_residuals);
  
  // 使用指数衰减函数
  // C_ceres = exp(-mean_residual / sigma)
  double confidence = std::exp(-mean_residual / kCeresResidualSigma);
  
  return std::min(1.0, std::max(0.0, confidence));
}

double ScanMatchConfidence::ComputeMatchRatio(
    const sensor::PointCloud& point_cloud,
    const mapping::Grid2D& grid,
    const transform::Rigid2d& pose) const {
  if (point_cloud.empty()) {
    return 0.0;
  }
  
  // Try to cast to ProbabilityGrid for probability access
  const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(&grid);
  if (prob_grid == nullptr) {
    // If not a ProbabilityGrid, use correspondence cost instead
    return 0.5;  // Default value for non-probability grids
  }
  
  int num_hit = 0;
  for (const auto& point : point_cloud) {
    // 将点转换到地图坐标系
    const Eigen::Vector2f transformed_point = 
        pose.cast<float>() * point.position.head<2>();
    
    // 检查栅格占用概率
    if (prob_grid->IsKnown(prob_grid->limits().GetCellIndex(transformed_point))) {
      float probability = prob_grid->GetProbability(
          prob_grid->limits().GetCellIndex(transformed_point));
      if (probability >= kMinOccupancyProbability) {
        ++num_hit;
      }
    }
  }
  
  return static_cast<double>(num_hit) / point_cloud.size();
}

double ScanMatchConfidence::ComputeAngleCoverageEntropy(
    const sensor::PointCloud& point_cloud,
    const mapping::Grid2D& grid,
    const transform::Rigid2d& pose,
    int num_sectors) const {
  if (point_cloud.empty()) {
    return 0.0;
  }
  
  // Try to cast to ProbabilityGrid for probability access
  const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(&grid);
  if (prob_grid == nullptr) {
    // If not a ProbabilityGrid, use default value
    return 0.5;
  }
  
  // 将360度分成num_sectors个扇区
  std::vector<int> sector_counts(num_sectors, 0);
  int total_matched = 0;
  
  for (const auto& point : point_cloud) {
    // 计算点的角度
    double angle = std::atan2(point.position.y(), point.position.x());
    int sector = static_cast<int>((angle + M_PI) / (2.0 * M_PI) * num_sectors) % num_sectors;
    
    // 检查是否匹配到地图
    const Eigen::Vector2f transformed_point = 
        pose.cast<float>() * point.position.head<2>();
    if (prob_grid->IsKnown(prob_grid->limits().GetCellIndex(transformed_point))) {
      float probability = prob_grid->GetProbability(
          prob_grid->limits().GetCellIndex(transformed_point));
      if (probability >= kMinOccupancyProbability) {
        sector_counts[sector]++;
        total_matched++;
      }
    }
  }
  
  if (total_matched == 0) {
    return 0.0;
  }
  
  // 计算熵: H = -Σ p_i * log(p_i)
  double entropy = 0.0;
  for (int count : sector_counts) {
    if (count > 0) {
      double p = static_cast<double>(count) / total_matched;
      entropy -= p * std::log(p);
    }
  }
  
  // 归一化（最大熵为log(num_sectors)）
  return entropy / std::log(num_sectors);
}

double ScanMatchConfidence::ComputeCovarianceConfidence(
    const Eigen::Matrix3d& covariance) const {
  // 提取位置部分的2x2协方差矩阵
  Eigen::Matrix2d cov_pos = covariance.block<2, 2>(0, 0);
  
  // 计算特征值
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov_pos);
  Eigen::Vector2d eigenvalues = solver.eigenvalues();
  
  // 计算椭圆面积
  double area = M_PI * std::sqrt(eigenvalues(0) * eigenvalues(1));
  
  // 计算条件数
  double condition_number = eigenvalues(1) / std::max(eigenvalues(0), 1e-10);
  condition_number = std::min(condition_number, kMaxConditionNumber);
  
  // 综合置信度
  // C_cov = exp(-α * A) * (1 / (1 + β * log(κ)))
  double area_factor = std::exp(-kCovarianceAlpha * area);
  double condition_factor = 1.0 / (1.0 + kCovarianceBeta * std::log(condition_number));
  
  return area_factor * condition_factor;
}

ScanMatchConfidence::Result ScanMatchConfidence::Evaluate(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose,
    const Eigen::Matrix3d& covariance,
    double csm_score,
    double ceres_final_cost,
    int num_residuals,
    const mapping::Grid2D* grid) const {
  
  Result result;
  
  // L1.1: CSM置信度
  result.csm_confidence = ComputeCSMConfidence(csm_score);
  
  // L1.2: Ceres置信度
  result.ceres_confidence = ComputeCeresConfidence(ceres_final_cost, num_residuals);
  
  // L1.3: 匹配率和角度覆盖
  if (grid != nullptr && !point_cloud.empty()) {
    result.match_ratio = ComputeMatchRatio(point_cloud, *grid, pose);
    result.angle_coverage_entropy = ComputeAngleCoverageEntropy(
        point_cloud, *grid, pose);
  } else {
    result.match_ratio = 0.5;  // 默认值
    result.angle_coverage_entropy = 0.5;
  }
  
  // L1.4: 协方差置信度
  result.covariance_confidence = ComputeCovarianceConfidence(covariance);
  
  // 综合置信度（加权平均）
  // 这里使用默认权重，可由外部配置
  const double w1 = 0.3;  // CSM
  const double w2 = 0.3;  // Ceres
  const double w3 = 0.2;  // 匹配率
  const double w4 = 0.2;  // 协方差
  
  result.total_confidence = 
      w1 * result.csm_confidence +
      w2 * result.ceres_confidence +
      w3 * result.match_ratio +
      w4 * result.covariance_confidence;
  
  return result;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
