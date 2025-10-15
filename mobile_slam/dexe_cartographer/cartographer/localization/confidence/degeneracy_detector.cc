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

#include "cartographer/localization/confidence/degeneracy_detector.h"
#include <cmath>
#include <algorithm>
#include <random>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

std::vector<DegeneracyDetector::Line> DegeneracyDetector::FitLinesRANSAC(
    const sensor::PointCloud& point_cloud,
    int max_lines,
    double distance_threshold,
    int min_inliers) const {
  
  std::vector<Line> lines;
  
  if (point_cloud.size() < static_cast<size_t>(min_inliers * 2)) {
    return lines;
  }
  
  std::vector<bool> used(point_cloud.size(), false);
  std::random_device rd;
  std::mt19937 gen(rd());
  
  for (int line_idx = 0; line_idx < max_lines && lines.size() < static_cast<size_t>(max_lines); ++line_idx) {
    // RANSAC迭代
    int best_inliers = 0;
    Line best_line;
    
    const int max_iterations = 100;
    for (int iter = 0; iter < max_iterations; ++iter) {
      // 随机选择两个未使用的点
      std::vector<size_t> available_indices;
      for (size_t i = 0; i < point_cloud.size(); ++i) {
        if (!used[i]) {
          available_indices.push_back(i);
        }
      }
      
      if (available_indices.size() < 2) {
        break;
      }
      
      std::uniform_int_distribution<> dis(0, available_indices.size() - 1);
      size_t idx1 = available_indices[dis(gen)];
      size_t idx2 = available_indices[dis(gen)];
      
      if (idx1 == idx2) {
        continue;
      }
      
      // 构造直线
      Eigen::Vector2f p1 = point_cloud[idx1].position.head<2>();
      Eigen::Vector2f p2 = point_cloud[idx2].position.head<2>();
      Eigen::Vector2f dir = (p2 - p1).normalized();
      
      if (dir.norm() < 1e-6) {
        continue;
      }
      
      Line line;
      line.point = p1;
      line.direction = dir;
      
      // 统计内点
      std::vector<size_t> inliers;
      for (size_t i = 0; i < point_cloud.size(); ++i) {
        if (!used[i]) {
          Eigen::Vector2f p = point_cloud[i].position.head<2>();
          if (line.DistanceToPoint(p) < distance_threshold) {
            inliers.push_back(i);
          }
        }
      }
      
      if (static_cast<int>(inliers.size()) > best_inliers) {
        best_inliers = inliers.size();
        best_line = line;
      }
    }
    
    // 如果找到足够好的直线，添加并标记内点
    if (best_inliers >= min_inliers) {
      lines.push_back(best_line);
      
      // 标记内点为已使用
      for (size_t i = 0; i < point_cloud.size(); ++i) {
        if (!used[i]) {
          Eigen::Vector2f p = point_cloud[i].position.head<2>();
          if (best_line.DistanceToPoint(p) < distance_threshold) {
            used[i] = true;
          }
        }
      }
    } else {
      break;  // 找不到更多直线
    }
  }
  
  return lines;
}

bool DegeneracyDetector::AreParallel(
    const Line& line1, const Line& line2, double angle_threshold) const {
  double dot_product = std::abs(line1.direction.dot(line2.direction));
  // 如果方向向量接近平行或反平行
  return dot_product > std::cos(angle_threshold);
}

bool DegeneracyDetector::DetectCorridor(const sensor::PointCloud& point_cloud) const {
  // 拟合直线
  auto lines = FitLinesRANSAC(point_cloud);
  
  if (lines.size() < 2) {
    return false;
  }
  
  // 检测平行线对
  int parallel_pairs = 0;
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t j = i + 1; j < lines.size(); ++j) {
      if (AreParallel(lines[i], lines[j])) {
        // 检查距离（走廊宽度应在合理范围内，如1-10米）
        Eigen::Vector2f vec = lines[j].point - lines[i].point;
        double distance = std::abs(vec.dot(
            Eigen::Vector2f(-lines[i].direction.y(), lines[i].direction.x())));
        
        if (distance > 1.0 && distance < 10.0) {
          ++parallel_pairs;
        }
      }
    }
  }
  
  // 如果有至少一对平行墙面，认为是走廊
  return parallel_pairs >= 1;
}

double DegeneracyDetector::ComputeFeatureDistribution(
    const sensor::PointCloud& point_cloud) const {
  
  if (point_cloud.empty()) {
    return 0.0;
  }
  
  // 将360度分成扇区
  std::vector<int> sector_counts(kNumSectors, 0);
  
  for (const auto& point : point_cloud) {
    double angle = std::atan2(point.position.y(), point.position.x());
    int sector = static_cast<int>((angle + M_PI) / (2.0 * M_PI) * kNumSectors) % kNumSectors;
    sector_counts[sector]++;
  }
  
  // 计算分布的标准差（归一化）
  double mean = static_cast<double>(point_cloud.size()) / kNumSectors;
  double variance = 0.0;
  
  for (int count : sector_counts) {
    double diff = count - mean;
    variance += diff * diff;
  }
  variance /= kNumSectors;
  
  double std_dev = std::sqrt(variance);
  
  // 归一化：完全均匀分布时std_dev=0，返回1
  // 分布越不均匀，置信度越低
  double max_std_dev = mean;  // 理论最大标准差
  return 1.0 - std::min(1.0, std_dev / max_std_dev);
}

double DegeneracyDetector::EvaluateHessianCondition(
    const Eigen::Matrix3d& covariance) const {
  
  // 提取位置部分
  Eigen::Matrix2d cov_pos = covariance.block<2, 2>(0, 0);
  
  // 计算特征值（Hessian的逆）
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov_pos);
  Eigen::Vector2d eigenvalues = solver.eigenvalues();
  
  // 条件数
  double condition_number = eigenvalues(1) / std::max(eigenvalues(0), 1e-10);
  
  return condition_number;
}

DegeneracyDetector::Result DegeneracyDetector::Evaluate(
    const sensor::PointCloud& point_cloud,
    const Eigen::Matrix3d& covariance) const {
  
  Result result;
  
  // 检测走廊
  result.is_corridor = DetectCorridor(point_cloud);
  
  // 计算特征分布
  result.feature_distribution = ComputeFeatureDistribution(point_cloud);
  
  // 评估Hessian条件数
  result.hessian_condition_num = EvaluateHessianCondition(covariance);
  
  // 计算退化因子
  if (result.is_corridor && result.hessian_condition_num > kConditionNumberThreshold) {
    // 检测到走廊且条件数很大，严重退化
    result.degeneracy_factor = 0.3;
  } else if (result.is_corridor || result.hessian_condition_num > kConditionNumberThreshold) {
    // 部分退化
    result.degeneracy_factor = 0.6;
  } else {
    // 结合特征分布
    result.degeneracy_factor = 
        0.5 + 0.5 * result.feature_distribution;
  }
  
  return result;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
