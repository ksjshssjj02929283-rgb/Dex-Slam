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
#include <sstream>
#include <iomanip>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

// Result打印方法实现
std::string DegeneracyDetector::Result::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  
  oss << "DegeneracyDetector Result:\n";
  oss << "  Degeneracy Factor:   " << degeneracy_factor << "\n";
  oss << "  ├─ Is Corridor:      " << (is_corridor ? "YES" : "NO") << "\n";
  oss << "  ├─ Parallel Walls:   " << num_parallel_walls << "\n";
  oss << "  ├─ Feature Dist:     " << feature_distribution << "\n";
  oss << "  └─ Hessian Cond:     " << hessian_condition_num << "\n";
  
  return oss.str();
}

void DegeneracyDetector::Result::Print() const {
  LOG(INFO) << ToString();
}

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

bool DegeneracyDetector::DetectCorridorFromCloud(const sensor::PointCloud& point_cloud) const {
  // 拟合直线
  auto lines = FitLinesRANSAC(point_cloud);
  
  if (lines.size() < 2) {
    return false;
  }
  
  // 指标1: 计算几何平行度 (Parallelism Ratio)
  double parallelism_ratio = ComputeParallelismRatio(lines);
  
  // 指标2: 计算观测方向熵 (Observable Direction Entropy)
  double entropy_degeneracy = ComputeObservationEntropy(point_cloud);
  
  // 加权融合判断（几何平行度占60%，方向熵占40%）
  double combined_score = 0.6 * parallelism_ratio + 0.4 * entropy_degeneracy;
  
  // 阈值判断：组合得分 > 0.6 认为是走廊
  return combined_score > 0.6;
}

DegeneracyDetector::CorridorDetectionResult DegeneracyDetector::DetectCorridorFromLines(
    const std::vector<sensor::LineData>& lines) const {
  
  CorridorDetectionResult result;
  result.is_corridor = false;
  result.parallelism_ratio = 0.0;
  
  if (lines.size() < 2) {
    return result;
  }
  
  // 计算平行线对和平行度比例
  int parallel_pairs = 0;
  int total_pairs = 0;
  constexpr double kAngleThreshold = 0.1396;  // 8度
  
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t j = i + 1; j < lines.size(); ++j) {
      const auto& line1 = lines[i];
      const auto& line2 = lines[j];
      
      // 计算方向向量
      Eigen::Vector2d dir1 = (line1.end() - line1.begin()).normalized();
      Eigen::Vector2d dir2 = (line2.end() - line2.begin()).normalized();
      
      // 计算角度差（考虑周期性）
      double angle1 = std::atan2(dir1.y(), dir1.x());
      double angle2 = std::atan2(dir2.y(), dir2.x());
      double angle_diff = std::abs(angle1 - angle2);
      angle_diff = std::min(angle_diff, M_PI - angle_diff);
      
      if (angle_diff < kAngleThreshold) {
        // 额外检查距离（走廊宽度应在合理范围内，如1-10米）
        Eigen::Vector2d vec = line2.begin() - line1.begin();
        Eigen::Vector2d normal(-dir1.y(), dir1.x());
        double distance = std::abs(vec.dot(normal));
        
        // 走廊宽度应在合理范围内（1-10米）
        if (distance > 1.0 && distance < 10.0) {
          ++parallel_pairs;
        }
      }
      ++total_pairs;
    }
  }
  
  // 计算平行度比例
  result.parallelism_ratio = static_cast<double>(parallel_pairs) / total_pairs;
  
  // 判断是否为走廊（平行度 > 0.7）
  result.is_corridor = result.parallelism_ratio > 0.7;
  
  return result;
}

// ========== 新增：计算几何平行度（从RANSAC拟合的直线） ==========
double DegeneracyDetector::ComputeParallelismRatio(
    const std::vector<Line>& lines) const {
  
  if (lines.size() < 2) {
    return 0.0;
  }
  
  int parallel_count = 0;
  int total_pairs = 0;
  constexpr double kAngleThreshold = 0.1396;  // 8度
  
  for (size_t i = 0; i < lines.size(); ++i) {
    for (size_t j = i + 1; j < lines.size(); ++j) {
      // 计算角度差（考虑周期性）
      double angle1 = std::atan2(lines[i].direction.y(), lines[i].direction.x());
      double angle2 = std::atan2(lines[j].direction.y(), lines[j].direction.x());
      double angle_diff = std::abs(angle1 - angle2);
      angle_diff = std::min(angle_diff, M_PI - angle_diff);
      
      if (angle_diff < kAngleThreshold) {
        // 额外检查距离（走廊宽度应在合理范围内，如1-10米）
        Eigen::Vector2f vec = lines[j].point - lines[i].point;
        Eigen::Vector2f normal(-lines[i].direction.y(), lines[i].direction.x());
        double distance = std::abs(vec.dot(normal));
        
        if (distance > 1.0 && distance < 10.0) {
          ++parallel_count;
        }
      }
      ++total_pairs;
    }
  }
  
  return static_cast<double>(parallel_count) / total_pairs;
}

// ========== 新增：计算观测方向熵（方案中的指标2） ==========
double DegeneracyDetector::ComputeObservationEntropy(
    const sensor::PointCloud& point_cloud) const {
  
  if (point_cloud.empty()) {
    return 0.0;
  }
  
  // 将360度分成18个扇区（每20度一个）
  constexpr int kNumBins = 18;
  std::vector<int> histogram(kNumBins, 0);
  
  // 统计每个方向的点数
  for (const auto& point : point_cloud) {
    double angle = std::atan2(point.position.y(), point.position.x());
    int bin = static_cast<int>((angle + M_PI) / (2.0 * M_PI) * kNumBins) % kNumBins;
    histogram[bin]++;
  }
  
  // 计算香农熵
  double entropy = 0.0;
  double total_points = static_cast<double>(point_cloud.size());
  
  for (int count : histogram) {
    if (count > 0) {
      double prob = count / total_points;
      entropy -= prob * std::log(prob);
    }
  }
  
  // 归一化（最大熵 = log(18) ≈ 2.89）
  double max_entropy = std::log(static_cast<double>(kNumBins));
  double normalized_entropy = entropy / max_entropy;
  
  // 转换为退化度（熵越低，退化越严重）
  double degeneracy_score = 1.0 - normalized_entropy;
  
  return degeneracy_score;
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
  
  // ========== 多指标退化评估（方案优化版） ==========
  
  // 指标1: 走廊检测（已包含几何平行度+方向熵）
  result.is_corridor = DetectCorridorFromCloud(point_cloud);
  
  // 拟合直线并计算平行度比例
  auto lines = FitLinesRANSAC(point_cloud);
  double parallelism_ratio = ComputeParallelismRatio(lines);
  
  // 指标2: 特征分布均匀性（原有指标，保留）
  result.feature_distribution = ComputeFeatureDistribution(point_cloud);
  
  // 指标3: Hessian条件数（运动约束退化）
  result.hessian_condition_num = EvaluateHessianCondition(covariance);

  // 使用平行度比例来计算geometry_score（0.0=无平行，1.0=完全平行）
  // geometry_score越高，表示几何特征越好（退化越小）
  double geometry_score = 1.0 - parallelism_ratio;  // 转换：平行度高 -> 退化高 -> geometry_score低
  double entropy_score = result.feature_distribution;       // 特征分布均匀性
  double constraint_score = std::tanh(result.hessian_condition_num / 50.0);  // 约束退化度
  
  // 三指标加权融合（几何30%、熵35%、约束35%）
  double degeneracy_score = 
      0.30 * (1.0 - geometry_score) +      // 几何退化：平行度越高，这项越大
      0.35 * (1.0 - entropy_score) +       // 信息退化
      0.35 * constraint_score;              // 约束退化
  
  // 转换为置信度（0=严重退化，1=无退化）
  result.degeneracy_factor = 1.0 - degeneracy_score;
  
  // 确保在合理范围内
  result.degeneracy_factor = std::max(0.0, std::min(1.0, result.degeneracy_factor));
  
  // ========== DEBUG输出 ==========
  // LOG(INFO) << "[DegeneracyDetector::Evaluate] Metrics Debug:";
  // LOG(INFO) << "  - Is Corridor: " << (result.is_corridor ? "YES" : "NO");
  // LOG(INFO) << "  - Parallelism Ratio: " << parallelism_ratio;
  // LOG(INFO) << "  - Feature Distribution: " << result.feature_distribution;
  // LOG(INFO) << "  - Hessian Condition Num: " << result.hessian_condition_num;
  // LOG(INFO) << "  - Geometry Score: " << geometry_score;
  // LOG(INFO) << "  - Entropy Score: " << entropy_score;
  // LOG(INFO) << "  - Constraint Score: " << constraint_score;
  // LOG(INFO) << "  - Degeneracy Score: " << degeneracy_score;
  // LOG(INFO) << "  - Final Degeneracy Factor: " << result.degeneracy_factor;

  return result;
}

DegeneracyDetector::Result DegeneracyDetector::EvaluateWithLines(
    const sensor::PointCloud& point_cloud,
    const std::vector<sensor::LineData>& lines,
    const Eigen::Matrix3d& covariance) const {
  
  Result result;
  
  // ========== 多指标退化评估（使用预提取直线） ==========
  
  // 指标1: 走廊检测（使用预提取直线，避免RANSAC）
  // 同时获取is_corridor和parallelism_ratio
  auto corridor_result = DetectCorridorFromLines(lines);
  result.is_corridor = corridor_result.is_corridor;
  double parallelism_ratio = corridor_result.parallelism_ratio;
  
  // 指标2: 特征分布均匀性
  result.feature_distribution = ComputeFeatureDistribution(point_cloud);
  
  // 指标3: Hessian条件数
  result.hessian_condition_num = EvaluateHessianCondition(covariance);
  
  // ========== 加权融合计算退化因子（与Evaluate一致） ==========
  
  // 使用平行度比例来计算geometry_score（0.0=无平行，1.0=完全平行）
  // geometry_score越高，表示几何特征越好（退化越小）
  double geometry_score = 1.0 - parallelism_ratio;  // 转换：平行度高 -> 退化高 -> geometry_score低
  double entropy_score = result.feature_distribution;
  double constraint_score = std::tanh(result.hessian_condition_num / 50.0);
  
  double degeneracy_score = 
      0.30 * (1.0 - geometry_score) +      // 几何退化：平行度越高，这项越大
      0.35 * (1.0 - entropy_score) +       // 信息退化
      0.35 * constraint_score;              // 约束退化
  
  result.degeneracy_factor = 1.0 - degeneracy_score;
  result.degeneracy_factor = std::max(0.0, std::min(1.0, result.degeneracy_factor));
  
  // ========== DEBUG输出 ==========
  // LOG(INFO) << "[DegeneracyDetector::EvaluateWithLines] Metrics Debug:";
  // LOG(INFO) << "  - Input Lines Count: " << lines.size();
  // LOG(INFO) << "  - Is Corridor: " << (result.is_corridor ? "YES" : "NO");
  // LOG(INFO) << "  - Parallelism Ratio: " << parallelism_ratio;
  // LOG(INFO) << "  - Feature Distribution: " << result.feature_distribution;
  // LOG(INFO) << "  - Hessian Condition Num: " << result.hessian_condition_num;
  // LOG(INFO) << "  - Geometry Score: " << geometry_score;
  // LOG(INFO) << "  - Entropy Score: " << entropy_score;
  // LOG(INFO) << "  - Constraint Score: " << constraint_score;
  // LOG(INFO) << "  - Degeneracy Score: " << degeneracy_score;
  // LOG(INFO) << "  - Final Degeneracy Factor: " << result.degeneracy_factor;
  
  return result;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
