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

#ifndef CARTOGRAPHER_LOCALIZATION_CONFIDENCE_DEGENERACY_DETECTOR_H_
#define CARTOGRAPHER_LOCALIZATION_CONFIDENCE_DEGENERACY_DETECTOR_H_

#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "Eigen/Core"
#include <vector>

namespace cartographer {
namespace localization {
namespace confidence {

// L4层：几何退化检测器
class DegeneracyDetector {
 public:
  struct Result {
    bool is_corridor;              // 是否检测到走廊
    double feature_distribution;   // 特征分布得分 [0,1]
    double degeneracy_factor;      // 退化因子 [0,1]，1表示无退化
    int num_parallel_walls;        // 平行墙面数量
    double hessian_condition_num;  // Hessian矩阵条件数
    
    // 打印评估项
    std::string ToString() const;
    void Print() const;
  };

  DegeneracyDetector() = default;

  // 走廊检测结果
  struct CorridorDetectionResult {
    bool is_corridor;           // 是否为走廊
    double parallelism_ratio;   // 平行度比例 [0,1]
  };

  // 检测走廊环境
  // 使用RANSAC拟合直线，检测平行墙面
  bool DetectCorridorFromCloud(const sensor::PointCloud& point_cloud) const;

  // 检测走廊环境（使用预提取的直线数据）
  // 直接使用merged data中已提取的直线，避免重复RANSAC拟合
  // 返回走廊检测结果和平行度比例
  CorridorDetectionResult DetectCorridorFromLines(
      const std::vector<sensor::LineData>& lines) const;

  // 计算特征分布均匀性
  // 基于点云在不同方向的分布
  double ComputeFeatureDistribution(const sensor::PointCloud& point_cloud) const;

  // 评估Hessian矩阵条件数（从协方差计算）
  double EvaluateHessianCondition(const Eigen::Matrix3d& covariance) const;

  // 综合评估
  Result Evaluate(
      const sensor::PointCloud& point_cloud,
      const Eigen::Matrix3d& covariance) const;

  // 综合评估（使用预提取的直线数据）
  // 避免重复RANSAC拟合，减少运算量
  Result EvaluateWithLines(
      const sensor::PointCloud& point_cloud,
      const std::vector<sensor::LineData>& lines,
      const Eigen::Matrix3d& covariance) const;

 private:
  // RANSAC直线拟合
  struct Line {
    Eigen::Vector2f point;     // 线上一点
    Eigen::Vector2f direction; // 方向向量（归一化）
    
    double DistanceToPoint(const Eigen::Vector2f& p) const {
      Eigen::Vector2f v = p - point;
      Eigen::Vector2f proj = v.dot(direction) * direction;
      return (v - proj).norm();
    }
  };

  std::vector<Line> FitLinesRANSAC(
      const sensor::PointCloud& point_cloud,
      int max_lines = 10,
      double distance_threshold = 0.1,
      int min_inliers = 20) const;

  bool AreParallel(const Line& line1, const Line& line2, double angle_threshold = 0.174) const; // 10度

  // ========== 新增：多指标退化评估辅助函数 ==========
  
  // 计算几何平行度（从RANSAC拟合的直线）
  double ComputeParallelismRatio(const std::vector<Line>& lines) const;
  
  // 计算观测方向熵（方案指标2：Observable Direction Entropy）
  double ComputeObservationEntropy(const sensor::PointCloud& point_cloud) const;
  
  // 参数
  static constexpr double kConditionNumberThreshold = 50.0;  // 条件数阈值
  static constexpr int kNumSectors = 36;  // 扇区数量（角度划分）
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_DEGENERACY_DETECTOR_H_
