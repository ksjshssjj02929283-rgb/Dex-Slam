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

#ifndef CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MAP_COHERENCE_EVALUATOR_H_
#define CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MAP_COHERENCE_EVALUATOR_H_

#include "cartographer/sensor/point_cloud.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace localization {
namespace confidence {

// L3层：地图一致性评估器
class MapCoherenceEvaluator {
 public:
  struct Result {
    double submap_match_score;  // 子图匹配得分 [0,1]
    double map_entropy;         // 地图熵 [0,1]
    double total_confidence;    // 综合置信度 [0,1]
    
    // 打印评估项
    std::string ToString() const;
    void Print() const;
  };

  MapCoherenceEvaluator() = default;

  // 计算子图匹配得分
  // 评估当前扫描与地图的一致性
  double ComputeSubmapMatchScore(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose,
      const mapping::ProbabilityGrid& grid) const;

  // 计算局部地图熵（环境复杂度）
  // pose: 当前位姿
  // grid: 概率栅格地图
  // radius_meters: 评估半径(默认5m)
  double ComputeLocalMapEntropy(
      const transform::Rigid2d& pose,
      const mapping::ProbabilityGrid& grid,
      double radius_meters = 5.0) const;

  // 综合评估
  Result Evaluate(
      const sensor::PointCloud& point_cloud,
      const transform::Rigid2d& pose,
      const mapping::Grid2D* grid = nullptr) const;

 private:
  // 参数
  static constexpr double kGoodMatchThreshold = 0.7;  // 良好匹配阈值
  static constexpr double kLowEntropyThreshold = 0.5;  // 低熵阈值（空旷环境）
  static constexpr double kHighEntropyThreshold = 1.5; // 高熵阈值（丰富特征）
};

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_CONFIDENCE_MAP_COHERENCE_EVALUATOR_H_
