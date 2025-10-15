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

#include "cartographer/localization/confidence/map_coherence_evaluator.h"
#include <cmath>
#include <algorithm>
#include "glog/logging.h"

namespace cartographer {
namespace localization {
namespace confidence {

double MapCoherenceEvaluator::ComputeSubmapMatchScore(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose,
    const mapping::ProbabilityGrid& grid) const {
  
  if (point_cloud.empty()) {
    return 0.0;
  }
  
  double sum_probability = 0.0;
  int valid_points = 0;
  
  for (const auto& point : point_cloud) {
    // 转换到地图坐标系
    const Eigen::Vector2f transformed_point = 
        pose.cast<float>() * point.position.head<2>();
    
    const Eigen::Array2i cell_index = 
        grid.limits().GetCellIndex(transformed_point);
    
    if (grid.IsKnown(cell_index)) {
      sum_probability += grid.GetProbability(cell_index);
      ++valid_points;
    }
  }
  
  if (valid_points == 0) {
    return 0.0;
  }
  
  return sum_probability / valid_points;
}

double MapCoherenceEvaluator::ComputeLocalMapEntropy(
    const transform::Rigid2d& pose,
    const mapping::ProbabilityGrid& grid,
    double radius_meters) const {
  
  // 计算搜索范围（栅格单元数）
  const double resolution = grid.limits().resolution();
  const int radius_cells = static_cast<int>(radius_meters / resolution);
  
  // 获取中心栅格索引
  const Eigen::Array2i center_cell = 
      grid.limits().GetCellIndex(pose.translation().cast<float>());
  
  // 统计占用、自由和未知栅格数量
  int num_occupied = 0;
  int num_free = 0;
  int total_cells = 0;
  
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      // 检查是否在圆形范围内
      if (dx * dx + dy * dy > radius_cells * radius_cells) {
        continue;
      }
      
      Eigen::Array2i cell = center_cell + Eigen::Array2i(dx, dy);
      
      if (grid.limits().Contains(cell)) {
        if (grid.IsKnown(cell)) {
          float prob = grid.GetProbability(cell);
          if (prob > 0.55) {
            ++num_occupied;
          } else if (prob < 0.45) {
            ++num_free;
          }
          ++total_cells;
        }
      }
    }
  }
  
  if (total_cells == 0) {
    return 0.0;
  }
  
  // 计算熵
  // H = -Σ p_i * log(p_i)
  double entropy = 0.0;
  
  double p_occupied = static_cast<double>(num_occupied) / total_cells;
  double p_free = static_cast<double>(num_free) / total_cells;
  
  if (p_occupied > 0) {
    entropy -= p_occupied * std::log(p_occupied);
  }
  if (p_free > 0) {
    entropy -= p_free * std::log(p_free);
  }
  
  // 归一化到[0,1]，最大熵为log(2)
  return entropy / std::log(2.0);
}

MapCoherenceEvaluator::Result MapCoherenceEvaluator::Evaluate(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose,
    const mapping::Grid2D* grid) const {
  
  Result result;
  
  if (grid != nullptr) {
    // Dynamically cast to ProbabilityGrid for probability access
    const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(grid);
    if (prob_grid != nullptr) {
      // 计算子图匹配得分
      result.submap_match_score = ComputeSubmapMatchScore(
          point_cloud, pose, *prob_grid);
      
      // 计算地图熵
      result.map_entropy = ComputeLocalMapEntropy(pose, *prob_grid);
    } else {
      // For non-probability grids, use default values
      result.submap_match_score = 0.5;
      result.map_entropy = 0.5;
    }
    
    // 综合置信度
    // 高熵表示环境复杂，定位更可靠
    double entropy_confidence = std::tanh(result.map_entropy * 1.5);
    
    result.total_confidence = 
        0.6 * result.submap_match_score +
        0.4 * entropy_confidence;
  } else {
    // 没有地图信息，使用默认值
    result.submap_match_score = 0.5;
    result.map_entropy = 0.5;
    result.total_confidence = 0.5;
  }
  
  return result;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
