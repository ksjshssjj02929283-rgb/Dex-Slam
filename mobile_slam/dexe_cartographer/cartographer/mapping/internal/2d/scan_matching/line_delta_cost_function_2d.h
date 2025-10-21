// Copyright (c) 2021 Geekplus Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_
#define CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_

#include "cartographer/mapping/2d/line_map_2d.h"
#include "ceres/ceres.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
struct Line
{
  Eigen::Vector2d begin;
  Eigen::Vector2d end;
  double angle() const
  {
    return atan2(end[1] - begin[1], end[0] - begin[0]);
  }
};

ceres::CostFunction * CreateLineDeltaCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line);
ceres::CostFunction * CreateLineDeltaWithAngleCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line,
  const transform::Rigid2d & initial_pose);
ceres::CostFunction * CreateLineDeltaWithoutInitPoseCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line);

// ============================================================================
// 方案B: 改进版本函数声明
// ============================================================================

// 改进版本1: 对称性距离 + 双端点 + 中点 + 自适应权重
// 返回2个残差: [距离误差, 角度误差]
ceres::CostFunction* CreateImprovedLineDeltaCostFunction2D(
  const double parallel_scaling_factor, 
  const double angle_scaling_factor,
  const sensor::LineData& line_data, 
  const sensor::LineData& target_line);

// 改进版本2: 添加初始位姿约束 + 对称性距离
// 返回4个残差: [距离误差, 角度误差, 平移先验误差, 旋转先验误差]
ceres::CostFunction* CreateImprovedLineDeltaWithInitPoseCostFunction2D(
  const double parallel_scaling_factor, 
  const double angle_scaling_factor,
  const double pose_prior_weight,
  const sensor::LineData& line_data, 
  const sensor::LineData& target_line,
  const transform::Rigid2d& initial_pose);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_
