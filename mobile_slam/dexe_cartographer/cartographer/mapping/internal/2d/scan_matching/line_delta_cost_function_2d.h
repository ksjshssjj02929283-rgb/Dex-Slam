// Copyright (c) 2026 Dexforce Inc
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
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_
