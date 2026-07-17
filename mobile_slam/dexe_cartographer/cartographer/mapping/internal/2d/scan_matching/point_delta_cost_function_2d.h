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

#ifndef CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__POINT_DELTA_COST_FUNCTION_2D_H_
#define CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__POINT_DELTA_COST_FUNCTION_2D_H_

#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{

ceres::CostFunction * CreatePointDeltaCostFunction2D(
  const double scaling_factor,
  const sensor::PointCloud::PointType& point_src,
  const sensor::PointCloud::PointType& point_target);

ceres::CostFunction * CreatePointLineCostFunction2D(
  const double scaling_factor,
  const sensor::PointCloud::PointType& point_src,
  const sensor::PointCloud::PointType& point_a,
  const sensor::PointCloud::PointType& point_b);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__POINT_DELTA_COST_FUNCTION_2D_H_
