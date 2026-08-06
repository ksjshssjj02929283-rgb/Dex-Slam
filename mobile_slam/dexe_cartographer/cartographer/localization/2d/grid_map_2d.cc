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

#include "cartographer/localization/2d/grid_map_2d.h"

#include <memory>
#include <vector>
#include <utility>
#include <algorithm>

#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"

namespace cartographer
{
namespace localization
{

mapping::ValueConversionTables GridMap2D::conversion_tables_ = mapping::ValueConversionTables();

GridMap2D::GridMap2D(
  const mapping::MapLimits & limits)
: mapping::ProbabilityGrid(limits, &conversion_tables_)
{
}

GridMap2D::GridMap2D(const mapping::ProbabilityGrid& source_grid)
: mapping::ProbabilityGrid(source_grid.ToProto(), &conversion_tables_)
{
  // 通过Proto方式复制，保证完整复制所有数据（包括protected成员）
}

GridMap2D::GridMap2D(
  const mapping::proto::Grid2D& proto,
  mapping::ValueConversionTables* conversion_tables)
: mapping::ProbabilityGrid(proto, conversion_tables)
{
}

GridMap2D::~GridMap2D()
{
}


}  // namespace localization
}  // namespace cartographer
