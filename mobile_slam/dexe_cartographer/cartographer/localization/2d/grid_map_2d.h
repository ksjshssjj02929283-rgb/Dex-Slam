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

#ifndef CARTOGRAPHER__LOCALIZATION__2D__GRID_MAP_2D_HPP_
#define CARTOGRAPHER__LOCALIZATION__2D__GRID_MAP_2D_HPP_

#include <memory>
#include <vector>
#include <utility>
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer
{
namespace localization
{

class GridMap2D : public mapping::ProbabilityGrid
{
public:
  GridMap2D(
    const mapping::MapLimits & limits);
  ~GridMap2D();

  static mapping::ValueConversionTables conversion_tables_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER__LOCALIZATION__2D__GRID_MAP_2D_HPP_
