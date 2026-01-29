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
// See the License for the specific langua ge governing permissions and
// limitations under the License.

#ifndef CARTOGRAPHER__SENSOR__MAP_DATA_HPP_
#define CARTOGRAPHER__SENSOR__MAP_DATA_HPP_

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/line_data.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

namespace cartographer
{
namespace localization
{
struct MapDataInfo
{
  std::string grid_filename;
  std::string cloud_filename;
  std::string highlight_cloud_filename;
  std::string line_filename;
  double origin[3];
  int negate = 0;
  double occ_th = 0.;
  double free_th = 0.;
  double resolution;
};


typedef std::vector<Eigen::Vector2d> ReflectorMapData;
typedef std::vector<sensor::LaneDataWithNormal> LineMapData;

// struct MapData
// {
//   MapData()
//   : map_id(""),
//     map_md5(""),
//     grid_map(std::make_shared<mapping::Grid2D>()),
//     laser_line_map(std::make_shared<LineMapData>())
//     {}

//   std::string map_id, map_md5;
//   std::shared_ptr<mapping::Grid2D> grid_map;
//   std::shared_ptr<LineMapData> laser_line_map;
// };

// std::shared_ptr<mapping::Grid2D> ToLocalizationGridMap2D(
//   const GridMapData & grid_map_parser,
//   const WeightLineMapData & weight_line_map,
//   const WeightAreaMapData & weight_area_map);
// std::shared_ptr<mapping::Grid2D> ToLocalizationGridMap2D(
//   const GridMapData & grid_map_parser);

// grid_map_parsers为active map的路径候选集
// 注意最后一个元素必须为static map的路径，防止active文件被损坏或者还未被生成
// 需要检查动态地图的尺寸和静态地图是否相同
// std::shared_ptr<localization::DynamicGrid>
// ToLocalizationDynamicGridMap2D(
//   int static_map_width, int static_map_height,
//   const std::vector<GridMapData> & grid_map_parsers,
//   const WeightLineMapData & weight_line_map,
//   const WeightAreaMapData & weight_area_map);
// std::shared_ptr<mapping::Submap2D> ToMappingInitMap2D(
//   const GridMapData & grid_map_parser, const LineMapData & line_map);

// std::shared_ptr<localization::InflationMap2D> ToInflationGridMap2D(
//   const mapping::MapLimits & limits, double max_occ_dist,
//   const std::string & grid_map_parser);

}  // namespace localization

}  // namespace cartographer

#endif  // CARTOGRAPHER__SENSOR__MAP_DATA_HPP_
