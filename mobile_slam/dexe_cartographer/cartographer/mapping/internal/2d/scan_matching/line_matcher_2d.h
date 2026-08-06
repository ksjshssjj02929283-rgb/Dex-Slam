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

#ifndef CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_MATCHER_2D_H_
#define CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_MATCHER_2D_H_
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/proto/scan_matching/line_matcher_options.pb.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
proto::LineMatcherOptions CreateLineMatcherOptions2D(
  common::LuaParameterDictionary * parameter_dictionary);
class LineMatcher2D
{
private:
  /* data */
  const proto::LineMatcherOptions options_;

public:
  explicit LineMatcher2D(const proto::LineMatcherOptions & options)
  : options_(options) {}

  ~LineMatcher2D() {}

  Eigen::Vector2d GetCross(
    const sensor::LineData & line_1,
    const sensor::LineData & line_2);
  Eigen::Vector2d ComputeContactPoint(
    const sensor::LineData & line,
    const Eigen::Vector2d & point);
  bool GetNearestLaneLine(
    const transform::Rigid2d & pose_prediction,
    const sensor::LineData & data,
    const LineMap2DList & line_map_2d, int & local_id);
  bool GetNearestLaneLine(
    const transform::Rigid2d & pose_prediction,
    const sensor::LineData & data,
    const LineMap2DList & line_map_2d, int & local_id,
    sensor::LineData & new_data);
  void TrackLineFromMap(
    const transform::Rigid2d &pose_prediction,
    const std::vector<sensor::LineData> &line_data,
    const LineMap2DList &line_map_2d,
    std::map<int, std::vector<sensor::LineData>>& tracked_line_data);
  void TrackLineFromMap(
    const transform::Rigid2d &pose_prediction,
    const std::map<int, std::vector<sensor::LineData>>& map_line_data,
    const LineMap2DList &line_map_2d,
    std::map<int, std::vector<sensor::LineData>>& tracked_line_data);


  // bool GetNearestLaneLine(const transform::Rigid2d &pose_prediction, const
  // sensor::LineData &data,
  //                         const GlobalLineMap &global_line_map, int
  //                         &global_id);
};
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_MATCHER_2D_H_
