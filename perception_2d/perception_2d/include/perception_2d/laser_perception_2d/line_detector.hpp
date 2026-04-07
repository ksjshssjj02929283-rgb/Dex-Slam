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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__LINE_DETECTOR_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__LINE_DETECTOR_HPP_
#include <list>
#include <utility>
#include <vector>

#include "perception_2d/laser_perception_2d/point_set.hpp"
#include "perception_2d/laser_perception_2d/segment.hpp"
#include "perception_2d/laser_perception_2d/config.hpp"
#include "perception_2d/laser_perception_2d/visualization.hpp"

namespace perception_2d
{
class LineDetector
{
public:
  explicit LineDetector(const Config config);
  std::list<Segment> detectLines(
    const std::list<Point> input_points,
    std::vector<std::pair<int, std::vector<int>>> point_cluster);
  void detectSegments(const PointSet & point_set);
  void mergeSegments();
  void checkLength();
  bool compareSegments(const Segment & s1, const Segment & s2, Segment & merged_segment);
  bool checkSegmentsProximity(const Segment & s1, const Segment & s2);
  bool checkSegmentsCollinearity(const Segment & segment, const Segment & s1, const Segment & s2);
  void UpdateParams(const std::string & param_name, double & value);

private:
  std::list<Point> input_points_;
  std::list<Segment> segments_;
  // Visualization visualization_;
  double p_max_merge_spread_;
  double p_max_merge_separation_;
  double min_line_length_;
  double merge_angle_;
  double min_point_nmber_;
  double split_dist_;
};
}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__LINE_DETECTOR_HPP_
