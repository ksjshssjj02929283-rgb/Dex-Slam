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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_H_
#define CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_H_
#include <list>
#include <utility>
#include <vector>

#include "cartographer/laser_perception/point_set.h"
#include "cartographer/laser_perception/segment.h"
#include "cartographer/laser_perception/config.h"
#include "cartographer/laser_perception/visualization.h"
#include "cartographer/laser_perception/proto/laser_perception_options.pb.h"

namespace cartographer
{
namespace laser_perception
{
class LineDetector
{
public:
  explicit LineDetector(const proto::LaserPerceptionOptions & options);
  std::list<Segment> detectLines(
    const std::list<Point> & input_points,
    const std::vector<std::pair<int, std::vector<int>>> & point_cluster);
  void detectSegments(const PointSet & point_set);
  void mergeSegments();
  void checkLength();
  bool compareSegments(const Segment & s1, const Segment & s2, Segment & merged_segment);
  bool checkSegmentsProximity(const Segment & s1, const Segment & s2);
  bool checkSegmentsCollinearity(const Segment & segment, const Segment & s1, const Segment & s2);

private:
  std::list<Point> input_points_;
  std::list<Segment> segments_;
  Visualization visualization_;
  double p_max_merge_spread_;
  double p_max_merge_separation_;
  double min_line_length_;
  double merge_angle_;
  double min_point_nmber_;
  double split_dist_;
};

}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__LINE_DETECTOR_H_
