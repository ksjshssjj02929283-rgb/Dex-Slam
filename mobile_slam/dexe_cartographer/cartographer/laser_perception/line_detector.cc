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
#include "cartographer/laser_perception/line_detector.h"
#include <list>
#include <utility>
#include <vector>
#include "cartographer/laser_perception/figure_fitting.h"
#include "cartographer/laser_perception/math_tool.h"

namespace cartographer
{
namespace laser_perception
{
LineDetector::LineDetector(const proto::LaserPerceptionOptions & options)
{
  p_max_merge_separation_ = 0.05;
  p_max_merge_spread_ = 0.05;
  min_line_length_ = options.min_line_length();
  merge_angle_ = 0.0005;
  min_point_nmber_ = 5;
  split_dist_ = 0.02;
}

std::list<Segment> LineDetector::detectLines(
  const std::list<Point> & input_points,
  const std::vector<std::pair<int, std::vector<int>>> & point_cluster)
{
  input_points_.clear();
  segments_.clear();
  input_points_ = input_points;
  PointSet point_set;
  PointIterator point = input_points_.begin();
  for (const auto & item : point_cluster) {
    point_set.begin = point;
    for (size_t i = 0; i < item.second.size(); i++) {
      point_set.end = point;
      point++;
    }
    point_set.num_points = item.second.size();
    detectSegments(point_set);
  }
  mergeSegments();
  checkLength();
  LinesAndTextInfo lines_text_info;
  lines_text_info.frame = "scan";
  lines_text_info.color = blue;
  for (auto segment : segments_) {
    cv::Point2d start_pose(segment.first_point.x, segment.first_point.y);
    cv::Point2d end_pose(segment.last_point.x, segment.last_point.y);
    lines_text_info.lines.push_back(std::make_pair(start_pose, end_pose));
  }
  visualization_.linesAndTextVisual(lines_text_info);

  return segments_;
}

void LineDetector::detectSegments(const PointSet & point_set)
{
  Segment segment(*point_set.begin, *point_set.end);  // Use Iterative End Point Fit

  // if (p_use_split_and_merge_)
  //   segment = fitSegment(point_set);

  PointIterator set_divider;
  double max_distance = 0.0;
  double distance = 0.0;

  int split_index = 0;  // Natural index of splitting point (counting from 1)
  int point_index = 0;  // Natural index of current point in the set

  // Seek the point of division
  for (PointIterator point = point_set.begin; point != point_set.end; ++point) {
    ++point_index;
    if ((distance = segment.distanceTo(*point)) >= max_distance) {
      max_distance = distance;
      set_divider = point;
      split_index = point_index;
    }
  }

  // Split the set only if the sub-groups are not 'small'
  // printf("max distance:%f\n", max_distance);
  if (max_distance > split_dist_ && split_index > 2 && split_index < point_set.num_points - 2) {
    // Clone the dividing point for each group
    set_divider = input_points_.insert(set_divider, *set_divider);

    PointSet subset1, subset2;
    subset1.begin = point_set.begin;
    subset1.end = set_divider;
    subset1.num_points = split_index;
    subset1.is_visible = point_set.is_visible;

    subset2.begin = ++set_divider;
    subset2.end = point_set.end;
    subset2.num_points = point_set.num_points - split_index;
    subset2.is_visible = point_set.is_visible;

    detectSegments(subset1);
    detectSegments(subset2);
  } else if (point_set.num_points > min_point_nmber_) {  // Add the segment
    segment = fitSegment(point_set);
    segments_.push_back(segment);
  }
}

void LineDetector::mergeSegments()
{
  for (auto i = segments_.begin(); i != segments_.end(); ++i) {
    for (auto j = i; j != segments_.end(); ++j) {
      Segment merged_segment;

      if (compareSegments(*i, *j, merged_segment)) {
        auto temp_itr = segments_.insert(i, merged_segment);
        segments_.erase(i);
        segments_.erase(j);
        i = --temp_itr;  // Check the new segment against others
        break;
      }
    }
  }
}

void LineDetector::checkLength()
{
  for (auto i = segments_.begin(); i != segments_.end(); ) {
    if (i->length() < min_line_length_) {
      i = segments_.erase(i);
    } else {
      i++;
    }
  }
}

bool LineDetector::compareSegments(const Segment & s1, const Segment & s2, Segment & merged_segment)
{
  if (&s1 == &s2) {
    return false;
  }

  // Segments must be provided counter-clockwise
  if (s1.first_point.cross(s2.first_point) < 0.0) {
    return compareSegments(s2, s1, merged_segment);
  }

  if (checkSegmentsProximity(s1, s2)) {
    std::vector<PointSet> point_sets;
    point_sets.insert(point_sets.end(), s1.point_sets.begin(), s1.point_sets.end());
    point_sets.insert(point_sets.end(), s2.point_sets.begin(), s2.point_sets.end());

    Segment segment = fitSegment(point_sets);

    if (checkSegmentsCollinearity(segment, s1, s2)) {
      merged_segment = segment;
      return true;
    }
  }

  return false;
}

bool LineDetector::checkSegmentsProximity(const Segment & s1, const Segment & s2)
{
  Point s1_vector = s1.last_point - s1.first_point;
  Point s2_vector = s2.last_point - s2.first_point;
  double angle_error = fabs(s1_vector.angle() - s2_vector.angle());
  return (angle_error < merge_angle_) &&
         (s1.trueDistanceTo(s2.first_point) < p_max_merge_separation_ ||
         s1.trueDistanceTo(s2.last_point) < p_max_merge_separation_ ||
         s2.trueDistanceTo(s1.first_point) < p_max_merge_separation_ ||
         s2.trueDistanceTo(s1.last_point) < p_max_merge_separation_);
}

bool LineDetector::checkSegmentsCollinearity(
  const Segment & segment, const Segment & s1,
  const Segment & s2)
{
  return segment.distanceTo(s1.first_point) < p_max_merge_spread_ &&
         segment.distanceTo(s1.last_point) < p_max_merge_spread_ &&
         segment.distanceTo(s2.first_point) < p_max_merge_spread_ &&
         segment.distanceTo(s2.last_point) < p_max_merge_spread_;
}

}  // namespace laser_perception
}  // cartographer
