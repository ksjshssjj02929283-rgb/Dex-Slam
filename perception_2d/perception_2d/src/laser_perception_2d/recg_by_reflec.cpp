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
#include <vector>
#include <utility>
#include "perception_2d/laser_perception_2d/math_tool.hpp"
#include "perception_2d/laser_perception_2d/rec_by_reflec.hpp"

namespace perception_2d
{

RecgByReflec::RecgByReflec(const Config config)
{
  reflector_error_ = config.reflector_error;
  reflector_radius_ = config.reflector_radius;
}

void RecgByReflec::UpdateParams(
  const std::string & param_name, double & value)
{
  if (param_name == "reflector_error") {
    reflector_error_ = value;
  } else if (param_name == "reflector_radius") {
    reflector_radius_ = value;
  }
}

std::vector<FeatureMsg> RecgByReflec::getReflector(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<FeatureMsg> target_map;

  for (auto points : laserPointSet) {
    if (points.first != 2) {
      continue;
    }
    double length_model = 2 * reflector_radius_;

    double length_scan = chordLength_point(
      msg.sync_points_base_link[points.second.front()],
      msg.sync_points_base_link[points.second.back()]);
    double length_error = length_model - length_scan;
    if (fabs(length_error) < reflector_error_) {
      double x = msg.sync_points_base_link[points.second[points.second.size() / 2]][0];
      double y = msg.sync_points_base_link[points.second[points.second.size() / 2]][1];
      double center_x = x + reflector_radius_ * (x / sqrt(x * x + y * y));
      double center_y = y + reflector_radius_ * (y / sqrt(x * x + y * y));
      FeatureMsg feature;
      feature.id = 0;
      feature.target_name = "reflector";
      feature.x = center_x;
      feature.y = center_y;
      feature.theta = 0;
      target_map.push_back(feature);
    } else {
      // LOG(INFO) << "Too large length error length_model: " <<
      //   length_model << " length_scan: " << length_scan;
    }
  }
  return target_map;
}

std::vector<FeatureMsg> RecgByReflec::getReflector(
  const sensor::TimedPointCloudOriginData & timed_point_cloud_data,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<FeatureMsg> target_map;

  PointsAndTextInfo points_visual;
  points_visual.frame = "base_link";
  points_visual.color = red;
  points_visual.text = "hear";

  for (auto points : laserPointSet) {
    if (points.first != 2) {
      continue;
    }
    double length_model = 2 * reflector_radius_;

    double length_scan = chordLength_point(
      timed_point_cloud_data.ranges[points.second.front()].point_time.position,
      timed_point_cloud_data.ranges[points.second.back()].point_time.position);
    double length_error = length_model - length_scan;
    if (fabs(length_error) < reflector_error_) {
      double x =
        timed_point_cloud_data.ranges[points.second[points.second.size() /
          2]].point_time.position[0];
      double y =
        timed_point_cloud_data.ranges[points.second[points.second.size() /
          2]].point_time.position[1];
      double center_x = x + reflector_radius_ * (x / sqrt(x * x + y * y));
      double center_y = y + reflector_radius_ * (y / sqrt(x * x + y * y));
      FeatureMsg feature;
      feature.id = 0;
      feature.target_name = "reflector";
      feature.x = center_x;
      feature.y = center_y;
      feature.theta = 0;
      target_map.push_back(feature);
    } else {
      // printf("Too large length error length_model:%f\tlength_scan:%f\n",
      // length_model, length_scan);
      // LOG(INFO) << "Too large length error length_model: "
      // << length_model << " length_scan: " << length_scan;
    }
  }
  return target_map;
}

}  // namespace perception_2d
