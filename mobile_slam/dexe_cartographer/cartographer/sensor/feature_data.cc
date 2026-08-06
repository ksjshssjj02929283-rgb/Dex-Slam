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
#include "cartographer/sensor/feature_data.h"

namespace cartographer
{
namespace sensor
{
std::vector<sensor::LineData> SyncLaneData(
  const OdometryData & odom,
  const std::vector<sensor::LaneDataWithOdom> & lane_data)
{
  // LOG(INFO) << "get lane data:" << lane_data.size();
  std::vector<sensor::LineData> res;
  for (size_t i = 0; i < lane_data.size(); i++) {
    transform::Rigid2d delta =
      transform::Project2D(odom.pose.inverse() * lane_data[i].odom.pose);
    if (delta.translation().norm() > 0.3) {
      continue;
    }
    Eigen::Vector3d tmp_begin{lane_data[i].lane_data.data.line_data.begin[0],
      lane_data[i].lane_data.data.line_data.begin[1], 0.0};
    Eigen::Vector3d tmp_end{lane_data[i].lane_data.data.line_data.end[0],
      lane_data[i].lane_data.data.line_data.end[1], 0.0};
    Eigen::Vector3d transform_begin = lane_data[i].lane_data.pose_scan_to_tracking * tmp_begin;
    Eigen::Vector3d transform_end = lane_data[i].lane_data.pose_scan_to_tracking * tmp_end;

    sensor::LineData tmp_line_data(
      delta *
      Eigen::Vector2d{transform_begin[0], transform_begin[1]},
      delta *
      Eigen::Vector2d{transform_end[0], transform_end[1]},
      lane_data[i].lane_data.line_weight);
    // LOG(INFO) << "tmp_line_data: " << tmp_line_data.begin() << " " << tmp_line_data.end();
    res.push_back(tmp_line_data);
  }
  return res;
}

std::vector<sensor::LandmarkPointData> SyncLandmarkData(
  const OdometryData & odom,
  const std::vector<sensor::LandmarkPointDataWithOdom> & landmark_data)
{
  std::vector<sensor::LandmarkPointData> res;
  for (size_t i = 0; i < landmark_data.size(); ++i) {
    transform::Rigid2d delta =
      transform::Project2D(odom.pose.inverse() * landmark_data[i].odom.pose);
    if (delta.translation().norm() > 0.3) {
      continue;
    }

    Eigen::Vector3d tmp_landmark_data{landmark_data[i].landmark_point_data.point[0],
      landmark_data[i].landmark_point_data.point[1],
      0.0};
    Eigen::Vector3d transform_landmark_data =
      landmark_data[i].landmark_point_data.pose_scan_to_tracking * tmp_landmark_data;
    sensor::LandmarkPointData tmp_data(
      landmark_data[i].landmark_point_data.time,
      delta *
      Eigen::Vector2d{transform_landmark_data[0], transform_landmark_data[1]});
    res.push_back(tmp_data);
  }
  return res;
}
}  // namespace sensor
}  // namespace cartographer
