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

#ifndef PERCEPTION_2D__SENSOR__TIMED_POINT_CLOUD_DATA_HPP_
#define PERCEPTION_2D__SENSOR__TIMED_POINT_CLOUD_DATA_HPP_

#include <Eigen/Core>
#include <vector>
#include "perception_2d/common/time.hpp"
#include "perception_2d/sensor/point_cloud.hpp"
#include "perception_2d/transform/rigid_transform.hpp"
#include "perception_2d/sensor/odometry_data.hpp"

namespace perception_2d
{
namespace sensor
{

struct TimedPointCloudData
{
  common::Time time;
  // Eigen::Vector3f origin;
  transform::Rigid3d origin;
  TimedPointCloud ranges;
};

struct PointCloudDataWithIntensities
{
  TimedPointCloudData points;
  std::vector<float> intensities;
};

struct TimedPointCloudOriginData
{
  struct RangeMeasurement
  {
    TimedRangefinderPoint point_time;
    float intensity;
    size_t origin_index;
  };
  common::Time time;
  // std::vector<Eigen::Vector3f> origins;
  std::vector<transform::Rigid3d> origins;
  std::vector<RangeMeasurement> ranges;
};

}  // namespace sensor
}  // namespace perception_2d

#endif  // PERCEPTION_2D__SENSOR__TIMED_POINT_CLOUD_DATA_HPP_
