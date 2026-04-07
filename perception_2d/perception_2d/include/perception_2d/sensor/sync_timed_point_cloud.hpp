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
#ifndef PERCEPTION_2D__SENSOR__SYNC_TIMED_POINT_CLOUD_HPP_
#define PERCEPTION_2D__SENSOR__SYNC_TIMED_POINT_CLOUD_HPP_
#include <vector>
#include <string>
#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/sensor/rangefinder_point.hpp"
#include "perception_2d/sensor/point_cloud.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d/sensor/range_data.hpp"
namespace perception_2d
{
namespace sensor
{
using TimedPointCloud = std::vector<TimedRangefinderPoint>;
struct TimedPointCloudWithIntensities
{
  TimedPointCloud points;
  std::vector<float> intensities;
};

template<class T>
inline TimedPointCloudWithIntensities operator*(
  const transform::Rigid3<T> & lhs,
  const TimedPointCloudWithIntensities & point_cloud)
{
  TimedPointCloudWithIntensities transformed_point_cloud;
  for (const auto & point : point_cloud.points) {
    // std::cout<<(lhs * point).position<<std::endl;
    transformed_point_cloud.points.push_back(lhs * point);
  }
  for (const auto & intensity : point_cloud.intensities) {
    transformed_point_cloud.intensities.push_back(intensity);
  }
  return transformed_point_cloud;
}

struct SyncTimedPointCloudData
{
  common::Time time;
  // Eigen::Vector3d origin;
  transform::Rigid3d origin;
  TimedPointCloudWithIntensities point_cloud;
};

struct PointCloudWithOrigin
{
  std::string frame_id;
  // Eigen::Vector3d origin;
  transform::Rigid3d origin;
  std::vector<Eigen::Vector3f> sync_points_base_link;
  std::vector<Eigen::Vector3f> origin_points_odom;
  std::vector<float> intensities;
};

struct SyncPointCloudWithOdom
{
  common::Time stamp;
  OdometryData odom;
  std::vector<PointCloudWithOrigin> sync_point_clouds;
};

struct SyncRangeData
{
  // Eigen::Vector3d origin;
  Eigen::Vector3f origin;
  PointCloud hits;
  PointCloud misses;
  PointCloud origin_points_odom;
  // std::vector<Eigen::Vector3f> hits;
  // std::vector<Eigen::Vector3f> misses;
  // std::vector<Eigen::Vector3f> origin_points_odom;
};

RangeData TransformSyncRangeData(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform);
struct RangeDataWithOdom
{
  common::Time stamp;
  OdometryData odom;
  std::vector<SyncRangeData> range_datas;
};

struct  RangeDataNormalWithOdom
{
  RangeDataWithOdom range_data;
  std::vector<std::vector<Eigen::Vector2f>> normals;
};

RangeDataNormal TransformSyncRangeDataNormal(
  const std::vector<SyncRangeData> & sync_range_data,
  const std::vector<Normals> & normals,
  const transform::Rigid3f & transform);


}  // namespace sensor
}  // namespace perception_2d
#endif  // PERCEPTION_2D__SENSOR__SYNC_TIMED_POINT_CLOUD_HPP_
