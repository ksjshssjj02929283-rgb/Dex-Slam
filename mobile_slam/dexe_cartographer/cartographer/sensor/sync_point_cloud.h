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

#ifndef CARTOGRAPHER__SENSOR__SYNC_POINT_CLOUD_H_
#define CARTOGRAPHER__SENSOR__SYNC_POINT_CLOUD_H_
#include <vector>

#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/normals.h"

namespace cartographer
{
namespace sensor
{
/*
struct SyncTimedRangefinderPoint
{
  Eigen::Vector3f position;
  float time;
};

inline SyncTimedRangefinderPoint FromProto(
    const proto::SyncTimedRangefinderPoint& timed_rangefinder_point_proto) {
  return {transform::ToEigen(timed_rangefinder_point_proto.position()),
          timed_rangefinder_point_proto.time()};
}

inline proto::SyncTimedRangefinderPoint ToProto(
    const SyncTimedRangefinderPoint& timed_rangefinder_point) {
  proto::SyncTimedRangefinderPoint proto;
  *proto.mutable_position() =
      transform::ToProto(timed_rangefinder_point.position);
  proto.set_time(timed_rangefinder_point.time);
  return proto;
}*/

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

proto::TimedPointCloudWithIntensities ToProto(
  const TimedPointCloudWithIntensities & timed_point_cloud_with_intensities);
TimedPointCloudWithIntensities FromProto(
  const proto::TimedPointCloudWithIntensities &
  timed_point_cloud_with_intensities);

struct SyncTimedPointCloudData
{
  common::Time time;
  transform::Rigid3d origin;
  TimedPointCloudWithIntensities point_cloud;
};
proto::SyncTimedPointCloudData ToProto(
  const SyncTimedPointCloudData & timed_point_cloud_data);
SyncTimedPointCloudData FromProto(const proto::SyncTimedPointCloudData & proto);

struct PointCloudWithOrigin
{
  Eigen::Vector3d origin;
  std::vector<Eigen::Vector3f> sync_points_base_link;
  std::vector<Eigen::Vector3f> origin_points_odom;
  std::vector<float> intensities;
};
proto::PointCloudWithOrigin ToProto(
  const PointCloudWithOrigin & point_cloud_with_origin);
PointCloudWithOrigin FromProto(const proto::PointCloudWithOrigin & proto);

struct SyncRangeData
{
  // Eigen::Vector3d origin;
  Eigen::Vector3f origin;
  PointCloud hits;
  std::vector<float> intensities;
  PointCloud misses;
  PointCloud origin_points_odom;
  // std::vector<Eigen::Vector3f> hits;
  // std::vector<Eigen::Vector3f> misses;
  // std::vector<Eigen::Vector3f> origin_points_odom;
};

proto::SyncRangeData ToProto(const SyncRangeData & range_data);
SyncRangeData FromProto(const proto::SyncRangeData & proto);

RangeData TransformSyncRangeData(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform);
RangeDataWithIntensity TransformSyncRangeDataWithIntensity(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform);
PointCloudWithIntensity GetHighLightCloud(
  const RangeDataWithIntensity & data,
  const double & intensity_threshold);
struct RangeDataWithOdom
{
  common::Time stamp;
  OdometryData odom;
  std::vector<SyncRangeData> range_datas;
};

struct RangeDataNormalWithOdom
{
  RangeDataWithOdom range_data;
  std::vector<std::vector<Eigen::Vector2f>> normals;
};

proto::RangeDataWithOdom ToProto(const RangeDataWithOdom & range_data_with_odom);
RangeDataWithOdom FromProto(const proto::RangeDataWithOdom & proto);

RangeDataNormal TransformSyncRangeDataNormal(
  const std::vector<SyncRangeData> & sync_range_data,
  const std::vector<Normals> & normals, const transform::Rigid3f & transform);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER__SENSOR__SYNC_POINT_CLOUD_H_
