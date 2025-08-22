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
#include <vector>
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer
{
namespace sensor
{

// sensor::RangeData ToRangeData(SyncRangeData& sync_range_data) {

// }

RangeData TransformSyncRangeData(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform)
{
  RangeData reset;
  reset.origin = transform::Project2D(transform).ToVector();
  for (int i = 0; i < static_cast<int>(sync_range_data.size()); i++) {
    const auto & data = sync_range_data[i];
    PointCloud hits = TransformPointCloud(data.hits, transform);
    PointCloud misses = TransformPointCloud(data.misses, transform);
    reset.returns.InsertCloud(hits);
    reset.misses.InsertCloud(misses);
  }
  return reset;
}

RangeDataWithIntensity TransformSyncRangeDataWithIntensity(
  const std::vector<SyncRangeData> & sync_range_data,
  const transform::Rigid3f & transform)
{
  RangeDataWithIntensity res;
  res.range_data = TransformSyncRangeData(sync_range_data, transform);
  for (size_t i = 0; i < sync_range_data.size(); i++) {
    res.intensitys.insert(
      res.intensitys.end(), sync_range_data[i].intensities.begin(),
      sync_range_data[i].intensities.end());
  }
  return res;
}

PointCloudWithIntensity GetHighLightCloud(
  const RangeDataWithIntensity & data,
  const double & intensity_threshold)
{
  PointCloudWithIntensity res;
  if (intensity_threshold < 0) {
    return res;
  }
  if (data.range_data.returns.size() != data.intensitys.size()) {
    LOG(WARNING) << "data.range_data.returns.size() != data.intensitys.size()";
    return res;
  }
  for (size_t i = 0; i < data.intensitys.size(); i++) {
    if (data.intensitys[i] > intensity_threshold) {
      res.push_back({data.range_data.returns[i].position, data.intensitys[i]});
    }
  }
  return res;
}

proto::TimedPointCloudWithIntensities ToProto(
  const TimedPointCloudWithIntensities & timed_point_cloud_with_intensities)
{
  proto::TimedPointCloudWithIntensities proto;
  for (const auto & point : timed_point_cloud_with_intensities.points) {
    *proto.add_points() = ToProto(point);
  }

  for (const float intensity : timed_point_cloud_with_intensities.intensities) {
    proto.add_intensities(intensity);
  }
  return proto;
}

TimedPointCloudWithIntensities FromProto(
  proto::TimedPointCloudWithIntensities & proto)
{
  TimedPointCloudWithIntensities timed_point_cloud_with_intensities;
  timed_point_cloud_with_intensities.points.reserve(proto.points_size());
  for (const auto & point_proto : proto.points()) {
    timed_point_cloud_with_intensities.points.push_back(FromProto(point_proto));
  }
  timed_point_cloud_with_intensities.intensities.reserve(proto.intensities_size());
  for (const auto & intensity : proto.intensities()) {
    timed_point_cloud_with_intensities.intensities.push_back(intensity);
  }
  return timed_point_cloud_with_intensities;
}
// struct TimedPointCloudData
// {
//   common::Time time;
//   Eigen::Vector3d origin;
//   TimedPointCloudWithIntensities point_cloud;
// };
proto::SyncTimedPointCloudData ToProto(
  const SyncTimedPointCloudData & timed_point_cloud_data)
{
  proto::SyncTimedPointCloudData proto;
  proto.set_time(common::ToUniversal(timed_point_cloud_data.time));
  *proto.mutable_origin() = transform::ToProto(timed_point_cloud_data.origin);
  *proto.mutable_point_cloud() = ToProto(timed_point_cloud_data.point_cloud);
  return proto;
}
SyncTimedPointCloudData FromProto(
  const proto::SyncTimedPointCloudData & proto)
{
  SyncTimedPointCloudData timed_point_cloud_data;
  // timed_point_cloud_data.time = common::FromUniversal(proto.time());
  // timed_point_cloud_data.origin = transform::ToEigen(proto.origin());
  // timed_point_cloud_data.point_cloud = FromProto(proto.point_cloud());
  return timed_point_cloud_data;
}
/*struct PointCloudWithOrigin
{
  Eigen::Vector3d origin;
  std::vector<Eigen::Vector3f> sync_points_base_link;
  std::vector<Eigen::Vector3f> origin_points_odom;
  std::vector<float> intensities;
};*/
proto::PointCloudWithOrigin ToProto(
  const PointCloudWithOrigin & point_cloud_with_origin)
{
  proto::PointCloudWithOrigin proto;
  *proto.mutable_origin() = transform::ToProto(point_cloud_with_origin.origin);
  for (const auto point : point_cloud_with_origin.sync_points_base_link) {
    *proto.add_sync_points_base_link() = transform::ToProto(point);
  }
  for (const auto point : point_cloud_with_origin.origin_points_odom) {
    *proto.add_origin_points_odom() = transform::ToProto(point);
  }
  for (const auto intensity : point_cloud_with_origin.intensities) {
    proto.add_intensities(intensity);
  }
  return proto;
}

PointCloudWithOrigin FromProto(
  const proto::PointCloudWithOrigin & proto)
{
  PointCloudWithOrigin point_cloud_with_origin;
  point_cloud_with_origin.origin = transform::ToEigen(proto.origin());
  for (const auto & point : proto.sync_points_base_link()) {
    point_cloud_with_origin.sync_points_base_link.push_back(transform::ToEigen(point));
  }
  for (const auto & point : proto.origin_points_odom()) {
    point_cloud_with_origin.origin_points_odom.push_back(transform::ToEigen(point));
  }
  for (const auto & intensity : proto.intensities()) {
    point_cloud_with_origin.intensities.push_back(intensity);
  }
  return point_cloud_with_origin;
}

/*struct SyncPointCloudWithOdom
{
  common::Time stamp;
  Odometry odom;
  std::vector<PointCloudWithOrigin> sync_point_clouds;
};*/
proto::SyncPointCloudWithOdom ToProto(
  const SyncPointCloudWithOdom & sync_point_cloud_with_odom)
{
  proto::SyncPointCloudWithOdom proto;
  proto.set_stamp(common::ToUniversal(sync_point_cloud_with_odom.stamp));
  *proto.mutable_odom() = ToProto(sync_point_cloud_with_odom.odom);
  for (const auto & point_cloud_with_origin : sync_point_cloud_with_odom.sync_point_clouds) {
    *proto.add_sync_point_clouds() = ToProto(point_cloud_with_origin);
  }
  return proto;
}
SyncPointCloudWithOdom FromProto(
  const proto::SyncPointCloudWithOdom & proto)
{
  SyncPointCloudWithOdom sync_point_cloud_with_odom;
  sync_point_cloud_with_odom.stamp = common::FromUniversal(proto.stamp());
  sync_point_cloud_with_odom.odom = FromProto(proto.odom());
  for (const auto & sync_point_cloud : proto.sync_point_clouds()) {
    sync_point_cloud_with_odom.sync_point_clouds.push_back(FromProto(sync_point_cloud));
  }
  return sync_point_cloud_with_odom;
}
/*struct RangeData
{
  Eigen::Vector3d origin;
  std::vector<Eigen::Vector3f> hits;
  std::vector<Eigen::Vector3f> misses;
  std::vector<Eigen::Vector3f> origin_points_odom;
};*/
proto::SyncRangeData ToProto(
  const SyncRangeData & range_data)
{
  proto::SyncRangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  for (const auto & hit : range_data.hits) {
    Eigen::Vector3f point = hit.position;
    *proto.add_hits() = transform::ToProto(point);
  }
  for (const auto & miss : range_data.misses) {
    Eigen::Vector3f point = miss.position;
    *proto.add_misses() = transform::ToProto(point);
  }
  for (const auto & origin_point : range_data.origin_points_odom) {
    Eigen::Vector3f point = origin_point.position;
    *proto.add_origin_points_odom() = transform::ToProto(point);
  }
  return proto;
}
SyncRangeData FromProto(
  const proto::SyncRangeData & proto)
{
  SyncRangeData range_data;
  range_data.origin = transform::ToEigen(proto.origin());
  for (const auto & hit : proto.hits()) {
    Eigen::Vector3f position = transform::ToEigen(hit);
    RangefinderPoint point{position};
    range_data.hits.push_back(point);
  }
  for (const auto & miss : proto.misses()) {
    Eigen::Vector3f position = transform::ToEigen(miss);
    RangefinderPoint point{position};
    range_data.misses.push_back(point);
    // range_data.misses.push_back(transform::ToEigen(miss));
  }
  for (const auto & origin_point : proto.origin_points_odom()) {
    Eigen::Vector3f position = transform::ToEigen(origin_point);
    RangefinderPoint point{position};
    range_data.origin_points_odom.push_back(point);
    // range_data.origin_points_odom.push_back(transform::ToEigen(origin_point));
  }
  return range_data;
}
/*struct RangeDataWithOdom
{
  common::Time stamp;
  Odometry odom;
  std::vector<RangeData> range_datas;
};*/
proto::RangeDataWithOdom ToProto(
  const RangeDataWithOdom & range_data_with_odom)
{
  proto::RangeDataWithOdom proto;
  proto.set_stamp(common::ToUniversal(range_data_with_odom.stamp));
  *proto.mutable_odom() = ToProto(range_data_with_odom.odom);
  for (const auto & range_data : range_data_with_odom.range_datas) {
    *proto.add_range_datas() = ToProto(range_data);
  }
  return proto;
}
RangeDataWithOdom FromProto(
  const proto::RangeDataWithOdom & proto)
{
  RangeDataWithOdom range_data_with_odom;
  range_data_with_odom.stamp = common::FromUniversal(proto.stamp());
  range_data_with_odom.odom = FromProto(proto.odom());
  for (const auto & range_data : proto.range_datas()) {
    range_data_with_odom.range_datas.push_back(FromProto(range_data));
  }
  return range_data_with_odom;
}


RangeDataNormal TransformSyncRangeDataNormal(
  const std::vector<SyncRangeData> & sync_range_data,
  const std::vector<Normals> & normals,
  const transform::Rigid3f & transform)
{
  CHECK_EQ(sync_range_data.size(), normals.size());
  RangeDataNormal reset;
  reset.range_data.origin = transform::Project2D(transform).ToVector();
  for (int i = 0; i < static_cast<int>(sync_range_data.size()); i++) {
    CHECK_EQ(sync_range_data[i].hits.size(), normals[i].size());
    PointCloud hits = TransformPointCloud(sync_range_data[i].hits, transform);
    PointCloud misses = TransformPointCloud(sync_range_data[i].misses, transform);
    reset.range_data.returns.InsertCloud(hits);
    reset.range_data.misses.InsertCloud(misses);
    Normals normals_temp = TransformNormals(normals[i], transform);
    reset.normals.insert(
      reset.normals.end(),
      normals_temp.begin(), normals_temp.end());
  }
  return reset;
}

}  // namespace sensor
}  // namespace cartographer
