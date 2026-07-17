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

#include "cartographer/sensor/landmark_data.h"

#include <vector>
#include "cartographer/transform/transform.h"

namespace cartographer
{
namespace sensor
{

proto::LandmarkData ToProto(const LandmarkData & landmark_data)
{
  proto::LandmarkData proto;
  proto.set_timestamp(common::ToUniversal(landmark_data.time));
  for (const auto & observation : landmark_data.landmark_observations) {
    auto * item = proto.add_landmark_observations();
    item->set_id(observation.id);
    *item->mutable_landmark_to_tracking_transform() =
      transform::ToProto(observation.landmark_to_tracking_transform);
    item->set_translation_weight(observation.translation_weight);
    item->set_rotation_weight(observation.rotation_weight);
  }
  return proto;
}

LandmarkData FromProto(const proto::LandmarkData & proto)
{
  LandmarkData landmark_data;
  landmark_data.time = common::FromUniversal(proto.timestamp());
  for (const auto & item : proto.landmark_observations()) {
    landmark_data.landmark_observations.push_back(
              {
                item.id(),
                transform::ToRigid3(item.landmark_to_tracking_transform()),
                item.translation_weight(),
                item.rotation_weight(),
              });
  }
  return landmark_data;
}


proto::LandmarkPointData ToProto(const LandmarkPointData & landmark_data)
{
  proto::LandmarkPointData proto;
  proto.set_timestamp(common::ToUniversal(landmark_data.time));
  *proto.mutable_point() = transform::ToProto(landmark_data.point);
  return proto;
}

LandmarkPointData FromProto(const proto::LandmarkPointData & proto)
{
  LandmarkPointData landmark_data;
  landmark_data.time = common::FromUniversal(proto.timestamp());
  landmark_data.point = transform::ToEigen(proto.point());
  return landmark_data;
}

proto::AngleStructData ToProto(const AngleStructData & angle_struct_data)
{
  proto::AngleStructData proto;
  proto.set_timestamp(common::ToUniversal(angle_struct_data.time));
  *proto.mutable_point() = transform::ToProto(angle_struct_data.point);
  proto.set_theta(angle_struct_data.theta);
  return proto;
}

AngleStructData FromProto(const proto::AngleStructData & proto)
{
  AngleStructData angle_struct_data;
  angle_struct_data.time = common::FromUniversal(proto.timestamp());
  angle_struct_data.point = transform::ToEigen(proto.point());
  angle_struct_data.theta = proto.theta();
  return angle_struct_data;
}

void TransformLandmarkPointData(
  const std::vector<LandmarkPointData> & data,
  const transform::Rigid2d & transform,
  std::vector<LandmarkPointData> & res)
{
  res.clear();
  if (data.size() == 0) {
    return;
  }
  for (size_t i = 0; i < data.size(); i++) {
    Eigen::Vector2d point = transform * data[i].point;
    res.push_back(
      LandmarkPointData(
        data[i].time, data[i].pose_scan_to_tracking, point));
  }
}

struct ClusterCenter
{
  Eigen::Vector2d center;
  std::vector<Eigen::Vector2d> member;
};


void ReflectorsClustering(
  const std::vector<Eigen::Vector2d> & reflector_data,
  std::vector<Eigen::Vector2d> & res)
{
  std::vector<ClusterCenter> cluster_centers;
  for (size_t i = 0; i < reflector_data.size(); i++) {
    double min_dis = -1;
    size_t center_id = 0;
    for (size_t j = 0; j < cluster_centers.size(); j++) {
      double dis = (reflector_data[i] - cluster_centers[j].center).norm();
      if (min_dis < 0 || dis < min_dis) {
        center_id = j;
        min_dis = dis;
      }
    }
    if (min_dis < 0.5 && min_dis >= 0) {
      cluster_centers[center_id].center =
        (cluster_centers[center_id].center * cluster_centers[center_id].member.size() +
        reflector_data[i]) / (cluster_centers[center_id].member.size() + 1);
      cluster_centers[center_id].member.push_back(reflector_data[i]);
    } else {
      cluster_centers.push_back(
        {reflector_data[i], {reflector_data[i]}});
    }
  }

  // 清除外点
  for (size_t i = 0; i < cluster_centers.size(); i++) {
    auto & cluster_center = cluster_centers[i];
    if (cluster_center.member.size() < 3) {
      continue;
    }
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    int num = 0;
    for (size_t j = 0; j < cluster_center.member.size(); j++) {
      if ((cluster_center.member[j] - cluster_center.center).norm() < 0.2) {
        center += cluster_center.member[j];
        num++;
      }
    }
    if (num > 0) {
      res.push_back(center / num);
    }
  }
}
}  // namespace sensor
}  // namespace cartographer
