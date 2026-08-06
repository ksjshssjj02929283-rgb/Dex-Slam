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

#ifndef CARTOGRAPHER__SENSOR__LANDMARK_DATA_H_
#define CARTOGRAPHER__SENSOR__LANDMARK_DATA_H_
#include <vector>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer
{
namespace sensor
{

struct LandmarkObservation
{
  std::string id;
  transform::Rigid3d landmark_to_tracking_transform;
  double translation_weight;
  double rotation_weight;
};

struct LandmarkData
{
  common::Time time;
  std::vector<LandmarkObservation> landmark_observations;
};

struct LandmarkPointData
{
  LandmarkPointData(
    const common::Time & t,
    const Eigen::Vector2d & p)
  : time(t), pose_scan_to_tracking(transform::Rigid3d()), point(p) {}
  LandmarkPointData()
  {
    pose_scan_to_tracking = transform::Rigid3d();
  }
  LandmarkPointData(
    const common::Time & t,
    const transform::Rigid3d & pose,
    const Eigen::Vector2d & p)
  : time(t), pose_scan_to_tracking(pose), point(p) {}

  common::Time time;
  transform::Rigid3d pose_scan_to_tracking;
  Eigen::Vector2d point;
};

void TransformLandmarkPointData(
  const std::vector<LandmarkPointData> & data,
  const transform::Rigid2d & transform,
  std::vector<LandmarkPointData> & res);

struct LandmarkPointDataWithOdom
{
  OdometryData odom;
  LandmarkPointData landmark_point_data;
};

struct AngleStructData
{
  common::Time time;
  Eigen::Vector3d point;
  double theta;
};

struct ReflectorPointData
{
  common::Time time;
  transform::Rigid3d pose_scan_to_tracking;
  std::vector<cartographer::sensor::LandmarkPointData> data;
};

void ReflectorsClustering(
  const std::vector<Eigen::Vector2d> & reflector_data,
  std::vector<Eigen::Vector2d> & res);

// Converts 'landmark_data' to a proto::LandmarkData.
proto::LandmarkData ToProto(const LandmarkData & landmark_data);

// Converts 'proto' to an LandmarkData.
LandmarkData FromProto(const proto::LandmarkData & proto);

proto::LandmarkPointData ToProto(const LandmarkPointData & landmark_data);
LandmarkPointData FromProto(const proto::LandmarkPointData & proto);


proto::AngleStructData ToProto(const AngleStructData & angle_struct_data);
AngleStructData FromProto(const proto::AngleStructData & proto);
}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER__SENSOR__LANDMARK_DATA_H_
