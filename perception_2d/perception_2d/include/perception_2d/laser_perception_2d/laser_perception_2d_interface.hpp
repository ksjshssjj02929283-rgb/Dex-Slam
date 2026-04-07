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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_INTERFACE_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_INTERFACE_HPP_
#include <string>
#include <vector>
#include <tuple>

#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d/sensor/sync_timed_point_cloud.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "perception_2d/laser_perception_2d/math_tool.hpp"
#include "perception_2d/laser_perception_2d/visualization.hpp"


namespace perception_2d
{

class LaserPerception2DInterface
{
public:
  LaserPerception2DInterface() {}
  virtual ~LaserPerception2DInterface() {}

  struct SensorId
  {
    enum class SensorType
    {
      RANGE = 0,
      IMU,
      ODOMETRY,
      FIXED_FRAME_POSE,
      LANDMARK,
      LOCAL_SLAM_RESULT
    };

    SensorType type;
    std::string id;

    bool operator==(const SensorId & other) const
    {
      return std::forward_as_tuple(type, id) ==
             std::forward_as_tuple(other.type, other.id);
    }

    bool operator<(const SensorId & other) const
    {
      return std::forward_as_tuple(type, id) <
             std::forward_as_tuple(other.type, other.id);
    }
  };

  LaserPerception2DInterface(const LaserPerception2DInterface &) = delete;
  LaserPerception2DInterface & operator=(const LaserPerception2DInterface &) =
  delete;
  virtual bool AddSensorData(
    const std::string & sensor_id,
    // const sensor::TimedPointCloudData& timed_point_cloud_data) = 0;
    const sensor::PointCloudDataWithIntensities & timed_point_cloud_data) = 0;
  virtual bool AddSensorData(
    const std::string & sensor_id,
    const sensor::SyncTimedPointCloudData & range_data_identity) = 0;
  virtual void AddSensorData(
    const std::string & sensor_id,
    const sensor::OdometryData & odometry_data) = 0;
  virtual bool AddSensorData(
    const std::string & sensor_id,
    const common::Time & time,
    const transform::Rigid3d & sensor_to_tracking,
    const sensor::TimedPointCloudWithIntensities& timed_point_cloud_data) = 0;
  virtual void HandleFeatureMapInfoMessage(
    const std::string & topic,
    const std::vector<FeatureInfo> & feature_map) = 0;
  virtual void UpdateTransformFromOdomToMap(const transform::Rigid3d & odom_to_map) = 0;
  virtual TargetMap LaserUpdate(
    const sensor::TimedPointCloudOriginData & point_cloud) = 0;
  // virtual std::tuple<std::vector<FeatureMsgWithOrigin>, TargetMap> LaserUpdate(
  //   const sensor::SyncPointCloudWithOdom & msg) = 0;
  // virtual std::tuple<std::vector<std::shared_ptr<FeatureMsgWithOrigin>> , TargetMap> 
  // LaserUpdate(
  //       const sensor::SyncPointCloudWithOdom & msg) = 0;

  virtual bool LaserUpdate(
    const sensor::SyncPointCloudWithOdom & msg,
    std::vector<std::shared_ptr<FeatureMsgWithOrigin>>& target_map_full,
    TargetMap& pre_model_target_map) = 0;
  virtual TargetMap GetTargetMsg() = 0;
  // virtual const std::vector<std::shared_ptr<FeatureMsgWithOrigin>>& GetLineAndRef() = 0;
  virtual bool GetLineAndRef(std::vector<std::shared_ptr<FeatureMsgWithOrigin>> & feature_map) = 0;
  virtual sensor::SyncPointCloudWithOdom GetSyncPointCloud() = 0;
  virtual void UpdateParams(const std::string & param_name, double & value) = 0;
  virtual void UpdateParams(const std::string & param_name, int & value) = 0;
  virtual void UpdateParams(const std::string & param_name, bool & value) = 0;
  // virtual bool GetRackInfo(RackInfo & rack_info) = 0;
  LinesAndTextInfo pre_info_;
  virtual void clear() = 0;

  virtual void ResetStatus() = 0;
  // virtual std::tuple<LinesAndTextInfo, LinesAndTextInfo> GetPrePointVisInfo() = 0;
};
}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_INTERFACE_HPP_
  // bool GetRackInfo(RackInfo & rack_info);
