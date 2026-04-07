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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__FEATURE_MAP_BRIDGE_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__FEATURE_MAP_BRIDGE_HPP_
#include <nav_interfaces/msg/target_info.hpp>
#include <memory>
#include <string>

#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/transform/rigid_transform.hpp"
#include "perception_2d/transform/transform.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d_ros2/tf_bridge.hpp"
#include "perception_2d/laser_perception_2d/laser_perception_2d_interface.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "perception_2d/laser_perception_2d/visualization.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


namespace perception_ros2
{

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class FeatureMapBridge
{
public:
  explicit FeatureMapBridge(
    const std::string & tracking_frame,
    double lookup_transform_timeout_sec,
    tf2_ros::Buffer * tf_buffer,
    perception_2d::LaserPerception2DInterface * laser_perception_2d);

  FeatureMapBridge(const FeatureMapBridge &) = delete;
  FeatureMapBridge & operator=(const FeatureMapBridge &) = delete;

  void UpdateFeatureMap(
    const std::string & topic,
    const nav_interfaces::msg::TargetInfo::ConstSharedPtr & msg);

  void UpdateTransformFromOdomToMap(
    const perception_2d::transform::Rigid3d & odom_to_map);

  perception_2d::TargetMap GetTargetPose();
  // const std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>>& GetLineAndRef();
  bool GetLineAndRef(std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> & feature_map);
  perception_2d::sensor::SyncPointCloudWithOdom GetSyncPointCloud();
//   std::tuple<perception_2d::LinesAndTextInfo, perception_2d::LinesAndTextInfo>
//             GetPrePointVisInfo();
//   const TfBridge& tf_bridge() const;

private:
  const TfBridge tf_bridge_;
  ::perception_2d::LaserPerception2DInterface * const
  laser_perception_2d_;
};

}  // namespace perception_ros2

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__FEATURE_MAP_BRIDGE_HPP_
