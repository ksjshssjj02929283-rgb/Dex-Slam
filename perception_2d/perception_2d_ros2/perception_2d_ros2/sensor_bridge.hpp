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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__SENSOR_BRIDGE_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__SENSOR_BRIDGE_HPP_

#include <memory>
#include <map>
#include <string>
#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/transform/rigid_transform.hpp"
#include "perception_2d/transform/transform.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d_ros2/tf_bridge.hpp"
#include "perception_2d/laser_perception_2d/laser_perception_2d_interface.hpp"
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
class SensorBridge
{
public:
  explicit SensorBridge(
    int num_subdivisions_per_laser_scan, const std::string & tracking_frame,
    double lookup_transform_timeout_sec, tf2_ros::Buffer * tf_buffer,
    perception_2d::LaserPerception2DInterface * laser_perception_2d);

  SensorBridge(const SensorBridge &) = delete;
  SensorBridge & operator=(const SensorBridge &) = delete;

  std::unique_ptr<perception_2d::sensor::OdometryData> 
  ToOdometryData(
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void HandleOdometryMessage(
    const std::string & sensor_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void HandleLaserScanMessage(
    const std::string & sensor_id,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);
  void HandlePointCloud2Message(
    const std::string & sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  const TfBridge & tf_bridge() const;

private:
  void HandleLaserScan(
    const std::string & sensor_id, 
    perception_2d::common::Time start_time,
    const std::string & frame_id,
    const perception_2d::sensor::PointCloudWithIntensities & points);

  const int num_subdivisions_per_laser_scan_;
  std::map<std::string, perception_2d::common::Time>
  sensor_to_previous_subdivision_time_;
  const TfBridge tf_bridge_;
  perception_2d::LaserPerception2DInterface * const
  laser_perception_2d_;
};

}  // namespace perception_ros2

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__SENSOR_BRIDGE_HPP_
