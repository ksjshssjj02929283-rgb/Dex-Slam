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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__MSG_CONVERSION_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__MSG_CONVERSION_HPP_

#include <string>
#include <tuple>
#include "rclcpp/rclcpp.hpp"
#include "perception_2d/common/time.hpp"
#include "perception_2d/sensor/point_cloud.hpp"
#include "perception_2d/transform/rigid_transform.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_interfaces/msg/line_array.hpp"

namespace perception_ros2
{
std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateLineFeatureMsg(
          const rclcpp::Time& time,
          const std::string& frame_id, 
          int& marker_id, 
          const std::vector<perception_2d::LineData>& lane_map,
          bool vis_line);
sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
  int64_t timestamp, const std::string & frame_id,
  const ::perception_2d::sensor::TimedPointCloud & point_cloud);

geometry_msgs::msg::Transform ToGeometryMsgTransform(
  const ::perception_2d::transform::Rigid3d & rigid3d);

geometry_msgs::msg::Pose ToGeometryMsgPose(
  const ::perception_2d::transform::Rigid3d & rigid3d);

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d & vector3d);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan & msg);

std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan & msg);

std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2 & msg);

::perception_2d::transform::Rigid3d ToRigid3d(
  const geometry_msgs::msg::TransformStamped & transform);

::perception_2d::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose & pose);

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3 & vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion & quaternion);

::perception_2d::transform::Rigid3d ToRigid3d(
  const double & x, const double & y, const double & yaw);

nav_interfaces::msg::LineArray ToLineData();

std::pair<Eigen::Vector3f, Eigen::Vector3f> TransferLine(
  const std::pair<Eigen::Vector3f, Eigen::Vector3f> & line,
  const perception_2d::transform::Rigid3d & origin);

}  // namespace perception_ros2

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__MSG_CONVERSION_HPP_
