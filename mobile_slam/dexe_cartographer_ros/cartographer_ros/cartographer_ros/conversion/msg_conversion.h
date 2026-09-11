/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CONVERSION_MSG_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CONVERSION_MSG_CONVERSION_H

#include <string>
#include <utility>
#include <vector>

#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace cartographer_ros {

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud);

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud);

geometry_msgs::msg::Transform ToGeometryMsgTransform(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Pose ToGeometryMsgPose(
    const ::cartographer::transform::Rigid3d& rigid3d);

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d);

// Converts ROS message to point cloud. Returns the time when the last point
// was acquired (different from the ROS timestamp). Timing of points is given in
// the fourth component of each point relative to `Time`.
std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan& msg);

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2& msg);

::cartographer::sensor::LandmarkData ToLandmarkData(
    const cartographer_ros_msgs::msg::LandmarkList& landmark_list);

::cartographer::transform::Rigid3d ToRigid3d(
    const geometry_msgs::msg::TransformStamped& transform);

::cartographer::transform::Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose);

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3);

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion);

// Converts from WGS84 (latitude, longitude, altitude) to ECEF.
Eigen::Vector3d LatLongAltToEcef(double latitude, double longitude,
                                 double altitude);

// Returns a transform that takes ECEF coordinates from nearby points to a local
// frame that has z pointing upwards.
cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(double latitude,
                                                              double longitude);

// Points to an occupancy grid message at a specific resolution from painted
// submap slices obtained via ::cartographer::io::PaintSubmapSlices(...).
std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
    const cartographer::io::PaintSubmapSlicesResult& painted_slices,
    const double resolution, const std::string& frame_id,
    const rclcpp::Time& time);

struct COLOR_RGB {
  double a;
  double r;
  double g;
  double b;
};

enum COLOR {
  null = 0,
  yellow = 1,
  red = 2,
  green = 3,
  blue = 4,
  purple = 5,
  orange = 6,
  pink = 7,
  cyan = 8,
  white = 9,
  black = 10,
  gray = 11,
  brown = 12,
  magenta = 13,
  maroon = 14,
  olive = 15,
};

std_msgs::msg::ColorRGBA GetColor(const COLOR& color);

// 把一组线段打包进单个 LINE_LIST Marker。
//
// 线特征曾按「一条线段一个 Marker」发布，marker.id 直接取自线跟踪 id；而同一个
// 跟踪 id 在一帧里可能对应多条观测线段（line_data_in_local 的 value 是 vector），
// 于是同一个 MarkerArray 内出现重复的 (ns, id)，RViz 报 Duplicate Marker Check。
// LINE_LIST 本身就支持一个 Marker 承载多条互不相连的线段，统一打包后 id 冲突不再
// 可能出现，消息体也从 N 个 Marker 降到 1 个。
//
// lines 为空、或线段全部含 NaN/Inf 时，返回 action=DELETE 的 Marker：空点集的
// LINE_LIST 会让 RViz 报错，而发空 MarkerArray 又清不掉上一帧的残留线段。
visualization_msgs::msg::Marker CreateLineListMsg(
    const rclcpp::Time& time, const std::string& frame_id,
    const std::string& ns, int id,
    const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& lines,
    COLOR pub_color = null, double scale = 0.15, double alpha = 1.0);
std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateLineMapMsg(
          const rclcpp::Time& time,
          const std::string& frame_id, 
          int& marker_id, 
          const std::shared_ptr<cartographer::sensor::LineMapData> lane_map,
          COLOR pub_color = null, bool with_normal = false);

// std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateCodeMapMsg(
//           const rclcpp::Time& time,
//           const std::string& frame_id, 
//           int& marker_id, 
//           const std::shared_ptr<std::vector<matrix::slam::lidar::slam_core::sensor::CodeData>> code_map);

std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateReflectorMapMsg(
          const rclcpp::Time& time,
          const std::string& frame_id, 
          int& marker_id, 
          const std::shared_ptr<std::vector<Eigen::Vector2d>> reflector_map);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_CONVERSION_MSG_CONVERSION_H
