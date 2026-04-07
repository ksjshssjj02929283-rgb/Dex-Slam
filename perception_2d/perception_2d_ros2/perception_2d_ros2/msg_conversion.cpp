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

#include "perception_2d_ros2/msg_conversion.hpp"

#include <cmath>
#include <string>
#include <tuple>

#include "perception_2d/common/math.hpp"
#include "perception_2d/common/port.hpp"
#include "perception_2d/common/time.hpp"
#include "perception_2d/transform/transform.hpp"
#include "perception_2d_ros2/time_conversion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace
{

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT
{
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  float unused_padding[2];
};

}  // namespace

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZT, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIT,
  (float, x, x)(float, y, y)(float, z, z)(
    float, intensity,
    intensity)(float, time, time))

namespace perception_ros2
{
namespace
{

// The ros::sensor_msgs::msg::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;

using ::perception_2d::sensor::PointCloudWithIntensities;
using ::perception_2d::transform::Rigid3d;

sensor_msgs::msg::PointCloud2 PreparePointCloud2Message(
  const int64_t timestamp,
  const std::string & frame_id,
  const int num_points)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = ToRos(::perception_2d::common::FromUniversal(timestamp));
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = num_points;
  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.is_bigendian = false;
  msg.point_step = 16;
  msg.row_step = 16 * msg.width;
  msg.is_dense = true;
  msg.data.resize(16 * num_points);
  return msg;
}

// For sensor_msgs::msg::LaserScan.
bool HasEcho(float) {return true;}

float GetFirstEcho(float range) {return range;}

// For sensor_msgs::msg::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::msg::LaserEcho & echo)
{
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::msg::LaserEcho & echo)
{
  return echo.echoes[0];
}

// For sensor_msgs::msg::LaserScan and sensor_msgs::msg::MultiEchoLaserScan.
template<typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::perception_2d::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType & msg)
{
  CHECK_GE(msg.range_min, 0.f);
  CHECK_GE(msg.range_max, msg.range_min);
  if (msg.angle_increment > 0.f) {
    CHECK_GT(msg.angle_max, msg.angle_min);
  } else {
    CHECK_GT(msg.angle_min, msg.angle_max);
  }
  PointCloudWithIntensities point_cloud;
  float angle = msg.angle_min;
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const auto & echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const perception_2d::sensor::TimedRangefinderPoint point{
          rotation * (first_echo * Eigen::Vector3f::UnitX()),
          i * msg.time_increment};
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto & echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  ::perception_2d::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += perception_2d::common::FromSeconds(duration);
    for (auto & point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(
  const sensor_msgs::msg::PointCloud2 & pc2,
  const std::string & field_name)
{
  for (const auto & field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
  const int64_t timestamp, const std::string & frame_id,
  const ::perception_2d::sensor::TimedPointCloud & point_cloud)
{
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  // Directly write to the data buffer
  float * data_ptr = reinterpret_cast<float *>(msg.data.data());
  for (const perception_2d::sensor::TimedRangefinderPoint & point : point_cloud) {
    *data_ptr++ = point.position.x();
    *data_ptr++ = point.position.y();
    *data_ptr++ = point.position.z();
    *data_ptr++ = kPointCloudComponentFourMagic;
  }
  return msg;
}

std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan & msg)
{
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan & msg)
{
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::perception_2d::sensor::PointCloudWithIntensities,
  ::perception_2d::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2 & msg)
{
  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(msg, "intensity")) {
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZIT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto & point : pcl_point_cloud) {
        point_cloud.points.push_back(
          {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(point.intensity);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto & point : pcl_point_cloud) {
        point_cloud.points.push_back(
          {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(point.intensity);
      }
    }
  } else {
    // If we don't have an intensity field, just copy XYZ and fill in 1.0f.
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto & point : pcl_point_cloud) {
        point_cloud.points.push_back(
          {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(1.0f);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto & point : pcl_point_cloud) {
        point_cloud.points.push_back(
          {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(1.0f);
      }
    }
  }
  ::perception_2d::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += perception_2d::common::FromSeconds(duration);
    for (auto & point : point_cloud.points) {
      point.time -= duration;
      CHECK_LE(point.time, 0.f) <<
        "Encountered a point with a larger stamp than "
        "the last point in the cloud.";
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

Rigid3d ToRigid3d(const double & x, const double & y, const double & yaw)
{
  return Rigid3d(
    Eigen::Vector3d(x, y, 0),
    perception_2d::transform::RollPitchYaw(0, 0, yaw));
}

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped & transform)
{
  return Rigid3d(
    ToEigen(transform.transform.translation),
    ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::msg::Pose & pose)
{
  return Rigid3d(
    {pose.position.x, pose.position.y, pose.position.z},
    ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3 & vector3)
{
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion & quaternion)
{
  return Eigen::Quaterniond(
    quaternion.w, quaternion.x, quaternion.y,
    quaternion.z);
}

geometry_msgs::msg::Transform ToGeometryMsgTransform(const Rigid3d & rigid3d)
{
  geometry_msgs::msg::Transform transform;
  transform.translation.x = rigid3d.translation().x();
  transform.translation.y = rigid3d.translation().y();
  transform.translation.z = rigid3d.translation().z();
  transform.rotation.w = rigid3d.rotation().w();
  transform.rotation.x = rigid3d.rotation().x();
  transform.rotation.y = rigid3d.rotation().y();
  transform.rotation.z = rigid3d.rotation().z();
  return transform;
}

geometry_msgs::msg::Pose ToGeometryMsgPose(const Rigid3d & rigid3d)
{
  geometry_msgs::msg::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d & vector3d)
{
  geometry_msgs::msg::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateLineFeatureMsg(
          const rclcpp::Time& time,
          const std::string& frame_id, 
          int& marker_id, 
          const std::vector<perception_2d::LineData>& lane_map,
          bool vis_line){
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>>
     lane_map_and_lane_text = std::make_unique<std::vector<visualization_msgs::msg::Marker>>();
  visualization_msgs::msg::Marker lane_map_marker;
  lane_map_marker.id = marker_id++;
  if(vis_line){
    lane_map_marker.ns = "vis line feature";
  }else{
    lane_map_marker.ns = "line feature";
  }
  
  lane_map_marker.type = visualization_msgs::msg::Marker::LINE_LIST; // 线
  lane_map_marker.header.stamp = time;
  lane_map_marker.header.frame_id = frame_id;
  lane_map_marker.scale.x = 0.1;//kLineFeatureMarkerScale;
  lane_map_marker.pose.orientation.w = 1.0;
  lane_map_marker.action = visualization_msgs::msg::Marker::ADD;
  std_msgs::msg::ColorRGBA color;
  if (vis_line == true) {
    color.a = 0.4;
    color.r = 0.79;
    color.g = 0.32;
    color.b = 0.12;
  } else {
    color.a = 0.4;
    color.r = 0.79;
    color.g = 0.94;
    color.b = 0.12;
  }
  for(size_t i = 0; i < lane_map.size(); i++){
    geometry_msgs::msg::Point point1,point2;
    const perception_2d::LineData& line_data = lane_map[i];
    point1.x = line_data.begin[0]; 
    point1.y = line_data.begin[1]; 
    point1.z = 0;
    point2.x = line_data.end[0]; 
    point2.y = line_data.end[1]; 
    point2.z = 0;

    lane_map_marker.points.push_back(point1);
    lane_map_marker.points.push_back(point2);
    lane_map_marker.colors.push_back(color);
    lane_map_marker.colors.push_back(color);
  }
  lane_map_and_lane_text->push_back(lane_map_marker);
  return lane_map_and_lane_text;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> TransferLine(
  const std::pair<Eigen::Vector3f, Eigen::Vector3f> & line,
  const perception_2d::transform::Rigid3d & origin)
{
  std::pair<Eigen::Vector3f, Eigen::Vector3f> return_line;

  Eigen::Vector3d begin_point(static_cast<double>(line.first[0]),
    static_cast<double>(line.first[1]),
    0);

  Eigen::Vector3d end_point(static_cast<double>(line.second[0]),
    static_cast<double>(line.second[1]),
    0);

  Eigen::Vector3f trans_begin_point{
    static_cast<float>((origin * begin_point)[0]),
    static_cast<float>((origin * begin_point)[1]),
    0};
  Eigen::Vector3f trans_end_point{
    static_cast<float>((origin * end_point)[0]),
    static_cast<float>((origin * end_point)[1]),
    0};

  return_line.first = trans_begin_point;
  return_line.second = trans_end_point;
  return return_line;
}


}  // namespace perception_ros2
