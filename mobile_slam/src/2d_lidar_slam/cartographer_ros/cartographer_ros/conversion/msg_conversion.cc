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

#include "cartographer_ros/conversion/msg_conversion.h"

#include <cmath>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/transform/proto/transform.pb.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/conversion/time_conversion.h"
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
#include "rclcpp/rclcpp.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace {

// Sizes of PCL point types have to be 4n floats for alignment, as described in
// http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php
struct PointXYZT {
  float x;
  float y;
  float z;
  float time;
};

struct PointXYZIT {
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
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(float, time, time))

namespace cartographer_ros {
namespace {

// The ros::sensor_msgs::msg::PointCloud2 binary data contains 4 floats for each
// point. The last one must be this value or RViz is not showing the point cloud
// properly.
constexpr float kPointCloudComponentFourMagic = 1.;
constexpr double kLaneMapMarkerScale = 0.04;
constexpr double kLineFeatureMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.1;

using ::cartographer::sensor::LandmarkData;
using ::cartographer::sensor::LandmarkObservation;
using ::cartographer::sensor::PointCloudWithIntensities;
using ::cartographer::transform::Rigid3d;
using ::cartographer_ros_msgs::LandmarkEntry;
using ::cartographer_ros_msgs::LandmarkList;

sensor_msgs::msg::PointCloud2 PreparePointCloud2Message(const int64_t timestamp,
                                                   const std::string& frame_id,
                                                   const int num_points) {
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.stamp = ToRos(::cartographer::common::FromUniversal(timestamp));
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
bool HasEcho(float) { return true; }

float GetFirstEcho(float range) { return range; }

// For sensor_msgs::msg::MultiEchoLaserScan.
bool HasEcho(const sensor_msgs::msg::LaserEcho& echo) {
  return !echo.echoes.empty();
}

float GetFirstEcho(const sensor_msgs::msg::LaserEcho& echo) {
  return echo.echoes[0];
}

// For sensor_msgs::msg::LaserScan and sensor_msgs::msg::MultiEchoLaserScan.
template <typename LaserMessageType>
std::tuple<PointCloudWithIntensities, ::cartographer::common::Time>
LaserScanToPointCloudWithIntensities(const LaserMessageType& msg) {
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
    const auto& echoes = msg.ranges[i];
    if (HasEcho(echoes)) {
      const float first_echo = GetFirstEcho(echoes);
      if (msg.range_min <= first_echo && first_echo <= msg.range_max) {
        const Eigen::AngleAxisf rotation(angle, Eigen::Vector3f::UnitZ());
        const cartographer::sensor::TimedRangefinderPoint point{
            rotation * (first_echo * Eigen::Vector3f::UnitX()),
            i * msg.time_increment};
        point_cloud.points.push_back(point);
        if (msg.intensities.size() > 0) {
          CHECK_EQ(msg.intensities.size(), msg.ranges.size());
          const auto& echo_intensities = msg.intensities[i];
          CHECK(HasEcho(echo_intensities));
          point_cloud.intensities.push_back(GetFirstEcho(echo_intensities));
        } else {
          point_cloud.intensities.push_back(0.f);
        }
      }
    }
    angle += msg.angle_increment;
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

bool PointCloud2HasField(const sensor_msgs::msg::PointCloud2& pc2,
                         const std::string& field_name) {
  for (const auto& field : pc2.fields) {
    if (field.name == field_name) {
      return true;
    }
  }
  return false;
}

}  // namespace

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    const int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::TimedPointCloud& point_cloud) {
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const cartographer::sensor::TimedRangefinderPoint& point : point_cloud) {
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

sensor_msgs::msg::PointCloud2 ToPointCloud2Message(
    int64_t timestamp, const std::string& frame_id,
    const ::cartographer::sensor::PointCloud& point_cloud)
{
  auto msg = PreparePointCloud2Message(timestamp, frame_id, point_cloud.size());
  ::ros::serialization::OStream stream(msg.data.data(), msg.data.size());
  for (const cartographer::sensor::RangefinderPoint& point : point_cloud) {
    stream.next(point.position.x());
    stream.next(point.position.y());
    stream.next(point.position.z());
    stream.next(kPointCloudComponentFourMagic);
  }
  return msg;
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::LaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::MultiEchoLaserScan& msg) {
  return LaserScanToPointCloudWithIntensities(msg);
}

std::tuple<::cartographer::sensor::PointCloudWithIntensities,
           ::cartographer::common::Time>
ToPointCloudWithIntensities(const sensor_msgs::msg::PointCloud2& msg) {
  PointCloudWithIntensities point_cloud;
  // We check for intensity field here to avoid run-time warnings if we pass in
  // a PointCloud2 without intensity.
  if (PointCloud2HasField(msg, "intensity")) {
    if (PointCloud2HasField(msg, "time")) {
      pcl::PointCloud<PointXYZIT> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(point.intensity);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZI> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
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
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, point.time});
        point_cloud.intensities.push_back(1.0f);
      }
    } else {
      pcl::PointCloud<pcl::PointXYZ> pcl_point_cloud;
      pcl::fromROSMsg(msg, pcl_point_cloud);
      point_cloud.points.reserve(pcl_point_cloud.size());
      point_cloud.intensities.reserve(pcl_point_cloud.size());
      for (const auto& point : pcl_point_cloud) {
        point_cloud.points.push_back(
            {Eigen::Vector3f{point.x, point.y, point.z}, 0.f});
        point_cloud.intensities.push_back(1.0f);
      }
    }
  }
  ::cartographer::common::Time timestamp = FromRos(msg.header.stamp);
  if (!point_cloud.points.empty()) {
    const double duration = point_cloud.points.back().time;
    timestamp += cartographer::common::FromSeconds(duration);
    for (auto& point : point_cloud.points) {
      point.time -= duration;
      CHECK_LE(point.time, 0.f)
          << "Encountered a point with a larger stamp than "
             "the last point in the cloud.";
    }
  }
  return std::make_tuple(point_cloud, timestamp);
}

LandmarkData ToLandmarkData(const LandmarkList& landmark_list) {
  LandmarkData landmark_data;
  landmark_data.time = FromRos(landmark_list.header.stamp);
  for (const LandmarkEntry& entry : landmark_list.landmarks) {
    landmark_data.landmark_observations.push_back(
        {entry.id, ToRigid3d(entry.tracking_from_landmark_transform),
         entry.translation_weight, entry.rotation_weight});
  }
  return landmark_data;
}

Rigid3d ToRigid3d(const geometry_msgs::msg::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d ToRigid3d(const geometry_msgs::msg::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d ToEigen(const geometry_msgs::msg::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond ToEigen(const geometry_msgs::msg::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}

geometry_msgs::msg::Transform ToGeometryMsgTransform(const Rigid3d& rigid3d) {
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

geometry_msgs::msg::Pose ToGeometryMsgPose(const Rigid3d& rigid3d) {
  geometry_msgs::msg::Pose pose;
  pose.position = ToGeometryMsgPoint(rigid3d.translation());
  pose.orientation.w = rigid3d.rotation().w();
  pose.orientation.x = rigid3d.rotation().x();
  pose.orientation.y = rigid3d.rotation().y();
  pose.orientation.z = rigid3d.rotation().z();
  return pose;
}

geometry_msgs::msg::Point ToGeometryMsgPoint(const Eigen::Vector3d& vector3d) {
  geometry_msgs::msg::Point point;
  point.x = vector3d.x();
  point.y = vector3d.y();
  point.z = vector3d.z();
  return point;
}

Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(cartographer::common::DegToRad(latitude));
  const double cos_phi = std::cos(cartographer::common::DegToRad(latitude));
  const double sin_lambda = std::sin(cartographer::common::DegToRad(longitude));
  const double cos_lambda = std::cos(cartographer::common::DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

cartographer::transform::Rigid3d ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(cartographer::common::DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(cartographer::common::DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  return cartographer::transform::Rigid3d(rotation * -translation, rotation);
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> CreateOccupancyGridMsg(
  const cartographer::io::PaintSubmapSlicesResult& painted_slices,
  const double resolution, const std::string& frame_id,
  const rclcpp::Time& time) {
  auto occupancy_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();

  const int width = cairo_image_surface_get_width(painted_slices.surface.get());
  const int height =
    cairo_image_surface_get_height(painted_slices.surface.get());

  occupancy_grid->header.stamp = time;
  occupancy_grid->header.frame_id = frame_id;
  occupancy_grid->info.map_load_time = time;
  occupancy_grid->info.resolution = resolution;
  occupancy_grid->info.width = width;
  occupancy_grid->info.height = height;
  occupancy_grid->info.origin.position.x =
    -painted_slices.origin.x() * resolution;
  occupancy_grid->info.origin.position.y =
    (-height + painted_slices.origin.y()) * resolution;
  occupancy_grid->info.origin.position.z = 0.;
  occupancy_grid->info.origin.orientation.w = 1.;
  occupancy_grid->info.origin.orientation.x = 0.;
  occupancy_grid->info.origin.orientation.y = 0.;
  occupancy_grid->info.origin.orientation.z = 0.;

  const uint32_t* pixel_data = reinterpret_cast<uint32_t*>(
    cairo_image_surface_get_data(painted_slices.surface.get()));
  occupancy_grid->data.reserve(width * height);
  for (int y = height - 1; y >= 0; --y) {
  for (int x = 0; x < width; ++x) {
    const uint32_t packed = pixel_data[y * width + x];
    const unsigned char color = packed >> 16;
    const unsigned char observed = packed >> 8;
    const int value =
      observed == 0
        ? -1
        : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
    CHECK_LE(-1, value);
    CHECK_GE(100, value);
    occupancy_grid->data.push_back(value);
  }
  }

  return occupancy_grid;
}

std_msgs::msg::ColorRGBA GetColor(const COLOR& color)
{
  std_msgs::msg::ColorRGBA color_msg;
  color_msg.a = 0.5;
  switch(color) {
    default:
    case null:
      color_msg.r = 0; color_msg.g = 0.67; color_msg.b = 0.95;
      break;
    case yellow:
      color_msg.r = 1; color_msg.g = 1; color_msg.b = 0;
      break;
    case red:
      color_msg.r = 1; color_msg.g = 0; color_msg.b = 0;
      break;
    case green:
      color_msg.r = 0; color_msg.g = 1; color_msg.b = 0;
      break;
    case blue:
      color_msg.r = 0; color_msg.g = 0; color_msg.b = 1;
      break;
    case purple:
      color_msg.r = 1; color_msg.g = 0; color_msg.b = 1;
      break;
    case orange:
      color_msg.r = 1; color_msg.g = 0.5; color_msg.b = 0;
      break;
    case white:
      color_msg.r = 1; color_msg.g = 1; color_msg.b = 1;
      break;
    case black:
      color_msg.r = 0; color_msg.g = 0; color_msg.b = 0;
      break;
    case gray:
      color_msg.r = 0.5; color_msg.g = 0.5; color_msg.b = 0.5;
      break;
    case brown:
      color_msg.r = 0.5; color_msg.g = 0.25; color_msg.b = 0;
      break;
  }
  return color_msg;
}

visualization_msgs::msg::Marker CreateLineMsg(
    const int& id, const std::string& frame_id,
    std::pair<Eigen::Vector2d, Eigen::Vector2d> line,
    const COLOR pub_color)
{
  visualization_msgs::msg::Marker line_map_marker;
  std_msgs::msg::ColorRGBA color = GetColor(pub_color);
  
  line_map_marker.ns = "global_line";
  line_map_marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // 线
  line_map_marker.id = id;
  line_map_marker.header.stamp = rclcpp::Clock().now();
  line_map_marker.header.frame_id = frame_id;
  line_map_marker.scale.x = 0.15;  // kLaneMapMarkerScale;
  line_map_marker.pose.orientation.w = 1.0;
  line_map_marker.action = visualization_msgs::msg::Marker::ADD;
  geometry_msgs::msg::Point point1, point2;
  if (std::isnan(line.first[0]) || std::isnan(line.first[1]) ||
      std::isnan(line.second[0]) || std::isnan(line.second[1]))
  {
    return line_map_marker;
  }


  point1.x = line.first[0];
  point1.y = line.first[1];
  point1.z = 0;
  point2.x = line.second[0];
  point2.y = line.second[1];
  point2.z = 0;
  line_map_marker.points.push_back(point1);
  line_map_marker.points.push_back(point2);
  line_map_marker.color = color;
  return line_map_marker;
}

std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateLineMapMsg(
    const ros::Time& time, const std::string& frame_id, int& marker_id,
    const std::shared_ptr<cartographer::sensor::LineMapData>
        line_map,
    COLOR pub_color, bool with_normal) {
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>>
      line_map_and_lane_text =
          absl::make_unique<std::vector<visualization_msgs::msg::Marker>>();
  std_msgs::msg::ColorRGBA color;
  color.a = 0.5;
  if (pub_color == COLOR::null) {
    color.r = 0;
    color.g = 0.67;
    color.b = 0.95;
  } else if (pub_color == COLOR::yellow) {
    color.r = 0.5;
    color.g = 0.67;
    color.b = 0;
  }

  for (size_t i = 0; i < line_map->size(); i++) {
    visualization_msgs::msg::Marker line_map_marker, corrdinate_text, normal_marker;
    line_map_marker.id = marker_id++;
    corrdinate_text.id = marker_id++;
    if (with_normal) normal_marker.id = marker_id++;
    line_map_marker.ns = corrdinate_text.ns = "lane map";
    line_map_marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // 线
    line_map_marker.pose.orientation.w = 1.0;
    corrdinate_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    normal_marker.type = visualization_msgs::msg::Marker::LINE_LIST;  // 线
    line_map_marker.header.stamp = corrdinate_text.header.stamp =
        normal_marker.header.stamp = time;
    line_map_marker.header.frame_id = corrdinate_text.header.frame_id =
        normal_marker.header.frame_id = frame_id;
    // corrdinate_text.scale.x = 0.1;
    corrdinate_text.scale.z = 0.2;
    corrdinate_text.color.r = 1.0f;
    corrdinate_text.color.a = 1.0;
    line_map_marker.scale.x = 0.15;  // kLaneMapMarkerScale;
    normal_marker.scale.x = 0.03;
    const cartographer::sensor::LaneDataWithNormal&
        lane_data = (*line_map)[i];
    std::ostringstream ss;
    ss << "(begin: " << lane_data.data.line_data.begin(0) << " "
       << lane_data.data.line_data.begin(1)
       << " end: " << lane_data.data.line_data.end(0) << " "
       << lane_data.data.line_data.end(1) << " )";
    corrdinate_text.text = ss.str();
    corrdinate_text.pose.position.x =
        (lane_data.data.line_data.begin(0) + lane_data.data.line_data.end(0)) /
            2 +
        0.1;
    corrdinate_text.pose.position.y =
        (lane_data.data.line_data.begin(1) + lane_data.data.line_data.end(1)) /
            2 +
        0.1;
    if (std::isnan(lane_data.data.line_data.begin(0)) || std::isnan(lane_data.data.line_data.begin(1)) ||
        std::isnan(lane_data.data.line_data.end(0)) || std::isnan(lane_data.data.line_data.end(1)))
    {
      continue;
    }

    geometry_msgs::msg::Point point1, point2;
    point1.x = lane_data.data.line_data.begin(0);
    point1.y = lane_data.data.line_data.begin(1);
    point1.z = 0;
    point2.x = lane_data.data.line_data.end(0);
    point2.y = lane_data.data.line_data.end(1);
    point2.z = 0;
    line_map_marker.points.push_back(point1);
    line_map_marker.points.push_back(point2);
    line_map_marker.colors.push_back(color);
    line_map_marker.colors.push_back(color);
    if (with_normal) {
      Eigen::Vector2d normal = lane_data.normal.normalized();
      geometry_msgs::msg::Point normal_point1, normal_point2;
      normal_point1.x = lane_data.data.line_data.begin(0);
      normal_point1.y = lane_data.data.line_data.begin(1);
      normal_point1.z = 0;
      normal_point2.x = lane_data.data.line_data.begin(0) + normal(0);
      normal_point2.y = lane_data.data.line_data.begin(1) + normal(1);
      normal_point2.z = 0;
      normal_marker.points.push_back(normal_point1);
      normal_marker.points.push_back(normal_point2);
      normal_marker.colors.push_back(color);
      normal_marker.colors.push_back(color);
      line_map_and_lane_text->push_back(normal_marker);
    }
    line_map_and_lane_text->push_back(line_map_marker);
    line_map_and_lane_text->push_back(corrdinate_text);
  }
  return line_map_and_lane_text;
}


std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateReflectorMapMsg(
    const ros::Time& time, const std::string& frame_id, int& marker_id,
    const std::shared_ptr<std::vector<Eigen::Vector2d>> reflector_map) {
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>>
      reflector_map_and_reflector_text =
          absl::make_unique<std::vector<visualization_msgs::msg::Marker>>();
  visualization_msgs::msg::Marker reflector_map_marker;
  reflector_map_marker.id = marker_id++;
  reflector_map_marker.ns = "reflector map";
  reflector_map_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;  // 球体
  reflector_map_marker.header.stamp = time;
  reflector_map_marker.header.frame_id = frame_id;
  reflector_map_marker.scale.x = kLandmarkMarkerScale;
  reflector_map_marker.scale.y = kLandmarkMarkerScale;
  reflector_map_marker.scale.z = 0.1;
  reflector_map_marker.pose.orientation.w = 1.0;
  reflector_map_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker reflector_text_marker;
  reflector_text_marker.header.frame_id = frame_id;
  reflector_text_marker.header.stamp = ros::Time::now();
  reflector_text_marker.ns = "reflector map";
  reflector_text_marker.pose.orientation.w = 1.0;
  reflector_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  reflector_text_marker.scale.z = 0.05;
  reflector_text_marker.color.b = 0;
  reflector_text_marker.color.g = 1;
  reflector_text_marker.color.r = 0;
  reflector_text_marker.color.a = 1;

  std_msgs::msg::ColorRGBA color;
  color.a = 1;
  color.r = 1;
  color.g = 0;
  color.b = 0.63;
  for (size_t i = 0; i < reflector_map->size(); i++) {
    geometry_msgs::msg::Point point;
    const Eigen::Vector2d& reflector_data = (*reflector_map)[i];
    point.x = reflector_data(0);
    point.y = reflector_data(1);
    point.z = 0;
    reflector_map_marker.points.push_back(point);
    reflector_map_marker.colors.push_back(color);

    reflector_text_marker.id = marker_id++;
    reflector_text_marker.pose.position.x = point.x;
    reflector_text_marker.pose.position.y = point.y;
    reflector_text_marker.pose.position.z = 0.1;
    std::string pub_str =
        "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ")";
    reflector_text_marker.text = pub_str;
    reflector_map_and_reflector_text->push_back(reflector_text_marker);
  }
  reflector_map_and_reflector_text->push_back(reflector_map_marker);
  return reflector_map_and_reflector_text;
}


// std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> CreateCodeMapMsg(
//     const ros::Time& time, const std::string& frame_id, int& marker_id,
//     const std::shared_ptr<
//         std::vector<cartographer::sensor::CodeData>>
//         code_map) {
//   // visualization_msgs::msg::MarkerArray code_map_markers;
//   std::unique_ptr<std::vector<visualization_msgs::msg::Marker>>
//       code_map_and_code_text =
//           absl::make_unique<std::vector<visualization_msgs::msg::Marker>>();
//   visualization_msgs::msg::Marker code_map_marker;
//   code_map_marker.id = marker_id++;
//   code_map_marker.ns = "code map";
//   code_map_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;  // 立方体
//   code_map_marker.header.stamp = time;
//   code_map_marker.header.frame_id = frame_id;
//   code_map_marker.scale.x = kLandmarkMarkerScale;
//   code_map_marker.scale.y = kLandmarkMarkerScale;
//   code_map_marker.scale.z = 0.001;
//   code_map_marker.pose.orientation.w = 1.0;

//   visualization_msgs::msg::Marker code_text_marker;
//   code_text_marker.header.frame_id = frame_id;
//   code_text_marker.header.stamp = ros::Time::now();
//   code_text_marker.ns = "code map";
//   // code_text_marker.action = visualization_msgs::msg::Marker::ADD;
//   code_text_marker.pose.orientation.w = 1.0;
//   code_text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//   code_text_marker.scale.z = 0.05;
//   code_text_marker.color.b = 0;
//   code_text_marker.color.g = 0;
//   code_text_marker.color.r = 1;
//   code_text_marker.color.a = 1;

//   std_msgs::msg::ColorRGBA color;
//   color.a = 1;
//   color.r = 0;
//   color.g = 1;
//   color.b = 0;
//   for (size_t i = 0; i < code_map->size(); i++) {
//     geometry_msgs::msg::Point point;
//     const cartographer::sensor::CodeData& code_data =
//         (*code_map)[i];
//     point.x = code_data.pose.translation().x();
//     point.y = code_data.pose.translation().y();
//     point.z = 0;
//     code_map_marker.points.push_back(point);
//     code_map_marker.colors.push_back(color);

//     code_text_marker.id = marker_id++;
//     code_text_marker.pose.position.x = point.x;
//     code_text_marker.pose.position.y = point.y;
//     code_text_marker.pose.position.z = 0.1;
//     std::string pub_str = "(" + std::to_string(point.x) + "," +
//                           std::to_string(point.y) + ")" +
//                           "\n code:" + std::to_string(code_data.code);
//     code_text_marker.text = pub_str;
//     code_map_and_code_text->push_back(code_text_marker);
//   }
//   code_map_and_code_text->push_back(code_map_marker);
//   return code_map_and_code_text;
// }

}  // namespace cartographer_ros
