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
#include <string>
#include <memory>
#include "perception_2d_ros2/sensor_bridge.hpp"

#include "perception_2d_ros2/msg_conversion.hpp"
#include "perception_2d_ros2/time_conversion.hpp"

namespace perception_ros2
{


using perception_2d::transform::Rigid3d;

namespace
{

const std::string & CheckNoLeadingSlash(const std::string & frame_id)
{
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id <<
      " should not start with a /. See 1.7 in "
      "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

SensorBridge::SensorBridge(
  const int num_subdivisions_per_laser_scan,
  const std::string & tracking_frame,
  const double lookup_transform_timeout_sec, tf2_ros::Buffer * const tf_buffer,
  perception_2d::LaserPerception2DInterface * const 
    laser_perception_2d)
: num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
  tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
  laser_perception_2d_(laser_perception_2d) {}

std::unique_ptr<perception_2d::sensor::OdometryData> 
SensorBridge::ToOdometryData(
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const perception_2d::common::Time time = 
    FromRos(msg->header.stamp);
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
    time, "base_link");   // msg->child_frame_id
  return std::make_unique<perception_2d::sensor::OdometryData>(
    perception_2d::sensor::OdometryData{
      time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse(),
      Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,
                      msg->twist.twist.linear.z),
      Eigen::Vector3d(msg->twist.twist.angular.x,msg->twist.twist.angular.y,
                      msg->twist.twist.angular.z),
      Eigen::Matrix<double, 6, 6>::Zero()});
}

void SensorBridge::HandleOdometryMessage(
  const std::string & sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::unique_ptr<perception_2d::sensor::OdometryData> odometry_data = ToOdometryData(msg);
  // LOG(INFO) << "odom_ x = " << odometry_data->pose.translation()[0]
  //           << " y = " << odometry_data->pose.translation()[1];
  if (odometry_data != nullptr) {
    laser_perception_2d_->AddSensorData(
      sensor_id,
      perception_2d::sensor::OdometryData{odometry_data->time, odometry_data->pose,
                                          odometry_data->linear_velocity,
                                          odometry_data->angular_velocity,
                                          odometry_data->pose_covariance});
  }
}

void SensorBridge::HandleLaserScanMessage(
  const std::string & sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  perception_2d::sensor::PointCloudWithIntensities point_cloud;
  perception_2d::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
  const std::string & sensor_id,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  perception_2d::sensor::PointCloudWithIntensities point_cloud;
  perception_2d::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  // HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud.points);
  // HandlePointCloudWithIntensities(sensor_id, time, msg->header.frame, point_cloud);
}

const TfBridge & SensorBridge::tf_bridge() const {return tf_bridge_;}

void SensorBridge::HandleLaserScan(
  const std::string & sensor_id, const perception_2d::common::Time time,
  const std::string & frame_id,
  const perception_2d::sensor::PointCloudWithIntensities & points)
{
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // sensor frame to tracking frame(laser --> base_link)
  const auto sensor_to_tracking =
    tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  const auto odom_to_map =
    tf_bridge_.LookupToTracking(time, "map", "odom");

  if (odom_to_map != nullptr) {
    laser_perception_2d_->UpdateTransformFromOdomToMap(odom_to_map->cast<double>());
  }
  // perception_2d::sensor::PointCloudDataWithIntensities point_cloud_b;
  // if (sensor_to_tracking != nullptr) {
  //   point_cloud_b.intensities = points.intensities;
  //   point_cloud_b.points = perception_2d::sensor::TimedPointCloudData{
  //     time, sensor_to_tracking->cast<double>(),
  //     perception_2d::sensor::TransformTimedPointCloud(
  //       points.points, sensor_to_tracking->cast<float>())};
  //   laser_perception_2d_->AddSensorData(sensor_id, point_cloud_b);
  // }

  if(sensor_to_tracking != nullptr) {
    laser_perception_2d_->AddSensorData(sensor_id, time, *sensor_to_tracking,
                                        perception_2d::sensor::TimedPointCloudWithIntensities{
                                        perception_2d::sensor::TransformTimedPointCloud(
                                                                points.points,
                                                                ((*sensor_to_tracking)).cast<float>()),
                                                                points.intensities});
  }
}


}  // namespace perception_ros2
