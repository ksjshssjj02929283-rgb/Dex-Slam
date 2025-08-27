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

#include "cartographer_ros/mapping/sensor_bridge.h"
#include "cartographer/sensor/timed_point_cloud_data.h"

#include "absl/memory/memory.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/conversion/time_conversion.h"

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;

namespace {

const std::string& CheckNoLeadingSlash(const std::string& frame_id) {
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace


/**
 * @brief Construct a new Sensor Bridge:: Sensor Bridge object
 * 
 * @param num_subdivisions_per_laser_scan 雷达数据降采样（每隔几个采样一次）
 * @param ignore_out_of_order_messages 是否忽略乱序消息
 * @param tracking_frame 跟踪的坐标系（一般为base_link）
 * @param lookup_transform_timeout_sec 获取TF的超时时间
 * @param tf_buffer tf_buffer实例
 * @param trajectory_builder trajectory_builder实例
 */
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const bool ignore_out_of_order_messages, const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder)
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      ignore_out_of_order_messages_(ignore_out_of_order_messages),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) {}

std::unique_ptr<carto::sensor::OdometryData> SensorBridge::ToOdometryData(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  // noted by hz: 查询tracking_frame到里程计的child_frame_id的转换
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->child_frame_id));
  if (sensor_to_tracking == nullptr) {
    return nullptr;
  }
  carto::sensor::Twist twist;
  twist.linear = ToEigen(msg->twist.twist.linear);
  twist.angular = ToEigen(msg->twist.twist.angular);

  return absl::make_unique<carto::sensor::OdometryData>(
      carto::sensor::OdometryData{
          time, ToRigid3d(msg->pose.pose) * sensor_to_tracking->inverse(), twist});
}

/**
 * @brief 判断当前传感器乱序时是否要忽略该消息
 * 
 * @param sensor_id 传感器id
 * @param sensor_time 当前传感器数据时间
 * @return true 当前传感器时间小于上一次传感器时间时，返回真，即忽略该消息
 * @return false 否则返回假
 */
bool SensorBridge::IgnoreMessage(const std::string& sensor_id,
                                 cartographer::common::Time sensor_time) {
  if (!ignore_out_of_order_messages_) {
    return false;
  }
  // ignore_out_of_order_messages_ == true 忽视乱序消息
  auto it = latest_sensor_time_.find(sensor_id);
  if (it == latest_sensor_time_.end()) {
    return false;
  }
  // 当当前传感器时间小于上一次传感器时间时，返回真，即忽略该消息
  return sensor_time <= it->second;
}

void SensorBridge::HandleOdometryMessage(
    const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  std::unique_ptr<carto::sensor::OdometryData> odometry_data =
      ToOdometryData(msg);
  if (odometry_data != nullptr) {
    if (IgnoreMessage(sensor_id, odometry_data->time)) {
      LOG(WARNING) << "Ignored odometry message from sensor " << sensor_id
                   << " because sensor time " << odometry_data->time
                   << " is not before last odometry message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    latest_sensor_time_[sensor_id] = odometry_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::OdometryData{odometry_data->time, odometry_data->pose});
  }
}

void SensorBridge::HandleNavSatFixMessage(
    const std::string& sensor_id, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  const carto::common::Time time = FromRos(msg->header.stamp);
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::FixedFramePoseData{time, absl::optional<Rigid3d>()});
    return;
  }

  if (!ecef_to_local_frame_.has_value()) {
    ecef_to_local_frame_ =
        ComputeLocalFrameFromLatLong(msg->latitude, msg->longitude);
    LOG(INFO) << "Using NavSatFix. Setting ecef_to_local_frame with lat = "
              << msg->latitude << ", long = " << msg->longitude << ".";
  }

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::FixedFramePoseData{
                     time, absl::optional<Rigid3d>(Rigid3d::Translation(
                               ecef_to_local_frame_.value() *
                               LatLongAltToEcef(msg->latitude, msg->longitude,
                                                msg->altitude)))});
}

void SensorBridge::HandleLandmarkMessage(
    const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  auto landmark_data = ToLandmarkData(*msg);

  auto tracking_from_landmark_sensor = tf_bridge_.LookupToTracking(
      landmark_data.time, CheckNoLeadingSlash(msg->header.frame_id));
  if (tracking_from_landmark_sensor != nullptr) {
    for (auto& observation : landmark_data.landmark_observations) {
      observation.landmark_to_tracking_transform =
          *tracking_from_landmark_sensor *
          observation.landmark_to_tracking_transform;
    }
  }
  trajectory_builder_->AddSensorData(sensor_id, landmark_data);
}

/**
 * @brief 将IMU数据转换成carto的数据格式，并将IMU数据转换到tracking坐标系下
 * 
 * @param msg 
 * @return std::unique_ptr<carto::sensor::ImuData> 
 */
std::unique_ptr<carto::sensor::ImuData> SensorBridge::ToImuData(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  // 1. 检查IMU数据是否完成
  CHECK_NE(msg->linear_acceleration_covariance[0], -1)
      << "Your IMU data claims to not contain linear acceleration measurements "
         "by setting linear_acceleration_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";
  CHECK_NE(msg->angular_velocity_covariance[0], -1)
      << "Your IMU data claims to not contain angular velocity measurements "
         "by setting angular_velocity_covariance[0] to -1. Cartographer "
         "requires this data to work. See "
         "http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html.";

  const carto::common::Time time = FromRos(msg->header.stamp);
  // 2. 获取IMU到tracking坐标系下的变换
  const auto sensor_to_tracking = tf_bridge_.LookupToTracking(
      time, CheckNoLeadingSlash(msg->header.frame_id));
  if (sensor_to_tracking == nullptr) {
    LOG(WARNING) << "Could not find transform from "
                  << CheckNoLeadingSlash(msg->header.frame_id)
                  << " to tracking; Skipping IMU message.";
    return nullptr;
  }
  // 3. 没有获取到TF，警告需要提供TF
  CHECK(sensor_to_tracking->translation().norm() < 1e-5)
      << "The IMU frame must be colocated with the tracking frame. "
         "Transforming linear acceleration into the tracking frame will "
         "otherwise be imprecise.";
  // 4. 返回转换后的IMU数据
  return absl::make_unique<carto::sensor::ImuData>(carto::sensor::ImuData{
      time, sensor_to_tracking->rotation() * ToEigen(msg->linear_acceleration),
      sensor_to_tracking->rotation() * ToEigen(msg->angular_velocity)});
}

void SensorBridge::HandleImuMessage(const std::string& sensor_id,
                                    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  std::unique_ptr<carto::sensor::ImuData> imu_data = ToImuData(msg);
  if (imu_data != nullptr) {
    if (IgnoreMessage(sensor_id, imu_data->time)) {
      LOG(WARNING) << "Ignored IMU message from sensor " << sensor_id
                   << " because sensor time " << imu_data->time
                   << " is not before last IMU message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }
    latest_sensor_time_[sensor_id] = imu_data->time;
    trajectory_builder_->AddSensorData(
        sensor_id,
        carto::sensor::ImuData{imu_data->time, imu_data->linear_acceleration,
                               imu_data->angular_velocity});
  }
}

void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleMultiEchoLaserScanMessage(
    const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandlePointCloud2Message(
    const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleRangefinder(sensor_id, time, msg->header.frame_id, point_cloud);
}

const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

void SensorBridge::HandleLaserScan(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id,
    const carto::sensor::PointCloudWithIntensities& points) {
  if (points.points.empty()) {
    return;
  }
  CHECK_LE(points.points.back().time, 0.f);
  // 将激光雷达数据分成几个部分，每个部分单独进行跟踪
  // TODO(gaschler): Use per-point time instead of subdivisions.
  // num_subdivisions_per_laser_scan_代表一帧激光分批处理的批次
  for (int i = 0; i != num_subdivisions_per_laser_scan_; ++i) {
    const size_t start_index =
        points.points.size() * i / num_subdivisions_per_laser_scan_;
    const size_t end_index =
        points.points.size() * (i + 1) / num_subdivisions_per_laser_scan_;
    carto::sensor::PointCloudWithIntensities subdivision{
      carto::sensor::TimedPointCloud(points.points.begin() + start_index,
                                    points.points.begin() + end_index),
      std::vector<float>(points.intensities.begin() + start_index,
                         points.intensities.begin() + end_index)};
    if (start_index == end_index) {
      continue;
    }
    const double time_to_subdivision_end = subdivision.points.back().time;
    // `subdivision_time` is the end of the measurement so sensor::Collator will
    // send all other sensor data first.
    const carto::common::Time subdivision_time =
        time + carto::common::FromSeconds(time_to_subdivision_end);
    auto it = sensor_to_previous_subdivision_time_.find(sensor_id);
    if (it != sensor_to_previous_subdivision_time_.end() &&
        it->second >= subdivision_time) {
      LOG(WARNING) << "Ignored subdivision of a LaserScan message from sensor "
                   << sensor_id << " because previous subdivision time "
                   << it->second << " is not before current subdivision time "
                   << subdivision_time;
      continue;
    }
    sensor_to_previous_subdivision_time_[sensor_id] = subdivision_time;
    for (auto& point : subdivision.points) {
      point.time -= time_to_subdivision_end;
    }
    CHECK_EQ(subdivision.points.back().time, 0.f);
    HandleRangefinder(sensor_id, subdivision_time, frame_id, subdivision);
  }
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::PointCloudWithIntensities& ranges) {
  if (!ranges.points.empty()) {
    CHECK_LE(ranges.points.back().time, 0.f);
  }
  // 获取雷达外参
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));
  if (sensor_to_tracking != nullptr) {
    if (IgnoreMessage(sensor_id, time)) {
      LOG(WARNING) << "Ignored Rangefinder message from sensor " << sensor_id
                   << " because sensor time " << time
                   << " is not before last Rangefinder message time "
                   << latest_sensor_time_[sensor_id];
      return;
    }

    // 去掉外参高度的影响
    const Eigen::Vector3d trans_refined = Eigen::Vector3d(
                                                  sensor_to_tracking->translation().x(),
                                                  sensor_to_tracking->translation().y(),
                                                  0.0);
    auto sensor_to_tracking_refined = Rigid3d(trans_refined, sensor_to_tracking->rotation());

    latest_sensor_time_[sensor_id] = time;
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                        time, sensor_to_tracking_refined.translation().cast<float>(),
                        carto::transform::Rigid3d::Identity(),
                       carto::sensor::TransformTimedPointCloud(
                           ranges.points, sensor_to_tracking_refined.cast<float>()),
                        ranges.intensities});
  }
}

}  // namespace cartographer_ros
