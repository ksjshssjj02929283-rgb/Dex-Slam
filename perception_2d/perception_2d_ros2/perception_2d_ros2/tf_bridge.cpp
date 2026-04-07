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

// #include "absl/memory/memory.h"
#include <memory>
#include <string>
#include "perception_2d_ros2/msg_conversion.hpp"
#include "perception_2d_ros2/tf_bridge.hpp"

namespace perception_ros2
{

TfBridge::TfBridge(
  const std::string & tracking_frame,
  const double lookup_transform_timeout_sec,
  const tf2_ros::Buffer * buffer)
: tracking_frame_(tracking_frame),
  lookup_transform_timeout_sec_(lookup_transform_timeout_sec),
  buffer_(buffer) {}

std::unique_ptr<perception_2d::transform::Rigid3d> TfBridge::LookupToTracking(
  const perception_2d::common::Time time,
  const std::string & frame_id
  // ::perception_2d::transform::Rigid3d& pose
) const
{
  auto timeout = rclcpp::Duration::from_seconds(lookup_transform_timeout_sec_);
  std::unique_ptr<perception_2d::transform::Rigid3d> frame_id_to_tracking;
  try {
    const ::rclcpp::Time latest_tf_time =
      buffer_
      ->lookupTransform(
      tracking_frame_, frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME),
      timeout)
      .header.stamp;
    const ::rclcpp::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = rclcpp::Duration::from_seconds(0.0);
    }

    // pose = ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
    //                                           requested_time, timeout));
    // return true;
    return std::make_unique<perception_2d::transform::Rigid3d>(
      ToRigid3d(
        buffer_->lookupTransform(
          tracking_frame_, frame_id,
          requested_time, timeout)));
  } catch (const tf2::TransformException & ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

std::unique_ptr<perception_2d::transform::Rigid3d> TfBridge::LookupToTracking(
  perception_2d::common::Time time,
  const std::string & parent_frame_id, const std::string & child_frame_id) const
{
  auto timeout = rclcpp::Duration::from_seconds(lookup_transform_timeout_sec_);
  std::unique_ptr<perception_2d::transform::Rigid3d> frame_id_to_tracking;
  try {
    const ::rclcpp::Time latest_tf_time =
      buffer_
      ->lookupTransform(
      parent_frame_id, child_frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME),
      timeout)
      .header.stamp;
    const ::rclcpp::Time requested_time = ToRos(time);
    if (latest_tf_time >= requested_time) {
      // We already have newer data, so we do not wait. Otherwise, we would wait
      // for the full 'timeout' even if we ask for data that is too old.
      timeout = rclcpp::Duration::from_seconds(0.0);
    }

    // pose = ToRigid3d(buffer_->lookupTransform(tracking_frame_, frame_id,
    //                                           requested_time, timeout));
    // return true;
    return std::make_unique<perception_2d::transform::Rigid3d>(
      ToRigid3d(
        buffer_->lookupTransform(
          parent_frame_id, child_frame_id,
          requested_time, timeout)));
  } catch (const tf2::TransformException & ex) {
    LOG(WARNING) << ex.what();
  }
  return nullptr;
}

}  // namespace perception_ros2
