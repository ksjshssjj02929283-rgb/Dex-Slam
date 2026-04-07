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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__TF_BRIDGE_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__TF_BRIDGE_HPP_

#include <map>
#include <memory>
#include <string>

#include "perception_2d/transform/rigid_transform.hpp"
#include "tf2_ros/buffer.h"

#include "perception_2d_ros2/time_conversion.hpp"

namespace perception_ros2
{

class TfBridge
{
public:
  TfBridge(
    const std::string & tracking_frame,
    double lookup_transform_timeout_sec, const tf2_ros::Buffer * buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge &) = delete;
  TfBridge & operator=(const TfBridge &) = delete;

  // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time'.
  std::unique_ptr<perception_2d::transform::Rigid3d> LookupToTracking(
    perception_2d::common::Time time,
    const std::string & frame_id) const;

  std::unique_ptr<perception_2d::transform::Rigid3d> LookupToTracking(
    perception_2d::common::Time time,
    const std::string & parent_frame_id, const std::string & child_frame_id) const;

private:
  const std::string tracking_frame_;
  const double lookup_transform_timeout_sec_;
  const tf2_ros::Buffer * const buffer_;
};

}  // namespace perception_ros2

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__TF_BRIDGE_HPP_
