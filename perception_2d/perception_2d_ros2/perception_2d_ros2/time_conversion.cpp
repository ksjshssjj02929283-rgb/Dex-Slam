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

#include "perception_2d_ros2/time_conversion.hpp"

// #include "perception_2d/common/time.hpp"
// 

namespace perception_ros2
{

::rclcpp::Time ToRos(perception_2d::common::Time time)
{
  int64_t uts_timestamp = 
    perception_2d::common::ToUniversal(time);
  int64_t ns_since_unix_epoch =
    (uts_timestamp -
    perception_2d::common::kUtsEpochOffsetFromUnixEpochInSeconds *
    10000000ll) *
    100ll;
  return ::rclcpp::Time(ns_since_unix_epoch, RCL_ROS_TIME);
}

// // TODO(pedrofernandez): Write test.
perception_2d::common::Time
  FromRos(const ::rclcpp::Time & time)
{
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return ::perception_2d::common::FromUniversal(
    (time.nanoseconds() / 1000000000 +
    ::perception_2d::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
    10000000ll +
    ((time.nanoseconds() % 1000000000) + 50) / 100);    // + 50 to get the rounding correct.
}

}  // namespace perception_ros2
