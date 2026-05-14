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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_OPTION_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_OPTION_HPP_

#include <string>
#include "perception_2d/common/lua_parameter_dictionary.hpp"
#include "perception_2d/common/configuration_file_resolver.hpp"
#include "perception_2d/laser_perception_2d/config.hpp"
#include "perception_2d/laser_perception_2d/perception_processor.hpp"

namespace perception_ros2
{
struct SensorBrigeOptions
{
  int num_subdivisions_per_laser_scan;
  std::string tracking_frame;
};

struct NodeOptions
{
  SensorBrigeOptions sensor_bridge_options;
  perception_2d::PerceptionOptions perception_options;
  // perception_2d::Config config;
  bool use_odometry;
  int num_laser_scans;            // 激光数量；> 1 时配合 scan_topics 列出全部 topic
  double lookup_transform_timeout_sec;
};

NodeOptions CreateNodeOptions(
  ::perception_2d::common::LuaParameterDictionary * const
  lua_parameter_dictionary);

NodeOptions LoadOptions(
  const std::string & configuration_directory,
  const std::string & configuration_basename,
  const std::string & model_directory,
  const std::string & model_basename);


}  // namespace perception_ros2

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_OPTION_HPP_
