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
#include <utility>
#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include "perception_2d_ros2/node_option.hpp"
#include "perception_2d/laser_perception_2d/laser_perception_2d.hpp"

namespace perception_ros2
{

bool  CreateMapBuilderOptions(
  ::perception_2d::common::LuaParameterDictionary * const parameter_dictionary)
{
  bool use_pre = parameter_dictionary->GetBool("use_pre");
  return use_pre;
}

NodeOptions CreateNodeOptions(
  ::perception_2d::common::LuaParameterDictionary * const
  lua_parameter_dictionary,
  const std::string & model_file_path)
{
  NodeOptions options;
  LOG(INFO) << "start get lua";
  options.perception_options.config = perception_2d::CreateLaserPerception2DOption(
    lua_parameter_dictionary->GetDictionary("laser_perception_2d").get(),
    model_file_path);
  options.use_odometry = lua_parameter_dictionary->GetBool("use_odometry");
  options.num_laser_scans = lua_parameter_dictionary->GetInt("num_laser_scans");
  options.sensor_bridge_options.num_subdivisions_per_laser_scan =
    lua_parameter_dictionary->GetInt("num_subdivisions_per_laser_scan");
  options.sensor_bridge_options.tracking_frame = lua_parameter_dictionary->GetString(
    "tracking_frame");
  options.lookup_transform_timeout_sec =
    lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");

  if (options.use_odometry) {
    LOG(INFO) << "load successful";
  }

  LOG(INFO) << "return options";
  return options;
}

// std::tuple<NodeOptions, TrajectoryOptions> LoadOptions(
NodeOptions LoadOptions(
  const std::string & configuration_directory,
  const std::string & configuration_basename,
  const std::string & model_directory,
  const std::string & model_basename)
{
  auto file_resolver =
    std::make_unique<perception_2d::common::ConfigurationFileResolver>(
    std::vector<std::string>{configuration_directory});
  const std::string code =
    file_resolver->GetFileContentOrDie(configuration_basename);
  perception_2d::common::LuaParameterDictionary lua_parameter_dictionary(
    code, std::move(file_resolver));
  const std::string model_file_path = model_directory + "/" + model_basename;

  return CreateNodeOptions(&lua_parameter_dictionary, model_file_path);

//   return std::make_tuple(CreateNodeOptions(&lua_parameter_dictionary),
//                          CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace perception_ros2
