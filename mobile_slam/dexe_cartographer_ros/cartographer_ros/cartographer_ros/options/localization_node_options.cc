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

#include "cartographer_ros/options/localization_node_options.h"

#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/localization/localization_updater.h"
#include "glog/logging.h"

namespace cartographer_ros {

LocalizationNodeOptions CreateLocalizationNodeOptions(
  ::cartographer::common::LuaParameterDictionary* const
    lua_parameter_dictionary) {
  LocalizationNodeOptions options;
  options.localization_updater_options =
    ::cartographer::localization::CreateLocalizationUpdaterOptions(
      lua_parameter_dictionary->GetDictionary("localization_updater").get());
  options.map_frame = lua_parameter_dictionary->GetString("map_frame");
  options.lookup_transform_timeout_sec =
    lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  options.submap_publish_period_sec =
    lua_parameter_dictionary->GetDouble("submap_publish_period_sec");
  options.pose_publish_period_sec =
    lua_parameter_dictionary->GetDouble("pose_publish_period_sec");
  options.trajectory_publish_period_sec =
    lua_parameter_dictionary->GetDouble("trajectory_publish_period_sec");
  if (lua_parameter_dictionary->HasKey("publish_to_tf")) {
  options.publish_to_tf =
    lua_parameter_dictionary->GetBool("publish_to_tf");
  }
  if (lua_parameter_dictionary->HasKey("publish_tracked_pose")) {
  options.publish_tracked_pose =
    lua_parameter_dictionary->GetBool("publish_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("use_pose_extrapolator")) {
  options.use_pose_extrapolator =
    lua_parameter_dictionary->GetBool("use_pose_extrapolator");
  }
  if (lua_parameter_dictionary->HasKey("save_tracked_pose")) {
    options.save_tracked_pose = lua_parameter_dictionary->GetBool("save_tracked_pose");
  }
  if (lua_parameter_dictionary->HasKey("tracked_pose_format")) {
    options.tracked_pose_format = lua_parameter_dictionary->GetString("tracked_pose_format");
  }
  if (lua_parameter_dictionary->HasKey("tracked_pose_directory")) {
    options.tracked_pose_directory = lua_parameter_dictionary->GetString("tracked_pose_directory");
  }
  if (lua_parameter_dictionary->HasKey("tracked_pose_filename")) {
    options.tracked_pose_filename = lua_parameter_dictionary->GetString("tracked_pose_filename");
  }

  if (lua_parameter_dictionary->HasKey("mapping_occ_thresh")) {
    options.mapping_occ_thresh = lua_parameter_dictionary->GetDouble("mapping_occ_thresh");
  }
  if (lua_parameter_dictionary->HasKey("mapping_free_thresh")) {
    options.mapping_free_thresh = lua_parameter_dictionary->GetDouble("mapping_free_thresh");
  }

  if (lua_parameter_dictionary->HasKey("save_navigation_map")) {
    options.save_navigation_map = lua_parameter_dictionary->GetBool("save_navigation_map");
  }
  if (lua_parameter_dictionary->HasKey("navigation_map_filter_mode")) {
    options.navigation_map_filter_mode = lua_parameter_dictionary->GetString("navigation_map_filter_mode");
  }
  if (lua_parameter_dictionary->HasKey("save_debug_map")) {
    options.save_debug_map = lua_parameter_dictionary->GetBool("save_debug_map");
  }

  return options;
}

std::tuple<LocalizationNodeOptions, TrajectoryOptions> LoadLocalizationNodeOptions(
  const std::string& configuration_directory,
  const std::string& configuration_basename) {
  auto file_resolver =
    absl::make_unique<cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
    file_resolver->GetFileContentOrDie(configuration_basename);
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
    code, std::move(file_resolver));

  return std::make_tuple(CreateLocalizationNodeOptions(&lua_parameter_dictionary),
             CreateTrajectoryOptions(&lua_parameter_dictionary));
}

}  // namespace cartographer_ros
