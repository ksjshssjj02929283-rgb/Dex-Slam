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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_LOCALIZATION_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_LOCALIZATION_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/localization/proto/localization_updater_options.pb.h"
#include "cartographer_ros/options/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct LocalizationNodeOptions {
  ::cartographer::localization::proto::LocalizationUpdaterOptions localization_updater_options;
  std::string map_frame;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
  double trajectory_publish_period_sec;

  double gridmap_publish_period_sec;
  double linemap_publish_period_sec;
  double cloudmap_publish_period_sec;

  bool publish_to_tf = true;
  bool publish_tracked_pose = false;
  bool use_pose_extrapolator = true;

  bool save_tracked_pose = false;
  std::string tracked_pose_format = "tum";
  std::string tracked_pose_directory = "";
  std::string tracked_pose_filename = "tracked_pose.txt";

  float mapping_occ_thresh = 0.65;
  float mapping_free_thresh = 0.196;
};

LocalizationNodeOptions CreateLocalizationNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<LocalizationNodeOptions, TrajectoryOptions> LoadLocalizationNodeOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_LOCALIZATION_NODE_OPTIONS_H
