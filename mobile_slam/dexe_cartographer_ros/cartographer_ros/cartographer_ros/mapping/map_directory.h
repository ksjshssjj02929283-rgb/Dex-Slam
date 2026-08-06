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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAPPING_MAP_DIRECTORY_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAPPING_MAP_DIRECTORY_H_

#include <cstdint>
#include <filesystem>
#include <string>

namespace cartographer_ros {

struct MapDirectoryResetResult {
  bool success = false;
  bool replaced_existing = false;
  std::uintmax_t removed_entries = 0;
  std::filesystem::path map_directory;
  std::filesystem::path slam_directory;
  std::string error_message;
};

bool ValidateMapName(const std::string& map_name,
                     std::string* error_message);

// Recreates <map_folder>/<map_name>/slam from an empty map directory.
MapDirectoryResetResult ResetMapDirectory(
    const std::filesystem::path& map_folder, const std::string& map_name);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAPPING_MAP_DIRECTORY_H_
