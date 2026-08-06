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

#include "cartographer_ros/mapping/map_directory.h"

#include <system_error>

namespace cartographer_ros {
namespace fs = std::filesystem;

bool ValidateMapName(const std::string& map_name,
                     std::string* error_message) {
  auto fail = [error_message](const std::string& message) {
    if (error_message != nullptr) {
      *error_message = message;
    }
    return false;
  };

  if (map_name.empty()) {
    return fail("map_name must not be empty");
  }
  if (map_name.find('\0') != std::string::npos) {
    return fail("map_name must not contain NUL characters");
  }

  const fs::path name_path(map_name);
  if (name_path.is_absolute() || name_path.has_root_path() ||
      name_path.has_parent_path() || name_path.filename() != name_path ||
      name_path == "." || name_path == "..") {
    return fail("map_name must be a single directory name");
  }

  if (error_message != nullptr) {
    error_message->clear();
  }
  return true;
}

MapDirectoryResetResult ResetMapDirectory(const fs::path& map_folder,
                                           const std::string& map_name) {
  MapDirectoryResetResult result;
  if (map_folder.empty()) {
    result.error_message = "map_folder must not be empty";
    return result;
  }
  if (!ValidateMapName(map_name, &result.error_message)) {
    return result;
  }

  result.map_directory = map_folder / map_name;
  result.slam_directory = result.map_directory / "slam";

  std::error_code error;
  fs::create_directories(map_folder, error);
  if (error) {
    result.error_message = "failed to create map_folder: " + error.message();
    return result;
  }
  if (!fs::is_directory(map_folder, error) || error) {
    result.error_message = "map_folder is not a directory";
    return result;
  }

  result.replaced_existing = fs::exists(result.map_directory, error);
  if (error) {
    result.error_message = "failed to inspect map directory: " + error.message();
    return result;
  }

  result.removed_entries = fs::remove_all(result.map_directory, error);
  if (error) {
    result.error_message = "failed to remove existing map directory: " +
                           error.message();
    return result;
  }

  fs::create_directories(result.slam_directory, error);
  if (error || !fs::is_directory(result.slam_directory)) {
    result.error_message = "failed to create clean slam directory";
    if (error) {
      result.error_message += ": " + error.message();
    }
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace cartographer_ros
