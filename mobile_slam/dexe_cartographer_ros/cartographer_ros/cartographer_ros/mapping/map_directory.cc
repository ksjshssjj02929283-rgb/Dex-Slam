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

#include <ctime>
#include <string>
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

namespace {

// 回收站条目名后缀：<map_name>_<YYYYMMDD_HHMMSS>。同秒重复时由调用处加序号。
std::string CurrentTrashStamp() {
  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(ts);
}

}  // namespace

MapDirectoryResetResult ResetMapDirectory(const fs::path& map_folder,
                                           const std::string& map_name,
                                           const bool use_trash) {
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

  if (result.replaced_existing && use_trash) {
    // 整目录 rename 到回收站：一次操作覆盖 slam + assets + versions。
    const fs::path trash_root = map_folder / kMapTrashDirName;
    fs::create_directories(trash_root, error);
    if (error) {
      result.error_message =
          "failed to create map trash directory " + trash_root.string() + ": " +
          error.message();
      return result;
    }
    const std::string stamp = CurrentTrashStamp();
    fs::path dst = trash_root / (map_name + "_" + stamp);
    // 同秒内重复 overwrite（自动化测试常见）时追加序号，不覆盖上一份。
    for (int suffix = 2; fs::exists(dst, error) && suffix < 1000; ++suffix) {
      dst = trash_root / (map_name + "_" + stamp + "_" + std::to_string(suffix));
    }
    fs::rename(result.map_directory, dst, error);
    if (error) {
      // 典型原因：回收站与地图目录不在同一挂载点（rename 跨设备失败）。
      // 不静默退回硬删——那等于把「资产会被保留」的承诺悄悄撤销。
      result.error_message =
          "failed to move existing map directory to trash (" + dst.string() +
          "): " + error.message() +
          ". If map_folder and the trash dir are on different filesystems, "
          "rename cannot be atomic; fix the layout or pass use_trash=false "
          "to accept permanent deletion.";
      return result;
    }
    result.trashed_directory = dst;
    result.removed_entries = 0;
  } else {
    result.removed_entries = fs::remove_all(result.map_directory, error);
    if (error) {
      result.error_message = "failed to remove existing map directory: " +
                             error.message();
      return result;
    }
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
