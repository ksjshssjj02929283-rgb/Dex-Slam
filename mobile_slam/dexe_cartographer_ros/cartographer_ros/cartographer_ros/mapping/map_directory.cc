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

#include <cstdint>
#include <cstring>
#include <ctime>
#include <string>
#include <system_error>
#include <vector>

namespace cartographer_ros {
namespace fs = std::filesystem;

namespace {

// 解码一个合法的 UTF-8 码点；非法（截断、续字节错、超长编码、代理区、超出范围）返回 false
bool NextCodePoint(const std::string& s, size_t* i, uint32_t* cp) {
  const unsigned char c = static_cast<unsigned char>(s[*i]);
  int len = 0;
  if (c < 0x80) { *cp = c; *i += 1; return true; }
  if ((c & 0xE0) == 0xC0) { len = 2; *cp = c & 0x1F; }
  else if ((c & 0xF0) == 0xE0) { len = 3; *cp = c & 0x0F; }
  else if ((c & 0xF8) == 0xF0) { len = 4; *cp = c & 0x07; }
  else return false;
  if (*i + len > s.size()) return false;
  for (int k = 1; k < len; ++k) {
    const unsigned char cc = static_cast<unsigned char>(s[*i + k]);
    if ((cc & 0xC0) != 0x80) return false;
    *cp = (*cp << 6) | (cc & 0x3F);
  }
  if ((len == 2 && *cp < 0x80) || (len == 3 && *cp < 0x800) || (len == 4 && *cp < 0x10000)) return false;
  if ((*cp >= 0xD800 && *cp <= 0xDFFF) || *cp > 0x10FFFF) return false;
  *i += len;
  return true;
}

}  // namespace

// 与 map_manager 的 map_package::ValidateNewMapName 同一套规则（建图、改名、恢复、导入都按它）：
//   合法 UTF-8；不超过 kMaxMapNameBytes 字节；不是 . / ..，不以 . 或 - 开头；
//   首尾没有空格（含全角空格、不换行空格）；不含控制字符和不可见字符；
//   不含 / \ : * ? " < > | ' $ 和反引号（路径分隔、Windows 文件名、脚本引号与 shell 展开）
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
  if (map_name.size() > kMaxMapNameBytes) {
    return fail("map_name is too long: " + std::to_string(map_name.size()) +
                " bytes (limit " + std::to_string(kMaxMapNameBytes) +
                " bytes, about 66 Chinese characters)");
  }
  if (map_name == "." || map_name == "..") {
    return fail("map_name must not be '.' or '..'");
  }
  if (map_name[0] == '.' || map_name[0] == '-') {
    return fail("map_name must not start with '.' or '-'");
  }
  std::vector<uint32_t> code_points;
  for (size_t i = 0; i < map_name.size();) {
    uint32_t cp = 0;
    if (!NextCodePoint(map_name, &i, &cp)) {
      return fail("map_name is not valid UTF-8");
    }
    code_points.push_back(cp);
  }
  for (const uint32_t cp : code_points) {
    if (cp < 0x20 || cp == 0x7F || (cp >= 0x80 && cp <= 0x9F)) {
      return fail("map_name contains control characters");
    }
    if ((cp >= 0x200B && cp <= 0x200F) || (cp >= 0x202A && cp <= 0x202E) ||
        (cp >= 0x2060 && cp <= 0x2064) || cp == 0x2028 || cp == 0x2029 ||
        cp == 0xFEFF) {
      return fail("map_name contains invisible characters (zero-width / direction marks)");
    }
    if (cp < 0x80 && std::strchr("/\\:*?\"<>|'$`", static_cast<int>(cp)) != nullptr) {
      return fail("map_name must not contain any of / \\ : * ? \" < > | ' $ `");
    }
  }
  auto is_space = [](uint32_t cp) { return cp == 0x20 || cp == 0xA0 || cp == 0x3000; };
  if (is_space(code_points.front()) || is_space(code_points.back())) {
    return fail("map_name must not start or end with a space (including full-width space)");
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

bool IsLoadableMapDirectory(const fs::path& map_directory) {
  std::error_code error;
  return fs::is_regular_file(map_directory / "slam" / "map.yaml", error) ||
         fs::is_regular_file(map_directory / "map.yaml", error);
}

MapDirectoryDiscardResult DiscardMapDirectory(const MapDirectoryResetResult& session,
                                              const bool restore_previous) {
  MapDirectoryDiscardResult result;
  if (session.map_directory.empty()) {
    result.success = true;
    return result;
  }
  std::error_code error;
  fs::remove_all(session.map_directory, error);
  if (error) {
    result.error_message = "failed to remove unsaved map directory " +
                           session.map_directory.string() + ": " + error.message();
    return result;
  }
  if (restore_previous && !session.trashed_directory.empty()) {
    if (!fs::is_directory(session.trashed_directory, error)) {
      result.error_message = "archived previous map is missing: " +
                             session.trashed_directory.string();
      return result;
    }
    fs::rename(session.trashed_directory, session.map_directory, error);
    if (error) {
      result.error_message = "failed to restore previous map from " +
                             session.trashed_directory.string() + ": " + error.message();
      return result;
    }
    result.restored_previous = true;
  }
  result.success = true;
  return result;
}

}  // namespace cartographer_ros
