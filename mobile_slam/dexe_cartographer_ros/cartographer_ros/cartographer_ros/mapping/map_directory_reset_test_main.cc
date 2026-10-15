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

#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

class TemporaryDirectory {
 public:
  TemporaryDirectory()
      : path_(fs::temp_directory_path() /
              ("cartographer_ros_map_directory_reset_test_" +
               std::to_string(getpid()))) {
    std::error_code error;
    fs::remove_all(path_, error);
    fs::create_directories(path_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(path_, error);
  }

  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

void WriteFile(const fs::path& path, const std::string& contents) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path);
  stream << contents;
}

bool TestCreatesNewMapDirectory(const fs::path& test_root) {
  const auto result = cartographer_ros::ResetMapDirectory(
      test_root / "new_root", "new_map");
  return Expect(result.success, result.error_message) &&
         Expect(!result.replaced_existing,
                "new map must not be reported as replaced") &&
         Expect(fs::is_directory(result.slam_directory),
                "slam directory was not created");
}

// use_trash=false：保留旧的硬删语义（供不需要回收站的场景用）
bool TestRemovesEntireExistingMapDirectory(const fs::path& test_root) {
  const fs::path map_root = test_root / "maps";
  const fs::path map_directory = map_root / "office";
  WriteFile(map_directory / "slam/map.pbstream", "old slam");
  WriteFile(map_directory / "assets/map_assets.json", "old assets");
  WriteFile(map_directory / "versions/old.json", "old manifest");
  WriteFile(map_directory / "slam/backups/old/map.yaml", "old backup");

  const auto result = cartographer_ros::ResetMapDirectory(map_root, "office",
                                                          /*use_trash=*/false);
  if (!Expect(result.success, result.error_message) ||
      !Expect(result.replaced_existing, "existing map was not detected") ||
      !Expect(result.removed_entries > 0, "existing entries were not removed") ||
      !Expect(result.trashed_directory.empty(),
              "use_trash=false must not archive") ||
      !Expect(fs::is_empty(result.slam_directory),
              "new slam directory must be empty") ||
      !Expect(!fs::exists(map_directory / "assets"),
              "stale assets directory remains") ||
      !Expect(!fs::exists(map_directory / "versions"),
              "stale versions directory remains")) {
    return false;
  }

  return Expect(std::distance(fs::directory_iterator(map_directory),
                              fs::directory_iterator()) == 1,
                "rebuilt map directory must contain only slam/");
}

// 默认（use_trash=true）：整目录归档到 .map_trash/，资产必须可恢复
bool TestArchivesExistingMapDirectoryToTrash(const fs::path& test_root) {
  const fs::path map_root = test_root / "maps_trash";
  const fs::path map_directory = map_root / "office";
  WriteFile(map_directory / "slam/map.pbstream", "old slam");
  WriteFile(map_directory / "assets/map_assets.json", "old assets");
  WriteFile(map_directory / "versions/old.json", "old manifest");

  const auto result = cartographer_ros::ResetMapDirectory(map_root, "office");
  if (!Expect(result.success, result.error_message) ||
      !Expect(result.replaced_existing, "existing map was not detected") ||
      !Expect(result.removed_entries == 0,
              "archiving must not report removed entries") ||
      !Expect(!result.trashed_directory.empty(),
              "trashed_directory must be set when archiving") ||
      !Expect(fs::is_empty(result.slam_directory),
              "new slam directory must be empty")) {
    return false;
  }

  // 新地图目录只剩 slam/
  if (!Expect(!fs::exists(map_directory / "assets"),
              "stale assets directory remains in the new map dir") ||
      !Expect(!fs::exists(map_directory / "versions"),
              "stale versions directory remains in the new map dir")) {
    return false;
  }

  // 关键：三类内容都在回收站里，可整目录恢复
  const fs::path trashed = result.trashed_directory;
  if (!Expect(trashed.parent_path().filename() ==
                  cartographer_ros::kMapTrashDirName,
              "trash entry must live under .map_trash/") ||
      !Expect(trashed.filename().string().rfind("office_", 0) == 0,
              "trash entry must be named <map_name>_<timestamp>") ||
      !Expect(fs::is_regular_file(trashed / "slam/map.pbstream"),
              "archived slam data is missing") ||
      !Expect(fs::is_regular_file(trashed / "assets/map_assets.json"),
              "archived assets are missing") ||
      !Expect(fs::is_regular_file(trashed / "versions/old.json"),
              "archived versions are missing")) {
    return false;
  }
  return true;
}

// 同秒内重复 overwrite 不能互相覆盖（自动化测试与连点都会触发）
bool TestTrashKeepsMultipleArchivesInSameSecond(const fs::path& test_root) {
  const fs::path map_root = test_root / "maps_trash_twice";
  for (int i = 0; i < 2; ++i) {
    WriteFile(map_root / "office/assets/a.json", "gen" + std::to_string(i));
    const auto result = cartographer_ros::ResetMapDirectory(map_root, "office");
    if (!Expect(result.success, result.error_message)) return false;
  }
  const fs::path trash = map_root / cartographer_ros::kMapTrashDirName;
  const auto count = std::distance(fs::directory_iterator(trash),
                                   fs::directory_iterator());
  return Expect(count == 2,
                "two overwrites in the same second must keep two archives");
}

bool TestRejectsUnsafeMapNames(const fs::path& test_root) {
  const std::vector<std::string> unsafe_names = {
      "", ".", "..", "../outside", "nested/map", "/absolute"};
  for (const auto& name : unsafe_names) {
    const auto result =
        cartographer_ros::ResetMapDirectory(test_root / "safe_root", name);
    if (!Expect(!result.success, "unsafe map name accepted: " + name)) {
      return false;
    }
  }
  return true;
}

bool TestRejectsNonDirectoryMapRoot(const fs::path& test_root) {
  const fs::path map_root = test_root / "map_root_file";
  WriteFile(map_root, "not a directory");
  const auto result =
      cartographer_ros::ResetMapDirectory(map_root, "office");
  return Expect(!result.success, "regular file accepted as map root") &&
         Expect(fs::is_regular_file(map_root), "map root file was modified");
}

}  // namespace

int main() {
  TemporaryDirectory temporary_directory;
  const fs::path& root = temporary_directory.path();

  const bool success = TestCreatesNewMapDirectory(root) &&
                       TestRemovesEntireExistingMapDirectory(root) &&
                       TestArchivesExistingMapDirectoryToTrash(root) &&
                       TestTrashKeepsMultipleArchivesInSameSecond(root) &&
                       TestRejectsUnsafeMapNames(root) &&
                       TestRejectsNonDirectoryMapRoot(root);
  if (!success) {
    return 1;
  }

  std::cout << "All map directory reset tests passed." << std::endl;
  return 0;
}
