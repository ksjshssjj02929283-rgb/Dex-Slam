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
  std::string han67;
  for (int i = 0; i < 67; ++i) han67 += "图";
  const std::vector<std::string> unsafe_names = {
      "", ".", "..", "../outside", "nested/map", "/absolute",
      // 统一地图名规则：超长、隐藏 / 选项样式、首尾空白（含全角）、不可见字符、
      // Windows 与 shell 敏感字符、非法 UTF-8、控制字符
      han67, ".hidden", "-opt", " lead", "trail ", "全角\xE3\x80\x80",
      "零\xE2\x80\x8B宽", "a\\b", "a:b", "a*b", "a?b", "a\"b", "a<b", "a>b",
      "a|b", "a'b", "a$b", "a`b", "bad\xFF", "line\nbreak", "\xC0\xAF"};
  for (const auto& name : unsafe_names) {
    const auto result =
        cartographer_ros::ResetMapDirectory(test_root / "safe_root", name);
    if (!Expect(!result.success, "unsafe map name accepted: " + name)) {
      return false;
    }
  }
  return true;
}

// 中文名能建；66 个汉字（198 字节）能建，覆盖建图时旧图能进回收站（条目名 +16 字节仍在 255 内）
bool TestAcceptsChineseMapNames(const fs::path& test_root) {
  const fs::path map_root = test_root / "chinese_root";
  std::string han66;
  for (int i = 0; i < 66; ++i) han66 += "图";
  for (const std::string& name : {std::string("一楼大厅"), std::string("Floor 1 东区"), han66}) {
    auto first = cartographer_ros::ResetMapDirectory(map_root, name);
    if (!Expect(first.success, "Chinese map name rejected: " + name + " (" + first.error_message + ")")) {
      return false;
    }
    WriteFile(first.slam_directory / "map.yaml", "image: map.png\n");
    auto second = cartographer_ros::ResetMapDirectory(map_root, name);
    if (!Expect(second.success && !second.trashed_directory.empty() &&
                    fs::exists(second.trashed_directory / "slam" / "map.yaml"),
                "overwrite of Chinese map must archive the old one: " + name + " (" +
                    second.error_message + ")")) {
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

// 可加载判定与 map_manager 列地图同口径：只有 slam/map.yaml 或扁平 map.yaml 才算地图
bool TestIsLoadableMapDirectory(const fs::path& test_root) {
  const fs::path root = test_root / "loadable";
  fs::create_directories(root / "leftover/slam");
  WriteFile(root / "layered/slam/map.yaml", "image: map.png");
  WriteFile(root / "flat/map.yaml", "image: map.png");
  WriteFile(root / "pbstream_only/slam/map.pbstream", "x");
  return Expect(!cartographer_ros::IsLoadableMapDirectory(root / "leftover"),
                "empty slam/ left by a cancelled session must not count as a map") &&
         Expect(!cartographer_ros::IsLoadableMapDirectory(root / "pbstream_only"),
                "pbstream without map.yaml is not loadable") &&
         Expect(!cartographer_ros::IsLoadableMapDirectory(root / "missing"),
                "missing directory is not a map") &&
         Expect(cartographer_ros::IsLoadableMapDirectory(root / "layered"),
                "slam/map.yaml must count as a map") &&
         Expect(cartographer_ros::IsLoadableMapDirectory(root / "flat"),
                "flat map.yaml must count as a map");
}

// 新名字建图后取消：目录整个删掉，同名可以再建
bool TestDiscardFreshSession(const fs::path& test_root) {
  const fs::path map_root = test_root / "discard_fresh";
  const auto session = cartographer_ros::ResetMapDirectory(map_root, "t3");
  if (!Expect(session.success, session.error_message)) return false;
  const auto discard = cartographer_ros::DiscardMapDirectory(session, /*restore_previous=*/false);
  return Expect(discard.success, discard.error_message) &&
         Expect(!discard.restored_previous, "nothing to restore for a fresh session") &&
         Expect(!fs::exists(map_root / "t3"), "unsaved map directory must be removed") &&
         Expect(!fs::exists(map_root / cartographer_ros::kMapTrashDirName),
                "a fresh session must not leave anything in the trash");
}

// overwrite=true 覆盖已有地图后取消：旧地图（slam + assets）原样回到原位，回收站不留这份
bool TestDiscardRestoresPreviousMap(const fs::path& test_root) {
  const fs::path map_root = test_root / "discard_restore";
  const fs::path map_directory = map_root / "office";
  WriteFile(map_directory / "slam/map.yaml", "old yaml");
  WriteFile(map_directory / "slam/map.pbstream", "old slam");
  WriteFile(map_directory / "assets/map_assets.json", "old assets");
  const auto session = cartographer_ros::ResetMapDirectory(map_root, "office");
  if (!Expect(session.success, session.error_message) ||
      !Expect(!session.trashed_directory.empty(), "previous map must be archived")) {
    return false;
  }
  WriteFile(session.slam_directory / "map.pbstream", "half-written new session");
  const auto discard = cartographer_ros::DiscardMapDirectory(session, /*restore_previous=*/true);
  std::ifstream yaml(map_directory / "slam/map.yaml");
  std::string yaml_text((std::istreambuf_iterator<char>(yaml)), std::istreambuf_iterator<char>());
  std::ifstream pb(map_directory / "slam/map.pbstream");
  std::string pb_text((std::istreambuf_iterator<char>(pb)), std::istreambuf_iterator<char>());
  return Expect(discard.success, discard.error_message) &&
         Expect(discard.restored_previous, "previous map must be reported as restored") &&
         Expect(yaml_text == "old yaml" && pb_text == "old slam",
                "restored map must be the original content, not the cancelled session") &&
         Expect(fs::is_regular_file(map_directory / "assets/map_assets.json"),
                "restored map must keep its assets") &&
         Expect(!fs::exists(session.trashed_directory), "restored archive must leave the trash");
}

// 残留目录（不是地图）被归档后取消：不还原残留，删掉本次目录即可
bool TestDiscardLeftoverIsNotRestored(const fs::path& test_root) {
  const fs::path map_root = test_root / "discard_leftover";
  fs::create_directories(map_root / "t3/slam");
  const auto session = cartographer_ros::ResetMapDirectory(map_root, "t3");
  if (!Expect(session.success, session.error_message)) return false;
  const auto discard = cartographer_ros::DiscardMapDirectory(session, /*restore_previous=*/false);
  return Expect(discard.success, discard.error_message) &&
         Expect(!discard.restored_previous, "leftover must not be restored") &&
         Expect(!fs::exists(map_root / "t3"), "map directory must be gone after cancel");
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
                       TestAcceptsChineseMapNames(root) &&
                       TestRejectsNonDirectoryMapRoot(root) &&
                       TestIsLoadableMapDirectory(root) &&
                       TestDiscardFreshSession(root) &&
                       TestDiscardRestoresPreviousMap(root) &&
                       TestDiscardLeftoverIsNotRestored(root);
  if (!success) {
    return 1;
  }

  std::cout << "All map directory reset tests passed." << std::endl;
  return 0;
}
