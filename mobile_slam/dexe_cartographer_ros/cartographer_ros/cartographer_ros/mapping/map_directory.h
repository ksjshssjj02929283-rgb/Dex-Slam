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
  std::uintmax_t removed_entries = 0;   // 仅硬删路径使用；归档路径恒为 0
  std::filesystem::path map_directory;
  std::filesystem::path slam_directory;
  // 旧地图目录被整体归档到的位置（启用回收站且确实替换了已有目录时非空）
  std::filesystem::path trashed_directory;
  std::string error_message;
};

// 地图回收站目录名（约定，与 map_manager 的 map_trash_dirname 默认值一致）。
// 「.」前缀让 map_manager 的目录扫描天然跳过它。
inline constexpr const char* kMapTrashDirName = ".map_trash";

bool ValidateMapName(const std::string& map_name,
                     std::string* error_message);

// Recreates <map_folder>/<map_name>/slam from an empty map directory.
//
// ⚠️ 旧实现对 <map_folder>/<map_name>/ 直接 fs::remove_all —— 连同
// assets/（点位 / 路网 / 禁区 / 房间）与 versions/（manifest）一起永久删除，
// 无备份、不可恢复。一次误传 overwrite=true 就能抹掉现场标注几小时的资产。
//
// 现在默认改为**整目录 rename 到回收站**：
//   <map_folder>/<map_name>/ → <map_folder>/.map_trash/<map_name>_<YYYYMMDD_HHMMSS>/
// 一次 rename 覆盖 slam + assets + versions 全部。回收站刻意放在 map_folder
// 内部：同挂载点下 rename 是原子且瞬时的；跨挂载点会退化成 copy + delete，
// 既慢又非原子（中途失败两边都残）。
// 回收站的轮换清理由 map_manager 负责（保留份数 + 总容量双判据）。
//
// 'use_trash=false' 保留旧的硬删行为，供不需要回收站的场景（如单测）使用。
MapDirectoryResetResult ResetMapDirectory(
    const std::filesystem::path& map_folder, const std::string& map_name,
    bool use_trash = true);

// 目录里是否有一张可加载的地图：<dir>/slam/map.yaml 或旧扁平布局的 <dir>/map.yaml。
// 与 map_manager 列地图的口径一致——取消 / 中断建图留下的空 slam/ 不算地图，
// 既不出现在 get_maps 里，也不应让 start_mapping 判为重名。
bool IsLoadableMapDirectory(const std::filesystem::path& map_directory);

struct MapDirectoryDiscardResult {
  bool success = false;
  bool restored_previous = false;  // 已把开始建图时归档的旧地图从回收站移回原位
  std::string error_message;
};

// 撤销一次建图会话的目录改动（取消建图用）：删除本次 ResetMapDirectory 新建的
// <map_folder>/<map_name>/（里面只有本次未保存的内容）；restore_previous 为真且
// 开始时归档过旧目录，则把它从回收站移回原位，相当于这次建图没有发生过。
// restore_previous 为假（旧目录只是残留、不是地图）时归档留在回收站，不再还原。
MapDirectoryDiscardResult DiscardMapDirectory(const MapDirectoryResetResult& session,
                                              bool restore_previous);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAPPING_MAP_DIRECTORY_H_
