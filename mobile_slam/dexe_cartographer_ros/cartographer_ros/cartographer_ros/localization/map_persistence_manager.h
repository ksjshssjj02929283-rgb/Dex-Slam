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

#ifndef CARTOGRAPHER_ROS_LOCALIZATION_MAP_PERSISTENCE_MANAGER_H_
#define CARTOGRAPHER_ROS_LOCALIZATION_MAP_PERSISTENCE_MANAGER_H_

#include <chrono>
#include <filesystem>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "cartographer_ros/localization/atomic_dir_swap.h"
#include "cartographer_ros/localization/localization_node.h"

namespace cartographer_ros {

/**
 * @brief MapPersistenceManager
 *
 *   地图保存执行单元，采用崩溃安全的"staging + sentinel + 同卷 rename"流程，
 *   保存动作仅由显式手动触发（HandleSaveMap 服务 / StopLocalization 生命周期）。
 *
 *   核心流程：
 *     1. AtomicDirSwap::BeginStage() → 准备 <target>.staging/
 *     2. SerializeState(staging/map.pbstream)   ← 写 pbstream
 *     3. SaveMap(staging/map)                   ← 写 png + yaml
 *     4. AtomicDirSwap::CommitStage({pbstream,png,yaml}) → 写 sentinel + fsync
 *     5. AtomicDirSwap::ActivateAndCleanup()    → 切换 + 归档 + 轮换历史 backup
 *
 *   历史 backup 保留数量由 max_backups（默认 5）控制，通过构造参数注入。
 */
class MapPersistenceManager {
 public:
  /**
   * @param node            ROS2 节点（用于打日志）
   * @param get_loc_node    返回当前 LocalizationNode* 的 callable
   * @param get_map_paths   返回 {map_folder, map_name} 的 callable
   * @param max_backups     保留多少个历史 backup（默认 5）
   */
  MapPersistenceManager(
      rclcpp::Node* node,
      std::function<localization::LocalizationNode*()> get_loc_node,
      std::function<std::pair<std::string, std::string>()> get_map_paths,
      int max_backups = 5)
      : node_(node),
        get_loc_node_(std::move(get_loc_node)),
        get_map_paths_(std::move(get_map_paths)),
        max_backups_(max_backups < 0 ? 0 : max_backups) {
    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Initialized (manual-save only, "
                "max_backups=%d)", max_backups_);
  }

  ~MapPersistenceManager() = default;

  MapPersistenceManager(const MapPersistenceManager&) = delete;
  MapPersistenceManager& operator=(const MapPersistenceManager&) = delete;

  /**
   * @brief 立即执行一次崩溃安全保存。
   *        调用方须保证 localization_node 仍然存活
   *        （即在 FinishAllTrajectories() 之前调用）。
   * @return true 表示完整保存成功（含 commit + activate + 历史轮换）
   */
  bool SaveNow() {
    auto* loc = get_loc_node_();
    if (!loc) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] SaveNow: localization node is null");
      return false;
    }
    auto [map_folder, map_name] = get_map_paths_();
    if (map_folder.empty() || map_name.empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MapPersistenceManager] SaveNow: map_folder or map_name empty, skipping");
      return false;
    }

    const std::string target_dir = map_folder + "/" + map_name;
    AtomicDirSwap swap(target_dir, max_backups_);

    // Step 1: 准备 staging
    const std::string staging = swap.BeginStage();
    if (staging.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] BeginStage failed");
      return false;
    }

    // Step 2: 写文件到 staging
    const std::string pbstream_in_staging = staging + "/map.pbstream";
    const std::string image_base_in_staging = staging + "/map";  // 会生成 map.png + map.yaml(+pcd/line_map)
    if (!SavePbstream(loc, pbstream_in_staging)) return false;
    if (!SaveMapImage(loc, image_base_in_staging)) return false;

    // Step 2.5（已废弃）：chunk map 现已随 SavePbstream 整合进 staging/map.pbstream
    //   （见 LocalizationUpdater::SerializeStateToFile → io::WritePbStream chunk 参数），
    //   与建图侧一致，不再单独落盘 staging/chunks/。

    // Step 3: Commit（fsync + sentinel）。动态枚举 staging 下所有文件，
    //         全部纳入 SHA256 校验，确保三件套 + pcd/line 版本一致。
    namespace fs = std::filesystem;
    std::vector<std::string> commit_files;
    std::error_code ec_scan;
    for (auto& entry : fs::recursive_directory_iterator(staging, ec_scan)) {
      if (ec_scan) break;
      if (!entry.is_regular_file()) continue;
      const std::string rel = fs::relative(entry.path(), staging).string();
      if (rel == AtomicDirSwap::kSentinelName) continue;  // sentinel 自身不纳入
      commit_files.push_back(rel);
    }
    if (commit_files.empty()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] no files in staging to commit");
      return false;
    }
    if (!swap.CommitStage(commit_files)) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] CommitStage failed");
      return false;
    }

    // Step 4-5b: Activate + 历史归档 + 轮换
    if (!swap.ActivateAndCleanup()) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] ActivateAndCleanup failed");
      return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] SaveNow completed for target: %s",
                target_dir.c_str());
    return true;
  }

 private:
  bool SavePbstream(localization::LocalizationNode* loc,
                    const std::string& filename) {
    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Saving pbstream: %s", filename.c_str());
    const auto t0 = std::chrono::steady_clock::now();
    try {
      loc->SerializeState(filename, true /* include_unfinished_submaps */);
      const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - t0).count();
      RCLCPP_INFO(node_->get_logger(),
                  "[MapPersistenceManager] pbstream saved (took %ld ms)", ms);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] Failed to save pbstream: %s", e.what());
      return false;
    }
  }

  bool SaveMapImage(localization::LocalizationNode* loc,
                    const std::string& filestream) {
    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Saving map image: %s.png", filestream.c_str());
    try {
      const bool ok = loc->SaveMap(filestream, 0.05 /* resolution, unused */);
      if (ok) {
        RCLCPP_INFO(node_->get_logger(),
                    "[MapPersistenceManager] Map image saved: %s.png/.yaml",
                    filestream.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[MapPersistenceManager] SaveMap returned false for %s",
                    filestream.c_str());
      }
      return ok;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "[MapPersistenceManager] Failed to save map image: %s", e.what());
      return false;
    }
  }

  rclcpp::Node* node_;
  std::function<localization::LocalizationNode*()> get_loc_node_;
  std::function<std::pair<std::string, std::string>()> get_map_paths_;
  int max_backups_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LOCALIZATION_MAP_PERSISTENCE_MANAGER_H_
