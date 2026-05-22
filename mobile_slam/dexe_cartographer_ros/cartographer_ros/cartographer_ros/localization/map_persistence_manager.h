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
#include <functional>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "cartographer_ros/localization/localization_node.h"

namespace cartographer_ros {

/**
 * @brief MapPersistenceManager
 *
 *   地图保存执行单元：将 LocalizationNode 当前的 pbstream 状态和图像 (map.png/yaml)
 *   写入磁盘。仅支持显式手动触发（HandleSaveMap 服务或 StopLocalization
 *   生命周期），不再含任何 timer / odom / 静止检测的自动保存路径。
 *
 *   生命周期：
 *     - 由 LocalizationManagerNode 在 StartLocalization() 成功后创建。
 *     - 在 StopLocalization() 中先调用 SaveNow()，再 reset()。
 *     - 持有对 localization_node 的非拥有引用（通过 get_loc_node_ callback）。
 *
 *   后续 Phase 3 会在 SaveNow() 内部引入 staging+sentinel+rename 崩溃安全机制。
 */
class MapPersistenceManager {
 public:
  /**
   * @param node            ROS2 节点（用于打日志）
   * @param get_loc_node    返回当前 LocalizationNode* 的 callable；可返回 nullptr
   * @param get_map_paths   返回 {map_folder, map_name} 的 callable
   */
  MapPersistenceManager(
      rclcpp::Node* node,
      std::function<localization::LocalizationNode*()> get_loc_node,
      std::function<std::pair<std::string, std::string>()> get_map_paths)
      : node_(node),
        get_loc_node_(std::move(get_loc_node)),
        get_map_paths_(std::move(get_map_paths)) {
    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Initialized (manual-save only)");
  }

  ~MapPersistenceManager() = default;

  MapPersistenceManager(const MapPersistenceManager&) = delete;
  MapPersistenceManager& operator=(const MapPersistenceManager&) = delete;

  /**
   * @brief 立即执行一次保存（pbstream + map.png/yaml）。
   *        调用方须保证 localization_node 仍然存活
   *        （即在 FinishAllTrajectories() 之前调用）。
   * @return true 表示全部保存成功
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
                  "[MapPersistenceManager] SaveNow: map_folder or map_name is empty, skipping");
      return false;
    }
    bool ok = true;
    ok &= SavePbstream(loc, map_folder, map_name);
    ok &= SaveMapImage(loc, map_folder, map_name);
    return ok;
  }

 private:
  bool SavePbstream(localization::LocalizationNode* loc,
                    const std::string& map_folder,
                    const std::string& map_name) {
    const std::string filename = map_folder + "/" + map_name + "/map.pbstream";
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
                    const std::string& map_folder,
                    const std::string& map_name) {
    const std::string filestream = map_folder + "/" + map_name + "/map";
    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Saving map image: %s.png", filestream.c_str());
    try {
      const bool ok = loc->SaveMap(filestream, 0.05 /* resolution, unused */);
      if (ok) {
        RCLCPP_INFO(node_->get_logger(),
                    "[MapPersistenceManager] Map image saved: %s.png/.yaml", filestream.c_str());
      } else {
        RCLCPP_WARN(node_->get_logger(),
                    "[MapPersistenceManager] SaveMap returned false for %s (may be non-fatal)",
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
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LOCALIZATION_MAP_PERSISTENCE_MANAGER_H_
