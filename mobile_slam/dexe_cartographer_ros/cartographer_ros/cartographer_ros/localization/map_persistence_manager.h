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

#include <atomic>
#include <chrono>
#include <functional>
#include <string>
#include <utility>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "cartographer_ros/localization/localization_node.h"
#include "gflags/gflags.h"

// ---------------------------------------------------------------------------
// GFlag 声明（定义位于 loc_node_main.cc）
// 其他编译单元若需使用，只需 DECLARE_bool/DECLARE_double。
// ---------------------------------------------------------------------------
DECLARE_bool(enable_auto_save_map);
DECLARE_double(auto_save_interval_s);
DECLARE_double(auto_save_linear_velocity_threshold);
DECLARE_double(auto_save_angular_velocity_threshold);

namespace cartographer_ros {

/**
 * @brief MapPersistenceManager
 *
 *   统一管理地图的 pbstream 序列化和图像保存逻辑。
 *
 *   定时检查策略（两层设计）：
 *     - 外层：每 kCheckIntervalS 秒执行一次检查（短周期，默认 30 s）
 *     - 触发条件：距上次成功保存已超过 FLAGS_auto_save_interval_s 秒，
 *                 且当前机器人线速度和角速度均低于阈值（静止）
 *     - 这样机器人运动时只是跳过当次检查，静止后最多等 kCheckIntervalS 秒即可触发保存，
 *       而不会因为"运动时恰好到了整点"就永远错过本轮保存机会。
 *
 *   提供 SaveNow() 供外部（StopLocalization / save_map 服务）主动调用。
 *
 *   生命周期：
 *     - 由 LocalizationManagerNode 在 StartLocalization() 成功后创建。
 *     - 在 StopLocalization() 中先调用 Stop()（停止定时器），
 *       再调用 SaveNow()（最终保存），最后 reset()。
 *     - 持有对 localization_node 的非拥有裸指针；
 *       合法性由 LocalizationManagerNode 在 Stop() 之后不再调用来保证。
 */
class MapPersistenceManager {
 public:
  /// 检查定时器间隔（秒）—— 短周期，用于及时捕捉"静止窗口"
  static constexpr double kCheckIntervalS = 30.0;

  /**
   * @param node            ROS2 节点（用于创建订阅/定时器和打日志）
   * @param get_loc_node    返回当前 LocalizationNode* 的 callable；可返回 nullptr
   * @param get_map_paths   返回 {map_folder, map_name} 的 callable
   * @param odom_topic      里程计话题名（默认 "/odom"）
   */
  MapPersistenceManager(
      rclcpp::Node* node,
      std::function<localization::LocalizationNode*()> get_loc_node,
      std::function<std::pair<std::string, std::string>()> get_map_paths,
      const std::string& odom_topic = "/odom")
      : node_(node),
        get_loc_node_(std::move(get_loc_node)),
        get_map_paths_(std::move(get_map_paths)),
        last_save_time_(std::chrono::steady_clock::now()) {

    // 订阅里程计，实时更新 is_robot_static_
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(1).best_effort(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          OnOdometry(msg);
        });

    // 短周期检查定时器：每 kCheckIntervalS 秒检查一次是否应当保存
    check_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(kCheckIntervalS),
        [this]() { OnCheckTick(); });

    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Initialized. "
                "Save interval: %.0f s (%.1f min), "
                "check interval: %.0f s, "
                "velocity thresholds: linear=%.3f m/s, angular=%.3f rad/s, "
                "odom_topic=%s",
                FLAGS_auto_save_interval_s,
                FLAGS_auto_save_interval_s / 60.0,
                kCheckIntervalS,
                FLAGS_auto_save_linear_velocity_threshold,
                FLAGS_auto_save_angular_velocity_threshold,
                odom_topic.c_str());
  }

  ~MapPersistenceManager() { Stop(); }

  // 禁止拷贝
  MapPersistenceManager(const MapPersistenceManager&) = delete;
  MapPersistenceManager& operator=(const MapPersistenceManager&) = delete;

  /**
   * @brief 停止检查定时器和里程计订阅。
   *        在 StopLocalization() 中、调用 SaveNow() 之前调用，防止并发。
   */
  void Stop() {
    if (check_timer_) {
      check_timer_->cancel();
      check_timer_.reset();
    }
    if (odom_sub_) {
      odom_sub_.reset();
    }
  }

  /**
   * @brief 立即执行一次保存（pbstream + map.png/yaml），忽略时间间隔和速度条件。
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
    if (ok) {
      last_save_time_ = std::chrono::steady_clock::now();
    }
    return ok;
  }

  /**
   * @brief 查询是否正在执行周期性保存（用于防止外部并发调用）。
   */
  bool IsSaving() const { return is_saving_.load(); }

 private:
  // -------------------------------------------------------------------------
  // ROS 回调

  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr& msg) {
    const auto& lv = msg->twist.twist.linear;
    const auto& av = msg->twist.twist.angular;
    const double speed   = std::hypot(std::hypot(lv.x, lv.y), lv.z);
    const double angular = std::hypot(std::hypot(av.x, av.y), av.z);
    is_robot_static_.store(
        speed   < FLAGS_auto_save_linear_velocity_threshold &&
        angular < FLAGS_auto_save_angular_velocity_threshold);
  }

  /**
   * @brief 短周期检查回调（每 kCheckIntervalS 秒触发一次）。
   *
   *   触发保存的条件（两者同时满足）：
   *     1. 距上次成功保存已超过 FLAGS_auto_save_interval_s 秒
   *     2. 机器人当前处于静止状态
   *
   *   如果条件 1 满足但条件 2 不满足（机器人在运动），本次跳过；
   *   等到下一个 kCheckIntervalS 周期再次检查，静止后即可立刻触发保存，
   *   无需再等完整的 FLAGS_auto_save_interval_s。
   */
  void OnCheckTick() {
    // 检查是否已到保存间隔
    const auto now = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(now - last_save_time_).count();
    if (elapsed_s < FLAGS_auto_save_interval_s) {
      // 还没到保存时间，打印剩余时间（调试级别）
      RCLCPP_DEBUG(node_->get_logger(),
                   "[MapPersistenceManager] Check tick: %.0f s until next save (interval=%.0f s)",
                   FLAGS_auto_save_interval_s - elapsed_s,
                   FLAGS_auto_save_interval_s);
      return;
    }

    // 已到保存间隔，检查机器人是否静止
    if (!is_robot_static_.load()) {
      RCLCPP_INFO(node_->get_logger(),
                  "[MapPersistenceManager] Check tick: save interval elapsed (%.0f s) "
                  "but robot is moving, waiting for static...",
                  elapsed_s);
      return;
    }

    // 防止重入（手动调用 SaveNow 与定时器并发）
    bool expected = false;
    if (!is_saving_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MapPersistenceManager] Check tick: another save is in progress, skip");
      return;
    }

    RCLCPP_INFO(node_->get_logger(),
                "[MapPersistenceManager] Periodic save triggered: "
                "elapsed=%.0f s, interval=%.0f s, robot is static",
                elapsed_s, FLAGS_auto_save_interval_s);

    auto* loc = get_loc_node_();
    if (!loc) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MapPersistenceManager] Check tick: localization not running, skip");
      is_saving_.store(false);
      return;
    }

    auto [map_folder, map_name] = get_map_paths_();
    if (map_folder.empty() || map_name.empty()) {
      RCLCPP_WARN(node_->get_logger(),
                  "[MapPersistenceManager] Check tick: map path not configured, skip");
      is_saving_.store(false);
      return;
    }

    bool ok = true;
    ok &= SavePbstream(loc, map_folder, map_name);
    ok &= SaveMapImage(loc, map_folder, map_name);

    if (ok) {
      last_save_time_ = std::chrono::steady_clock::now();
      RCLCPP_INFO(node_->get_logger(),
                  "[MapPersistenceManager] Periodic save completed successfully, "
                  "next save in %.0f s", FLAGS_auto_save_interval_s);
    } else {
      // 保存失败：不更新 last_save_time_，下一个检查周期会再次尝试
      RCLCPP_WARN(node_->get_logger(),
                  "[MapPersistenceManager] Periodic save completed with errors, "
                  "will retry at next check tick");
    }

    is_saving_.store(false);
  }

  // -------------------------------------------------------------------------
  // 内部保存实现

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

  // -------------------------------------------------------------------------
  // 成员变量

  rclcpp::Node* node_;
  std::function<localization::LocalizationNode*()> get_loc_node_;
  std::function<std::pair<std::string, std::string>()> get_map_paths_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr check_timer_;   ///< 短周期检查定时器（kCheckIntervalS）

  std::atomic<bool> is_robot_static_{true};
  std::atomic<bool> is_saving_{false};
  std::chrono::steady_clock::time_point last_save_time_;  ///< 上次成功保存的时刻
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_LOCALIZATION_MAP_PERSISTENCE_MANAGER_H_
