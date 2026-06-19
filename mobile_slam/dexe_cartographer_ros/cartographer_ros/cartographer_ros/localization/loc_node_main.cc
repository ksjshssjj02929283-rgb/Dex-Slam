
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

#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <ctime>
#include <mutex>
#include <filesystem>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "absl/memory/memory.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/localization/atomic_dir_swap.h"
#include "cartographer_ros/localization/localization_node.h"
#include "cartographer_ros/localization/map_persistence_manager.h"
#include "nav_interfaces/srv/save_map.hpp"
#include "nav_interfaces/srv/set_pose.hpp"
#include "cartographer_ros/options/localization_node_options.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "gflags/gflags.h"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    map_filestream, "",
    "Filename of a pbstream to draw a map from.");

DEFINE_string(
    map_folder, "",
    "Filename of the map to be saved. If empty, a map will not be saved.");
DEFINE_string(
    map_name, "",
    "Name of the map to be saved. If empty, a map will not be saved.");

// ---------------------------------------------------------------------------
// 启动期 GFlags
// ---------------------------------------------------------------------------
DEFINE_bool(
    auto_load_map, false,
    "If true, automatically load the map specified by --map_folder/--map_name on startup. "
    "If false, skip map loading at startup and wait for the set_map_changed service to trigger.");

DEFINE_int32(
    map_backup_keep_count, 50,
    "Number of historical map backups to keep on disk (rotated after each save). "
    "0 = do not keep any historical backups. Each backup occupies one copy of the "
    "map folder size.");

namespace cartographer_ros {

/**
 * @brief LocalizationManager - 管理LocalizationNode生命周期的ROS2节点
 */
class LocalizationManagerNode : public rclcpp::Node {
public:
  explicit LocalizationManagerNode()
      : Node("localization_node"), 
        state_(LocalizationState::STOPPED) {
    
    RCLCPP_INFO(get_logger(), "Initializing LocalizationManager Node");
    
    // 按依赖顺序初始化组件
    InitializeTfComponents();
    InitializeServices();

    // 启动健康检查定时器
    health_check_timer_ = create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&LocalizationManagerNode::PerformHealthCheck, this));
    
    RCLCPP_INFO(get_logger(), "LocalizationManager Node initialized successfully");
    
    // 自动启动（延迟一小段时间确保所有组件就绪）
    auto_start_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
          auto_start_timer_->cancel();  // 只执行一次
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (FLAGS_auto_load_map) {
            RCLCPP_INFO(get_logger(), "auto_load_map=true: auto-starting localization with map...");
            if (StartLocalization()) {
              StartFilter();
              RCLCPP_INFO(get_logger(), "Localization auto-started successfully");
            } else {
              RCLCPP_ERROR(get_logger(), "Failed to auto-start localization");
            }
          } else {
            RCLCPP_INFO(get_logger(),
                        "auto_load_map=false: skipping map load at startup, "
                        "waiting for ~/set_map_changed service to trigger");
          }
        });
  }

  ~LocalizationManagerNode() {
    // RCLCPP_INFO(get_logger(), "Shutting down LocalizationManager Node");
    // Shutdown();
  }

  // 显式的shutdown方法，确保能够完成清理
  void Shutdown() {
    // 使用atomic标志避免重复调用
    bool expected = false;
    if (!shutdown_flag_.compare_exchange_strong(expected, true)) {
      return;  // 已经在执行shutdown
    }

    RCLCPP_INFO(get_logger(), "Performing shutdown sequence...");

    // 取消所有定时器（不需要锁）
    if (health_check_timer_) {
      health_check_timer_->cancel();
    }
    if (auto_start_timer_) {
      auto_start_timer_->cancel();
    }

    RCLCPP_INFO(get_logger(), "Shutdown sequence completed");
  }

private:
  // === State Management ===
  enum class LocalizationState {
    STOPPED,
    STARTING,
    RUNNING,
    STOPPING,
    ERROR
  };
  LocalizationState state_;
  std::mutex state_mutex_;
  std::atomic<bool> shutdown_flag_{false};

  // === Core Components ===
  std::unique_ptr<localization::LocalizationNode> localization_node_;
  std::unique_ptr<LocalizationNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  // === TF Components ===
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // === ROS2 Services ===
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_localization_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_localization_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_localization_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr change_map_srv_;
  rclcpp::Service<nav_interfaces::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_origin_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_global_relocalization_srv_;
  rclcpp::Service<nav_interfaces::srv::SetPose>::SharedPtr relocalization_srv_;
  // rollback_map_to 服务已移除——回退归 command_board 文件级处理。

  // filter service 调用使用临时 node + 独立 executor，不持有长期 client
  // （在 service handler 线程中调用同进程其他 service 会死锁，须独立 executor）

  // === Monitoring ===
  rclcpp::TimerBase::SharedPtr health_check_timer_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;

  // === Map Persistence ===
  std::unique_ptr<MapPersistenceManager> map_persistence_manager_;

  void InitializeServices() {
    // 创建服务
    start_localization_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start_localization",
        std::bind(&LocalizationManagerNode::HandleStartLocalization, this,
                  std::placeholders::_1, std::placeholders::_2));

    stop_localization_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop_localization",
        std::bind(&LocalizationManagerNode::HandleStopLocalization, this,
                  std::placeholders::_1, std::placeholders::_2));

    reset_localization_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_localization",
        std::bind(&LocalizationManagerNode::HandleResetLocalization, this,
                  std::placeholders::_1, std::placeholders::_2));

    change_map_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/set_map_changed",
        std::bind(&LocalizationManagerNode::HandleChangeMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    save_map_srv_ = create_service<nav_interfaces::srv::SaveMap>(
        "~/save_map",
        std::bind(&LocalizationManagerNode::HandleSaveMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    reset_map_origin_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_map_origin",
        std::bind(&LocalizationManagerNode::HandleResetMapOrigin, this,
                  std::placeholders::_1, std::placeholders::_2));

    trigger_global_relocalization_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/trigger_global_relocalization",
        std::bind(&LocalizationManagerNode::HandleTriggerGlobalRelocalization, this,
                  std::placeholders::_1, std::placeholders::_2));

    // 局部重定位（SetPose）：控制服务在外围层注册，转调核心 Relocalize()
    relocalization_srv_ = create_service<nav_interfaces::srv::SetPose>(
        "~/relocalization",
        std::bind(&LocalizationManagerNode::HandleRelocalization, this,
                  std::placeholders::_1, std::placeholders::_2));

    // ~/rollback_map_to 已移除——引擎不提供 rollback，回退由 command_board
    // 文件级处理（从 slam/backups 恢复 + 重下发）。底图 ring backup 仍由 save_map 维护。

    // filter service 客户端在调用时临时创建，不在此注册
  }

  void InitializeTfComponents() {
    constexpr double kTfBufferCacheTimeInSeconds = 10.0;
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        get_clock(), tf2::durationFromSec(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  void HandleStartLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_ == LocalizationState::RUNNING) {
      response->success = true;
      response->message = "Localization is already running";
      return;
    }

    response->success = StartLocalization();
    if (response->success) {
      StartFilter();
    }
    response->message = response->success ?
        "Localization started successfully" : "Failed to start localization";
  }

  void HandleStopLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);

    StopFilter();
    StopLocalization();
    
    response->success = true;
    response->message = "Localization stopped successfully";
  }

  void HandleResetLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    RCLCPP_INFO(get_logger(), "Resetting localization");

    StopFilter();
    StopLocalization();
    // ResetFilter();

    response->success = StartLocalization();
    if (response->success) {
      StartFilter();
    }
    response->message = response->success ?
        "Localization reset successfully" : "Failed to reset localization";
  }

  void HandleChangeMap(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    RCLCPP_INFO(get_logger(), "Changing map - getting new map info from map_manager");
    
    // 从map_manager获取新的地图信息
    std::string new_map_folder;
    std::string new_map_name;
    
    if (!GetMapInfoFromMapManager(new_map_folder, new_map_name)) {
      response->success = false;
      response->message = "Failed to get map info from map_manager";
      return;
    }
    
    RCLCPP_INFO(get_logger(), "New map - folder: %s, name: %s", 
                new_map_folder.c_str(), new_map_name.c_str());
    
    // 停止当前定位
    RCLCPP_INFO(get_logger(), "Stopping current localization...");
    StopFilter();
    StopLocalization();
    
    // 更新地图路径（更新全局FLAGS）
    FLAGS_map_folder = new_map_folder;
    FLAGS_map_name = new_map_name;
    
    // 使用新地图启动定位
    RCLCPP_INFO(get_logger(), "Starting localization with new map...");
    response->success = StartLocalization() && StartFilter();
    response->message = response->success ? 
        "Map changed successfully" : "Failed to start localization with new map";
  }

  bool GetMapInfoFromMapManager(std::string& map_folder, std::string& map_name) {
    // 创建临时节点用于参数获取
    try {
      RCLCPP_INFO(get_logger(), "Creating temporary node to fetch map_manager parameters...");
      
      // 创建临时节点
      auto temp_node = std::make_shared<rclcpp::Node>(
          "temp_param_client_node",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
      
      // 创建参数客户端
      auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
          temp_node, "map_manager");
      
      if (!param_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "map_manager parameter service not available");
        return false;
      }
      
      RCLCPP_INFO(get_logger(), "Connected to map_manager parameter service");
      
      // 获取 map_folder 和 map_name 参数
      auto params = param_client->get_parameters({"map_folder", "map_name"});
      
      if (params.size() != 2) {
        RCLCPP_ERROR(get_logger(), "Failed to get both map_folder and map_name parameters");
        return false;
      }
      
      // 检查并获取 map_folder
      if (params[0].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        map_folder = params[0].as_string();
      } else {
        RCLCPP_ERROR(get_logger(), "map_folder parameter is not a string");
        return false;
      }
      
      // 检查并获取 map_name
      if (params[1].get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        map_name = params[1].as_string();
      } else {
        RCLCPP_ERROR(get_logger(), "map_name parameter is not a string");
        return false;
      }
      
      if (map_folder.empty() || map_name.empty()) {
        RCLCPP_ERROR(get_logger(), "Got empty map_folder or map_name from map_manager");
        return false;
      }
      
      RCLCPP_INFO(get_logger(), "Successfully got map info from map_manager - folder: %s, name: %s",
                  map_folder.c_str(), map_name.c_str());
      
      return true;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception while getting parameters from map_manager: %s", e.what());
      return false;
    }
  }

  bool StartLocalization() {
    if (state_ == LocalizationState::RUNNING) {
      RCLCPP_WARN(get_logger(), "Localization is already running");
      return true;
    }

    state_ = LocalizationState::STARTING;

    try {
      // 启动恢复：扫描磁盘 staging/swap_backup/tombstone 残留，按决策表自愈
      // 必须在 ValidateMapResources 之前——半成品 staging/swap_backup 可能让 validate 误判
      if (!FLAGS_map_folder.empty() && !FLAGS_map_name.empty()) {
        const std::string target_dir = FLAGS_map_folder + "/" + FLAGS_map_name + "/slam";  // 底图在 slam/ 子目录
        AtomicDirSwap::RecoverIfInterrupted(target_dir);
      }

      // 验证地图资源
      if (!ValidateMapResources()) {
        state_ = LocalizationState::ERROR;
        return false;
      }

      // 创建配置选项 - 使用gflags
      node_options_ = std::make_unique<LocalizationNodeOptions>();
      trajectory_options_ = std::make_unique<TrajectoryOptions>();
      
      // 加载配置选项
      std::tie(*node_options_, *trajectory_options_) = 
          LoadLocalizationNodeOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

      // 创建定位更新器
      auto localization_updater = cartographer::localization::CreateLocalizationUpdater(
          node_options_->localization_updater_options);

      // 创建定位节点
      localization_node_ = std::make_unique<localization::LocalizationNode>(
          *node_options_, std::move(localization_updater), tf_buffer_, shared_from_this());

      // 加载状态文件
      std::string state_filename = FLAGS_map_folder + "/" + FLAGS_map_name + "/slam/map.pbstream";
      localization_node_->LoadState(state_filename, FLAGS_load_frozen_state);

      // 启动轨迹
      if (FLAGS_start_trajectory_with_default_topics) {
        localization_node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
      }

      state_ = LocalizationState::RUNNING;

      // 创建地图保存执行单元（仅手动保存：只能由 HandleSaveMap 触发；StopLocalization 不再附带保存）
      map_persistence_manager_ = std::make_unique<MapPersistenceManager>(
          this,
          [this]() -> localization::LocalizationNode* {
            return localization_node_.get();
          },
          [this]() -> std::pair<std::string, std::string> {
            return {FLAGS_map_folder, FLAGS_map_name};
          },
          FLAGS_map_backup_keep_count);

      RCLCPP_INFO(get_logger(), "Localization started successfully");
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to start localization: %s", e.what());
      state_ = LocalizationState::ERROR;
      return false;
    }
  }

  // 停定位 — 纯停止，不再附带保存。
  // 保存仅由 ~/save_map 服务显式触发；如需"停定位前保存"，调用方应先调 save_map 再调 stop_localization。
  void StopLocalization() {
    if (state_ == LocalizationState::STOPPED) {
      RCLCPP_INFO(get_logger(), "Localization already stopped");
      return;
    }

    RCLCPP_INFO(get_logger(), "Stopping localization...");
    state_ = LocalizationState::STOPPING;

    try {
      if (map_persistence_manager_) {
        map_persistence_manager_.reset();
      }

      if (localization_node_) {
        localization_node_->FinishAllTrajectories();
        localization_node_.reset();
      }

      node_options_.reset();
      trajectory_options_.reset();

      state_ = LocalizationState::STOPPED;
      RCLCPP_INFO(get_logger(), "=== Localization stopped successfully ===");

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error while stopping localization: %s", e.what());
      state_ = LocalizationState::ERROR;
    }
  }

  void HandleSaveMap(
      const std::shared_ptr<nav_interfaces::srv::SaveMap::Request> request,
      std::shared_ptr<nav_interfaces::srv::SaveMap::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ != LocalizationState::RUNNING) {
      response->success = false;
      response->message = "Localization is not running, cannot save map";
      RCLCPP_WARN(get_logger(), "save_map service called but localization is not running");
      return;
    }

    if (!map_persistence_manager_) {
      response->success = false;
      response->message = "MapPersistenceManager is not initialized";
      RCLCPP_ERROR(get_logger(), "save_map service called but MapPersistenceManager is null");
      return;
    }

    // localization 保存当前已加载地图（只更新底图，不碰 map asset）。
    // map_name 输入仅作记录：定位侧不支持 save-as（被保存的就是当前加载的地图）。
    if (!request->map_name.empty()) {
      RCLCPP_INFO(get_logger(),
                  "save_map: request map_name='%s'（定位侧保存当前已加载地图，忽略改名）",
                  request->map_name.c_str());
    }

    RCLCPP_INFO(get_logger(), "save_map service called, saving base map via MapPersistenceManager...");
    std::string map_path, backup_ts;
    const bool success = map_persistence_manager_->SaveNow(&map_path, &backup_ts);

    response->success = success;
    response->map_path = map_path;
    response->backup_ts = backup_ts;
    response->message = success ? "Map saved successfully"
                               : "Map save completed with errors (check logs)";
  }

  void HandleResetMapOrigin(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ != LocalizationState::RUNNING || !localization_node_) {
      response->success = false;
      response->message = "Localization is not running, cannot reset map origin";
      RCLCPP_WARN(get_logger(), "reset_map_origin called but localization is not running");
      return;
    }

    if (FLAGS_map_folder.empty() || FLAGS_map_name.empty()) {
      response->success = false;
      response->message = "map_folder or map_name is not set";
      RCLCPP_ERROR(get_logger(), "reset_map_origin: --map_folder and --map_name must be specified");
      return;
    }

    const std::string map_dir = FLAGS_map_folder + "/" + FLAGS_map_name + "/slam";  // 底图在 slam/ 子目录
    const std::string map_base = map_dir + "/map";
    const std::string pbstream_path = map_dir + "/map.pbstream";

    RCLCPP_INFO(get_logger(), "reset_map_origin: starting workflow...");

    // 1. 停止 EKF filter，避免 TF 变换期间出现冲突
    RCLCPP_INFO(get_logger(), "reset_map_origin [1/5]: stopping EKF filter...");
    StopFilter();

    // 2. 在定位运行时获取当前位姿，重置地图原点并保存新地图文件
    RCLCPP_INFO(get_logger(),
                "reset_map_origin [2/5]: resetting map origin (map_base=%s)...",
                map_base.c_str());
    const bool reset_success =
        localization_node_->ResetMapOriginAndSave(map_base, pbstream_path);

    if (!reset_success) {
      RCLCPP_ERROR(get_logger(), "reset_map_origin: ResetMapOriginAndSave failed, aborting");
      StartFilter();  // 尽力恢复 filter
      response->success = false;
      response->message = "Failed to reset map origin (check logs)";
      return;
    }

    // 3. 停止定位（StopLocalization 默认不保存——保存只由 save_map 服务触发）
    RCLCPP_INFO(get_logger(), "reset_map_origin [3/5]: stopping localization...");
    StopLocalization();

    // 4. 用新 pbstream 重新启动定位
    RCLCPP_INFO(get_logger(), "reset_map_origin [4/5]: restarting localization with new map...");
    const bool start_success = StartLocalization();

    if (!start_success) {
      RCLCPP_ERROR(get_logger(),
                   "reset_map_origin: failed to restart localization after origin reset");
      response->success = false;
      response->message = "Map origin reset succeeded but failed to restart localization";
      return;
    }

    // 5. 触发重定位：机器人当前位置即新地图原点，设初始位姿为 identity
    RCLCPP_INFO(get_logger(),
                "reset_map_origin [5/5]: triggering relocalization at new map origin...");
    localization_node_->TriggerGlobalRelocalization();

    // start_filter 内部已执行 reset()，滤波器将从新帧的首次位姿测量重新初始化
    StartFilter();

    RCLCPP_INFO(get_logger(), "reset_map_origin: completed successfully");
    response->success = true;
    response->message =
        "Map origin reset, localization restarted, relocalization triggered at new origin";
  }

  void HandleTriggerGlobalRelocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ != LocalizationState::RUNNING || !localization_node_) {
      response->success = false;
      response->message = "Localization is not running, cannot trigger relocalization";
      RCLCPP_WARN(get_logger(),
                  "trigger_global_relocalization called but localization is not running");
      return;
    }

    RCLCPP_INFO(get_logger(), "trigger_global_relocalization: invoking...");
    const bool ok = localization_node_->TriggerGlobalRelocalization();

    response->success = ok;
    response->message = ok
        ? "Global relocalization triggered successfully"
        : "Global relocalization failed (see node logs)";

    if (ok) {
      RCLCPP_INFO(get_logger(), "trigger_global_relocalization: success");
    } else {
      RCLCPP_ERROR(get_logger(), "trigger_global_relocalization: failed");
    }
  }

  // 局部重定位（SetPose）：外围层门面，转调核心 LocalizationNode::Relocalize()
  void HandleRelocalization(
      const std::shared_ptr<nav_interfaces::srv::SetPose::Request> request,
      std::shared_ptr<nav_interfaces::srv::SetPose::Response> response) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (state_ != LocalizationState::RUNNING || !localization_node_) {
      response->success = false;
      RCLCPP_WARN(get_logger(),
                  "relocalization called but localization is not running");
      return;
    }

    response->success = localization_node_->Relocalize(request->pose);
    if (response->success) {
      RCLCPP_INFO(get_logger(), "relocalization: initial pose set successfully");
    } else {
      RCLCPP_ERROR(get_logger(), "relocalization: failed (see node logs)");
    }
  }

  bool ValidateMapResources() {
    // 使用gflags参数
    if (FLAGS_map_folder.empty() || FLAGS_map_name.empty()) {
      RCLCPP_ERROR(get_logger(), "Need to specify --map_folder and --map_name");
      return false;
    }

    if (!std::filesystem::exists(FLAGS_map_folder)) {
      RCLCPP_ERROR(get_logger(), "Map folder does not exist: %s", FLAGS_map_folder.c_str());
      return false;
    }

    std::string map_path = FLAGS_map_folder + "/" + FLAGS_map_name + "/slam";  // 底图在 slam/ 子目录
    if (!std::filesystem::exists(map_path)) {
      RCLCPP_ERROR(get_logger(), "Map directory does not exist: %s", map_path.c_str());
      return false;
    }

    std::string pbstream_path = map_path + "/map.pbstream";
    if (!std::filesystem::exists(pbstream_path)) {
      RCLCPP_ERROR(get_logger(), "Map pbstream file does not exist: %s", pbstream_path.c_str());
      return false;
    }

    std::string yaml_path = map_path + "/map.yaml";
    if (!std::filesystem::exists(yaml_path)) {
      RCLCPP_ERROR(get_logger(), "Map yaml file does not exist: %s", yaml_path.c_str());
      return false;
    }

    return true;
  }

  // 在 service handler 线程内调用另一个 service 会死锁（主 executor 被占用无法 spin）。
  // 解决方案：每次调用创建临时 node + 独立 SingleThreadedExecutor，与主 executor 完全隔离。
  bool CallRobotLocalizationService(const std::string& service_name) {
    try {
      auto temp_node = std::make_shared<rclcpp::Node>(
          "temp_filter_client",
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
      auto temp_client = temp_node->create_client<std_srvs::srv::Trigger>(service_name);

      if (!temp_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
        return false;
      }

      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto future = temp_client->async_send_request(request);

      rclcpp::executors::SingleThreadedExecutor temp_executor;
      temp_executor.add_node(temp_node);

      const auto timeout = std::chrono::seconds(5);
      const auto start_time = std::chrono::steady_clock::now();

      while (rclcpp::ok() &&
             future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        temp_executor.spin_some();
        if (std::chrono::steady_clock::now() - start_time > timeout) {
          RCLCPP_ERROR(get_logger(), "Service %s call timeout", service_name.c_str());
          return false;
        }
      }

      auto response = future.get();
      if (!response->success) {
        RCLCPP_ERROR(get_logger(), "Service %s call failed: %s",
                     service_name.c_str(), response->message.c_str());
        return false;
      }

      RCLCPP_INFO(get_logger(), "Service %s call successful", service_name.c_str());
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception while calling %s: %s",
                   service_name.c_str(), e.what());
      return false;
    }
  }

  bool StartFilter() {
    return CallRobotLocalizationService("/robot_localization/start_filter");
  }

  bool StopFilter() {
    return CallRobotLocalizationService("/robot_localization/stop_filter");
  }

  bool ResetFilter() {
    return CallRobotLocalizationService("/robot_localization/reset_filter");
  }

  void PerformHealthCheck() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 简单的健康检查逻辑
    if (state_ == LocalizationState::RUNNING && !localization_node_) {
      RCLCPP_ERROR(get_logger(), "Localization node is null but state is RUNNING");
      state_ = LocalizationState::ERROR;
    }
    
    // 可以根据需要添加更多健康检查逻辑
  }
};

} // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // 验证必需参数
  if (FLAGS_configuration_directory.empty()) {
    std::cerr << "-configuration_directory is missing." << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  if (FLAGS_configuration_basename.empty()) {
    std::cerr << "-configuration_basename is missing." << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  // 创建节点
  auto node = std::make_shared<cartographer_ros::LocalizationManagerNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);

  try {
    executor.spin();
  } catch (const std::exception& e) {
    std::cerr << "[LocalizationManager] Exception in executor: " << e.what() << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
