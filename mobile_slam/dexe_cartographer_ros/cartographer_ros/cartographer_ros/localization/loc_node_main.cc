
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
#include <mutex>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "absl/memory/memory.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/localization/localization_node.h"
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
  }

  ~LocalizationManagerNode() {
    RCLCPP_INFO(get_logger(), "Shutting down LocalizationManager Node");
    StopLocalization();
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

  // === Service Clients ===
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_filter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_filter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_filter_client_;

  // === Monitoring ===
  rclcpp::TimerBase::SharedPtr health_check_timer_;

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

    // 创建服务客户端
    start_filter_client_ = create_client<std_srvs::srv::Trigger>(
        "/robot_localization/start_filter");
    stop_filter_client_ = create_client<std_srvs::srv::Trigger>(
        "/robot_localization/stop_filter");
    reset_filter_client_ = create_client<std_srvs::srv::Trigger>(
        "/robot_localization/reset_filter");
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

    // if (!StartFilter()) {
    //   response->success = false;
    //   response->message = "Failed to start robot localization filter";
    //   return;
    // }

    response->success = StartLocalization();
    response->message = response->success ? 
        "Localization started successfully" : "Failed to start localization";
  }

  void HandleStopLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    StopLocalization();
    StopFilter();
    
    response->success = true;
    response->message = "Localization stopped successfully";
  }

  void HandleResetLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    RCLCPP_INFO(get_logger(), "Resetting localization");
    
    StopLocalization();
    // ResetFilter();
    
    response->success = StartLocalization();
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
    StopLocalization();
    
    // 更新地图路径（更新全局FLAGS）
    FLAGS_map_folder = new_map_folder;
    FLAGS_map_name = new_map_name;
    
    // 使用新地图启动定位
    RCLCPP_INFO(get_logger(), "Starting localization with new map...");
    response->success = StartLocalization();
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
      std::string state_filename = FLAGS_map_folder + "/" + FLAGS_map_name + "/map.pbstream";
      localization_node_->LoadState(state_filename, FLAGS_load_frozen_state);

      // 加载地图数据
      std::string yaml_filepath = FLAGS_map_folder + "/" + FLAGS_map_name + "/map.yaml";
      localization_node_->LoadMapData(yaml_filepath);

      // 启动轨迹
      if (FLAGS_start_trajectory_with_default_topics) {
        localization_node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
      }

      state_ = LocalizationState::RUNNING;
      RCLCPP_INFO(get_logger(), "Localization started successfully");
      return true;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to start localization: %s", e.what());
      state_ = LocalizationState::ERROR;
      return false;
    }
  }

  void StopLocalization() {
    if (state_ == LocalizationState::STOPPED) {
      return;
    }

    state_ = LocalizationState::STOPPING;
    
    try {
      if (localization_node_) {
        localization_node_->FinishAllTrajectories();
        localization_node_.reset();
      }
      
      node_options_.reset();
      trajectory_options_.reset();
      
      state_ = LocalizationState::STOPPED;
      RCLCPP_INFO(get_logger(), "Localization stopped successfully");
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Error while stopping localization: %s", e.what());
      state_ = LocalizationState::ERROR;
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

    std::string map_path = FLAGS_map_folder + "/" + FLAGS_map_name;
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

  bool CallRobotLocalizationService(const std::string& service_name) {
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
    
    if (service_name.find("start_filter") != std::string::npos) {
      client = start_filter_client_;
    } else if (service_name.find("stop_filter") != std::string::npos) {
      client = stop_filter_client_;
    } else if (service_name.find("reset_filter") != std::string::npos) {
      client = reset_filter_client_;
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown service name: %s", service_name.c_str());
      return false;
    }

    if (!client->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Service %s call timeout", service_name.c_str());
      return false;
    }

    auto response = future.get();
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "Service %s call failed: %s", 
                   service_name.c_str(), response->message.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Service %s call successful", service_name.c_str());
    return true;
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
  
  auto node = std::make_shared<cartographer_ros::LocalizationManagerNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  
  return 0;
}
