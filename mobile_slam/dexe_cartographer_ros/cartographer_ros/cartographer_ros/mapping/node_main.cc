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
#include <filesystem>
#include <thread>
#include <chrono>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <atomic>
#include <mutex>
#include "nav_interfaces/srv/start_mapping.hpp"
#include "nav_interfaces/msg/mapping_state.hpp"


#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/mapping/map_directory.h"
#include "cartographer_ros/mapping/node.h"
#include "cartographer_ros/options/mapping_node_options.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

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
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");

DEFINE_string(
    map_folder, "",
    "Filename of the map to be saved. If empty, a map will not be saved.");
DEFINE_string(
    map_name, "",
    "Name of the map to be saved. If empty, a map will not be saved.");


DEFINE_double(
    resolution, 0.05,
    "Resolution of a grid cell in the drawn map.");

// DEFINE_string(
//     save_state_filename, "",
//     "If non-empty, serialize state and write it to disk before shutting down.");
// // added by hz
// DEFINE_string(initial_poses_filename, "",
//               "If non-empty, filename of a .yaml file to load, containing "
//               "predefined initial pose.");
// DEFINE_string(apriltag_poses_filename, "",
//               "If non-empty, filename of a .yaml file to load, containing "
//               "predefined apriltag poses.");
// DEFINE_string(location_name, "", "Set the location of the robot.");
// // end


namespace cartographer_ros {
namespace {

class MappingNode : public rclcpp::Node
{
public:
  explicit MappingNode()
  : rclcpp::Node("mapping")
  {
    // Publisher
    // ~/state：结构化建图状态（唯一状态出口，FQN /mapping/state；状态仅经 topic 发布）
    state_pub_ = this->create_publisher<nav_interfaces::msg::MappingState>("~/state", 10);

    // 建图控制服务放独立回调组：finish 期间阻塞较久，不能拖住默认组里的状态定时器
    ops_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    start_mapping_srv_ = this->create_service<nav_interfaces::srv::StartMapping>(
      "~/start_mapping",
      std::bind(&MappingNode::StartMappingCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, ops_group_);

    finish_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/finish_mapping",
      std::bind(&MappingNode::FinishMappingCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, ops_group_);

    cancel_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/cancel_mapping",
      std::bind(&MappingNode::CancelMappingCallback, this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, ops_group_);

    // 建图状态仅经 ~/state(topic) 发布，不提供查询 service；
    // 原 ~/set_map_name 已并入 start_mapping 请求，原 ~/save_map 与 finish_mapping 同构已删除

    // Clients for show map control (使用全路径，等价于 ROS1 版本)
    start_show_map_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/cartographer_occupancy_grid_node/start_showing_map");
    stop_show_map_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/cartographer_occupancy_grid_node/stop_showing_map");

    // Timer: 5Hz -> 0.2s
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MappingNode::Publish, this));
  }

  ~MappingNode()
  {
    if (is_mapping_) {
      FinishMapping();
    }
  }

private:
  // ---- Service Callbacks (ROS2 签名) ----
  // 构建结构化建图状态（~/state topic 发布用）
  nav_interfaces::msg::MappingState BuildMappingState() {
    nav_interfaces::msg::MappingState s;
    s.header.stamp = this->now();
    if (is_saving_) {
      s.status = nav_interfaces::msg::MappingState::SAVING;
    } else if (is_mapping_) {
      s.status = nav_interfaces::msg::MappingState::MAPPING;
    } else {
      s.status = nav_interfaces::msg::MappingState::IDLE;
    }
    s.progress = progress_.load();
    std::lock_guard<std::mutex> lock(state_mutex_);
    s.map_name = map_name_;
    s.map_path = map_path_;
    s.stage = stage_;
    s.error_code = error_code_;
    return s;
  }

  // 统一填写返回字段（各对外 srv 响应内联 success / error_code / message）
  template <typename Response>
  static void FillResult(Response* r, bool success, int32_t code, const std::string& message) {
    r->success = success;
    r->error_code = code;
    r->message = message;
  }

  void SetStage(const std::string& stage) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stage_ = stage;
  }

  void SetErrorCode(int32_t code) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_code_ = code;
  }



  void StartMappingCallback(const std::shared_ptr<nav_interfaces::srv::StartMapping::Request> req,
                            std::shared_ptr<nav_interfaces::srv::StartMapping::Response> res)
  {
    using MS = nav_interfaces::msg::MappingState;
    if (is_saving_) {
      LOG(ERROR) << "Map saving is in progress; start_mapping rejected.";
      FillResult(res.get(), false, MS::E_MAP_SAVING, "Map saving in progress; wait for ~/state to become IDLE");
      return;
    }
    using MS = nav_interfaces::msg::MappingState;
    if (is_mapping_) {
      LOG(ERROR) << "Mapping node is already running.";
      FillResult(res.get(), false, MS::E_MAP_ALREADY_RUNNING, "Mapping already running");
      return;
    }
    if (FLAGS_map_folder.empty()) {
      LOG(ERROR) << "Need to specify --map_folder.";
      FillResult(res.get(), false, MS::E_MAP_FOLDER_MISSING, "map_folder is not configured");
      return;
    }
    std::string name_error;
    if (!ValidateMapName(req->map_name, &name_error)) {
      LOG(ERROR) << "Invalid map name '" << req->map_name << "': " << name_error;
      FillResult(res.get(), false, MS::E_MAP_NAME_INVALID, "Invalid map_name: " + name_error);
      return;
    }
    if (!EnsureDirectoryExists(FLAGS_map_folder)) {
      FillResult(res.get(), false, MS::E_MAP_FOLDER_MISSING,
                 "map_folder does not exist and cannot be created: " + FLAGS_map_folder);
      return;
    }
    const std::filesystem::path map_directory =
        std::filesystem::path(FLAGS_map_folder) / req->map_name;
    if (std::filesystem::exists(map_directory) && !req->overwrite) {
      LOG(ERROR) << "Map '" << req->map_name << "' already exists and overwrite is false.";
      FillResult(res.get(), false, MS::E_MAP_NAME_EXISTS,
                 "Map already exists; set overwrite=true to recreate: " + req->map_name);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name_ = req->map_name;
      map_path_ = FLAGS_map_folder + "/" + req->map_name + "/slam";
      stage_.clear();
      error_code_ = MS::E_NONE;
    }
    progress_ = 0.0f;
    // A new mapping session always owns a clean map directory. This removes
    // stale slam/assets/versions data before any trajectory is started.
    const MapDirectoryResetResult reset_result =
        ResetMapDirectory(FLAGS_map_folder, map_name_);
    if (!reset_result.success) {
      LOG(ERROR) << "Failed to reset map directory '"
                 << reset_result.map_directory.string()
                 << "': " << reset_result.error_message;
      FillResult(res.get(), false, MS::E_MAP_DIR_RESET_FAILED,
                 "Failed to reset map directory: " + reset_result.error_message);
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        map_name_.clear();
        map_path_.clear();
      }
      return;
    }
    if (reset_result.replaced_existing) {
      LOG(WARNING) << "Recreated existing map directory '"
                   << reset_result.map_directory.string() << "' (removed "
                   << reset_result.removed_entries << " entries).";
    } else {
      LOG(INFO) << "Created map directory '"
                << reset_result.map_directory.string() << "'.";
    }

    // TF2 (ROS2): Buffer 绑定到本节点的 Clock
    constexpr double kTfBufferCacheTimeInSeconds = 10.0;
    (void)kTfBufferCacheTimeInSeconds; // 缓存时长由 Buffer 默认处理
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    node_options_ = std::make_unique<MappingNodeOptions>();
    trajectory_options_ = std::make_unique<TrajectoryOptions>();

    // 1. 解析节点和轨迹的配置文件(tie会自动根据参数名，对应node_options和trajectory_options)
    std::tie(*node_options_, *trajectory_options_) =
      LoadMappingOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    // 2. 构建 map_builder
    auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options_->map_builder_options);
    // 3. 创建 Cartographer ROS Node（节点发布的topic、服务等在其中定义）
    mapping_node_ =
          std::make_unique<cartographer_ros::Node>(*node_options_, std::move(map_builder),
                                                    tf_buffer_, this->shared_from_this(),
                                                    FLAGS_collect_metrics);
    // 4. 读取pstream文件（子图、约束、轨迹等信息）
    // TODO： 扩建模式下，才需要 load_state_filename
    if (!FLAGS_load_state_filename.empty()) {
      mapping_node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }
    // 5. 创建轨迹优化器，订阅ros消息
    if (FLAGS_start_trajectory_with_default_topics) {
      mapping_node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
    }

    is_mapping_ = true;
    trajectories_finished_ = false;  // 重置标志

    StartShowMap();
    FillResult(res.get(), true, MS::E_NONE, "OK");
  }

  // finish 为触发动作：本回调同步执行收尾管线，期间经 ~/state 上报 progress/stage。
  // 状态定时器在节点默认回调组，与本服务所在的 ops_group_ 由 MultiThreadedExecutor 并行调度，
  // 因此 SAVING 期间 ~/state 照常以 5 Hz 发布；调用方无需等待本响应，盯 ~/state 即可。
  void FinishMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (is_saving_) {
      res->success = false;
      res->message = "Finish already in progress; watch ~/state";
      return;
    }
    if (!is_mapping_) {
      res->success = false;
      res->message = "Mapping not running";
      return;
    }
    std::string map_path;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_path = map_path_;
    }
    if (!RunFinishPipeline()) {
      res->success = false;
      res->message =
          "Failed to save map files; see ~/state error_code and call finish_mapping again to retry";
      return;
    }
    res->success = true;
    res->message = "Map saved to " + map_path;
  }

  void CancelMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (is_saving_) {
      res->success = false;
      res->message = "Cannot cancel while saving; wait for ~/state to become IDLE";
      return;
    }
    if (!is_mapping_) {
      // 未在建图：清除残留地图名/路径，等价于重置（幂等）
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool had_name = !map_name_.empty();
      map_name_.clear();
      map_path_.clear();
      res->success = true;
      res->message = had_name ? "Map name cleared, ready for new mapping" : "Already idle";
      return;
    }
    progress_ = 0.0f;
    SetStage("");
    SetErrorCode(nav_interfaces::msg::MappingState::E_NONE);
    LOG(INFO) << "Cancelling mapping without saving. Resetting mapping node.";
    // 停止显示地图
    StopShowMap();
    // 直接重置建图节点，不保存任何数据
    ResetMappingNode();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name_.clear();
      map_path_.clear();
    }
    res->success = true;
    res->message = "Mapping cancelled successfully";
  }
  // ---- Helpers ----
  bool EnsureDirectoryExists(const std::string& path) {
    try {
      if (path.empty()) return false;
      if (std::filesystem::exists(path)) {
        if (std::filesystem::is_directory(path)) return true;
        LOG(ERROR) << "Path exists but is not a directory: '" << path << "'.";
        return false;
      }
      return std::filesystem::create_directories(path);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Filesystem error for path '" << path << "': " << e.what();
      return false;
    }
  }

  // 同步调用 Trigger（等待存在 + 调用）
  bool CallTriggerSync(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client) {
    using namespace std::chrono_literals;
    if (!client) return false;

    // 等 5s 服务可用
    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(), "%s not exists", client->get_service_name());
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    // 简单同步等待（增加超时时间到 5s）
    auto status = future.wait_for(5s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "%s call timeout", client->get_service_name());
      return false;
    }
    auto resp = future.get();
    if (resp->success) {
      RCLCPP_INFO(this->get_logger(), "%s call success", client->get_service_name());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s call failed: %s", client->get_service_name(), resp->message.c_str());
      return false;
    }
  }

  bool StartShowMap() {
    return CallTriggerSync(start_show_map_client_);
  }

  bool StopShowMap() {
    return CallTriggerSync(stop_show_map_client_);
  }

  // 重置建图节点（清理所有资源）
  void ResetMappingNode() {
    LOG(INFO) << "Resetting mapping node...";
    
    mapping_node_.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    is_mapping_ = false;
    is_saving_ = false;
    trajectories_finished_ = false;  // 重置标志
    
    LOG(INFO) << "Mapping node reset completed.";
  }

  // 收尾管线：结束轨迹 → 全局优化 → 写 pbstream（逐 node 进度 0~90）→ 写栅格/yaml/pcd → 100。
  // 失败时保持建图态（可重试），返回 false。
  bool RunFinishPipeline() {
    using MS = nav_interfaces::msg::MappingState;
    std::string map_name;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name = map_name_;
      error_code_ = MS::E_NONE;
    }
    is_saving_ = true;  // 进入保存/收尾阶段（~/state 报 SAVING）
    progress_ = 0.0f;
    // 完成所有轨迹并进行最终优化（只执行一次）
    SetStage("FINISHING_TRAJECTORIES");
    if (!trajectories_finished_) {
      LOG(INFO) << "Finishing all trajectories and running final optimization...";
      mapping_node_->FinishAllTrajectories();
      SetStage("OPTIMIZING");
      mapping_node_->RunFinalOptimization();
      trajectories_finished_ = true;
      LOG(INFO) << "Trajectories finished and optimized.";
    } else {
      LOG(INFO) << "Trajectories already finished, skipping optimization.";
    }
    if (node_options_->save_tracked_pose) {
      mapping_node_->SaveOptimizedPoses(int(0));
    }
    // 写 pbstream：内部逐 node 生成全局静态子图，进度按 node 数映射到 0~90
    SetStage("SERIALIZING");
    mapping_node_->SetSaveProgressCallback([this](size_t done, size_t total) {
      progress_ = total == 0 ? 0.0f : 90.0f * static_cast<float>(done) / static_cast<float>(total);
    });
    const std::string slam_dir = FLAGS_map_folder + "/" + map_name + "/slam";
    const std::string state_filename = slam_dir + "/map.pbstream";
    mapping_node_->SerializeState(state_filename, true /* include_unfinished_submaps */);
    mapping_node_->SetSaveProgressCallback(nullptr);
    progress_ = 90.0f;
    LOG(INFO) << "State saved to '" << state_filename << "'.";
    // 写栅格/yaml/pcd（带校验；yaml 内含面积统计）
    SetStage("WRITING_GRID");
    const std::string map_filestream = slam_dir + "/map";
    const bool save_success = mapping_node_->SaveMap(map_filestream, FLAGS_resolution);
    if (!save_success) {
      LOG(ERROR) << "Failed to save map files for " << map_filestream
                 << ". Call finish_mapping again to retry.";
      is_saving_ = false;  // 保存失败，退出保存态（仍在建图，可重试）
      SetStage("");
      SetErrorCode(MS::E_MAP_SAVE_FAILED);
      return false;
    }
    LOG(INFO) << "Map saved successfully. Resetting mapping node.";
    ResetMappingNode();
    progress_ = 100.0f;
    SetStage("DONE");
    return true;
  }
  // 析构时兜底保存（不关心结果）
  void FinishMapping() {
    RunFinishPipeline();
  }
  void Publish() {
    // ~/state：结构化建图状态（唯一状态出口，仅 topic）
    state_pub_->publish(BuildMappingState());
  }
private:
  // 状态
  std::string map_name_;                       // 受 state_mutex_ 保护
  std::string map_path_;                       // 受 state_mutex_ 保护
  std::string stage_;                          // 受 state_mutex_ 保护
  int32_t error_code_{0};                      // 受 state_mutex_ 保护
  std::mutex state_mutex_;
  std::atomic<float> progress_{0.0f};          // 0~100，定时器与保存线程并发读写
  std::atomic<bool> is_mapping_{false};
  std::atomic<bool> is_saving_{false};         // 正在保存/收尾建图（~/state 报 SAVING）
  std::atomic<bool> trajectories_finished_{false};  // 标记轨迹是否已完成和优化

  // Cartographer 持有
  std::unique_ptr<cartographer_ros::Node> mapping_node_;
  std::unique_ptr<MappingNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  // ROS2 通信
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<nav_interfaces::msg::MappingState>::SharedPtr state_pub_;

  rclcpp::CallbackGroup::SharedPtr ops_group_;
  rclcpp::Service<nav_interfaces::srv::StartMapping>::SharedPtr start_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mapping_srv_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_show_map_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_show_map_client_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // 在ros2中，要优先初始化rclcpp，否则无法识别launch中的变量

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto node = std::make_shared<cartographer_ros::MappingNode>();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
