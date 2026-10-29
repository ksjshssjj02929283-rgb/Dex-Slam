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

#include <algorithm>
#include <iostream>
#include <memory>
#include <filesystem>
#include <thread>
#include <chrono>
#include <fstream>
#include <cmath>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <atomic>
#include <mutex>
#include "nav_interfaces/srv/start_mapping.hpp"
#include "nav_interfaces/msg/mapping_state.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"


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

using SteadyClock = std::chrono::steady_clock;
constexpr double kBatteryThreshold = 60.;
constexpr double kBatteryTimeoutMs = 10000.;
constexpr int kFaultReportGraceMs = 500;

struct BatterySnapshot {
  double percentage;
  SteadyClock::time_point expires_at;
  bool fresh(SteadyClock::time_point now) const { return now <= expires_at; }
  bool low() const { return percentage < kBatteryThreshold; }
};

bool ParseNumber(const std::string& text, double* value) {
  try {
    size_t end = 0;
    *value = std::stod(text, &end);
    return end == text.size() && std::isfinite(*value);
  } catch (const std::exception&) {
    return false;
  }
}

std::optional<BatterySnapshot> ParseBatteryDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray& msg,
    SteadyClock::time_point now) {
  std::optional<double> percentage;
  double age = 0.;
  bool invalid = false;
  for (const auto& status : msg.status) {
    if (status.name.size() < 10 ||
        status.name.compare(status.name.size() - 10, 10, "PowerBoard") != 0)
      continue;
    std::optional<std::string> percentage_text, age_text;
    for (const auto& value : status.values) {
      if (value.key == "battery_percentage") percentage_text = value.value;
      if (value.key == "battery_age_ms") age_text = value.value;
    }
    if (!percentage_text) continue;
    double current = 0., current_age = 0.;
    if (!age_text || !ParseNumber(*percentage_text, &current) ||
        !ParseNumber(*age_text, &current_age) || current < 0. ||
        current > 100. || current_age < 0. || current_age > kBatteryTimeoutMs) {
      invalid = true;
      continue;
    }
    if (percentage && *percentage != current) invalid = true;
    percentage = current;
    age = std::max(age, current_age);
  }
  if (!percentage || invalid) return std::nullopt;
  return BatterySnapshot{
      *percentage, now + std::chrono::duration_cast<SteadyClock::duration>(
                             std::chrono::duration<double, std::milli>(
                                 kBatteryTimeoutMs - age))};
}

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

    // 电量更新独立于耗时的建图操作，仅保留最新诊断。
    battery_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions diagnostic_options;
    diagnostic_options.callback_group = battery_group_;
    battery_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_agg", rclcpp::QoS(1).best_effort(),
        std::bind(&MappingNode::OnDiagnostics, this, std::placeholders::_1),
        diagnostic_options);

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

    // Keep the watchdog independent of potentially slow sensor callbacks.
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&MappingNode::Publish, this),
        battery_group_);
  }

  ~MappingNode()
  {
    if (is_mapping_) {
      FinishMapping();
    }
  }

  void SetExitCallback(std::function<void()> callback) {
    exit_callback_ = std::move(callback);
  }
  bool exit_requested() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return exit_deadline_.has_value();
  }

private:
  bool HasMappingQualityFaultLocked() const {
    return quality_fault_ && quality_fault_->fault.load();
  }

  bool HasMappingQualityFault() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return HasMappingQualityFaultLocked();
  }

  void OnDiagnostics(
      const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
    const auto battery = ParseBatteryDiagnostics(*msg, SteadyClock::now());
    std::lock_guard<std::mutex> lock(state_mutex_);
    battery_ = battery;
  }

  std::optional<BatterySnapshot> FreshBatteryLocked() const {
    if (battery_ && battery_->fresh(SteadyClock::now())) return battery_;
    return std::nullopt;
  }

  int32_t ExitErrorLocked() const {
    if (HasMappingQualityFaultLocked()) {
      return nav_interfaces::msg::MappingState::E_MAP_QUALITY_BAD;
    }
    return exit_deadline_ ? error_code_
                          : nav_interfaces::msg::MappingState::E_NONE;
  }

  void RequestExit(int32_t error_code) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (exit_deadline_) return;
    error_code_ = error_code;
    error_message_.clear();
    exit_deadline_ =
        SteadyClock::now() + std::chrono::milliseconds(kFaultReportGraceMs);
  }

  // ---- Service Callbacks (ROS2 签名) ----
  // 构建结构化建图状态（~/state topic 发布用）
  nav_interfaces::msg::MappingState BuildMappingState() {
    nav_interfaces::msg::MappingState s;
    s.header.stamp = this->now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    const auto fault = ExitErrorLocked();
    if (fault != 0) {
      s.status = nav_interfaces::msg::MappingState::ERROR;
    } else if (is_saving_) {
      s.status = nav_interfaces::msg::MappingState::SAVING;
    } else if (is_mapping_) {
      s.status = nav_interfaces::msg::MappingState::MAPPING;
    } else {
      s.status = nav_interfaces::msg::MappingState::IDLE;
    }
    s.progress = progress_.load();
    s.map_name = map_name_;
    s.map_path = map_path_;
    s.stage = fault != 0 ? "" : stage_;
    s.error_code = fault != 0
                       ? fault
                       : (error_code_ != 0 ? error_code_ : battery_error_code_);
    if (s.error_code == nav_interfaces::msg::MappingState::E_MAP_QUALITY_BAD) {
      s.error_message = "Mapping quality fault";
    } else if (s.error_code != 0 && s.error_code == error_code_) {
      s.error_message = error_message_;
    }
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
    if (!exit_deadline_) {
      error_code_ = code;
      error_message_.clear();
    }
  }

  void StartMappingCallback(const std::shared_ptr<nav_interfaces::srv::StartMapping::Request> req,
                            std::shared_ptr<nav_interfaces::srv::StartMapping::Response> res)
  {
    using MS = nav_interfaces::msg::MappingState;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto fault = ExitErrorLocked();
      if (fault != 0) {
        FillResult(res.get(), false, fault,
                   "Mapping process is exiting; restart it before starting a "
                   "new session");
        return;
      }
    }
    if (is_saving_) {
      LOG(ERROR) << "Map saving is in progress; start_mapping rejected.";
      FillResult(res.get(), false, MS::E_MAP_SAVING, "Map saving in progress; wait for ~/state to become IDLE");
      return;
    }
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
    const std::filesystem::path map_directory =
        std::filesystem::path(FLAGS_map_folder) / req->map_name;
    // 重名只看「是否已有一张可加载的地图」（与 get_maps 同口径）；取消 / 中断建图
    // 留下的残留目录不算，照常开始，残留由 ResetMapDirectory 归档进回收站
    const bool previous_is_map = IsLoadableMapDirectory(map_directory);
    if (previous_is_map && !req->overwrite) {
      LOG(ERROR) << "Map '" << req->map_name << "' already exists and overwrite is false.";
      FillResult(res.get(), false, MS::E_MAP_NAME_EXISTS,
                 "Map already exists; set overwrite=true to recreate: " + req->map_name);
      return;
    }
    std::optional<BatterySnapshot> battery;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      battery = FreshBatteryLocked();
      if (!battery) {
        error_code_ = MS::E_MAP_SENSOR_ABNORMAL;
        error_message_ =
            "Battery diagnostics missing, invalid or stale; mapping was "
            "not started; retry with fresh data";
        FillResult(res.get(), false, MS::E_MAP_SENSOR_ABNORMAL,
                   "Battery diagnostics missing, invalid or stale; mapping was "
                   "not started; retry with fresh data");
        return;
      }
    }
    if (battery->low()) {
      FillResult(res.get(), false, MS::E_MAP_BATTERY_LOW,
                 "Battery " + std::to_string(battery->percentage) +
                     "% is below " +
                     std::to_string(kBatteryThreshold) +
                     "%; mapping was not started; mapping process will exit");
      RequestExit(MS::E_MAP_BATTERY_LOW);
      return;
    }
    node_options_ = std::make_unique<MappingNodeOptions>();
    trajectory_options_ = std::make_unique<TrajectoryOptions>();

    // 1. 解析节点和轨迹的配置文件(tie会自动根据参数名，对应node_options和trajectory_options)
    std::tie(*node_options_, *trajectory_options_) =
      LoadMappingOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    if (!EnsureDirectoryExists(FLAGS_map_folder)) {
      FillResult(res.get(), false, MS::E_MAP_FOLDER_MISSING,
                 "map_folder does not exist and cannot be created: " + FLAGS_map_folder);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      battery_error_code_ = 0;
      map_name_ = req->map_name;
      map_path_ = FLAGS_map_folder + "/" + req->map_name + "/slam";
      stage_.clear();
      error_code_ = MS::E_NONE;
      error_message_.clear();
      initial_scan_fault_ = false;
    }
    progress_ = 0.0f;
    // A new mapping session always owns a clean map directory. Existing
    // slam/assets/versions data is archived to <map_folder>/.map_trash/
    // instead of being deleted: map assets take hours to author by hand,
    // and a mistaken overwrite=true used to wipe them irrecoverably.
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
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      session_reset_ = reset_result;
      session_restore_previous_ = previous_is_map;
    }
    if (reset_result.replaced_existing && !previous_is_map) {
      LOG(INFO) << "Map directory '" << reset_result.map_directory.string()
                << "' held no loadable map (leftover of a cancelled or interrupted session); "
                   "archived to '" << reset_result.trashed_directory.string() << "'.";
    } else if (reset_result.replaced_existing) {
      if (!reset_result.trashed_directory.empty()) {
        LOG(WARNING) << "Recreated existing map directory '"
                     << reset_result.map_directory.string()
                     << "'; previous content (slam + assets + versions) "
                        "archived to '"
                     << reset_result.trashed_directory.string()
                     << "'. map_manager rotates that trash by count and size.";
      } else {
        LOG(WARNING) << "Recreated existing map directory '"
                     << reset_result.map_directory.string() << "' (removed "
                     << reset_result.removed_entries
                     << " entries, NOT archived).";
      }
    } else {
      LOG(INFO) << "Created map directory '"
                << reset_result.map_directory.string() << "'.";
    }

    // TF2 (ROS2): Buffer 绑定到本节点的 Clock
    constexpr double kTfBufferCacheTimeInSeconds = 10.0;
    (void)kTfBufferCacheTimeInSeconds; // 缓存时长由 Buffer 默认处理
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 2. 构建 map_builder
    auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options_->map_builder_options);
    // 3. 创建 Cartographer ROS Node（节点发布的topic、服务等在其中定义）
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      // The manager owns Node, so its back-reference must not keep the manager
      // alive.
      mapping_node_ = std::make_unique<cartographer_ros::Node>(
          *node_options_, std::move(map_builder), tf_buffer_,
          rclcpp::Node::SharedPtr(this, [](rclcpp::Node*) {}),
          FLAGS_collect_metrics);
      // Capture before starting trajectories; retain through teardown.
      quality_fault_ = mapping_node_->GetMappingQualityFaultFlag();
    }
    // 4. 读取pstream文件（子图、约束、轨迹等信息）
    // TODO： 扩建模式下，才需要 load_state_filename
    if (!FLAGS_load_state_filename.empty()) {
      mapping_node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }
    // 5. 创建轨迹优化器，订阅ros消息
    if (FLAGS_start_trajectory_with_default_topics) {
      mapping_node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      health_started_ = SteadyClock::now();
      is_mapping_ = true;
    }
    trajectories_finished_ = false;  // 重置标志

    StartShowMap();
    FillResult(res.get(), true, MS::E_NONE, "OK");
  }

  // finish 为触发动作：本回调同步执行收尾管线，期间经 ~/state 上报
  // progress/stage。 状态定时器与电量诊断共用独立回调组，与 ops_group_ 由
  // MultiThreadedExecutor 并行调度， 因此 SAVING 期间 ~/state 照常以 5 Hz
  // 发布；调用方无需等待本响应，盯 ~/state 即可。
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
    if (HasMappingQualityFault() || exit_requested()) {
      res->success = false;
      res->message = "Mapping fault; process is exiting";
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
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      health_started_.reset();
    }
    progress_ = 0.0f;
    SetStage("");
    SetErrorCode(nav_interfaces::msg::MappingState::E_NONE);
    LOG(INFO) << "Cancelling mapping without saving. Resetting mapping node.";
    // 停止显示地图
    StopShowMap();
    // 直接重置建图节点，不保存任何数据
    ResetMappingNode();
    MapDirectoryResetResult session;
    bool restore_previous = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_name_.clear();
      map_path_.clear();
      session = session_reset_;
      restore_previous = session_restore_previous_;
      session_reset_ = MapDirectoryResetResult{};
      session_restore_previous_ = false;
    }
    // 撤销本次建图对地图目录的改动：删掉未保存的目录，覆盖建图时还原原来的地图，
    // 这样取消后同名可以直接再建，覆盖了的旧图也不会凭空消失
    const MapDirectoryDiscardResult discard = DiscardMapDirectory(session, restore_previous);
    res->success = true;
    if (!discard.success) {
      LOG(ERROR) << "Mapping cancelled, but cleaning the unsaved map directory failed: "
                 << discard.error_message;
      res->message = "Mapping cancelled, but map directory cleanup failed: " + discard.error_message;
    } else if (discard.restored_previous) {
      LOG(INFO) << "Mapping cancelled; previous map restored to '" << session.map_directory.string() << "'.";
      res->message = "Mapping cancelled; previous map restored";
    } else {
      res->message = "Mapping cancelled successfully";
    }
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
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      health_started_.reset();
    }

    // Finish before marking idle; the destructor also finishes as a fallback.
    // Retain the shared fault flag, including faults raised by flushed samples.
    if (mapping_node_) mapping_node_->FinishAllTrajectories();
    std::unique_ptr<cartographer_ros::Node> finished_node;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      finished_node = std::move(mapping_node_);
      battery_error_code_ = 0;
      is_mapping_ = false;
      is_saving_ = false;
    }
    finished_node.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    trajectories_finished_ = false;  // 重置标志
    // 地图原点重置的幂等标志在 Node 内部，mapping_node_ 重建时自然复位

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
      if (!exit_deadline_) {
        error_code_ = MS::E_NONE;
        error_message_.clear();
      }
      health_started_.reset();
    }
    // Exception firewall. Everything below touches the live pose graph, the
    // submap list and the filesystem; an escaping exception would unwind into
    // the service callback and end in std::terminate() -> SIGABRT, discarding
    // the entire in-memory map that this very call is trying to write out.
    // Convert it into the same "failed, retryable" outcome the explicit
    // save-failure branch produces, so a retry can still recover the map.
    try {
      return RunFinishPipelineUnlocked(map_name);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Finish pipeline aborted by exception: " << e.what()
                 << ". Mapping state is kept; call finish_mapping again to retry.";
      is_saving_ = false;
      SetStage("");
      SetErrorCode(MS::E_MAP_SAVE_FAILED);
      return false;
    } catch (...) {
      LOG(ERROR) << "Finish pipeline aborted by a non-std::exception. "
                    "Mapping state is kept; call finish_mapping again to retry.";
      is_saving_ = false;
      SetStage("");
      SetErrorCode(MS::E_MAP_SAVE_FAILED);
      return false;
    }
  }

  bool RunFinishPipelineUnlocked(const std::string& map_name) {
    using MS = nav_interfaces::msg::MappingState;
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

    // 保存前重置地图起点以对齐结构化环境（来自 cdd81f3；rebase 到 apply_for_xwiz
    // 后，原先挂在旧 FinishMapping 上的调用移植到重构后的 RunFinishPipeline）。
    // 调用前提：FinishAllTrajectories + RunFinalOptimization 已完成、尚未
    // SaveOptimizedPoses / SerializeState —— 这样 pbstream / png / yaml / pcd
    // 全部产物天然落在新坐标系，不需要事后各自变换。
    // 幂等性由 Node::ResetMapOriginToAlignStructure 内部标志保证：保存失败重试
    // 不会重复施加旋转。
    // 刻意不新增 stage：MappingState.msg 的 stage 取值是对外契约
    //（FINISHING_TRAJECTORIES / OPTIMIZING / SERIALIZING / WRITING_GRID / DONE），
    // 对齐属于收尾优化的一部分，沿用当前 stage，不在冲突解决里悄悄改契约。
    mapping_node_->ResetMapOriginToAlignStructure();
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
    {
      // 已保存：本次会话的目录改动成立，之后的 cancel 不再回滚
      std::lock_guard<std::mutex> lock(state_mutex_);
      session_reset_ = MapDirectoryResetResult{};
      session_restore_previous_ = false;
    }
    progress_ = 100.0f;
    SetStage("DONE");
    return true;
  }
  // 析构时兜底保存（不关心结果）
  void FinishMapping() {
    RunFinishPipeline();
  }
  // Runs even when missing input has stopped all matching callbacks.
  void CheckMappingData() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!is_mapping_ || is_saving_ || !health_started_ || !quality_fault_)
      return;
    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> health_lock(quality_fault_->mutex);
    const auto topic_for = [this](const std::string& id) {
      return get_node_topics_interface()->resolve_topic_name(id);
    };
    std::string insufficient;
    for (const auto& entry : quality_fault_->initial_scans) {
      const auto& scan = entry.second;
      if (!scan.insufficient()) continue;
      if (!insufficient.empty()) insufficient += "; ";
      insufficient +=
          "topic=" + topic_for(entry.first) +
          " outside=" + std::to_string(scan.outside) +
          " total=" + std::to_string(scan.total) + " ratio=" +
          std::to_string(scan.total == 0
                             ? 0.
                             : static_cast<double>(scan.outside) / scan.total);
    }
    if (!insufficient.empty() && (!exit_deadline_ || initial_scan_fault_)) {
      const std::string message =
          "雷达有效点数不足，可能是受遮挡或超量程无回波: " + insufficient +
          " frame=base_footprint rect=(-0.3,0.3)x(-0.3,0.3)";
      if (message != error_message_) LOG(ERROR) << message;
      error_message_ = message;
      error_code_ = nav_interfaces::msg::MappingState::E_MAP_SENSOR_ABNORMAL;
      initial_scan_fault_ = true;
      if (!exit_deadline_)
        exit_deadline_ = now + std::chrono::milliseconds(kFaultReportGraceMs);
      return;
    }
    if (exit_deadline_) return;
    // Allow initial TF/submap creation; normal keyframe interval is about 2 s.
    if (now - *health_started_ < std::chrono::seconds(15)) return;
    std::string unavailable;
    double age = 0.;
    bool never_received = false;
    for (const auto& entry : quality_fault_->inputs) {
      const auto last = entry.second.received.value_or(*health_started_);
      if (now - last >= std::chrono::seconds(5)) {
        unavailable = entry.first;
        never_received = !entry.second.received.has_value();
        age = std::chrono::duration<double>(now - last).count();
        break;
      }
    }
    if (unavailable.empty()) {
      const auto last =
          quality_fault_->last_valid_match.value_or(*health_started_);
      if (now - last >= std::chrono::seconds(15)) {
        unavailable = "valid_match";
        age = std::chrono::duration<double>(now - last).count();
      }
    }
    if (unavailable.empty()) return;
    error_code_ = nav_interfaces::msg::MappingState::E_MAP_SENSOR_ABNORMAL;
    error_message_ =
        unavailable == "valid_match"
            ? "Valid matching timeout: source=valid_match timeout_s=15"
            : std::string(never_received
                              ? "No valid sensor data received: topic="
                              : "Valid sensor data timeout: topic=") +
                  topic_for(unavailable) + " timeout_s=5";
    error_message_ += " age_s=" + std::to_string(age);
    exit_deadline_ = now + std::chrono::milliseconds(kFaultReportGraceMs);
    LOG(ERROR) << error_message_;
    LOG(ERROR) << "Mapping data unavailable source=" << unavailable
               << " age_s=" << age << " error_code=" << error_code_;
  }

  void Publish() {
    const bool already_exiting = exit_requested();
    if (HasMappingQualityFault() && !already_exiting) {
      RequestExit(nav_interfaces::msg::MappingState::E_MAP_QUALITY_BAD);
    }
    CheckMappingData();
    if (!already_exiting && exit_requested()) {
      if (stop_show_map_client_->service_is_ready()) {
        try {
          stop_show_map_client_->async_send_request(
              std::make_shared<std_srvs::srv::Trigger::Request>());
        } catch (const std::exception& e) {
          LOG(WARNING) << "Cannot stop map display during exit: " << e.what();
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_mapping_ || is_saving_) {
        if (const auto battery = FreshBatteryLocked()) {
          battery_error_code_ =
              battery->low()
                  ? nav_interfaces::msg::MappingState::E_MAP_BATTERY_LOW
                  : 0;
        } else {
          RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 10000,
              "Mapping battery diagnostics unavailable or stale");
        }
      }
    }
    state_pub_->publish(BuildMappingState());
    bool exit_due;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      exit_due = exit_deadline_ && SteadyClock::now() >= *exit_deadline_;
    }
    if (exit_due && exit_callback_) exit_callback_();
  }
 private:
  // 状态
  std::string map_name_;                       // 受 state_mutex_ 保护
  std::string map_path_;                       // 受 state_mutex_ 保护
  std::string stage_;                          // 受 state_mutex_ 保护
  std::string error_message_;
  bool initial_scan_fault_ = false;
  int32_t error_code_{0};                      // 受 state_mutex_ 保护
  std::mutex state_mutex_;
  // Protected by state_mutex_; retained after mapping_node_ is released.
  std::shared_ptr<const cartographer::mapping::MappingQualityState>
      quality_fault_;
  std::optional<BatterySnapshot> battery_;
  int32_t battery_error_code_ = 0;
  std::optional<SteadyClock::time_point> exit_deadline_;
  std::optional<SteadyClock::time_point> health_started_;
  std::function<void()> exit_callback_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      battery_sub_;
  // 本次建图会话对地图目录做的重置（取消时据此回滚），受 state_mutex_ 保护
  MapDirectoryResetResult session_reset_;
  bool session_restore_previous_ = false;      // 开始时原目录是一张可加载地图，取消时要还原
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
  rclcpp::CallbackGroup::SharedPtr battery_group_;
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

  {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    node->SetExitCallback([&executor] { executor.cancel(); });
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node->SetExitCallback({});
  }
  const bool failed = node->exit_requested();
  node.reset();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
