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
#include "nav_interfaces/srv/set_map_name.hpp"
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
  : rclcpp::Node("mapping_node")
  {
    // Publisher
    // ~/state：结构化建图状态（唯一状态出口，FQN /mapping_node/state；状态仅经 topic 发布）
    state_pub_ = this->create_publisher<nav_interfaces::msg::MappingState>("~/state", 10);

    start_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/start_mapping",
      std::bind(&MappingNode::StartMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    finish_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/finish_mapping",
      std::bind(&MappingNode::FinishMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    cancel_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/cancel_mapping",
      std::bind(&MappingNode::CancelMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    save_map_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "~/save_map",
      std::bind(&MappingNode::SaveMapCallback, this, std::placeholders::_1, std::placeholders::_2));

    set_map_name_srv_ = this->create_service<nav_interfaces::srv::SetMapName>(
      "~/set_map_name",
      std::bind(&MappingNode::SetMapName, this, std::placeholders::_1, std::placeholders::_2));

    // 建图状态仅经 ~/state(topic) 发布，不提供查询 service

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
  nav_interfaces::msg::MappingState BuildMappingState() const {
    nav_interfaces::msg::MappingState s;
    if (is_saving_) {
      s.status = nav_interfaces::msg::MappingState::SAVING;
    } else if (is_mapping_) {
      s.status = nav_interfaces::msg::MappingState::MAPPING;
    } else {
      s.status = nav_interfaces::msg::MappingState::IDLE;
    }
    s.map_name = map_name_;
    return s;
  }

  void StartMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (is_mapping_) {
      LOG(ERROR) << "Mapping node is already running.";
      res->success = false;
      res->message = "Already running";
      return;
    }

    if (FLAGS_map_folder.empty() || map_name_.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      res->success = false;
      res->message = "Missing map_folder or map_name";
      return;
    }

    std::string map_name_error;
    if (!ValidateMapName(map_name_, &map_name_error)) {
      LOG(ERROR) << "Invalid map name '" << map_name_ << "': "
                 << map_name_error;
      res->success = false;
      res->message = "Invalid map_name: " + map_name_error;
      return;
    }

    // A new mapping session always owns a clean map directory. This removes
    // stale slam/assets/versions data before any trajectory is started.
    const MapDirectoryResetResult reset_result =
        ResetMapDirectory(FLAGS_map_folder, map_name_);
    if (!reset_result.success) {
      LOG(ERROR) << "Failed to reset map directory '"
                 << reset_result.map_directory.string()
                 << "': " << reset_result.error_message;
      res->success = false;
      res->message = "Failed to reset map directory: " +
                     reset_result.error_message;
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

    res->success = true;
    res->message = "OK";
  }

  void FinishMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!is_mapping_) {
      // 即使没有在建图，也清除map_name，相当于重置
      if (!map_name_.empty()) {
        LOG(INFO) << "Mapping node is not running, but clearing map name: " << map_name_;
        map_name_.clear();
        res->success = true;
        res->message = "Map name cleared, ready for new mapping";
      } else {
        LOG(INFO) << "Mapping node is not running and no map name set.";
        res->success = true;
        res->message = "Already idle";
      }
      return;
    }
    
    is_saving_ = true;  // 进入保存/收尾阶段（~/state 报 SAVING）
    // 完成所有轨迹并进行最终优化（只执行一次）
    if (!trajectories_finished_) {
      LOG(INFO) << "Finishing all trajectories and running final optimization...";
      mapping_node_->FinishAllTrajectories();
      mapping_node_->RunFinalOptimization();
      trajectories_finished_ = true;
      LOG(INFO) << "Trajectories finished and optimized.";
    } else {
      LOG(INFO) << "Trajectories already finished, skipping optimization.";
    }

    // 保存前重置地图起点以对齐结构化环境（幂等：本次建图只应用一次，保存失败重试不会重复应用）
    ResetMapOriginToAlignStructure();

    if (node_options_->save_tracked_pose) {
      mapping_node_->SaveOptimizedPoses(int(0));
    }

    // 保存 pbstream 文件
    const std::string state_filename = FLAGS_map_folder + "/" + map_name_ + "/slam/map.pbstream";
    if (!state_filename.empty()) {
      mapping_node_->SerializeState(state_filename, true /* include_unfinished_submaps */);
      LOG(INFO) << "State saved to '" << state_filename << "'.";
    }

    // 保存地图文件（带校验）
    const std::string map_filestream = FLAGS_map_folder + "/" + map_name_ + "/slam/map";
    bool save_success = mapping_node_->SaveMap(map_filestream, FLAGS_resolution);
    
    if (!save_success) {
      LOG(ERROR) << "Failed to save map files for " << map_filestream 
                 << ". You can call finish_mapping again to retry.";
      res->success = false;
      res->message = "Failed to save map files. Please check logs and call finish_mapping again to retry.";
      is_saving_ = false;  // 保存失败，退出保存态（仍在建图）
      return;  // 保存失败，不重置节点，保持 is_mapping_ = true 和 trajectories_finished_ = true
    }
    
    LOG(INFO) << "Map saved successfully. Resetting mapping node.";
    
    // 保存成功后重置建图节点和标志
    ResetMappingNode();

    res->success = true;
    res->message = "OK";
  }

  void CancelMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!is_mapping_) {
      // 即使没有在建图，也清除map_name，相当于重置
      if (!map_name_.empty()) {
        LOG(INFO) << "Mapping node is not running, but clearing map name: " << map_name_;
        map_name_.clear();
        res->success = true;
        res->message = "Map name cleared, ready for new mapping";
      } else {
        LOG(INFO) << "Mapping node is not running and no map name set.";
        res->success = true;
        res->message = "Already idle";
      }
      return;
    }
    
    LOG(INFO) << "Cancelling mapping without saving. Resetting mapping node.";
    
    // 停止显示地图
    StopShowMap();
    
    // 直接重置建图节点，不保存任何数据
    ResetMappingNode();

    res->success = true;
    res->message = "Mapping cancelled successfully";
  }

  void SaveMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!is_mapping_) {
      LOG(INFO) << "Mapping node is not running.";
      res->success = false;
      res->message = "Mapping not running";
      return;
    }
    
    // 先停止显示地图
    StopShowMap();
    
    // 执行完整的结束流程
    FinishMapping();
    
    // 检查是否成功（通过 is_mapping_ 状态判断）
    if (is_mapping_) {
      // 如果 is_mapping_ 仍为 true，说明 FinishMapping 中的保存失败了
      res->success = false;
      res->message = "Failed to save map files. Please check logs.";
    } else {
      res->success = true;
      res->message = "OK";
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
    
    mapping_node_.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    map_name_.clear();
    is_mapping_ = false;
    is_saving_ = false;
    trajectories_finished_ = false;  // 重置标志
    map_origin_reset_done_ = false;  // 重置标志

    LOG(INFO) << "Mapping node reset completed.";
  }

  // 保存地图前重置地图起点（整体旋转使栅格对齐结构化环境）：把位姿图所有
  // 全局位姿变换到新坐标系，使后续 SerializeState/SaveMap 以新坐标系产出
  // pbstream 与地图文件。
  // 调用前提：FinishAllTrajectories + RunFinalOptimization 已完成、尚未
  // SerializeState（pbstream 内的位姿图与 global_static_submap 都从位姿图
  // 节点重建，自然落在新系）。
  // 幂等：一次建图会话只决策一次（包括"估计失败 → 不重置"的决策），
  // 保存失败重试不会重复估计/应用。
  //
  // 新原点朝向 new_yaw 由全局线特征在线估计（位姿图节点线特征聚合 →
  // 结构化方向直方图 → 正交主方向，见 EstimateStructureAlignmentYaw），
  // 位置 (x, y) 为预留接口，纯角度重置时保持 0，即绕 map 原点
  // ——机器人建图起点——旋转。估计失败（无可靠正交结构）时跳过重置。
  void ResetMapOriginToAlignStructure() {
    if (!node_options_->reset_map_origin_to_align_structure) {
      return;
    }
    if (map_origin_reset_done_) {
      LOG(INFO) << "Map origin already reset in this mapping session, skipping.";
      return;
    }

    // 从全局线特征估计对齐角度
    cartographer::mapping::StructureAlignmentOptions align_options;
    align_options.min_line_length = node_options_->reset_map_origin_min_line_length;
    align_options.min_inliers = node_options_->reset_map_origin_min_inliers;
    align_options.min_observations = node_options_->reset_map_origin_min_observations;
    align_options.max_candidate_lines = node_options_->reset_map_origin_max_candidate_lines;
    align_options.orthogonality_tolerance_deg =
        node_options_->reset_map_origin_orthogonality_tolerance_deg;
    align_options.min_secondary_support_ratio =
        node_options_->reset_map_origin_min_secondary_support_ratio;
    const auto estimated_yaw =
        mapping_node_->EstimateStructureAlignmentYaw(align_options);
    if (!estimated_yaw.has_value()) {
      LOG(WARNING) << "No reliable orthogonal structure found in map line "
                      "features, skipping map origin reset (identity kept).";
      map_origin_reset_done_ = true;  // 决策已完成（结果 = 恒等），不再重试
      return;
    }

    // 位置 (x, y) 为预留接口，纯角度重置保持 0
    const double new_x = 0.0;
    const double new_y = 0.0;
    const double new_yaw = estimated_yaw.value();

    // T_new_origin：新原点在旧地图坐标系下的位姿；施加其逆变换（旧系 → 新系）
    const cartographer::transform::Rigid3d T_new_origin(
        Eigen::Vector3d(new_x, new_y, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ())));
    LOG(INFO) << "Resetting map origin to align with structure: "
              << "new origin in old map frame x=" << new_x
              << ", y=" << new_y << ", yaw=" << new_yaw << " rad ("
              << (new_yaw * 180.0 / M_PI) << " deg)";
    mapping_node_->ApplyMapOriginTransform(T_new_origin.inverse());
    map_origin_reset_done_ = true;
  }

  void FinishMapping() {
    is_saving_ = true;  // 进入保存/收尾阶段（~/state 报 SAVING）
    // 结束优化与保存
    mapping_node_->FinishAllTrajectories();
    mapping_node_->RunFinalOptimization();

    // 保存前重置地图起点以对齐结构化环境（幂等：本次建图只应用一次）
    ResetMapOriginToAlignStructure();

    if (node_options_->save_tracked_pose) {
      mapping_node_->SaveOptimizedPoses(int(0));
    }

    const std::string state_filename = FLAGS_map_folder + "/" + map_name_ + "/slam/map.pbstream";
    if (!state_filename.empty()) {
      mapping_node_->SerializeState(state_filename, true /* include_unfinished_submaps */);
      LOG(INFO) << "State saved to '" << state_filename << "'.";
    }

    const std::string map_filestream = FLAGS_map_folder + "/" + map_name_ + "/slam/map";
    bool save_success = mapping_node_->SaveMap(map_filestream, FLAGS_resolution);
    
    if (!save_success) {
      LOG(ERROR) << "Failed to save map files in FinishMapping()";
      is_saving_ = false;  // 保存失败，退出保存态
      return;
    }
    
    LOG(INFO) << "Map saved successfully in FinishMapping(). Resetting mapping node.";
    
    // 保存成功后重置建图节点
    ResetMappingNode();
  }

  void SetMapName(const std::shared_ptr<nav_interfaces::srv::SetMapName::Request> req,
                  std::shared_ptr<nav_interfaces::srv::SetMapName::Response> res)
  {
    if (FLAGS_map_folder.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      res->success = false;
      return;
    }

    std::string map_name_error;
    if (!ValidateMapName(req->map_name, &map_name_error)) {
      LOG(ERROR) << "Invalid map name '" << req->map_name << "': "
                 << map_name_error;
      res->success = false;
      return;
    }
    if (!EnsureDirectoryExists(FLAGS_map_folder)) {
      res->success = false;
      return;
    }

    map_name_ = req->map_name;
    const std::filesystem::path map_directory =
        std::filesystem::path(FLAGS_map_folder) / map_name_;
    RCLCPP_INFO(this->get_logger(),
                "Set map name to %s; directory will be recreated when mapping starts",
                map_directory.c_str());
    res->success = true;
  }

  void Publish() {
    // ~/state：结构化建图状态（唯一状态出口，仅 topic）
    state_pub_->publish(BuildMappingState());
  }
private:
  // 状态
  std::string map_name_;
  bool is_mapping_{false};
  bool is_saving_{false};               // 正在保存/收尾建图（~/state 报 SAVING）
  bool trajectories_finished_{false};  // 标记轨迹是否已完成和优化
  bool map_origin_reset_done_{false};  // 本次建图会话是否已应用过地图原点重置（防重试时重复施加）

  // Cartographer 持有
  std::unique_ptr<cartographer_ros::Node> mapping_node_;
  std::unique_ptr<MappingNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  // ROS2 通信
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<nav_interfaces::msg::MappingState>::SharedPtr state_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
  rclcpp::Service<nav_interfaces::srv::SetMapName>::SharedPtr set_map_name_srv_;

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
