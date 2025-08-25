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

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int16.hpp"
#include "dcm_srvs/srv/set_map_name.hpp"
#include "dcm_srvs/srv/get_string.hpp"

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/mapping/node.h"
#include "cartographer_ros/options/mapping_node_options.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "boost/filesystem.hpp"

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
  MappingNode()
  : rclcpp::Node("mapping_node")
  {
    // Publisher
    mapping_status_pub_ = this->create_publisher<std_msgs::msg::Int16>("mapping_status", 10);



    // Services (在节点命名空间下暴露，与 ROS1 "~" 私有等价)
    start_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "start_mapping",
      std::bind(&MappingNode::StartMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    finish_mapping_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "finish_mapping",
      std::bind(&MappingNode::FinishMappingCallback, this, std::placeholders::_1, std::placeholders::_2));

    set_map_name_srv_ = this->create_service<dcm_srvs::srv::SetMapName>(
      "set_map_name",
      std::bind(&MappingNode::SetMapName, this, std::placeholders::_1, std::placeholders::_2));

    get_mapping_status_srv_ = this->create_service<dcm_srvs::srv::GetString>(
      "get_mapping_status",
      std::bind(&MappingNode::GetMappingStatus, this, std::placeholders::_1, std::placeholders::_2));

    // Clients for show map control (使用全路径，等价于 ROS1 版本)
    start_show_map_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/cartographer_occupancy_grid_node/start_mapping");
    stop_show_map_client_ = this->create_client<std_srvs::srv::Trigger>(
      "/cartographer_occupancy_grid_node/stop_mapping");

    // Timer: 5Hz -> 0.2s
    publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&MappingNode::Publish, this));
  }

  ~MappingNode() override
  {
    if (is_mapping_) {
      FinishMapping();
    }
  }

private:
  // ---- Service Callbacks (ROS2 签名) ----
  void GetMappingStatus(const std::shared_ptr<dcm_srvs::srv::GetString::Request> /*req*/,
                        std::shared_ptr<dcm_srvs::srv::GetString::Response> res)
  {
    res->data = is_mapping_ ? "mapping" : "idle";
    res->success = true;
  }

  void StartMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    StartShowMap();

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

    // 创建目录
    if (!EnsureDirectoryExists(FLAGS_map_folder)) {
      res->success = false;
      res->message = "Failed to create map_folder";
      return;
    }
    const std::string this_map_folder = FLAGS_map_folder + "/" + map_name_;
    if (!EnsureDirectoryExists(this_map_folder)) {
      res->success = false;
      res->message = "Failed to create map subfolder";
      return;
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
    node_ = std::make_unique<Node>(*node_options_, std::move(map_builder), tf_buffer_.get(),
                                   FLAGS_collect_metrics);
    // 4. 读取pstream文件（子图、约束、轨迹等信息）
    // TODO： 扩建模式下，才需要 load_state_filename
    if (!FLAGS_load_state_filename.empty()) {
      node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }
    // 5. 创建轨迹优化器，订阅ros消息
    if (FLAGS_start_trajectory_with_default_topics) {
      node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
    }

    is_mapping_ = true;

    res->success = true;
    res->message = "OK";
  }

  void FinishMappingCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!is_mapping_) {
      LOG(INFO) << "Mapping node is not running.";
      res->success = true;
      res->message = "Already idle";
      return;
    }
    FinishMapping();
    StopShowMap();

    res->success = true;
    res->message = "OK";
  }


  // ---- Helpers ----
  bool EnsureDirectoryExists(const std::string& path) {
    // 建议在 ROS2 中改用 std::filesystem（C++17）
    namespace fs = std::filesystem;
    try {
      if (path.empty()) return false;
      if (fs::exists(path)) return true;
      return fs::create_directories(path);
    } catch (const std::exception& e) {
      LOG(ERROR) << "Filesystem error for path '" << path << "': " << e.what();
      return false;
    }
  }

  // 同步调用 Trigger（等待存在 + 调用）
  bool CallTriggerSync(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client) {
    using namespace std::chrono_literals;
    if (!client) return false;

    // 等 1s 服务可用
    if (!client->wait_for_service(1s)) {
      RCLCPP_ERROR(this->get_logger(), "%s not exists", client->get_service_name());
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    // 简单同步等待（可根据需要改为异步）
    auto status = future.wait_for(2s);
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

  void FinishMapping() {
    // 结束优化与保存
    node_->FinishAllTrajectories();
    node_->RunFinalOptimization();

    if (node_options_->save_tracked_pose) {
      node_->SaveOptimizedPoses(int(0));
    }

    const std::string state_filename = FLAGS_map_folder + "/" + map_name_ + "/map.pbstream";
    if (!state_filename.empty()) {
      node_->SerializeState(state_filename, true /* include_unfinished_submaps */);
      LOG(INFO) << "State saved to '" << state_filename << "'.";
    }

    const std::string map_filestream = FLAGS_map_folder + "/" + map_name_ + "/map";
    node_->SaveMap(map_filestream, FLAGS_resolution);

    node_.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    map_name_.clear();
    is_mapping_ = false;
  }

  void SetMapName(const std::shared_ptr<dcm_srvs::srv::SetMapName::Request> req,
                  std::shared_ptr<dcm_srvs::srv::SetMapName::Response> res)
  {
    map_name_ = req->map_name;

    if (FLAGS_map_folder.empty() || map_name_.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      res->success = false;
      return;
    }

    if (!EnsureDirectoryExists(FLAGS_map_folder)) {
      res->success = false;
      return;
    }
    const std::string this_map_folder = FLAGS_map_folder + "/" + map_name_;
    if (!EnsureDirectoryExists(this_map_folder)) {
      res->success = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Set map name to %s", this_map_folder.c_str());
    res->success = true;
  }

  void Publish() {
    auto msg = std_msgs::msg::Int16();
    msg.data = is_mapping_ ? 1 : -1;
    mapping_status_pub_->publish(msg);
  }
private:
  // 状态
  std::string map_name_;
  bool is_mapping_{false};

  // Cartographer 持有
  std::unique_ptr<Node> node_;
  std::unique_ptr<MappingNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  // ROS2 通信
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mapping_status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mapping_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr finish_mapping_srv_;
  rclcpp::Service<dcm_srvs::srv::SetMapName>::SharedPtr set_map_name_srv_;
  rclcpp::Service<dcm_srvs::srv::GetString>::SharedPtr get_mapping_status_srv_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_show_map_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_show_map_client_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty()) << "-configuration_basename is missing.";

  rclcpp::init(argc, argv);
  auto node = std::make_shared<cartographer_ros::MappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
