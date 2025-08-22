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

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "absl/memory/memory.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/localization/localization_node.h"
#include "cartographer_ros/options/localization_node_options.h"
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
namespace {

class LocalizationROSNode
{
public:
  LocalizationROSNode()
  {
    ros::NodeHandle nh_private("~");
    start_localization_srv_ = nh_private.advertiseService("start_localization", &LocalizationROSNode::StartLocalizationCallback, this);
    stop_localization_srv_ = nh_private.advertiseService("stop_localization", &LocalizationROSNode::StopLocalizationCallback, this);
    reset_localization_srv_ = nh_private.advertiseService("reset_localization", &LocalizationROSNode::ResetLocalizationCallback, this);

    StartLocalization();
  }

  bool StartLocalization()
  {
    // 首先获取map name和map folder
    // std::string map_name, map_folder;

    // ros::param::get("/map_manager/map_name", map_name);
    // ros::param::get("/map_manager/map_folder", map_folder);

    // if (map_name.empty() || map_folder.empty()) {
    //   ROS_ERROR("map_name or map_folder is empty");
    //   return false;
    // }

    // std::string this_map_folder = map_folder + "/" + map_name_;

    if (is_activating_) {
      LOG(ERROR) << "Localization node is already activated.";
      return false;
    }

    if (node_ != nullptr) {
      StopLocalization();
      LOG(WARNING) << "Previous localization node is not stopped, stop it first.";
      return false;
    }

    if (FLAGS_map_folder.empty() || FLAGS_map_name.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      return false;
    }

    if (!boost::filesystem::exists(FLAGS_map_folder)) {
      LOG(ERROR) << "Map folder does not exist: " << FLAGS_map_folder;
      return false;
    }

    std::string this_map_folder = FLAGS_map_folder + "/" + FLAGS_map_name;
    if (!boost::filesystem::exists(this_map_folder)) {
      LOG(ERROR) << "Map: " << this_map_folder << " is not exist.";
      return false;
    }

    constexpr double kTfBufferCacheTimeInSeconds = 10.;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(::ros::Duration(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    LOG(INFO) << "Start LocalizationNode";

    node_options_ = std::make_unique<LocalizationNodeOptions>();
    trajectory_options_ = std::make_unique<TrajectoryOptions>();
    // 1. 解析节点和轨迹的配置文件(tie会自动根据参数名，对应node_options和trajectory_options)
    std::tie(*node_options_, *trajectory_options_) =
      LoadLocalizationNodeOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    // 2. 创建map_builder
    auto localization_updater =
      cartographer::localization::CreateLocalizationUpdater(node_options_->localization_updater_options);
    // 3. 创建ros node（节点发布的topic、服务等在其中定义）
    node_ = std::make_unique<localization::LocalizationNode>(*node_options_, std::move(localization_updater), tf_buffer_.get());
    // 4. 读取pstream文件（子图、约束、轨迹等信息）
    // TODO： 扩建模式下，才需要 load_state_filename
    std::string state_filename = FLAGS_map_folder + "/" + FLAGS_map_name + "/map.pbstream";
    if (!boost::filesystem::exists(state_filename)) {
      LOG(ERROR) << "Map: " << FLAGS_map_name << " STATE FILE is not exist.";
      return false;
    }

    std::string yaml_filepath = FLAGS_map_folder + "/" + FLAGS_map_name + "/map.yaml";
    if (!boost::filesystem::exists(yaml_filepath)) {
      LOG(ERROR) << "Map: " << FLAGS_map_name << " YAML FILE is not exist.";
      return false;
    }

    node_->LoadState(state_filename, FLAGS_load_frozen_state);
    node_->LoadMapData(yaml_filepath);

    if (FLAGS_start_trajectory_with_default_topics) {
      node_->StartTrajectoryWithDefaultTopics(*trajectory_options_);
    }

    is_activating_ = true;
    return true;
  }

  void StopLocalization()
  {
    if (is_activating_) {
      LOG(WARNING) << "Localization node is already deactivated.";
      return;
    }

    LOG(INFO) << "Stop LocalizationNode";
    // 6. 结束优化
    if (node_ != nullptr) {
      node_->FinishAllTrajectories();
    }

    node_.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
  }

  bool StartLocalizationCallback(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res)
  {
    StartFilter();
    res.success = StartLocalization();
    if (!res.success) {
      ROS_ERROR("Start localization failed");
    }
    return res.success;
  }

  bool StopLocalizationCallback(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res)
  {
    StopLocalization();
    StopFilter();
    res.success = true;
    return res.success;
  }

  bool ResetLocalizationCallback(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res)
  {
    ROS_INFO("Reset localization");
    StopLocalization();
    ResetFilter();
    res.success = StartLocalization();
    if (!res.success) {
      ROS_ERROR("Start localization failed");
    }
    return res.success;
  }

  bool StartFilter()
  {
    ros::ServiceClient client =
      ros::NodeHandle().serviceClient<std_srvs::Trigger>("/robot_localization/start_filter");
    std_srvs::Trigger cmd;

    if (!client.waitForExistence(ros::Duration(1.0)) ||
        !client.exists())
    {
      ROS_ERROR("%s not exists", client.getService().c_str());
      return false;
    }

    if (client.call(cmd)) {
      ROS_INFO("%s call success", client.getService().c_str());
      return true;
    } else {
      ROS_ERROR("%s call failed", client.getService().c_str());
      return false;
    }

    return false;
  }

  bool StopFilter()
  {
    ros::ServiceClient client =
      ros::NodeHandle().serviceClient<std_srvs::Trigger>("/robot_localization/stop_filter");
    std_srvs::Trigger cmd;

    if (!client.waitForExistence(ros::Duration(1.0)) ||
        !client.exists())
    {
      ROS_ERROR("%s not exists", client.getService().c_str());
      return false;
    }

    if (client.call(cmd)) {
      ROS_INFO("%s call success", client.getService().c_str());
      return true;
    } else {
      ROS_ERROR("%s call failed", client.getService().c_str());
      return false;
    }

    return false;
  }

  bool ResetFilter()
  {
    ros::ServiceClient client =
      ros::NodeHandle().serviceClient<std_srvs::Trigger>("/robot_localization/reset_filter");
    std_srvs::Trigger cmd;

    if (!client.waitForExistence(ros::Duration(1.0)) ||
        !client.exists())
    {
      ROS_ERROR("%s not exists", client.getService().c_str());
      return false;
    }

    if (client.call(cmd)) {
      ROS_INFO("%s call success", client.getService().c_str());
      return true;
    } else {
      ROS_ERROR("%s call failed", client.getService().c_str());
      return false;
    }

    return false;
  }

private:
  std::string map_folder_ = "";
  std::string map_name_ = "";

  std::unique_ptr<localization::LocalizationNode> node_;
  std::unique_ptr<LocalizationNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  ros::ServiceServer start_localization_srv_;
  ros::ServiceServer stop_localization_srv_;
  ros::ServiceServer reset_localization_srv_;
  ros::ServiceServer set_map_changed_srv_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool is_activating_ = false;
};

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
    << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "localization_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::LocalizationROSNode node;

  ::ros::spin();

  ::ros::shutdown();
}
