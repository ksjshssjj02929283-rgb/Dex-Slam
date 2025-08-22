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
#include <std_msgs/Int16.h>
#include "dcm_srvs/SetMapName.h"
#include "dcm_srvs/GetString.h"

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

namespace cartographer_ros {
namespace {

class MappingNode
{
public:
  MappingNode()
  {
    ros::NodeHandle nh_private("~");
    start_mapping_srv_ = nh_private.advertiseService("start_mapping", &MappingNode::StartMappingCallback, this);
    finish_mapping_srv_ = nh_private.advertiseService("finish_mapping", &MappingNode::FinishMappingCallback, this);

    set_map_name_srv_ = nh_private.advertiseService("set_map_name", &MappingNode::SetMapName, this);

    get_mapping_status_srv_ = nh_private.advertiseService("get_mapping_status", &MappingNode::GetMappingStatus, this);

    mapping_status_pub_ = nh_private.advertise<std_msgs::Int16>("mapping_status", 1);
    publish_timer_ = nh_private.createTimer(ros::Duration(0.2), &MappingNode::Publish, this);
  }

  ~MappingNode()
  {
    if (is_mapping_) {
      FinishMapping();
    }
  }

  bool StartShowMap()
  {
    ros::NodeHandle nh;
    ros::ServiceClient client =
      nh.serviceClient<std_srvs::Trigger>("/cartographer_occupancy_grid_node/start_mapping");
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

  bool StopShowMap()
  {
    ros::NodeHandle nh;
    ros::ServiceClient client =
      nh.serviceClient<std_srvs::Trigger>("/cartographer_occupancy_grid_node/stop_mapping");
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

  bool GetMappingStatus(dcm_srvs::GetString::Request &req,
                        dcm_srvs::GetString::Response &res)
  {
    if (is_mapping_) {
      res.data = "mapping";
    } else {
      res.data = "idle";
    }

    res.success = true;
    return true;
  }

  bool StartMappingCallback(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res)
  {
    StartShowMap();

    if (is_mapping_) {
      LOG(ERROR) << "Mapping node is already running.";
      res.success = false;
      return false;
    }
 
    if (FLAGS_map_folder.empty() || map_name_.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      res.success = false;
      return false;
    }

    if (!boost::filesystem::exists(FLAGS_map_folder)) {
      bool create_folder_success = boost::filesystem::create_directory(FLAGS_map_folder);
      if (!create_folder_success) {
        LOG(ERROR) << "Failed to create folder: " << FLAGS_map_folder;
        res.success = false;
        return false;
      }
    }

    std::string this_map_folder = FLAGS_map_folder + "/" + map_name_;
    if (!boost::filesystem::exists(this_map_folder)) {
      bool create_folder_success = boost::filesystem::create_directory(this_map_folder);
      if (!create_folder_success) {
        LOG(ERROR) << "Failed to create folder: " << this_map_folder;
        res.success = false;
        return false;
      }
    }

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    // tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    // tf2_ros::TransformListener tf(tf_buffer);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(::ros::Duration(kTfBufferCacheTimeInSeconds));
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    node_options_ = std::make_unique<MappingNodeOptions>();
    trajectory_options_ = std::make_unique<TrajectoryOptions>();
    // 1. 解析节点和轨迹的配置文件(tie会自动根据参数名，对应node_options和trajectory_options)
    std::tie(*node_options_, *trajectory_options_) =
      LoadMappingOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
    // 2. 创建map_builder
    auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options_->map_builder_options);
    // 3. 创建ros node（节点发布的topic、服务等在其中定义）
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

    res.success = true;
    return res.success;
  }

  bool FinishMappingCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res)
  {
    if (!is_mapping_) {
      LOG(INFO) << "Mapping node is not running.";
      res.success = true;
      return res.success;
    }
    FinishMapping();
    StopShowMap();

    res.success = true;
    return res.success;
  }

  void FinishMapping()
  {
    // 6. 结束优化
    node_->FinishAllTrajectories();
    node_->RunFinalOptimization();
    // 7. 保存轨迹
    if (node_options_->save_tracked_pose) {
      node_->SaveOptimizedPoses(int(0));
    }

    std::string state_filename = FLAGS_map_folder + "/" + map_name_ + "/map.pbstream";
    // 8. 结束的时候是否保存建图信息
    if (!state_filename.empty()) {
      node_->SerializeState(state_filename,
                          true /* include_unfinished_submaps */);
      LOG(INFO) << "State saved to '" << state_filename << "'. \n";
    }
    std::string map_filestream = FLAGS_map_folder + "/" + map_name_ + "/map";
    node_->SaveMap(map_filestream, FLAGS_resolution);

    node_.reset();
    node_options_.reset();
    trajectory_options_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    map_name_ = "";  // 每次建图完，清除地图名字
    is_mapping_ = false;
  }

  bool SetMapName(dcm_srvs::SetMapName::Request &req,
                  dcm_srvs::SetMapName::Response &res)
  {
    map_name_ = req.map_name;

    if (FLAGS_map_folder.empty() || map_name_.empty()) {
      LOG(ERROR) << "Need to specify --map_folder and --map_name.";
      res.success = false;
      return false;
    }

    if (!boost::filesystem::exists(FLAGS_map_folder)) {
      bool create_folder_success = boost::filesystem::create_directory(FLAGS_map_folder);
      if (!create_folder_success) {
        LOG(ERROR) << "Failed to create folder: " << FLAGS_map_folder;
        res.success = false;
        return false;
      }
    }

    std::string this_map_folder = FLAGS_map_folder + "/" + map_name_;
    if (!boost::filesystem::exists(this_map_folder)) {
      bool create_folder_success = boost::filesystem::create_directory(this_map_folder);
      if (!create_folder_success) {
        LOG(ERROR) << "Failed to create folder: " << this_map_folder;
        res.success = false;
        return false;
      }
    }

    ROS_INFO("Set map name to %s", this_map_folder.c_str());

    res.success = true;
    return res.success;
  }

  void Publish(const ros::TimerEvent& event)
  {
    std_msgs::Int16 mapping_status_msg;
    mapping_status_msg.data = 0;
    if (is_mapping_) {
      mapping_status_msg.data = 1;
    } else {
      mapping_status_msg.data = -1;
    }
    mapping_status_pub_.publish(mapping_status_msg);
  }

private:
  std::string map_name_ = "";

  std::unique_ptr<Node> node_;
  std::unique_ptr<MappingNodeOptions> node_options_;
  std::unique_ptr<TrajectoryOptions> trajectory_options_;

  bool is_mapping_ = false;
  ros::Timer publish_timer_;
  ros::Publisher mapping_status_pub_;

  ros::ServiceServer start_mapping_srv_;
  ros::ServiceServer finish_mapping_srv_;
  ros::ServiceServer set_map_name_srv_;
  ros::ServiceServer mapping_status_srv_;
  ros::ServiceServer get_mapping_status_srv_;

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
  CHECK(!FLAGS_configuration_basename.empty())
    << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "mapping_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::MappingNode node;

  ::ros::spin();

  ::ros::shutdown();
}
