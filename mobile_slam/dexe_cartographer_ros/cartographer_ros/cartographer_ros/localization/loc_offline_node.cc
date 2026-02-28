/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer_ros/localization/loc_offline_node.h"
#include "cartographer_ros/options/localization_node_options.h"

#include <errno.h>
#include <string>
#ifndef WIN32
#include <sys/resource.h>
#endif
#include <time.h>

#include <chrono>
#include <regex>

#include "absl/strings/str_split.h"
#include "cartographer_ros/localization/localization_node.h"
#include "cartographer_ros/mapping/playable_bag.h"
#include "cartographer_ros/mapping/urdf_reader.h"
#include "gflags/gflags.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "urdf/model.h"
#include "rclcpp/exceptions.hpp"

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
DEFINE_string(bag_filenames, "",
              "Comma-separated list of bags to process for localization testing.");
DEFINE_string(urdf_filenames, "",
              "Comma-separated list of one or more URDF files that contain "
              "static links for the sensor configuration(s).");
DEFINE_bool(use_bag_transforms, true,
            "Whether to read, use and republish transforms from bags.");
DEFINE_string(map_filestream, "",
              "Path to the pbstream file to load for localization.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(keep_running, false,
            "Keep running the offline node after all messages from the bag "
            "have been processed.");
DEFINE_double(skip_seconds, 0,
              "Optional amount of seconds to skip from the beginning "
              "(i.e. when the earliest bag starts.).");
DEFINE_string(pose_output_file, "",
              "If specified, save the localization poses to this file.");

namespace cartographer_ros {
namespace localization {

constexpr char kClockTopic[] = "clock";
constexpr char kTfStaticTopic[] = "/tf_static";
constexpr char kTfTopic[] = "/tf";
constexpr double kClockPublishFrequencySec = 1. / 30.;
const rclcpp::Duration kDelay(1.0, 0);

void RunLocOfflineNode(const LocalizationUpdaterFactory& localization_updater_factory,
                       rclcpp::Node::SharedPtr loc_offline_node) {
  // 1. 加载配置参数和bag包
  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  LOG(INFO) << "Configuration directory: " << FLAGS_configuration_directory;
  
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  LOG(INFO) << "Configuration basename: " << FLAGS_configuration_basename;
  
  CHECK(!FLAGS_bag_filenames.empty())
      << "-bag_filenames is missing.";
  
  CHECK(!FLAGS_map_filestream.empty())
      << "-map_filestream is missing. Localization requires a map.";
  LOG(INFO) << "Map file: " << FLAGS_map_filestream;
  
  // 解析bag文件名
  std::regex regex(",");
  std::vector<std::string> bag_filenames(
      std::sregex_token_iterator(
          FLAGS_bag_filenames.begin(), FLAGS_bag_filenames.end(), regex, -1),
      std::sregex_token_iterator());
  
  // 加载定位配置
  cartographer_ros::LocalizationNodeOptions node_options;
  cartographer_ros::TrajectoryOptions trajectory_options;
  
  std::tie(node_options, trajectory_options) =
      LoadLocalizationNodeOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  
  // Since we preload the transform buffer, we should never have to wait for a transform
  node_options.lookup_transform_timeout_sec = 0.;
  
  auto localization_updater = localization_updater_factory(
      node_options.localization_updater_options);
  
  const std::chrono::time_point<std::chrono::steady_clock> start_time =
      std::chrono::steady_clock::now();
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
          loc_offline_node->get_clock(),
          tf2::durationFromSec(10),
          loc_offline_node);
  
  // 从urdf文件中读取静态变换
  std::vector<geometry_msgs::msg::TransformStamped> urdf_transforms;
  if (!FLAGS_urdf_filenames.empty()) {
    std::vector<std::string> urdf_filenames(
        std::sregex_token_iterator(
            FLAGS_urdf_filenames.begin(), FLAGS_urdf_filenames.end(), regex, -1),
        std::sregex_token_iterator());
    for (const auto& urdf_filename : urdf_filenames) {
      const auto current_urdf_transforms =
          ReadStaticTransformsFromUrdf(urdf_filename, tf_buffer);
      urdf_transforms.insert(urdf_transforms.end(),
                            current_urdf_transforms.begin(),
                            current_urdf_transforms.end());
    }
  }
  tf_buffer->setUsingDedicatedThread(true);
  
  // 创建LocalizationNode
  LocalizationNode node(node_options, std::move(localization_updater), 
                       tf_buffer, loc_offline_node);
  
  // 加载地图 - 两步加载：先LoadState，再LoadMapData
  LOG(INFO) << "Loading map from: " << FLAGS_map_filestream;
  node.LoadState(FLAGS_map_filestream, FLAGS_load_frozen_state);
  
  // 构建map.yaml路径并加载地图数据（关键步骤！）
  std::string map_yaml_path = FLAGS_map_filestream;
  size_t pos = map_yaml_path.rfind("/map.pbstream");
  if (pos != std::string::npos) {
    map_yaml_path.replace(pos, std::string("/map.pbstream").length(), "/map.yaml");
  }
  // LOG(INFO) << "Loading map data from: " << map_yaml_path;
  // node.LoadMapData(map_yaml_path);
  
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher =
      loc_offline_node->create_publisher<tf2_msgs::msg::TFMessage>(
          kTfTopic, kLatestOnlyPublisherQueueSize);
  
  ::tf2_ros::StaticTransformBroadcaster static_tf_broadcaster(loc_offline_node);
  
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher =
      loc_offline_node->create_publisher<rosgraph_msgs::msg::Clock>(
          kClockTopic, kLatestOnlyPublisherQueueSize);
  
  if (urdf_transforms.size() > 0) {
    static_tf_broadcaster.sendTransform(urdf_transforms);
  }
  
  rosgraph_msgs::msg::Clock clock;
  
  // 计算期望的传感器ID
  const auto expected_sensor_ids = node.ComputeExpectedSensorIds(trajectory_options);
  
  std::map<std::pair<int /* bag_index */, std::string>,
           cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      bag_topic_to_sensor_id;
  
  PlayableBagMultiplexer playable_bag_multiplexer(loc_offline_node);
  
  for (size_t current_bag_index = 0; current_bag_index < bag_filenames.size();
       ++current_bag_index) {
    const std::string& bag_filename = bag_filenames.at(current_bag_index);
    if (!rclcpp::ok()) {
      return;
    }
    
    for (const auto& expected_sensor_id : expected_sensor_ids) {
      LOG(INFO) << "Expected sensor ID: " << expected_sensor_id.id;
      // 应用ROS2话题重映射
      std::string topic_name = expected_sensor_id.id;
      std::string resolved_topic = loc_offline_node->get_node_base_interface()->
          resolve_topic_or_service_name(topic_name, false);
      const auto bag_resolved_topic = std::make_pair(
          static_cast<int>(current_bag_index),
          resolved_topic);
      LOG(INFO) << "Resolved topic: " << topic_name << " -> " << resolved_topic;
      
      if (bag_topic_to_sensor_id.count(bag_resolved_topic) != 0) {
        LOG(ERROR) << "Sensor " << expected_sensor_id.id << " of bag "
                   << current_bag_index << " resolves to topic "
                   << bag_resolved_topic.second << " which is already used by "
                   << " sensor "
                   << bag_topic_to_sensor_id.at(bag_resolved_topic).id;
      }
      bag_topic_to_sensor_id[bag_resolved_topic] = expected_sensor_id;
    }
    
    auto serializer = rclcpp::Serialization<tf2_msgs::msg::TFMessage>();
    playable_bag_multiplexer.AddPlayableBag(PlayableBag(
        bag_filename, current_bag_index, kDelay,
        [&tf_publisher, tf_buffer, loc_offline_node, serializer]
        (std::shared_ptr<rosbag2_storage::SerializedBagMessage> msg) {
          if (msg->topic_name == kTfTopic || msg->topic_name == kTfStaticTopic) {
            if (FLAGS_use_bag_transforms) {
              tf2_msgs::msg::TFMessage tf_message;
              rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
              try {
                serializer.deserialize_message(&serialized_msg, &tf_message);
                for (auto& transform : tf_message.transforms) {
                  try {
                    tf_buffer->setTransform(transform, "unused_authority",
                                          msg->topic_name == kTfStaticTopic);
                  } catch (const tf2::TransformException& ex) {
                    LOG(WARNING) << ex.what();
                  }
                }
                tf_publisher->publish(tf_message);
              } catch (const rclcpp::exceptions::RCLError& rcl_error) {
                return true;
              }
            }
            return false;
          } else {
            return true;
          }
        }));
  }
  
  std::set<std::string> bag_topics;
  std::stringstream bag_topics_string;
  for (const auto& topic : playable_bag_multiplexer.topics()) {
    std::string resolved_topic = loc_offline_node->get_node_base_interface()->
        resolve_topic_or_service_name(topic, false);
    bag_topics.insert(resolved_topic);
    bag_topics_string << resolved_topic << ",";
  }
  
  bool print_topics = false;
  for (const auto& entry : bag_topic_to_sensor_id) {
    const std::string& resolved_topic = entry.first.second;
    if (bag_topics.count(resolved_topic) == 0) {
      LOG(WARNING) << "Expected resolved topic \"" << resolved_topic
                   << "\" not found in bag file(s).";
      print_topics = true;
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Available topics in bag file(s) are "
                 << bag_topics_string.str();
  }
  
  // 启动定位轨迹  
  const rclcpp::Time begin_time =
      playable_bag_multiplexer.IsMessageAvailable()
          ? playable_bag_multiplexer.PeekMessageTime()
          : rclcpp::Time();
  
  int trajectory_id = -1;  // Will be initialized when first message arrives
  
  auto laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::LaserScan>();
  auto multi_echo_laser_scan_serializer = rclcpp::Serialization<sensor_msgs::msg::MultiEchoLaserScan>();
  auto pcl2_serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
  auto imu_serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
  auto odom_serializer = rclcpp::Serialization<nav_msgs::msg::Odometry>();
  auto nav_sat_fix_serializer = rclcpp::Serialization<sensor_msgs::msg::NavSatFix>();
  auto landmark_list_serializer = rclcpp::Serialization<cartographer_ros_msgs::msg::LandmarkList>();
  
  // 处理bag消息
  while (playable_bag_multiplexer.IsMessageAvailable()) {
    if (!::rclcpp::ok()) {
      return;
    }
    
    const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
    const rosbag2_storage::SerializedBagMessage& msg = std::get<0>(next_msg_tuple);
    const int bag_index = std::get<1>(next_msg_tuple);
    const std::string topic_type = std::get<2>(next_msg_tuple);
    
    if (msg.time_stamp <
        (begin_time.nanoseconds() + rclcpp::Duration(FLAGS_skip_seconds, 0).nanoseconds())) {
      continue;
    }
    
    // Lazily add trajectory only when the first message arrives
    // This ensures the map is fully loaded before trajectory starts
    if (trajectory_id < 0) {
      LOG(INFO) << "Starting localization trajectory...";
      trajectory_id = node.AddTrajectory(trajectory_options);
      LOG(INFO) << "Started trajectory with ID: " << trajectory_id;
    }
    
    const auto bag_topic = std::make_pair(
        bag_index,
        loc_offline_node->get_node_base_interface()->
            resolve_topic_or_service_name(msg.topic_name, false));
    auto it = bag_topic_to_sensor_id.find(bag_topic);
    
    if (it != bag_topic_to_sensor_id.end()) {
      const std::string& sensor_id = it->second.id;
      
      if (topic_type == "sensor_msgs/msg/LaserScan") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg =
            std::make_shared<sensor_msgs::msg::LaserScan>();
        laser_scan_serializer.deserialize_message(&serialized_msg, laser_scan_msg.get());
        node.HandleLaserScanMessage(trajectory_id, sensor_id, laser_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/MultiEchoLaserScan") {
        // LocalizationNode doesn't have HandleMultiEchoLaserScanMessage
        // Skip multi-echo laser scans for now
        LOG(WARNING) << "MultiEchoLaserScan not supported in localization, skipping...";
      } else if (topic_type == "sensor_msgs/msg/PointCloud2") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::PointCloud2::SharedPtr pcl2_scan_msg =
            std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl2_serializer.deserialize_message(&serialized_msg, pcl2_scan_msg.get());
        node.HandlePointCloud2Message(trajectory_id, sensor_id, pcl2_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/Imu") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::Imu::SharedPtr imu_scan_msg =
            std::make_shared<sensor_msgs::msg::Imu>();
        imu_serializer.deserialize_message(&serialized_msg, imu_scan_msg.get());
        node.HandleImuMessage(trajectory_id, sensor_id, imu_scan_msg);
      } else if (topic_type == "nav_msgs/msg/Odometry") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        nav_msgs::msg::Odometry::SharedPtr odom_scan_msg =
            std::make_shared<nav_msgs::msg::Odometry>();
        odom_serializer.deserialize_message(&serialized_msg, odom_scan_msg.get());
        node.HandleOdometryMessage(trajectory_id, sensor_id, odom_scan_msg);
      } else if (topic_type == "sensor_msgs/msg/NavSatFix") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        sensor_msgs::msg::NavSatFix::SharedPtr nav_sat_fix_msg =
            std::make_shared<sensor_msgs::msg::NavSatFix>();
        nav_sat_fix_serializer.deserialize_message(&serialized_msg, nav_sat_fix_msg.get());
        node.HandleNavSatFixMessage(trajectory_id, sensor_id, nav_sat_fix_msg);
      } else if (topic_type == "cartographer_ros_msgs/msg/LandmarkList") {
        rclcpp::SerializedMessage serialized_msg(*msg.serialized_data);
        cartographer_ros_msgs::msg::LandmarkList::SharedPtr landmark_list_msg =
            std::make_shared<cartographer_ros_msgs::msg::LandmarkList>();
        landmark_list_serializer.deserialize_message(&serialized_msg, landmark_list_msg.get());
        node.HandleLandmarkMessage(trajectory_id, sensor_id, landmark_list_msg);
      }
    }
    
    clock.clock = rclcpp::Time(msg.time_stamp);
    clock_publisher->publish(clock);
    rclcpp::spin_some(loc_offline_node);
  }
  
  // 确保时钟在bag处理完成后继续发布
  auto clock_republish_timer = loc_offline_node->create_wall_timer(
      std::chrono::milliseconds(int(kClockPublishFrequencySec * 1000)),
      [&clock_publisher, &clock]() {
        clock_publisher->publish(clock);
      });
  
  node.FinishAllTrajectories();
  
  const std::chrono::time_point<std::chrono::steady_clock> end_time =
      std::chrono::steady_clock::now();
  const double wall_clock_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time)
          .count();
  
  LOG(INFO) << "Elapsed wall clock time: " << wall_clock_seconds << " s";
#ifdef __linux__
  timespec cpu_timespec = {};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu_timespec);
  LOG(INFO) << "Elapsed CPU time: "
            << (cpu_timespec.tv_sec + 1e-9 * cpu_timespec.tv_nsec) << " s";
  rusage usage;
  CHECK_EQ(getrusage(RUSAGE_SELF, &usage), 0) << strerror(errno);
  LOG(INFO) << "Peak memory usage: " << usage.ru_maxrss << " KiB";
#endif
  
  // 保存定位轨迹位姿
  if (node_options.save_tracked_pose) {
    LOG(INFO) << "Saving localization tracked poses...";
    // 使用配置文件中的目录和文件名
    std::string pose_dir = node_options.tracked_pose_directory;
    std::string pose_filename = node_options.tracked_pose_filename;
    
    // 如果指定了pose_output_file参数，则使用参数值
    if (!FLAGS_pose_output_file.empty()) {
      // 解析出目录和文件名
      size_t last_slash = FLAGS_pose_output_file.find_last_of("/");
      if (last_slash != std::string::npos) {
        pose_dir = FLAGS_pose_output_file.substr(0, last_slash + 1);
        pose_filename = FLAGS_pose_output_file.substr(last_slash + 1);
      } else {
        pose_filename = FLAGS_pose_output_file;
      }
    }
    
    LOG(INFO) << "Pose output directory: " << pose_dir;
    LOG(INFO) << "Pose output filename: " << pose_filename;
    
    // 创建输出目录（如果不存在）
    std::string mkdir_cmd = "mkdir -p " + pose_dir;
    int ret = system(mkdir_cmd.c_str());
    if (ret != 0) {
      LOG(ERROR) << "Failed to create pose directory: " << pose_dir;
    } else {
      LOG(INFO) << "Pose directory ready: " << pose_dir;
    }
    
    // 注意：localization的trajectory_id是在运行时动态分配的
    // 我们需要保存所有轨迹的位姿，但通常只有一个trajectory_id
    if (trajectory_id >= 0) {
      // TODO: 需要在LocalizationNode中添加SaveOptimizedPoses方法
      // 或者实现一个独立的pose保存机制
      LOG(WARNING) << "Pose saving for localization is not yet fully implemented.";
      LOG(WARNING) << "Trajectory ID: " << trajectory_id;
      LOG(WARNING) << "Please use publish_tracked_pose option for runtime pose tracking.";
    } else {
      LOG(WARNING) << "No valid trajectory ID found, skipping pose save.";
    }
  }
  
  if (FLAGS_keep_running) {
    LOG(INFO) << "Finished processing and waiting for shutdown.";
    rclcpp::spin(loc_offline_node);
  }
}

}  // namespace localization
}  // namespace cartographer_ros
