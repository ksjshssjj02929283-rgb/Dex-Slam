// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <utility>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "perception_2d_ros2/ros_log_sink.hpp"
#include "perception_2d_ros2/node.hpp"
#include "perception_2d/common/time.hpp"
#include "perception_2d/laser_perception_2d/perception_processor.hpp"
#include "tf2_ros/transform_listener.h"

DEFINE_string(
  configuration_directory, "",
  "First directory in which configuration files are searched, "
  "second is always the Cartographer installation to allow "
  "including files from there.");
DEFINE_string(
  configuration_basename, "",
  "Basename, i.e. not containing any directory prefix, of the "
  "configuration file.");
DEFINE_string(
  model_directory, "",
  "First directory in which configuration files are searched, "
  "second is always the Cartographer installation to allow "
  "including files from there.");
DEFINE_string(
  model_basename, "",
  "Basename, i.e. not containing any directory prefix, of the "
  "configuration file.");

namespace perception_ros2
{


void InitParams(const std::shared_ptr<rclcpp::Node>& node, NodeOptions& node_options)
{
  node->declare_parameter("use_odometry", node_options.use_odometry);
  node->get_parameter("use_odometry", node_options.use_odometry);
  
  node->declare_parameter("num_laser_scans", node_options.num_laser_scans);
  node->get_parameter("num_laser_scans", node_options.num_laser_scans);
  
  node->declare_parameter("lookup_transform_timeout_sec", node_options.lookup_transform_timeout_sec);
  node->get_parameter("lookup_transform_timeout_sec", node_options.lookup_transform_timeout_sec);
  
  node->declare_parameter("num_subdivisions_per_laser_scan", 
    node_options.sensor_bridge_options.num_subdivisions_per_laser_scan);
  node->get_parameter("num_subdivisions_per_laser_scan", 
    node_options.sensor_bridge_options.num_subdivisions_per_laser_scan);
  
  node->declare_parameter("tracking_frame", node_options.sensor_bridge_options.tracking_frame);
  node->get_parameter("tracking_frame", node_options.sensor_bridge_options.tracking_frame);
  
  node->declare_parameter("predict_rec", node_options.perception_options.config.predict_rec);
  node->get_parameter("predict_rec", node_options.perception_options.config.predict_rec);
  
  node->declare_parameter("line_rec", node_options.perception_options.config.line_rec);
  node->get_parameter("line_rec", node_options.perception_options.config.line_rec);
  
  node->declare_parameter("shape_rec", node_options.perception_options.config.shape_rec);
  node->get_parameter("shape_rec", node_options.perception_options.config.shape_rec);
  
  node->declare_parameter("vmark_rec", node_options.perception_options.config.vmark_rec);
  node->get_parameter("vmark_rec", node_options.perception_options.config.vmark_rec);
  
  node->declare_parameter("intensities_min", node_options.perception_options.config.intensity_threshold);
  node->get_parameter("intensities_min", node_options.perception_options.config.intensity_threshold);
  
  node->declare_parameter("range_max", node_options.perception_options.config.range_max);
  node->get_parameter("range_max", node_options.perception_options.config.range_max);
  
  node->declare_parameter("range_min", node_options.perception_options.config.range_min);
  node->get_parameter("range_min", node_options.perception_options.config.range_min);
  
  node->declare_parameter("search_distance", node_options.perception_options.config.search_distance);
  node->get_parameter("search_distance", node_options.perception_options.config.search_distance);
  
  node->declare_parameter("search_angle", node_options.perception_options.config.max_ang_error);
  node->get_parameter("search_angle", node_options.perception_options.config.max_ang_error);
  
  node->declare_parameter("max_error", node_options.perception_options.config.max_dis_error);
  node->get_parameter("max_error", node_options.perception_options.config.max_dis_error);
  
  node->declare_parameter("min_select_points", node_options.perception_options.config.min_select_points);
  node->get_parameter("min_select_points", node_options.perception_options.config.min_select_points);
  
  node->declare_parameter("reflector_error", node_options.perception_options.config.reflector_error);
  node->get_parameter("reflector_error", node_options.perception_options.config.reflector_error);
  
  node->declare_parameter("min_cluster_points", node_options.perception_options.config.min_cluster_points);
  node->get_parameter("min_cluster_points", node_options.perception_options.config.min_cluster_points);
  
  node->declare_parameter("reflector_radius", node_options.perception_options.config.reflector_radius);
  node->get_parameter("reflector_radius", node_options.perception_options.config.reflector_radius);
  
  node->declare_parameter("start_rec_dis", node_options.perception_options.config.start_rec_dis);
  node->get_parameter("start_rec_dis", node_options.perception_options.config.start_rec_dis);
  
  node->declare_parameter("min_line_length", node_options.perception_options.config.min_line_length);
  node->get_parameter("min_line_length", node_options.perception_options.config.min_line_length);
}

void Run()
{
  CHECK(!FLAGS_configuration_directory.empty()) <<
    "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty()) <<
    "-configuration_basename is missing.";
  CHECK(!FLAGS_model_directory.empty()) <<
    "-model_directory is missing.";
  CHECK(!FLAGS_model_basename.empty()) <<
    "-model_basename is missing.";

  auto ros_node = std::make_shared<rclcpp::Node>("perception_2d_node");
  
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock());
  tf_buffer->setUsingDedicatedThread(true);
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  
  NodeOptions node_options = LoadOptions(
    FLAGS_configuration_directory, FLAGS_configuration_basename,
    FLAGS_model_directory, FLAGS_model_basename);
  
  InitParams(ros_node, node_options);
  
  perception_2d::PerceptionOptions perception_options = node_options.perception_options;
  auto perception_processor = std::make_unique<perception_2d::PerceptionProcessor>(
    perception_options);
  
  auto node = std::make_shared<Node>(
    ros_node, node_options, std::move(perception_processor), tf_buffer.get());
  
  node->StartRecoginizeWithDefaultTopics(node_options);

  rclcpp::spin(ros_node);
}

}  // namespace perception_ros2

int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  rclcpp::init(argc, argv);

  perception_ros2::ScopedRosLogSink ros_log_sink;
  perception_ros2::Run();
  
  rclcpp::shutdown();
  return 0;
}
