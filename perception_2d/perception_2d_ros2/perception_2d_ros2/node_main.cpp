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


// declare_parameter(name, default) 把 Lua 默认值注入 ROS2 参数体系；
// yaml 列出了相同 key 时，get_parameter 拿到的是 yaml 值（覆盖 Lua 默认）。
// 顺序与 params_perception_2d.yaml 对齐。
//
// 一个小工具：declare + get 同名参数，避免重复字段拼写
namespace {
template <typename T>
void DeclareAndGet(
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & name,
  T & value)
{
  node->declare_parameter(name, value);
  node->get_parameter(name, value);
}
}  // namespace

void InitParams(const std::shared_ptr<rclcpp::Node>& node, NodeOptions& node_options)
{
  auto & cfg = node_options.perception_options.config;
  auto & sbo = node_options.sensor_bridge_options;

  // === 拓扑 ===
  DeclareAndGet(node, "use_odometry", node_options.use_odometry);
  DeclareAndGet(node, "num_laser_scans", node_options.num_laser_scans);
  DeclareAndGet(node, "num_subdivisions_per_laser_scan", sbo.num_subdivisions_per_laser_scan);
  DeclareAndGet(node, "tracking_frame", sbo.tracking_frame);
  DeclareAndGet(node, "lookup_transform_timeout_sec", node_options.lookup_transform_timeout_sec);

  // === 检测器开关 ===
  DeclareAndGet(node, "predict_rec", cfg.predict_rec);
  DeclareAndGet(node, "reflect_rec", cfg.reflect_rec);
  DeclareAndGet(node, "line_rec",    cfg.line_rec);
  DeclareAndGet(node, "shape_rec",   cfg.shape_rec);
  DeclareAndGet(node, "vmark_rec",   cfg.vmark_rec);
  DeclareAndGet(node, "angle_rec",   cfg.angle_rec);

  // === 数据预处理 ===
  DeclareAndGet(node, "intensities_min",    cfg.intensity_threshold);
  DeclareAndGet(node, "connect_threshold",  cfg.connect_threshold);
  DeclareAndGet(node, "range_max",          cfg.range_max);
  DeclareAndGet(node, "range_min",          cfg.range_min);
  DeclareAndGet(node, "min_cluster_points", cfg.min_cluster_points);

  // === 匹配阈值 ===
  DeclareAndGet(node, "search_distance",   cfg.search_distance);
  DeclareAndGet(node, "search_angle",      cfg.max_ang_error);     // yaml 名 vs config 字段名差异
  DeclareAndGet(node, "max_error",         cfg.max_dis_error);     // 同上
  DeclareAndGet(node, "start_rec_dis",     cfg.start_rec_dis);
  DeclareAndGet(node, "min_select_points", cfg.min_select_points);

  // === 反光板 ===
  DeclareAndGet(node, "reflector_error",  cfg.reflector_error);
  DeclareAndGet(node, "reflector_radius", cfg.reflector_radius);

  // === 直线检测 ===
  DeclareAndGet(node, "min_line_length", cfg.min_line_length);

  // === 形状匹配 ===
  DeclareAndGet(node, "shape_match_score",         cfg.shape_match_score);
  DeclareAndGet(node, "shape_match_close_degree",  cfg.shape_match_close_degree);
  DeclareAndGet(node, "shape_dis_to_robot_center", cfg.shape_dis_to_robot_center);

  // === 模型文件（yaml 设置可覆盖 launch 的 -model_basename gflag） ===
  std::string model_file_override;
  node->declare_parameter("model_file", std::string(""));
  node->get_parameter("model_file", model_file_override);
  if (!model_file_override.empty()) {
    cfg.model_file = model_file_override;
    LOG(INFO) << "[InitParams] model_file overridden by yaml: " << cfg.model_file;
  }
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
