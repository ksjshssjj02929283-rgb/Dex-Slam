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

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
// noted by hz: 通过 gflags 或 glog
// 从命令行获取参数，这些都可以在launch文件中设置参数
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
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
// added by hz
DEFINE_string(initial_poses_filename, "",
              "If non-empty, filename of a .yaml file to load, containing "
              "predefined initial pose.");
DEFINE_string(apriltag_poses_filename, "",
              "If non-empty, filename of a .yaml file to load, containing "
              "predefined apriltag poses.");
DEFINE_string(location_name, "", "Set the location of the robot.");
// end

namespace cartographer_ros {
namespace {

void Run() {
  // added by hz
  bool flag_localization = false;
  // end
  rclcpp::Node::SharedPtr cartographer_node =
      rclcpp::Node::make_shared("cartographer_node");

  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  // noted by hz: 存放tf的buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer =
      std::make_shared<tf2_ros::Buffer>(
          cartographer_node->get_clock(),
          tf2::durationFromSec(kTfBufferCacheTimeInSeconds), cartographer_node);
  // noted by hz: 监听tf
  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  // noted by hz: 通过 gflags 或 glog
  // 从命令行获取参数，这些都可以在launch文件中设置参数 比如：./your_program
  // --configuration_directory=/path/to/configs
  NodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  // noted by hz: 实例化SLAM算法，初始化地图构建器，位姿图 优化问题
  // 是否合并多激光雷达数据
  auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  // noted by hz: 注册SLAM算法节点，并传入参数
  auto node = std::make_shared<cartographer_ros::Node>(
      node_options, std::move(map_builder), tf_buffer, cartographer_node,
      FLAGS_collect_metrics);
  // noted by hz: 如果pbstream不为空，载入pbstream文件(纯定位模式会用到)
  // 加载pbstream文件的函数调用逻辑
  // node_main.cpp ->
  // node.cpp ->
  // map_builder_bridge.cpp ->
  // map_builder_interface.h(在这里LoadState是纯虚函数)
  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    // added by hz
    flag_localization = true;
    // end
  }
  // noted by hz: 使用默认topic开始轨迹
  // 建图逻辑在这里实现？？？
  // node_main ->
  //
  // node / void StartTrajectoryWithDefaultTopics(const TrajectoryOptions&
  // options) ->
  //
  // node / int AddTrajectory(const TrajectoryOptions& options) ->
  //
  // map_builder_bridge / int AddTrajectory(const
  // std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&expected_sensor_ids,const
  // TrajectoryOptions& trajectory_options) ->
  //
  // map_builder_interface / int AddTrajectoryBuilder(const std::set<SensorId>
  // &expected_sensor_ids,const proto::TrajectoryBuilderOptions
  // &trajectory_options,LocalSlamResultCallback local_slam_result_callback)
  // override ->
  //    这里map_builder_interface只是个接口，实际实现是在map_builder

  std::cout << "FLAGS_initial_poses_filename: " << FLAGS_initial_poses_filename
            << std::endl;
  std::cout << "FLAGS_apriltag_poses_filename: "
            << FLAGS_apriltag_poses_filename << std::endl;
  std::cout << "FLAGS_location_name: " << FLAGS_location_name << "\n"
            << std::endl;
  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(
        trajectory_options, flag_localization, FLAGS_initial_poses_filename,
        FLAGS_apriltag_poses_filename, FLAGS_location_name);
  }

  rclcpp::spin(cartographer_node);
  // noted by hz: 结束所有活动状态的轨迹
  node->FinishAllTrajectories();
  // noted by hz: 所有轨迹结束时，再执行一次优化
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                         true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::GLOG_ERROR);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}
