/*
 * Copyright 2017 The Cartographer Authors
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
#include "cartographer/cloud/client/map_builder_stub.h"
#include "cartographer_ros/mapping/node.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "cartographer_ros/options/mapping_node_options.h"

#include "gflags/gflags.h"
#include "glog/logging.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
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
DEFINE_string(server_address, "localhost:50051",
              "gRPC server address to stream the sensor data to.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_map_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file "
              "to load, containing a saved SLAM state. "
              "Unless --upload_load_state_file is set, the filepath refers "
              "to the gRPC server's file system.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(upload_load_state_file, false,
            "Upload the .pbstream file from a local path to the (remote) gRPC "
            "server instead of loading it from the server file system.");
DEFINE_string(client_id, "",
              "Cartographer client ID to use when connecting to the server.");

namespace cartographer_ros {
namespace {

void Run(rclcpp::Node::SharedPtr node) {
  // TF buffer/listener setup in ROS 2:
  // - Buffer can be constructed with a clock interface (node's clock)
  // - Cache time can be set via constructor duration if desired
  constexpr double kTfBufferCacheTimeInSeconds = 10.0;
  auto clock = node->get_clock();

  tf2_ros::Buffer tf_buffer{clock};
  tf_buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tf_listener(tf_buffer);

  MappingNodeOptions node_options;
  TrajectoryOptions trajectory_options;
  std::tie(node_options, trajectory_options) =
      LoadMappingOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder = absl::make_unique<::cartographer::cloud::MapBuilderStub>(
      FLAGS_server_address, FLAGS_client_id);

  if (!FLAGS_load_state_filename.empty() && !FLAGS_upload_load_state_file) {
    map_builder->LoadStateFromFile(FLAGS_load_state_filename,
                                   FLAGS_load_frozen_state);
  }

  Node carto_node(node_options, std::move(map_builder), &tf_buffer,
                  FLAGS_collect_metrics);

  if (!FLAGS_load_state_filename.empty() && FLAGS_upload_load_state_file) {
    carto_node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    carto_node.StartTrajectoryWithDefaultTopics(trajectory_options);
  }

  rclcpp::spin(node);

  carto_node.FinishAllTrajectories();
  carto_node.RunFinalOptimization();

  if (!FLAGS_save_map_filename.empty()) {
    carto_node.SerializeState(FLAGS_save_map_filename,
                              false /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";
  CHECK(!FLAGS_client_id.empty()) << "-client_id is missing.";

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("cartographer_grpc_node");

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  cartographer_ros::Run(node);

  rclcpp::shutdown();
  return 0;
}