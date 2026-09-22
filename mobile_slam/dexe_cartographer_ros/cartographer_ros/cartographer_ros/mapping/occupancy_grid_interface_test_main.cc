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

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "cartographer/common/port.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"

// Keep the production node implementation in this test translation unit so
// that the regression exercises the real submap callback without widening the
// node's public API only for testing.
#define main cartographer_occupancy_grid_node_program_main
#include "cartographer_ros/mapping/occupancy_grid_node_main.cc"
#undef main

namespace {

using namespace std::chrono_literals;

template <typename Predicate>
bool WaitUntil(Predicate predicate, const std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return predicate();
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto occupancy_node =
      std::make_shared<cartographer_ros::Node>(0.05, 0.05);
  auto mapping_node = std::make_shared<rclcpp::Node>("mapping");
  auto observer_node =
      std::make_shared<rclcpp::Node>("occupancy_grid_interface_test");

  std::atomic<int> query_count{0};
  const auto query_service =
      mapping_node->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
          "~/submap_query",
          [&query_count](
              const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr,
              cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr
                  response) {
            ++query_count;
            response->status.code =
                cartographer_ros_msgs::msg::StatusCode::OK;
            response->submap_version = 1;

            std::string compressed_cells;
            cartographer::common::FastGzipString(
                std::string({static_cast<char>(0), static_cast<char>(255)}),
                &compressed_cells);
            cartographer_ros_msgs::msg::SubmapTexture texture;
            texture.cells.assign(compressed_cells.begin(),
                                 compressed_cells.end());
            texture.width = 1;
            texture.height = 1;
            texture.resolution = 0.05;
            texture.slice_pose.orientation.w = 1.;
            response->textures.push_back(std::move(texture));
          });

  const auto submap_list_publisher =
      mapping_node->create_publisher<cartographer_ros_msgs::msg::SubmapList>(
          "~/submap_list", rclcpp::QoS(10));

  std::atomic<int> map_count{0};
  const auto map_subscription =
      observer_node->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map", rclcpp::QoS(10).transient_local(),
          [&map_count](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr&) {
            ++map_count;
          });
  const auto start_client =
      observer_node->create_client<std_srvs::srv::Trigger>(
          "/cartographer_occupancy_grid_node/start_showing_map");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(occupancy_node);
  executor.add_node(mapping_node);
  executor.add_node(observer_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool passed = true;
  if (!start_client->wait_for_service(3s) ||
      !WaitUntil([&submap_list_publisher]() {
        return submap_list_publisher->get_subscription_count() > 0;
      }, 3s)) {
    std::cerr << "FAILED: occupancy grid interfaces were not discovered"
              << std::endl;
    passed = false;
  }

  if (passed) {
    auto start_future = start_client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (start_future.wait_for(3s) != std::future_status::ready ||
        !start_future.get()->success) {
      std::cerr << "FAILED: start_showing_map did not succeed" << std::endl;
      passed = false;
    }
  }

  if (passed) {
    cartographer_ros_msgs::msg::SubmapList message;
    message.header.stamp = mapping_node->now();
    message.header.frame_id = "map";
    cartographer_ros_msgs::msg::SubmapEntry entry;
    entry.trajectory_id = 0;
    entry.submap_index = 0;
    entry.submap_version = 1;
    entry.pose.orientation.w = 1.;
    message.submap.push_back(entry);
    submap_list_publisher->publish(message);

    const bool query_observed =
        WaitUntil([&query_count]() { return query_count.load() > 0; }, 3s);
    const bool map_observed =
        WaitUntil([&map_count]() { return map_count.load() > 0; }, 3s);
    if (!query_observed || !map_observed) {
      std::cerr << "FAILED: /mapping/submap_list did not produce /map; "
                << "query_count=" << query_count.load()
                << " map_count=" << map_count.load() << std::endl;
      passed = false;
    }
  }

  executor.cancel();
  spin_thread.join();
  executor.remove_node(observer_node);
  executor.remove_node(mapping_node);
  executor.remove_node(occupancy_node);
  rclcpp::shutdown();

  if (!passed) {
    return 1;
  }
  std::cout << "Occupancy grid interface regression test passed."
            << std::endl;
  return 0;
}
