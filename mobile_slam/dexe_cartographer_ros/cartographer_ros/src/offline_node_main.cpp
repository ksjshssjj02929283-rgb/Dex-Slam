/**
 * @file offline_node_main.cpp
 * @author ZheHe
 * @brief 执行离线建图
 * @version 0.1
 * @date 2023-05-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/offline_node.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  // noted by hz: 传入配置参数，实例化SLAM算法
  const cartographer_ros::MapBuilderFactory map_builder_factory = [](
      const ::cartographer::mapping::proto::MapBuilderOptions&
          map_builder_options) {
    return ::cartographer::mapping::CreateMapBuilder(map_builder_options);
  };
  // noted by hz: 注册离线建图的节点
  rclcpp::Node::SharedPtr cartographer_offline_node =
      rclcpp::Node::make_shared("cartographer_offline_node");
  cartographer_ros::RunOfflineNode(map_builder_factory, cartographer_offline_node);

  rclcpp::shutdown();
}
