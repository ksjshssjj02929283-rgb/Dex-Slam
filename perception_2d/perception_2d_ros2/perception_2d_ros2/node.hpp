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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_interfaces/msg/target_info.hpp>
#include <nav_interfaces/msg/target_map_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_interfaces/msg/line_array.hpp>
#include <nav_interfaces/msg/docking_info.hpp>
#include <set>
#include <memory>
#include <string>
#include <vector>
#include "perception_2d_ros2/node_constants.hpp"
#include "perception_2d_ros2/ros_log_sink.hpp"
#include "perception_2d_ros2/sensor_bridge.hpp"
#include "perception_2d_ros2/perception_processor_bridge.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "perception_2d_ros2/msg_conversion.hpp"


namespace perception_ros2
{
struct PerceptionTopics
{
  std::string laser_scan_topic;
  std::string odometry_topic;
  std::string update_feature_map_topic;
};

static constexpr double color_list[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // r, g , b

class Node
{
public:
  Node(
    std::shared_ptr<rclcpp::Node> node,
    const NodeOptions & node_options,
    std::unique_ptr<perception_2d::PerceptionProcessorInterface> perception_processor,
    tf2_ros::Buffer * const tf_buffer);
  ~Node();

  Node(const Node &) = delete;
  Node & operator=(const Node &) = delete;

  void StartRecoginizeWithDefaultTopics(const NodeOptions & node_options);

  void HandleOdometryMessage(
    const std::string & sensor_id,
    const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  void HandleLaserScanMessage(
    const std::string & sensor_id,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg);

  void HandleFeatureMapInfoMessage(
    const std::string & topic,
    const nav_interfaces::msg::TargetInfo::ConstSharedPtr & msg);
  std::set<perception_2d::LaserPerception2DInterface::SensorId>
  ComputeExpectedSensorIds(
    const NodeOptions & options,
    const PerceptionTopics & topics) const;
  void linesAndTextVisual(
    perception_2d::LinesAndTextInfo lines_text_info,
    std::string ns, rclcpp::Time stamp);

  std::shared_ptr<rclcpp::Node> node_handle();

  void ActivatePerceptionCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  void DeactivatePerceptionCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

private:
  void PublishTargetTF();
  void PublishPrePoint();
  void PublishLineAndRef();
  
  struct Subscriber
  {
    rclcpp::SubscriptionBase::SharedPtr subscriber;
    std::string topic;
  };
  
  std::shared_ptr<rclcpp::Node> node_handle_;
  std::vector<Subscriber> subscribers_;
  std::map<std::string, std::string> topic_frame_map_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> point_cloud_pubs_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> line_pubs_;
  std::map<std::string, rclcpp::PublisherBase::SharedPtr> ref_pubs_;
  rclcpp::Publisher<nav_interfaces::msg::TargetMapInfo>::SharedPtr target_map_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_vis_pub;
  std::vector<rclcpp::TimerBase::SharedPtr> wall_timers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr feature_detected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr reflector_pub_;
  rclcpp::Publisher<nav_interfaces::msg::LineArray>::SharedPtr line_pub_;
  rclcpp::Publisher<nav_interfaces::msg::DockingInfo>::SharedPtr docking_info_pub_;
  PerceptionProcessorBridge perception_processor_bridge_;
  SensorBridge * sensor_bridge_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr active_perception_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr deactive_perception_server_;
  bool is_activate_;

  void SubscribeTopics(const PerceptionTopics & topics);
  void LaunchSubscribers(const PerceptionTopics & topics, const NodeOptions & options);
};
}  // namespace perception_ros2


#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_HPP_
