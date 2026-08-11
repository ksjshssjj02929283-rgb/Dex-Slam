#pragma once

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <optional>

namespace lightning {

class MapOdomTFBroadcaster : public rclcpp::Node {
   public:
    explicit MapOdomTFBroadcaster();

   private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void PublishMapOdomTF(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    geometry_msgs::msg::TransformStamped ComputeMapToOdom(const geometry_msgs::msg::TransformStamped& map_to_base,
                                                          const geometry_msgs::msg::TransformStamped& odom_to_base);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::mutex pose_mutex_;
    geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_;

    std::string odom_topic_;
    std::string pose_topic_;
    std::string odom_frame_;
    std::string base_link_frame_;
    std::string map_frame_;

    bool initialized_ = false;
};

}  // namespace lightning