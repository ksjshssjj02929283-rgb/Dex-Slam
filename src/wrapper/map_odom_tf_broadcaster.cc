#include "wrapper/map_odom_tf_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lightning {

MapOdomTFBroadcaster::MapOdomTFBroadcaster() : Node("map_odom_tf_broadcaster") {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/feedback/odom");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/lightning/pose/current");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OdomCallback(msg); });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { PoseCallback(msg); });

    RCLCPP_INFO(get_logger(), "MapOdomTFBroadcaster initialized");
    RCLCPP_INFO(get_logger(), "  odom_topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  pose_topic: %s", pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  odom_frame: %s", odom_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  base_link_frame: %s", base_link_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  map_frame: %s", map_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  TF publish mode: event-driven (triggered by odom msg)");
}

void MapOdomTFBroadcaster::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) { PublishMapOdomTF(msg); }

void MapOdomTFBroadcaster::PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    latest_pose_ = msg;
}

void MapOdomTFBroadcaster::PublishMapOdomTF(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::PoseStamped::SharedPtr pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!latest_pose_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "No pose data received yet, waiting for /lightning/pose/current");
            return;
        }
        pose = latest_pose_;
    }

    geometry_msgs::msg::TransformStamped map_to_base;
    map_to_base.header.stamp = pose->header.stamp;
    map_to_base.header.frame_id = map_frame_;
    map_to_base.child_frame_id = base_link_frame_;
    map_to_base.transform.translation.x = pose->pose.position.x;
    map_to_base.transform.translation.y = pose->pose.position.y;
    map_to_base.transform.translation.z = pose->pose.position.z;
    map_to_base.transform.rotation = pose->pose.orientation;

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header = odom_msg->header;
    odom_to_base.child_frame_id = odom_msg->child_frame_id;
    odom_to_base.transform.translation.x = odom_msg->pose.pose.position.x;
    odom_to_base.transform.translation.y = odom_msg->pose.pose.position.y;
    odom_to_base.transform.translation.z = odom_msg->pose.pose.position.z;
    odom_to_base.transform.rotation = odom_msg->pose.pose.orientation;

    geometry_msgs::msg::TransformStamped map_to_odom = ComputeMapToOdom(map_to_base, odom_to_base);

    tf_broadcaster_->sendTransform(map_to_odom);

    if (!initialized_) {
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "map->odom TF broadcasting started (100Hz, triggered by odom)");
    }
}

geometry_msgs::msg::TransformStamped MapOdomTFBroadcaster::ComputeMapToOdom(
    const geometry_msgs::msg::TransformStamped& map_to_base, const geometry_msgs::msg::TransformStamped& odom_to_base) {
    tf2::Transform map_to_base_tf, odom_to_base_tf, map_to_odom_tf;

    tf2::fromMsg(map_to_base.transform, map_to_base_tf);
    tf2::fromMsg(odom_to_base.transform, odom_to_base_tf);

    tf2::Transform base_to_odom = odom_to_base_tf.inverse();

    map_to_odom_tf = map_to_base_tf * base_to_odom;

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = odom_to_base.header.stamp;
    map_to_odom_msg.header.frame_id = map_frame_;
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom_tf);

    return map_to_odom_msg;
}

}  // namespace lightning

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lightning::MapOdomTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}