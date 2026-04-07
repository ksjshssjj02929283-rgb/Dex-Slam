#!/bin/bash

# Script to convert ROS1 perception_2d_ros1 files to ROS2 perception_2d_ros2
# This script performs basic text replacement from ROS1 API to ROS2 API

SRC_DIR="/home/fengbro/dexforce/0_slam/add_perception_2d/dexe_mobile_slam/perception_2d/perception_2d_ros1/perception_2d_ros1"
DST_DIR="/home/fengbro/dexforce/0_slam/add_perception_2d/dexe_mobile_slam/perception_2d/perception_2d_ros2/perception_2d_ros2"

# List of files to convert (exclude node_main.cpp as it's already manually converted)
FILES=(
    "ros_log_sink.hpp"
    "ros_log_sink.cpp"
    "node_constants.hpp"
    "node_constants.cpp"
    "node.hpp"
    "node.cpp"
    "node_option.hpp"
    "node_option.cpp"
    "time_conversion.hpp"
    "time_conversion.cpp"
    "msg_conversion.hpp"
    "msg_conversion.cpp"
    "tf_bridge.hpp"
    "tf_bridge.cpp"
    "sensor_bridge.hpp"
    "sensor_bridge.cpp"
    "perception_processor_bridge.hpp"
    "perception_processor_bridge.cpp"
    "feature_map_bridge.hpp"
    "feature_map_bridge.cpp"
)

# Copy and convert each file
for file in "${FILES[@]}"; do
    echo "Converting $file..."
    cp "$SRC_DIR/$file" "$DST_DIR/$file"
    
    # Perform sed replacements for ROS1 to ROS2 conversion
    sed -i 's/perception_ros1/perception_ros2/g' "$DST_DIR/$file"
    sed -i 's/PERCEPTION_2D_ROS1/PERCEPTION_2D_ROS2/g' "$DST_DIR/$file"
    sed -i 's/perception_2d_ros1/perception_2d_ros2/g' "$DST_DIR/$file"
    sed -i 's/#include <ros\/ros.h>/#include <rclcpp\/rclcpp.hpp>/g' "$DST_DIR/$file"
    sed -i 's/ros::NodeHandle/rclcpp::Node/g' "$DST_DIR/$file"
    sed -i 's/ros::Publisher/rclcpp::Publisher/g' "$DST_DIR/$file"
    sed -i 's/ros::Subscriber/rclcpp::Subscription/g' "$DST_DIR/$file"
    sed -i 's/ros::Time/rclcpp::Time/g' "$DST_DIR/$file"
    sed -i 's/ros::Duration/rclcpp::Duration/g' "$DST_DIR/$file"
    sed -i 's/ros::WallTimer/rclcpp::TimerBase/g' "$DST_DIR/$file"
    sed -i 's/ros::ServiceServer/rclcpp::Service/g' "$DST_DIR/$file"
    sed -i 's/::ros::/rclcpp::/g' "$DST_DIR/$file"
    sed -i 's/\.ConstPtr/::ConstSharedPtr/g' "$DST_DIR/$file"
    
done

echo "Basic conversion complete. Manual review and fixes required!"
echo "Key areas to review:"
echo "  - Node initialization and lifecycle"
echo "  - Publisher/Subscriber creation syntax"
echo "  - Timer callbacks"
echo "  - Parameter handling"
echo "  - TF2 usage"
echo "  - Service callbacks"
