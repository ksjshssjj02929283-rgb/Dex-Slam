//
// Created by xiang on 25-3-24.
//

#ifndef LIGHTNING_ROS_UTILS_H
#define LIGHTNING_ROS_UTILS_H

#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/eigen_types.h"
#include "common/point_def.h"

namespace lightning {

inline double ToSec(const builtin_interfaces::msg::Time &time) { return double(time.sec) + 1e-9 * time.nanosec; }
inline uint64_t ToNanoSec(const builtin_interfaces::msg::Time &time) { return time.sec * 1e9 + time.nanosec; }

/// ROS TransformStamped 转 SE3
inline SE3 TransformToSE3(const geometry_msgs::msg::TransformStamped &tf) {
    Eigen::Quaterniond q(tf.transform.rotation.w, tf.transform.rotation.x, tf.transform.rotation.y,
                         tf.transform.rotation.z);
    Eigen::Vector3d t(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    return SE3(q, t);
}

/// SE3 转 ROS TransformStamped
inline geometry_msgs::msg::TransformStamped SE3ToTransform(double timestamp, const SE3 &pose,
                                                           const std::string &frame_id,
                                                           const std::string &child_frame) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame;

    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    msg.transform.translation.x = t.x();
    msg.transform.translation.y = t.y();
    msg.transform.translation.z = t.z();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    return msg;
}

/// SE3 转 ROS PoseStamped
inline geometry_msgs::msg::PoseStamped SE3ToPoseStamped(double timestamp, const SE3 &pose,
                                                        const std::string &frame_id) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = rclcpp::Time(static_cast<uint64_t>(timestamp * 1e9));
    msg.header.frame_id = frame_id;

    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    msg.pose.position.x = t.x();
    msg.pose.position.y = t.y();
    msg.pose.position.z = t.z();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    return msg;
}

/// geometry_msgs::Pose 转 SE3
inline SE3 PoseToSE3(const geometry_msgs::msg::Pose &pose) {
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
    return SE3(q, t);
}

/// geometry_msgs::PoseStamped 转 SE3
inline SE3 PoseStampedToSE3(const geometry_msgs::msg::PoseStamped &pose_msg) {
    Eigen::Quaterniond q(pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                         pose_msg.pose.orientation.z);
    Eigen::Vector3d t(pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
    return SE3(q, t);
}

}  // namespace lightning

#endif  // LIGHTNING_ROS_UTILS_H