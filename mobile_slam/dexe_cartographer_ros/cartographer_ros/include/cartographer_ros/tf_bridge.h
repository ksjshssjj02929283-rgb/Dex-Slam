/**
 * @file tf_bridge.h
 * @author ZheHe
 * @brief 封装cartographer的刚体变换
 * @version 0.1
 * @date 2023-05-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/time_conversion.h"
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

class TfBridge {
 public:
  TfBridge(const std::string& tracking_frame,
           double lookup_transform_timeout_sec, const tf2_ros::Buffer* buffer);
  ~TfBridge() {}

  TfBridge(const TfBridge&) = delete;
  TfBridge& operator=(const TfBridge&) = delete;

  // Returns the transform for 'frame_id' to 'tracking_frame_' if it exists at
  // 'time'.
  std::unique_ptr<::cartographer::transform::Rigid3d> LookupToTracking(
      ::cartographer::common::Time time, const std::string& frame_id) const;

 private:
  const std::string tracking_frame_;
  const double lookup_transform_timeout_sec_;
  const tf2_ros::Buffer* const buffer_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TF_BRIDGE_H
