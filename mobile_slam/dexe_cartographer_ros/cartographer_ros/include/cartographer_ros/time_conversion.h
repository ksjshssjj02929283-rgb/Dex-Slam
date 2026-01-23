/**
 * @file time_conversion.h
 * @author ZheHe
 * @brief ros和cartographer之间 时间戳 格式的转换
 * @version 0.1
 * @date 2023-05-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cartographer/common/time.h"

namespace cartographer_ros {

rclcpp::Time ToRos(::cartographer::common::Time time);

::cartographer::common::Time FromRos(const rclcpp::Time& time);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
