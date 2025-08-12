/**
 * @file urdf_reader.h
 * @author ZheHe
 * @brief 从urdf读取静态tf
 * @version 0.1
 * @date 2023-05-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H

#include <vector>

#include "cartographer/common/port.h"
#include "tf2_ros/buffer.h"

namespace cartographer_ros {

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, std::shared_ptr<tf2_ros::Buffer> tf_buffer);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_URDF_READER_H
