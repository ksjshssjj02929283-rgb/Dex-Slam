/**
 * @file ros_map.h
 * @author ZheHe
 * @brief 
 * @version 0.1
 * @date 2023-05-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H

#include <string>

#include "Eigen/Core"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/2d/map_limits.h"

namespace cartographer_ros {

// Write 'image' as a pgm into 'file_writer'. The resolution is used in the
// comment only'
void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer);

// Write the corresponding yaml into 'file_writer'.
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& pgm_filename,
               ::cartographer::io::FileWriter* file_writer);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
