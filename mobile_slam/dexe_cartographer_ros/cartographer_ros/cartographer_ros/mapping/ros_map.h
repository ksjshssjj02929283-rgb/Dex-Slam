/*
 * Copyright 2017 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H

#include <string>

#include "Eigen/Core"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/image.h"
#include "cartographer/mapping/2d/map_limits.h"

namespace cartographer_ros {

// Write 'image' as a pgm into 'file_writer'. The resolution is used in the
// comment only'
void WritePgm(const ::cartographer::io::Image& image, const double resolution,
              ::cartographer::io::FileWriter* file_writer,
              double occ_thresh, double free_thresh);

// Write the corresponding yaml into 'file_writer'.
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh);

void WritePng(const ::cartographer::io::Image& image,
              const std::string file_name,
              double occ_thresh, double free_thresh);

// Generate a thresholded grayscale cv::Mat from an Image without writing to disk.
cv::Mat GeneratePngMat(const ::cartographer::io::Image& image,
                       double occ_thresh, double free_thresh);

// Run navigation-map noise removal on an in-memory mat and write the result.
void WriteNavigationMap(const cv::Mat& input_mat,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir);

// Convenience overload that reads the input PNG from disk (legacy).
void WriteNavigationMap(const std::string pgm_file_name,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir);


}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
