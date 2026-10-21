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

#include <cstdint>
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

// 栅格地图面积统计：total 取输出图像外框，effective 取最终保存图的自由像素；仅写入 yaml。
struct GridMapStats {
  double resolution = 0.0;
  uint32_t total_cells = 0;   // 输出图像 width × height（含画布 padding，与 map.png 一致）
  uint32_t free_cells = 0;    // 最终保存图（噪点滤除后）中按 free_thresh 判为自由的像素数
  double TotalAreaM2() const { return total_cells * resolution * resolution; }
  double EffectiveAreaM2() const { return free_cells * resolution * resolution; }
};

// Write the corresponding yaml into 'file_writer'.
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh,
               const GridMapStats* stats = nullptr,
               const std::string& created_at = "");   // 空 = 当前本地时间（新建图）；更新已有地图时传入原值沿用

// 读取已有 map.yaml 的 created_at（文件不存在 / 无键 → 空串）
std::string ReadCreatedAtFromYaml(const std::string& yaml_path);

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
                       const std::string& map_dir,
                       cv::Mat* filtered_out = nullptr);  // 非空时回传滤波后的最终图（与写出的 png 一致）

// Convenience overload that reads the input PNG from disk (legacy).
void WriteNavigationMap(const std::string pgm_file_name,
                       const std::string output_file_name,
                       const std::string& filter_mode,
                       bool save_debug_map,
                       const std::string& map_dir);


}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_ROS_MAP_H
