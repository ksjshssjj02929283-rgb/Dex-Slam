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
#include <vector>

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

// 从「渲染图」与「导航滤波后的最终图」统计面积，口径与建图侧
// MapBuilderBridge::SaveGridMap 完全一致：
//   total_cells = 渲染图外框 width × height（含画布 padding，与 map.png 一致）
//   free_cells  = 最终保存图中 pixel > 255*(1-free_thresh) 的像素数
// 离线工具（align / replace / merge / reset_origin）重新生成地图后必须重算并写入
// map.yaml —— 面积是 map_manager get_maps 对外暴露的字段，不重算就会退回 0。
// 'filtered_mat' 传空 Mat 时 free_cells = 0（调用方须自行说明口径）。
GridMapStats ComputeGridMapStats(const cv::Mat& rendered_mat,
                                 const cv::Mat& filtered_mat,
                                 double resolution, double free_thresh);

// 从既有 map.yaml 读取 'created_at'（本地时间 ISO 8601，无引号/有引号均可）。
// 语义：**内容采集时刻**，不是文件最后修改时刻 —— 后者由 map.yaml 的 mtime
// 表达（map_manager 的 MapInfo.updated_at 就取 mtime，永远不会因某个写入方
// 忘记更新而失真）。因此这个值在重写地图时要按"内容有没有换"决定去留：
//   - 对齐 / reset_origin / 定位侧保存：内容没变（只是转了角或更新了动态层）→ **保留**
//   - 替换：内容整体换成 src 那次勘测 → **取 src 的值**（取"现在"是错的，那是 updated_at）
//   - 合并：保留 tgt 的（tgt 是坐标系宿主）
// 键不存在时返回空串（传给 WriteYaml 即取当前时间）。
std::string ReadMapCreatedAt(const std::string& yaml_path);

// 比对既有 map.yaml 里的键与本次将要写出的键，列出**会被丢掉**的键。
// 工具是全量重写 map.yaml 的，建图侧将来新增任何键（如 created_at）都会被静默
// 抹掉；这个检查把「下次记得」变成一次机器告警。文件不存在时返回空。
std::vector<std::string> FindDroppedYamlKeys(const std::string& existing_yaml_path,
                                             bool with_area_stats);

// Write the corresponding yaml into 'file_writer'.
// 'created_at' 空 = 当前本地时间（新建图）；更新已有地图时传入原值沿用（见 ReadMapCreatedAt）。
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh,
               const GridMapStats* stats = nullptr,
               const std::string& created_at = "");

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
