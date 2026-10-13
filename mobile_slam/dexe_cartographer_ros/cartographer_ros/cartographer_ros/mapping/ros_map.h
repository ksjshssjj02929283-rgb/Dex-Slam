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
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
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
// 语义：**内容采集时刻**，不是最后编辑时刻 —— 后者是 map.yaml 的 last_edited_at 键
// （WriteYaml 新建时与 created_at 相同，WriteMapYamlMerged 每次重写取当前时间；
// map_manager 以 MapInfo.last_edited_at 透传，只反映底图，与资产文件无关）。
// 重写地图时按"内容有没有换"决定 created_at 的去留：
//   - 对齐 / reset_origin / 定位侧保存 / 合并：内容没换 → 由 WriteMapYamlMerged 从底文件原样沿用
//   - 替换：内容整体换成 src 那次勘测 → 调用方读 src 的值，经 MapYamlMergeOptions::created_at 覆盖
// 键不存在时返回空串。
std::string ReadMapCreatedAt(const std::string& yaml_path);

// map.yaml 的一组键值，值是已格式化好的 yaml 文本，按写出顺序排列。
using MapYamlEntries = std::vector<std::pair<std::string, std::string>>;

// 渲染写入方「负责」的键：image / resolution / origin / negate / occupied_thresh / free_thresh，
// 给出 stats 时再加 total_area_m2 / effective_area_m2，给出 data_version 时再加 data_version。
// 不再写 line_map / cloud_map / highlight_cloud_map：点云与线特征都固化在 map.pbstream，
// 需要文件时用 pbstream_export_tool 导出；合并写出时还会删掉旧地图里的这三个键。
// 键名与数值格式只在这里定义一处，WriteYaml 与 WriteMapYamlMerged 共用。
// 不含 created_at —— 它的去留按写入方语义另定（见 MapYamlMergeOptions）。
// 'data_version' 语义同 WriteYaml。
MapYamlEntries MakeMapYamlEntries(double resolution, const Eigen::Vector2d& origin,
                                  const std::string& map_file_name,
                                  double occ_thresh, double free_thresh,
                                  const absl::optional<std::string>& data_version,
                                  const GridMapStats* stats);

struct MapYamlMergeOptions {
  // 以哪份既有 map.yaml 为底。底里**不归本次写入方负责**的键逐行原样保留
  // （created_at、display_yaw 以及以后新增的任何扩展键，写入方都不用再逐个去补）。
  // 空串或文件不存在 = 新建，不继承任何旧键。
  // 底未必是输出文件：定位侧先写 <map>/slam.staging/ 再整体交换，底要取正式目录那份。
  std::string base_yaml_path;
  // created_at 覆盖：nullopt = 沿用底的值（底里没有则取当前时间）；
  // 有值 = 用它，空串表示取当前时间。只有替换要填（取源图的值）。
  absl::optional<std::string> created_at;
  // 本次写入让坐标系绕原点转了多少（弧度，即新原点在旧坐标系中的 yaw）。
  // 底里有 display_yaw 时把它加上并归一化到 (-π, π]，使屏幕上看到的朝向不变；
  // 底里没有就不新增。只有对齐与重设原点非 0。
  double display_yaw_delta = 0.0;
};

// 合并写出 map.yaml，取代「整份重写 + 逐个写入方补救丢键」：
//   1. 读底文件；
//   2. entries 里的键按新值覆盖（原位替换；底里没有的追加在末尾）；
//   3. 其余行逐字节保留（注释、空行、未知键一律不动）；
//   4. 写 <out>.tmp、fsync 后 rename，原子替换。
// 读改写全程持有底文件所在目录（无底时为输出目录）下 .map.yaml.lock 的 flock，
// 与写同一份 map.yaml 的其它进程互斥。
// 依赖型键统一在这里处理：渲染写入方一定重画了底图，entries 没给面积时删掉底里
// 的旧面积而不是留下陈旧值；created_at、display_yaw 按 opts 处理；
// last_edited_at 一律写当前时间（重写即编辑了底图）。
// 返回 false 表示写盘失败，原文件不受影响。
bool WriteMapYamlMerged(const std::string& out_yaml_path, const MapYamlEntries& entries,
                        const MapYamlMergeOptions& opts);

// Write the corresponding yaml into 'file_writer'.
// 'data_version'：与这份 yaml 一起写出的 pbstream 的数据版本
// （SerializationHeader.data_version），写成 yaml 键 data_version，
// map_manager 读它告诉上层这张图的 pbstream 存了哪些数据：
//   - "0.3.0" 等 → 原样写入；
//   - ""（旧版 pbstream）→ 写 "legacy"；
//   - absl::nullopt（这份 yaml 不对应 pbstream，如 assets_writer）→ 不写该键。
// 必须由调用方显式给出：各写入方写出的 pbstream 版本不同（替换取 source、
// 定位回写沿用加载图），漏给就编译失败，而不是悄悄写错。
// 只用于**新建**地图（建图完成、离线转换、assets_writer）；更新已有地图一律走 WriteMapYamlMerged。
// 'created_at' 空 = 当前本地时间；last_edited_at 写成与 created_at 相同。
void WriteYaml(const double resolution, const Eigen::Vector2d& origin,
               const std::string& map_file_name,
               ::cartographer::io::FileWriter* file_writer,
               double occ_thresh, double free_thresh,
               const absl::optional<std::string>& data_version,
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
