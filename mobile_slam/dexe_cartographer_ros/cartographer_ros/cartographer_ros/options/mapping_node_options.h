/*
 * Copyright 2016 The Cartographer Authors
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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_MAPPING_NODE_OPTIONS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_MAPPING_NODE_OPTIONS_H

#include <string>
#include <tuple>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer_ros/options/trajectory_options.h"

namespace cartographer_ros {

// Top-level options of Cartographer's ROS integration.
struct MappingNodeOptions {
  ::cartographer::mapping::proto::MapBuilderOptions map_builder_options;
  std::string map_frame;
  double lookup_transform_timeout_sec;
  double submap_publish_period_sec;
  double pose_publish_period_sec;
  double trajectory_publish_period_sec;
  bool publish_to_tf = true;
  bool publish_tracked_pose = false;
  bool use_pose_extrapolator = true;

  bool save_tracked_pose = false;
  std::string tracked_pose_format = "tum";
  std::string tracked_pose_directory = "";
  std::string tracked_pose_filename = "tracked_pose.txt";

  float mapping_occ_thresh = 0.65;
  float mapping_free_thresh = 0.196;
  
  // Navigation map options
  bool save_origin_map = false;           // save original (pre-filter) map as *_origin.png
  std::string navigation_map_filter_mode = "normal"; // "normal", "high", "super"
  bool save_debug_map = false; // Enable debug visualization output
  bool save_map_pcd = false;              // save map point cloud as *.pcd

  // 保存地图前重置地图原点（整体旋转使栅格对齐结构化环境）。
  // 重置角度由全局线特征在线估计（见 cartographer/mapping/2d/structure_alignment.h）。
  bool reset_map_origin_to_align_structure = false;
  // 对齐角度估计的阈值（填充 StructureAlignmentOptions）。
  // 观测次数 / RANSAC 内点等基础质量剔除已由上游 LineDataCulling 完成，
  // 此处不再重复设门槛：
  double reset_map_origin_min_line_length = 1.5;  // 结构线段最小长度（米）
  int reset_map_origin_max_candidate_lines = 20;  // 按长度取前 N 条候选
  double reset_map_origin_orthogonality_tolerance_deg = 10.0;  // 正交容差（度）
  // 次方向支撑量 / 主方向支撑量 的最小比值（低于则视为无正交结构，不重置）
  double reset_map_origin_min_secondary_support_ratio = 0.3;
};

MappingNodeOptions CreateMappingNodeOptions(
    ::cartographer::common::LuaParameterDictionary* lua_parameter_dictionary);

std::tuple<MappingNodeOptions, TrajectoryOptions> LoadMappingOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_OPTIONS_MAPPING_NODE_OPTIONS_H
