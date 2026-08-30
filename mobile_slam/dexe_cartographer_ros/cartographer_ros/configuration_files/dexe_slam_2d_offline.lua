-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  publish_to_tf = true,
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,
  use_odometry = true,
  ignore_out_of_order = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-2,
  trajectory_publish_period_sec = 0.5,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  -- [地图保存参数]
  mapping_occ_thresh = 0.6,
  mapping_free_thresh = 0.2,
  -- [导航地图配置]
  save_origin_map = false,  -- 是否保存原始地图（噪声剔除前）
  navigation_map_filter_mode = "normal",  -- 滤波强度: "normal", "high", "super"
  save_debug_map = false,  -- 是否保存调试图片（mask、连通域等中间结果）
  save_map_pcd = false,  -- 是否保存地图点云（.pcd）
  -- [地图起点重置]
  reset_map_origin_to_align_structure = true,  -- 保存地图前是否重置地图起点（旋转栅格对齐结构化环境；重置角度由全局线特征在线估计）
  -- 对齐角度估计阈值（仅在上面开关打开时生效）：
  reset_map_origin_min_line_length = 0.3,        -- 结构线段最小长度（米），短于此不参与估角（观测次数/内点的基础剔除已由 LineDataCulling 完成）
  reset_map_origin_max_candidate_lines = 20,     -- 通过门槛的线段中按长度取前 N 条候选
  reset_map_origin_orthogonality_tolerance_deg = 10.0,  -- 正交容差（度）
  reset_map_origin_min_secondary_support_ratio = 0.1,   -- 次方向支撑量/主方向支撑量 最小比值（低于则视为无正交结构，不重置）

  publish_tracked_pose = true,
  save_tracked_pose = true,
  tracked_pose_format = "tum",
  tracked_pose_directory = os.getenv("HOME") .. "/.dexe/data/",
  tracked_pose_filename = "carto_poses.txt",
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.data_preprocessor.min_range = 0.2
TRAJECTORY_BUILDER_2D.data_preprocessor.max_range = 15.
TRAJECTORY_BUILDER_2D.data_preprocessor.missing_data_ray_length = 10.
TRAJECTORY_BUILDER_2D.data_preprocessor.use_deskew = true
TRAJECTORY_BUILDER_2D.data_preprocessor.enable_feature_detection = true

-- 自适应体素滤波选项
TRAJECTORY_BUILDER_2D.data_preprocessor.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.data_preprocessor.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.data_preprocessor.adaptive_voxel_filter.max_range = 15.

-- 激光感知选项
TRAJECTORY_BUILDER_2D.data_preprocessor.laser_perception.min_cluster_points = 5
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- [[局部匹配参数]]
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(30.)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.line_weight = 60
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.point_weight = 60

TRAJECTORY_BUILDER_2D.keyframe_motion_filter.max_distance_meters = 0.25
TRAJECTORY_BUILDER_2D.keyframe_motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.keyframe_motion_filter.max_angle_radians = math.rad(15.)
TRAJECTORY_BUILDER_2D.map_insertion_motion_filter.max_distance_meters = 0.25
TRAJECTORY_BUILDER_2D.map_insertion_motion_filter.max_time_seconds = 30
TRAJECTORY_BUILDER_2D.map_insertion_motion_filter.max_angle_radians = math.rad(45.)

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- [[回环检测参数]]
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2

return options
