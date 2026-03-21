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

include "localization_updater.lua"
include "trajectory_builder.lua"

options = {
  localization_updater = LOCALIZATION_UPDATER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  publish_to_tf = false,
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.01,
  trajectory_publish_period_sec = 0.5,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_tracked_pose = true,
  save_tracked_pose = true,
  tracked_pose_format = "tum",
  tracked_pose_directory = os.getenv("HOME") .. "/.dexe/data/",
  tracked_pose_filename = "tracked_pose.txt",
  save_origin_map = false,
  navigation_map_filter_mode = "normal",
  save_debug_map = false,
  save_map_pcd = false,
}

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 2,
}

POSE_INITIALIZER_2D.use_global_matching = false

POSE_ESTIMATOR_2D.data_preprocessor.min_range = 0.2
POSE_ESTIMATOR_2D.data_preprocessor.max_range = 15.
POSE_ESTIMATOR_2D.data_preprocessor.missing_data_ray_length = 5.
POSE_ESTIMATOR_2D.data_preprocessor.use_deskew = true
POSE_ESTIMATOR_2D.data_preprocessor.use_odometry = true
POSE_ESTIMATOR_2D.data_preprocessor.enable_feature_detection = true

-- 自适应体素滤波选项
POSE_ESTIMATOR_2D.data_preprocessor.adaptive_voxel_filter.max_length = 0.5
POSE_ESTIMATOR_2D.data_preprocessor.adaptive_voxel_filter.min_num_points = 200
POSE_ESTIMATOR_2D.data_preprocessor.adaptive_voxel_filter.max_range = 15.

-- 激光感知选项
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.predict_rec = false
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.reflect_rec = false
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.shape_rec = false
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.line_rec = false
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.angle_rec = false
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.intensity_threshold = 30
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.connect_threshold = 0.05
POSE_ESTIMATOR_2D.data_preprocessor.laser_perception.min_cluster_points = 5
-- POSE_ESTIMATOR_2D.use_imu_data = false
POSE_ESTIMATOR_2D.use_online_correlative_scan_matching = false

-- [[局部匹配参数]]
POSE_ESTIMATOR_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1
POSE_ESTIMATOR_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1
POSE_ESTIMATOR_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
POSE_ESTIMATOR_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.)
POSE_ESTIMATOR_2D.ceres_scan_matcher.occupied_space_weight = 60
POSE_ESTIMATOR_2D.ceres_scan_matcher.translation_weight = 1
POSE_ESTIMATOR_2D.ceres_scan_matcher.rotation_weight = 1
POSE_ESTIMATOR_2D.ceres_scan_matcher.line_weight = 120
POSE_ESTIMATOR_2D.ceres_scan_matcher.point_weight = 120
POSE_ESTIMATOR_2D.keyframe_motion_filter.max_distance_meters = 0.2
POSE_ESTIMATOR_2D.keyframe_motion_filter.max_time_seconds = 0.5
POSE_ESTIMATOR_2D.keyframe_motion_filter.max_angle_radians = math.rad(45.)
POSE_ESTIMATOR_2D.map_insertion_motion_filter.max_distance_meters = 0.4
POSE_ESTIMATOR_2D.map_insertion_motion_filter.max_time_seconds = 30
POSE_ESTIMATOR_2D.map_insertion_motion_filter.max_angle_radians = math.rad(90.)

-- 需要测试num_range_data对定位的影响
POSE_ESTIMATOR_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.61
POSE_ESTIMATOR_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48
POSE_ESTIMATOR_2D.submaps.num_range_data = 20
POSE_ESTIMATOR_2D.submaps.grid_options_2d.resolution = 0.04

-- [[回环检测参数]]
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.optimize_every_n_nodes = 0

-- [[节点裁剪参数]]
-- 注意：定位模式不建议启用节点裁剪
-- POSE_GRAPH.glc_trimmer_2d = {
--   overlap_high_threshold = 0.8,
--   enable_glc = true,
-- }

return options
