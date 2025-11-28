include "map_builder.lua"
include "trajectory_builder.lua"

-- 定位模式配置（不依赖 mapping 配置）
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 坐标系配置
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  
  -- TF发布配置
  publish_to_tf = false,
  provide_odom_frame = false,  -- 定位模式不提供odom，避免与robot_localization冲突
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  
  -- 传感器配置
  use_odometry = false,  -- 定位模式不使用里程计，由robot_localization提供融合后的odom
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,  -- 双雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 超时和发布周期
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,  -- 200Hz位姿发布
  trajectory_publish_period_sec = 30e-3,
  
  -- 传感器采样率
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  
  -- 自定义话题配置
  localPoseTopic = "carto_local_pose",
  publish_tracked_pose = true,
  publish_local_pose = true,
  
  -- 定位初始化配置
  sub_initial_pose = false,  -- 不使用手动初始位姿（RViz 2D Pose Estimate）
  sub_apriltag_pose = false,  -- 不使用 AprilTag 初始化
  use_predefined_init_pose = true,  -- 使用预定义的初始位姿
}

-- 2D SLAM配置
MAP_BUILDER.use_trajectory_builder_2d = true

-- 前端配置（定位模式）
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 双雷达累积帧数
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 50.

-- CSM扫描匹配器配置
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.

-- 子图配置（定位模式下较小）
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

-- 纯定位模式配置：限制保留的子图数量
TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 3,  -- 定位模式只保留最近3个子图
}

-- 后端优化配置（定位模式下降低频率）
POSE_GRAPH.optimize_every_n_nodes = 80  -- 定位模式下降低优化频率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.40

return options