include "map_builder.lua"
include "trajectory_builder.lua"
-- options里面部分属于NodeOptions，部分属于TrajectoryOptions
options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "imu_chassis_link",             -- 将所有传感器数据转换到这个坐标系下，如果有imu，这里一般设置为imu_link
  published_frame = "odom",            -- tf: map -> footprint，其实footprint和base_link坐标系是静态tf转换
  odom_frame = "odom",                      -- 里程计的坐标系名字
  publish_to_tf = true,                     
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint。如果 机器人/bag 中提供odom坐标系，设置成false，不然机器人位姿会来回跳
  publish_frame_projected_to_2d = true,    -- 是否将坐标系投影到平面上
  use_pose_extrapolator = false,          -- 发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿，一般不使用位姿估计器

  use_odometry = false,                      -- 是否使用里程计,如果使用,一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 2,                      -- 使用单线激光数据的个数
  num_multi_echo_laser_scans = 0,           -- 使用multi_echo_laser_scans数据的个数
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

  --localization_local_frame = "carto_odom",  -- added by hz
  localPoseTopic = "carto_local_pose",      -- added by hz
  publish_tracked_pose = true,              -- added by hz
  publish_local_pose = true,                -- added by hz
  
  sub_initial_pose = false,
  sub_apriltag_pose = false,                     -- added by hz
  use_predefined_init_pose = false,              -- added by hz
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 如果使用imu数据，tracking_frame一定要设置为imu_link
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2 -- added by hz，使用双雷达时设置为2
TRAJECTORY_BUILDER_2D.min_range = 0.2
TRAJECTORY_BUILDER_2D.max_range = 50.
--TRAJECTORY_BUILDER_2D.min_z = 0.2      --在仅使用单线激光雷达的时候，需要注释掉z轴上的限制，不然会报错 local_trajectory_builder_2d.cc:218] Dropped empty horizontal range data
--TRAJECTORY_BUILDER_2D.max_z = 1.4
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- csm粗匹配
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

POSE_GRAPH.optimize_every_n_nodes = 160.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.40

return options
