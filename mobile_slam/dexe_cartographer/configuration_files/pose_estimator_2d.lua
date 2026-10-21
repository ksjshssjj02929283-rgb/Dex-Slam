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

include "pose_initializer_2d.lua"

POSE_ESTIMATOR_2D = {
  use_imu_data = true,
  min_range = 0.,
  max_range = 30.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  voxel_filter_size = 0.04,

  pose_initializer_2d = POSE_INITIALIZER_2D,
  auto_trigger_global_relocalization = true,  -- 控制是否自动触发全局重定位

  -- 一致性检查器参数：用于位姿一致性检查和质量验证
  consistency_checker = {
    translation_threshold = 0.05,        -- 平移误差阈值 (米)，5cm
    rotation_threshold = math.rad(30.),  -- 旋转误差阈值 (弧度)，30度
    max_linear_velocity = 1.0,           -- 最大线速度 (m/s)
    max_angular_velocity = math.rad(90.),-- 最大角速度 (rad/s)，45度/秒
    min_score = 0.3,                     -- 最小匹配分数阈值，用于验证和恢复
    -- 恢复（RecoverFromLocalMap）改用 FCSM 大窗口；旧 RTCSM 0.2 m/15° 与检查容差同量级搜不到
    fast_correlative_scan_matcher = {
      linear_search_window = 3.,           -- 局部图 ±15 m，恢复窗口不越出
      angular_search_window = math.rad(180.),
      branch_and_bound_depth = 7,
    },

    -- ===== 一致性/恢复现场快照（基础功能，默认开启）=====
    -- 一致性检查失败 / 恢复成功 / 恢复失败 三个时刻各存一张 PNG：
    -- 背景 = 本次匹配用的 local submap（灰度同 local_map.png），叠加当前位姿下的点云。
    -- 合并前各雷达的去畸变点云分色：若多雷达时间对齐有问题，两片会整体错开一个角度。
    -- PNG 编码在后台线程，定位主链路只付约 1-3 ms；队列满则丢弃新帧并计数。
    -- 文件名 = 当前帧时间戳 + 事件名；目录是环形的，写满后删最早的一张。
    -- 之所以能常开：只在一致性检查失败时产图，健康运行一张不写、零开销；
    -- 实测单张约 140 KB（±15m submap、scale=2），1000 张 ≈ 136 MB 稳态上限。
    debug_dump_enabled = true,
    debug_dump_dir = "",                 -- 空 = $HOME/.dexe/data/consistency_check
    debug_dump_max_files = 1000,         -- 保留最近 N 张，超出删最早；<=0 不限
    debug_dump_scale = 1,                -- 输出放大倍数
    debug_dump_point_radius = 2,         -- 雷达点半径(像素)，点太小看不清就调大；0 = 单像素
    -- 每帧匹配都存一张（不只失败时），落到 frames/ 子目录、独立环形区，
    -- 不会挤掉主目录里的失败现场。1.88Hz 下约 0.26 MB/s，1000 张约 9 分钟一轮。
    debug_dump_every_frame = false,

    -- 静态地图漂移门控：边定位边更新地图时拦截"自洽漂移"（在错误位姿更新地图）。
    -- 原理：现有三路信号都参照 odom 预测 + live local map（被同一漂移污染），对缓慢漂移失明。
    -- 用从不更新的 global_static_submap_ 做独立锚点：漂移时 live 重合度高但 static 低，
    -- 且独立再匹配出现位姿 delta → 连续若干帧确认后冻结地图更新。
    enable_drift_gate = true,            -- 是否启用静态地图漂移门控
    static_map_min_score = 0.30,         -- static map 重合度 / 再匹配得分下限（低于则 static 无法判定，跳过避免新区域误冻）
    drift_score_gap = 0.15,              -- (live 重合度 - static 重合度) 触发漂移指纹的最小差
    drift_trans_threshold = 0.12,        -- 独立再匹配相对估计位姿的平移 delta 阈值 (米)
    drift_rot_threshold = math.rad(6.),  -- 独立再匹配相对估计位姿的旋转 delta 阈值 (弧度)
    drift_persist_frames = 3,            -- 连续多少帧确认漂移才冻结地图（去抖）
  },

  adaptive_voxel_filter = {
    max_length = 0.5,
    min_num_points = 200,
    max_range = 50.,
  },

  data_preprocessor = {
    min_range = 0.,
    max_range = 30.,
    missing_data_ray_length = 5.,
    voxel_filter_size = 0.04,
    use_deskew = true,
    use_odometry = true,
    -- 里程计外插上限（秒）= odom delta 限幅：超过即保持最后位姿、twist 置零。底盘掉 OP 停发时防预测跑飞
    max_odometry_extrapolation_seconds = 0.2,
    enable_feature_detection = true,
    enable_multi_sensor_fusion = false,

    adaptive_voxel_filter = {
      max_length = 0.5,
      min_num_points = 200,
      max_range = 50.,
    },
    
    laser_perception = {
      min_cluster_points = 5,
      min_range = 0.1,
      max_range = 15.,
      highlight_intensity_threshold = 900,
      cluster_distance_threshold = 0.05,
      line_detector = {
        least_thresh = 0.03,
        min_line_length = 0.4,
        predict_distance = 0.05,
        seed_line_points = 10,
        min_line_points = 10,
      },
    },
  },

  loop_closure_adaptive_voxel_filter = {
    max_length = 0.9,
    min_num_points = 100,
    max_range = 50.,
  },

  local_map_updater = {
    min_x = -15.,
    max_x = 15.,
    min_y = -15.,
    max_y = 15.,
    min_z = -1.,
    max_z = 1.,
    update_threshold = 7.5,
    -- 动态地图策略:
    -- 0 (NONE): 不使用dynamic，也不更新dynamic map
    -- 1 (SHORT): 短时间保留（暂未实现）
    -- 2 (LONG): 长时间保留在内存，车辆移出区域后仍然保留，但不会存盘
    -- 3 (PERSISTENT): 更长时间的保留，会存盘，下次运行也会读取
    dynamic_map_policy = 3,  -- 默认使用 PERSISTENT 策略
    save_debug_data = true,
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.2,
    angular_search_window = math.rad(5.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 1.,
    translation_weight = 10.,
    rotation_weight = 40.,
    line_weight = 40.,
    point_weight = 40.,
    max_correspondence_distance = 0.1,
    local_weight_factor = 0.5;  -- 0 ~ 1
    use_improved_line_cost = true,
    line_distance_huber_scale = 0.1,
    line_angle_huber_scale = math.rad(5.),
    line_length_reference = 1.0,
    line_overlap_gap_weight = 0.5,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },

  keyframe_motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  map_insertion_motion_filter = {
    max_time_seconds = 5.,
    max_distance_meters = 0.2,
    max_angle_radians = math.rad(1.),
  },

  estimator_line_matcher = {
    line_delta_theta_threshold = math.rad(10.),
    line_delta_tra_elongation_threshold = 0.5,
    min_delta_tra_parallel = 0.1,
    max_match_distance = 12.0,
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  imu_gravity_time_constant = 10.,
  pose_extrapolator = {
    use_imu_based = false,
    constant_velocity = {
      imu_gravity_time_constant = 9.806,
      pose_queue_duration = 0.3,
    },
    imu_based = {
      pose_queue_duration = 5.,
      gravity_constant = 9.806,
      pose_translation_weight = 1.,
      pose_rotation_weight = 1.,
      imu_acceleration_weight = 1.,
      imu_rotation_weight = 1.,
      odometry_translation_weight = 1.,
      odometry_rotation_weight = 1.,
      solver_options = {
        use_nonmonotonic_steps = false;
        max_num_iterations = 10;
        num_threads = 1;
      },
    },
  },

  submaps = {
    num_range_data = 50,
    cloud_culling_prob = 0.65,
    min_line_length = 0.4,
    max_line_length = 7.0,
    line_culling_obs = 2,
    culling_interval = 5,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.04,
    },
    range_data_inserter = {
      range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
      probability_grid_range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.61,
        miss_probability = 0.48,
      },
      tsdf_range_data_inserter = {
        truncation_distance = 0.3,
        maximum_weight = 10.,
        update_free_space = false,
        normal_estimation_options = {
          num_normal_samples = 4,
          sample_radius = 0.5,
        },
        project_sdf_distance_to_scan_normal = true,
        update_weight_range_exponent = 0,
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
