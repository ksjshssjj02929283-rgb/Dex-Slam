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
include "amcl_core.lua"

POSE_ESTIMATOR_2D = {
  use_imu_data = true,
  min_range = 0.,
  max_range = 30.,
  min_z = -0.8,
  max_z = 2.,
  missing_data_ray_length = 5.,
  voxel_filter_size = 0.04,

  pose_initializer_2d = POSE_INITIALIZER_2D,
  amcl_core = AMCL_CORE,
  auto_trigger_global_relocalization = true,  -- 控制是否自动触发全局重定位

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
    enable_feature_detection = true,
    enable_multi_sensor_fusion = false,

    adaptive_voxel_filter = {
      max_length = 0.5,
      min_num_points = 200,
      max_range = 50.,
    },
    
    laser_perception = {
      predict_rec = false,
      reflect_rec = false,
      shape_rec = false,
      line_rec = false,
      angle_rec = false,
      intensity_threshold = 30,
      connect_threshold = 0.05,
      min_cluster_points = 5,
      search_distance = 0.2,
      max_dis_error = 0.02,
      max_ang_error = 0.1,
      start_rec_dis = 0.5,
      target_file = "",
      model_file = "",
      min_select_points = 10,
      reflector_error = 0.05,
      reflector_radius = 0.1,
      min_line_length = 0.3,
      shape_match_score = 0.8,
      shape_match_close_degree = 0.9,
      shape_dis_to_robot_center = 5.0,
      intensity_max_threshold = 255,
      laser_intensity_type = 0,
      intensity_max_range_threshold = 30.0,
      intensity_min_range_threshold = 0.1,
      reflector_error_range = 0.1,
      angle_increment = 0.00436332,
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
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.3,
    angular_search_window = math.rad(10.),
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
    min_delta_tra_parallel = 0.1;
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
