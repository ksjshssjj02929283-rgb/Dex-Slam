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

POSE_INITIALIZER_2D = {
  use_global_matching = false,
  global_localization_min_score = 0.3,

  global_matcher_type = "gpu",  -- "gpu" or "cpu"
  global_matcher_num_threads = 4,

  fast_correlative_scan_matcher = {
    linear_search_window = 5.,
    angular_search_window = math.rad(180.),
    branch_and_bound_depth = 9,
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.5,
    angular_search_window = math.rad(30.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 40,
    translation_weight = 20.,
    rotation_weight = 10.,
    line_weight = 40.,
    point_weight = 40.,
    max_correspondence_distance = 0.2,
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

  initializer_line_matcher = {
    line_delta_theta_threshold = math.rad(10.),
    line_delta_tra_elongation_threshold = 1.0,
    min_delta_tra_parallel = 0.1,
    max_match_distance = 12.0,
  },
}
