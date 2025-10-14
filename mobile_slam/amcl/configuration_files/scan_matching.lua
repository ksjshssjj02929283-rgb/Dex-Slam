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
options = {
  global_localization_min_score = 0.3,
  use_global_matching = false;
  voxel_filter_size = 0.025,

  fast_correlative_scan_matcher = {
    linear_search_window = 1.,
    angular_search_window = math.rad(30.),
    branch_and_bound_depth = 7,
  },

  ceres_scan_matcher = {
    occupied_space_weight = 50.,
    translation_weight = 10.,
    rotation_weight = 10.,
    point_weight = 50.,
    max_correspondence_distance = 0.1,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 30,
      num_threads = 1,
    },
  },

  use_online_correlative_scan_matching = true,
  real_time_correlative_scan_matcher = {
    linear_search_window = 0.3,
    angular_search_window = math.rad(5.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },
}

return options
