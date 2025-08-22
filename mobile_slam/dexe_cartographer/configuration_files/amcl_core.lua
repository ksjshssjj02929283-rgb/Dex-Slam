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

AMCL_CORE = {
  min_particles = 100,
  max_particles = 500,
  kld_err = 0.01,
  kld_z = 0.99,
  recovery_alpha_slow = 0.,
  recovery_alpha_fast = 0.,
  
  resample_interval = 1,

  max_beams = 100,

  odom_model_type = "diff",
  odom_alpha1 = 0.6,
  odom_alpha2 = 0.6,
  odom_alpha3 = 0.9,
  odom_alpha4 = 0.6,
  odom_alpha5 = 0.3,

  initial_cov_xx = 0.01,
  initial_cov_yy = 0.01,
  initial_cov_aa = 0.01,
}
