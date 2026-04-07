// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__CONFIG_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__CONFIG_HPP_
#include <string>

namespace perception_2d
{
typedef struct
{
  bool predict_rec = false;
  bool reflect_rec = false;
  bool shape_rec = false;
  bool line_rec = false;
  bool angle_rec = false;
  bool vmark_rec = false;
  int intensity_threshold = 220;
  double connect_threshold = 0.05;
  double range_max = 10;
  double range_min = 0.1;
  int min_cluster_points = 5;
  double search_distance = 0.25;
  double max_dis_error = 0.1;
  double max_ang_error = 0.2;
  double start_rec_dis = 2.0;
  std::string target_file = "";
  std::string model_file = "";
  int min_select_points = 4;
  double reflector_error = 0.04;
  double reflector_radius = 0.06;
  double min_line_length = 0.35;
  double shape_match_score = 0.7;
  double shape_match_close_degree = 0.05;
  double shape_dis_to_robot_center = 0.335;
} Config;

}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__CONFIG_HPP_
