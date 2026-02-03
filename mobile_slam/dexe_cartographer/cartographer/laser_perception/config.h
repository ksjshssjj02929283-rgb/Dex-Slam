// Copyright (c) 2021 Geekplus Inc
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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__CONFIG_H_
#define CARTOGRAPHER__LASER_PERCEPTION__CONFIG_H_
#include <string>
namespace cartographer
{
namespace laser_perception
{
typedef struct
{
  bool predict_rec = false;
  bool reflect_rec = false;
  bool shape_rec = false;
  bool line_rec = false;
  bool angle_rec = false;
  int intensity_threshold = 220;
  double connect_threshold = 0.05;
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
  double min_line_length = 0.4;
  double shape_match_score = 0.7;
  double shape_match_close_degree = 0.05;
  double shape_dis_to_robot_center = 0.335;
  int intensity_max_threshold = 235;
  int laser_intensity_type = 1;
  double intensity_max_range_threshold = 5;
  double intensity_min_range_threshold = 1.5;
  double reflector_error_range = 0.5;
  double angle_increment = 0.0057;
} Config;

}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__CONFIG_H_
