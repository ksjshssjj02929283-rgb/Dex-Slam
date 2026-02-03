/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/laser_perception/laser_perception_options.h"

namespace cartographer {
namespace laser_perception {

proto::LaserPerceptionOptions CreateLaserPerceptionOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LaserPerceptionOptions options;
  options.set_predict_rec(parameter_dictionary->GetBool("predict_rec"));
  options.set_reflect_rec(parameter_dictionary->GetBool("reflect_rec"));
  options.set_shape_rec(parameter_dictionary->GetBool("shape_rec"));
  options.set_line_rec(parameter_dictionary->GetBool("line_rec"));
  options.set_angle_rec(parameter_dictionary->GetBool("angle_rec"));

  options.set_intensity_threshold(parameter_dictionary->GetInt("intensity_threshold"));
  options.set_connect_threshold(parameter_dictionary->GetDouble("connect_threshold"));
  options.set_min_cluster_points(parameter_dictionary->GetInt("min_cluster_points"));
  options.set_search_distance(parameter_dictionary->GetDouble("search_distance"));
  options.set_max_dis_error(parameter_dictionary->GetDouble("max_dis_error"));
  options.set_max_ang_error(parameter_dictionary->GetDouble("max_ang_error"));
  options.set_start_rec_dis(parameter_dictionary->GetDouble("start_rec_dis"));

  options.set_target_file(parameter_dictionary->GetString("target_file"));
  options.set_model_file(parameter_dictionary->GetString("model_file"));

  options.set_min_select_points(parameter_dictionary->GetInt("min_select_points"));
  options.set_reflector_error(parameter_dictionary->GetDouble("reflector_error"));
  options.set_reflector_radius(parameter_dictionary->GetDouble("reflector_radius"));
  options.set_min_line_length(parameter_dictionary->GetDouble("min_line_length"));
  options.set_shape_match_score(parameter_dictionary->GetDouble("shape_match_score"));
  options.set_shape_match_close_degree(parameter_dictionary->GetDouble("shape_match_close_degree"));
  options.set_shape_dis_to_robot_center(parameter_dictionary->GetDouble("shape_dis_to_robot_center"));

  options.set_intensity_max_threshold(parameter_dictionary->GetInt("intensity_max_threshold"));
  options.set_laser_intensity_type(parameter_dictionary->GetInt("laser_intensity_type"));
  options.set_intensity_max_range_threshold(parameter_dictionary->GetDouble("intensity_max_range_threshold"));
  options.set_intensity_min_range_threshold(parameter_dictionary->GetDouble("intensity_min_range_threshold"));
  options.set_reflector_error_range(parameter_dictionary->GetDouble("reflector_error_range"));
  options.set_angle_increment(parameter_dictionary->GetDouble("angle_increment"));
  // CHECK_GT(options.intensity_threshold, 0) << "Intensity threshold must be greater than zero.";
  // CHECK_GT(options.connect_threshold, 0.0) << "Connect threshold must be greater than zero.";
  // CHECK_GT(options.min_cluster_points, 0) << "Minimum number of cluster points must be greater than zero.";
  // CHECK_GT(options.min_select_points, 0) << "Minimum number of selected points must be greater than zero.";
  // CHECK_GT(options.search_distance, 0.0) << "Search distance must be greater than zero.";
  // CHECK_GT(options.max_dis_error, 0.0) << "Maximum distance error must be greater than zero.";
  // CHECK_GE(options.max_ang_error, 0.0) << "Maximum angular error must be greater than or equal to zero.";
  // CHECK_GT(options.start_rec_dis, 0.0) << "Starting recognition distance must be greater than zero.";
  // CHECK(!options.target_file.empty()) << "Target file must not be empty.";
  // CHECK(!options.model_file.empty()) << "Model file must not be empty.";
  return options;
}

}  // namespace laser_perception
}  // namespace cartographer
