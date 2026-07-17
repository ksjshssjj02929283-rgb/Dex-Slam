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

proto::LineDetectorXOptions CreateLineDetectorXOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LineDetectorXOptions options;
  options.set_least_thresh(parameter_dictionary->GetDouble("least_thresh"));
  options.set_min_line_length(parameter_dictionary->GetDouble("min_line_length"));
  options.set_predict_distance(parameter_dictionary->GetDouble("predict_distance"));
  options.set_seed_line_points(parameter_dictionary->GetInt("seed_line_points"));
  options.set_min_line_points(parameter_dictionary->GetInt("min_line_points"));
  return options;
}

proto::LaserPerceptionOptions CreateLaserPerceptionOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LaserPerceptionOptions options;
  options.set_min_cluster_points(parameter_dictionary->GetInt("min_cluster_points"));
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_highlight_intensity_threshold(
      parameter_dictionary->GetInt("highlight_intensity_threshold"));
  options.set_cluster_distance_threshold(
      parameter_dictionary->GetDouble("cluster_distance_threshold"));
  *options.mutable_line_detector() = CreateLineDetectorXOptions(
      parameter_dictionary->GetDictionary("line_detector").get());
  return options;
}

}  // namespace laser_perception
}  // namespace cartographer
