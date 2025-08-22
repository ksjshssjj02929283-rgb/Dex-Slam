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

#include "cartographer/preprocess/data_preprocessor_options.h"

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace preprocess {
proto::DataPreprocessorOptions CreateDataPreprocessorOptions(
	common::LuaParameterDictionary* const parameter_dictionary) {
  proto::DataPreprocessorOptions options;
  options.set_min_range(parameter_dictionary->GetDouble("min_range"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  options.set_missing_data_ray_length(
	  parameter_dictionary->GetDouble("missing_data_ray_length"));
  options.set_use_deskew(
	  parameter_dictionary->GetBool("use_deskew"));
  if (parameter_dictionary->HasKey("use_odometry")) {
    options.set_use_odometry(
      parameter_dictionary->GetBool("use_odometry"));
  } else {
    options.set_use_odometry(true);
  }

  options.set_num_accumulated_range_data(
	  parameter_dictionary->GetInt("num_accumulated_range_data"));

  return options;
}

}  // namespace preprocess
}  // namespace cartographer
