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

#include "cartographer/localization/internal/2d/pose_initializer_options_2d.h"

#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace localization {

proto::PoseInitializerOptions2D CreatePoseInitializerOptions2D(
  common::LuaParameterDictionary* const parameter_dictionary) {
  proto::PoseInitializerOptions2D options;
  options.set_use_global_matching(
    parameter_dictionary->GetBool("use_global_matching"));
  options.set_global_localization_min_score(
    parameter_dictionary->GetDouble("global_localization_min_score"));
  *options.mutable_fast_correlative_scan_matcher_options() =
    mapping::scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
      parameter_dictionary
        ->GetDictionary("fast_correlative_scan_matcher")
        .get());
  
  options.set_use_online_correlative_scan_matching(
    parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_real_time_correlative_scan_matcher_options() =
    mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
      parameter_dictionary
        ->GetDictionary("real_time_correlative_scan_matcher")
        .get());

  *options.mutable_ceres_scan_matcher_options() =
    mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      parameter_dictionary->GetDictionary("ceres_scan_matcher").get());

  *options.mutable_line_matcher_options() =
    mapping::scan_matching::CreateLineMatcherOptions2D(
    parameter_dictionary->GetDictionary("initializer_line_matcher").get());
  return options;
}

}  // namespace localization
}  // namespace cartographer
