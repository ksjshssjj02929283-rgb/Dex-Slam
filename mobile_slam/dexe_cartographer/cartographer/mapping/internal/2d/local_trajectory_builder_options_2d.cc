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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_options_2d.h"

#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/preprocess/data_preprocessor_options.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "cartographer/mapping/pose_extrapolator_interface.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"

namespace cartographer {
namespace mapping {

proto::LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
  common::LuaParameterDictionary* const parameter_dictionary) {
  proto::LocalTrajectoryBuilderOptions2D options;
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_voxel_filter_size(
    parameter_dictionary->GetDouble("voxel_filter_size"));
  options.set_use_online_correlative_scan_matching(
    parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
  *options.mutable_data_preprocessor_options() =
    preprocess::CreateDataPreprocessorOptions(
      parameter_dictionary->GetDictionary("data_preprocessor").get());
  *options.mutable_loop_closure_adaptive_voxel_filter_options() =
    sensor::CreateAdaptiveVoxelFilterOptions(
      parameter_dictionary
        ->GetDictionary("loop_closure_adaptive_voxel_filter")
        .get());
  *options.mutable_real_time_correlative_scan_matcher_options() =
    mapping::scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
      parameter_dictionary
        ->GetDictionary("real_time_correlative_scan_matcher")
        .get());
  *options.mutable_ceres_scan_matcher_options() =
    mapping::scan_matching::CreateCeresScanMatcherOptions2D(
      parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
  *options.mutable_keyframe_motion_filter_options() = mapping::CreateMotionFilterOptions(
    parameter_dictionary->GetDictionary("keyframe_motion_filter").get());
  *options.mutable_map_insertion_motion_filter_options() = mapping::CreateMotionFilterOptions(
    parameter_dictionary->GetDictionary("map_insertion_motion_filter").get());
  *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
    parameter_dictionary->GetDictionary("pose_extrapolator").get());
  options.set_imu_gravity_time_constant(
    parameter_dictionary->GetDouble("imu_gravity_time_constant"));
  *options.mutable_submaps_options() = CreateSubmapsOptions2D(
    parameter_dictionary->GetDictionary("submaps").get());
  options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
  *options.mutable_line_matcher_options() =
    mapping::scan_matching::CreateLineMatcherOptions2D(
    parameter_dictionary->GetDictionary("local_line_matcher").get());
  return options;
}

}  // namespace mapping
}  // namespace cartographer
