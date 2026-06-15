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
#include "glog/logging.h"

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

  // 全局匹配器版本选择（向后兼容：旧 lua 无字段时默认 GPU 角度版 / shift=2 / 4 线程）。
  if (parameter_dictionary->HasKey("global_matcher_type")) {
    const std::string t =
        parameter_dictionary->GetString("global_matcher_type");
    if (t == "gpu") {
      options.set_global_matcher_type(proto::GLOBAL_MATCHER_GPU_ANGULAR);
    } else if (t == "cpu") {
      options.set_global_matcher_type(proto::GLOBAL_MATCHER_CPU_ANGULAR);
    } else if (t == "original") {
      options.set_global_matcher_type(proto::GLOBAL_MATCHER_ORIGINAL);
    } else {
      LOG(WARNING) << "Unknown global_matcher_type=\"" << t
                   << "\" (should be gpu/cpu/original), fallback to gpu.";
      options.set_global_matcher_type(proto::GLOBAL_MATCHER_GPU_ANGULAR);
    }
  } else {
    options.set_global_matcher_type(proto::GLOBAL_MATCHER_GPU_ANGULAR);
  }
  // 注：angular_block_shift 已写死为代码常量（算法安全参数，不从 lua 暴露）。
  options.set_global_matcher_num_threads(
      parameter_dictionary->HasKey("global_matcher_num_threads")
          ? parameter_dictionary->GetInt("global_matcher_num_threads")
          : 4);
  return options;
}

}  // namespace localization
}  // namespace cartographer
