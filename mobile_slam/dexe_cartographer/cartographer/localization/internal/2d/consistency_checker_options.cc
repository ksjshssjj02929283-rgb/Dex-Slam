/*
 * Copyright 2025 The Cartographer Authors
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

#include "cartographer/localization/internal/2d/consistency_checker_options.h"

namespace cartographer {
namespace localization {

proto::ConsistencyCheckerOptions CreateConsistencyCheckerOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::ConsistencyCheckerOptions options;
  
  options.set_translation_threshold(
      parameter_dictionary->GetDouble("translation_threshold"));
  options.set_rotation_threshold(
      parameter_dictionary->GetDouble("rotation_threshold"));
  options.set_max_linear_velocity(
      parameter_dictionary->GetDouble("max_linear_velocity"));
  options.set_max_angular_velocity(
      parameter_dictionary->GetDouble("max_angular_velocity"));
  options.set_min_score(
      parameter_dictionary->GetDouble("min_score"));

  // ===== 静态地图漂移门控参数（向后兼容：旧 lua 缺省时用默认值）=====
  options.set_enable_drift_gate(
      parameter_dictionary->HasKey("enable_drift_gate")
          ? parameter_dictionary->GetBool("enable_drift_gate")
          : true);
  options.set_static_map_min_score(
      parameter_dictionary->HasKey("static_map_min_score")
          ? parameter_dictionary->GetDouble("static_map_min_score")
          : 0.30);
  options.set_drift_score_gap(
      parameter_dictionary->HasKey("drift_score_gap")
          ? parameter_dictionary->GetDouble("drift_score_gap")
          : 0.15);
  options.set_drift_trans_threshold(
      parameter_dictionary->HasKey("drift_trans_threshold")
          ? parameter_dictionary->GetDouble("drift_trans_threshold")
          : 0.12);
  options.set_drift_rot_threshold(
      parameter_dictionary->HasKey("drift_rot_threshold")
          ? parameter_dictionary->GetDouble("drift_rot_threshold")
          : 0.105);
  options.set_drift_persist_frames(
      parameter_dictionary->HasKey("drift_persist_frames")
          ? parameter_dictionary->GetInt("drift_persist_frames")
          : 3);

  return options;
}

}  // namespace localization
}  // namespace cartographer
