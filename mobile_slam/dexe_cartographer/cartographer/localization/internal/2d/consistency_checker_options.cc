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
  
  return options;
}

}  // namespace localization
}  // namespace cartographer
