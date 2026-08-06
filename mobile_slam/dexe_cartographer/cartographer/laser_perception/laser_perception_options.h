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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_OPTIONS_H_
#define CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/laser_perception/proto/laser_perception_options.pb.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace laser_perception {

proto::LineDetectorXOptions CreateLineDetectorXOptions(
    common::LuaParameterDictionary* parameter_dictionary);

proto::LaserPerceptionOptions CreateLaserPerceptionOptions(
    common::LuaParameterDictionary* parameter_dictionary);

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_OPTIONS_H_
