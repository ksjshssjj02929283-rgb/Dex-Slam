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

#ifndef CARTOGRAPHER__LOCALIZATION__AMCL__CORE__AMCL_CORE_OPTIONS_H_
#define CARTOGRAPHER__LOCALIZATION__AMCL__CORE__AMCL_CORE_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/localization/proto/amcl_core_options.pb.h"

namespace cartographer {
namespace localization {
namespace amcl {

proto::AMCLCoreOptions CreateAMCLCoreOptions(
    common::LuaParameterDictionary* parameter_dictionary);

}  // namespace amcl
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER__LOCALIZATION__AMCL__CORE__AMCL_CORE_OPTIONS_2D_H_
