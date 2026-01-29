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

#ifndef CARTOGRAPHER_DATA_PREPROCESSOR_DATA_PREPROCESSOR_OPTIONS_H_
#define CARTOGRAPHER_DATA_PREPROCESSOR_DATA_PREPROCESSOR_OPTIONS_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/preprocess/proto/data_preprocessor_options.pb.h"

namespace cartographer {
namespace preprocess {

proto::DataPreprocessorOptions CreateDataPreprocessorOptions(
  common::LuaParameterDictionary* parameter_dictionary);

}  // namespace preprocess
}  // namespace cartographer

#endif  // CARTOGRAPHER_DATA_PREPROCESSOR_DATA_PREPROCESSOR_OPTIONS_H_
