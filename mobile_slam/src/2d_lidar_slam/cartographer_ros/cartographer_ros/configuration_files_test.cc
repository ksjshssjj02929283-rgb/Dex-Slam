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

#include <string>
#include <vector>

#include "cartographer_ros/options/mapping_node_options.h"
#include "gtest/gtest.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace cartographer_ros {
namespace {

class ConfigurationFilesTest : public ::testing::TestWithParam<const char*> {};

TEST_P(ConfigurationFilesTest, ValidateNodeOptions) {
  EXPECT_NO_FATAL_FAILURE({
    // 获取安装后的共享目录，例如：
    // <install>/share/cartographer_ros/configuration_files
    const std::string share_dir =
        ament_index_cpp::get_package_share_directory("cartographer_ros");
    const std::string config_dir = share_dir + "/configuration_files";

    LoadMappingOptions(config_dir, GetParam());
  });
}

// GTest 现代 API：INSTANTIATE_TEST_SUITE_P
INSTANTIATE_TEST_SUITE_P(
    ValidateAllNodeOptions, ConfigurationFilesTest,
    ::testing::Values("backpack_2d.lua", "backpack_2d_localization.lua",
                      "backpack_3d.lua", "backpack_3d_localization.lua",
                      "pr2.lua", "revo_lds.lua", "taurob_tracker.lua"));

}  // namespace
}  // namespace cartographer_ros
