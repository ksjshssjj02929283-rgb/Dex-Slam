/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOC_OFFLINE_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOC_OFFLINE_NODE_H

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/options/localization_node_options.h"

namespace cartographer_ros {
namespace localization {

using LocalizationUpdaterFactory =
    std::function<std::unique_ptr<::cartographer::localization::LocalizationUpdater>(
        const ::cartographer::localization::proto::LocalizationUpdaterOptions&)>;

void RunLocOfflineNode(const LocalizationUpdaterFactory& localization_updater_factory,
                       rclcpp::Node::SharedPtr loc_offline_node);

}  // namespace localization
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOC_OFFLINE_NODE_H
