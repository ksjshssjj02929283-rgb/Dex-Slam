/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/localization/loc_offline_node.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "gflags/gflags.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  
  // 创建LocalizationUpdater工厂函数
  const cartographer_ros::localization::LocalizationUpdaterFactory 
      localization_updater_factory = [](
          const ::cartographer::localization::proto::LocalizationUpdaterOptions&
              localization_updater_options) {
    return ::cartographer::localization::CreateLocalizationUpdater(
        localization_updater_options);
  };
  
  // 注册离线定位节点。统一域名：节点名 = "localization"（与在线定位一致）
  // → 话题/服务全部落 /localization/ 域，在线离线消费方通用。
  rclcpp::Node::SharedPtr loc_offline_node =
      rclcpp::Node::make_shared("localization");
  
  cartographer_ros::localization::RunLocOfflineNode(
      localization_updater_factory, loc_offline_node);

  rclcpp::shutdown();
  
  return 0;
}
