/*
 * @Author: Buddy_X guo_linfeng@foxmail.com
 * @Date: 2023-05-19 17:50:12
 * @LastEditors: Buddy_X guo_linfeng@foxmail.com
 * @LastEditTime: 2023-05-29 09:39:49
 * @FilePath: /0_dcm_ws/src/cartographer_ros/cartographer_ros/common/node_constants.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_COMMON_NODE_CONSTANTS_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_COMMON_NODE_CONSTANTS_H

#include <string>
#include <vector>

namespace cartographer_ros {

// Default topic names; expected to be remapped as needed.
constexpr char kLaserScanTopic[] = "scan";
constexpr char kMultiEchoLaserScanTopic[] = "echoes";
constexpr char kPointCloud2Topic[] = "points2";
constexpr char kImuTopic[] = "imu";
constexpr char kOdometryTopic[] = "/feedback/odom";
constexpr char kNavSatFixTopic[] = "fix";
constexpr char kLandmarkTopic[] = "landmark";
constexpr char kFinishTrajectoryServiceName[] = "finish_trajectory";
constexpr char kOccupancyGridTopic[] = "mapping_map";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kTrackedPoseTopic[] = "tracked_pose";
constexpr char kTrackedPathTopic[] = "tracked_path";
constexpr char kLocaliztionPoseTopic[] = "loc_pose";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kTrajectoryQueryServiceName[] = "trajectory_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr char kReadMetricsServiceName[] = "read_metrics";
constexpr char kHandleRelocalizationServiceName[] = "localization_node/relocalization";
constexpr char kHandleSetMapChangedServiceName[] = "localization_node/set_map_changed";
constexpr char kTrajectoryNodeListTopic[] = "trajectory_node_list";
constexpr char kLandmarkPosesListTopic[] = "landmark_poses_list";
constexpr char kConstraintListTopic[] = "constraint_list";
constexpr char kSetInitialPoseTopic[] = "initialpose";
constexpr char kGlobalPointCloudTopic[] = "global_cloud_map";
constexpr char kGlobalFeatureMapTopic[] = "global_feature_map";
constexpr char kLocalPointCloudTopic[] = "local_cloud_map";
constexpr char kLocalFeatureMapTopic[] = "local_feature_map";
constexpr char kMatchedFeatureTopic[] = "matched_feature";
constexpr char kScanMatchedPointCloudTopic[] = "scan_matched_points2";
constexpr char kLocalizationStateTopic[] = "localization_state";
constexpr char kParticlePosesTopic[] = "particle_poses";

constexpr double kConstraintPublishPeriodSec = 0.5;
constexpr double kTopicMismatchCheckDelaySec = 3.0;

constexpr int kInfiniteSubscriberQueueSize = 0;
constexpr int kLatestOnlyPublisherQueueSize = 1;

// For multiple topics adds numbers to the topic name and returns the list.
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   int num_topics);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_COMMON_NODE_CONSTANTS_H
