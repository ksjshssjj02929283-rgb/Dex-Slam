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
constexpr char kOccupancyGridTopic[] = "map";
constexpr char kSubmapListTopic[] = "submap_list";
constexpr char kTrackedPoseTopic[] = "tracked_pose";
constexpr char kTrackedPathTopic[] = "tracked_path";
// tracked_path 是标准可视化输出（非评估用全量轨迹）：滑窗只保留最近 N 个位姿。
// @ pose_publish_period_sec=0.1（10Hz）≈ 5 秒尾迹；原实现无上限追加，内存与带宽随运行时长线性增长。
constexpr size_t kTrackedPathWindowSize = 50;
// 相邻两帧位姿跳变超过该距离（米）视为发生了重定位，清空尾迹——否则会画出一条跨越地图的直线。
// 取值依据：10Hz 发布下正常运动帧间位移约 0.1m（1 m/s），留 5 倍余量；实测重定位跳变约 1.0m。
constexpr double kTrackedPathJumpThreshold = 0.5;
constexpr char kLocaliztionPoseTopic[] = "loc_pose";
constexpr char kSubmapQueryServiceName[] = "submap_query";
constexpr char kTrajectoryQueryServiceName[] = "trajectory_query";
constexpr char kStartTrajectoryServiceName[] = "start_trajectory";
constexpr char kWriteStateServiceName[] = "write_state";
constexpr char kGetTrajectoryStatesServiceName[] = "get_trajectory_states";
constexpr char kReadMetricsServiceName[] = "read_metrics";
constexpr char kHandleRelocalizationServiceName[] = "localization/relocalization";
constexpr char kHandleSetMapChangedServiceName[] = "localization/set_map_changed";
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
