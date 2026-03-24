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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_UPDATER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_UPDATER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>
#include <optional>

#include "absl/synchronization/mutex.h"

// Cartographer core
#include "cartographer/common/time.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer_ros/options/localization_node_options.h"
#include "cartographer_ros/mapping/sensor_bridge.h"
#include "cartographer_ros/mapping/tf_bridge.h"
#include "cartographer_ros/options/trajectory_options.h"
#include "cartographer_ros/common/map_loader.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif

namespace cartographer_ros {

/**
 * @brief map_builder_bridge是map_builder的ros接口，
 *        主要负责操作增加轨迹优化器、优化操作、获取当前地图信息、定位信息等操作
 * 
 */
class LocalizationUpdaterBridge {
 public:
  struct LocalTrajectoryData {
    // Contains the trajectory data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    struct LocalSlamData {
      ::cartographer::common::Time time;  // 时间
      ::cartographer::transform::Rigid3d local_pose;  // 本体在局部坐标系中的位姿 base_to_local
      ::cartographer::sensor::RangeData range_data_in_local;  // 局部坐标系下的激光数据
      std::shared_ptr<::cartographer::sensor::PointCloud> local_map;
      std::map<int, std::vector<::cartographer::sensor::LineData>> line_data_in_local;
      std::shared_ptr<::cartographer::mapping::LineMap2DList> line_map_2d_list;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    ::cartographer::transform::Rigid3d local_to_map;  // 局部坐标系到全局坐标系的变换
    std::unique_ptr<::cartographer::transform::Rigid3d> published_to_tracking;  // T_base_published
    TrajectoryOptions trajectory_options;  // 参数配置
  };

  LocalizationUpdaterBridge(
      const LocalizationNodeOptions& node_options,
      std::unique_ptr<::cartographer::localization::LocalizationUpdater> localization_updater,
      tf2_ros::Buffer* tf_buffer);

  LocalizationUpdaterBridge(const LocalizationUpdaterBridge&) = delete;
  LocalizationUpdaterBridge& operator=(const LocalizationUpdaterBridge&) = delete;

  // 状态/地图加载与控制
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  bool SetInitialPose(const ::cartographer::transform::Rigid3d& initial_pose);

  std::optional<::cartographer::localization::LocalizationState> GetLocalizationState() const;

  int AddTrajectory(
      const std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>& expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  bool SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // 保存地图（map.png + map.yaml）
  // map_filename 为不含后缀的路径前缀，例如 "/path/to/map"
  // is_from_pose_graph = true : 从 pose graph 所有轨迹节点重建地图（类似建图时的 SaveMap）
  // is_from_pose_graph = false: 从当前全局 submap（global_dynamic_submap 或 global_static_submap）生成
  bool SaveMap(const std::string& map_filename,
               const float occ_thresh,
               const float free_thresh,
               bool is_from_pose_graph = true);

  std::unique_ptr<::cartographer::io::SubmapTextures>
  FetchSubmapTextures(
      const ::cartographer::mapping::SubmapId& submap_id,
      cartographer_ros_msgs::srv::SubmapQuery::Response& response);

  void HandleSubmapQuery(
      const cartographer_ros_msgs::srv::SubmapQuery::Request& request,
      cartographer_ros_msgs::srv::SubmapQuery::Response& response);
  void HandleTrajectoryQuery(
      const cartographer_ros_msgs::srv::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::srv::TrajectoryQuery::Response& response);

  std::map<int /* trajectory_id */,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
  GetTrajectoryStates();

  cartographer_ros_msgs::msg::SubmapList GetSubmapList();

  std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
      LOCKS_EXCLUDED(mutex_);

  visualization_msgs::msg::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::msg::MarkerArray GetLandmarkPosesList();
  visualization_msgs::msg::MarkerArray GetConstraintList();

  ::cartographer::mapping::MapById<
      ::cartographer::mapping::NodeId, ::cartographer::mapping::TrajectoryNodePose>
  GetTrajectoryPoses() const;

  visualization_msgs::msg::MarkerArray GetFeatureMap();
  // std::vector<visualization_msgs::Marker> GetCodeMap(int& marker_id);
  sensor_msgs::msg::PointCloud2 GetCloudMap();
  std::vector<visualization_msgs::msg::Marker> GetLineMap(int& marker_id);
  std::vector<visualization_msgs::msg::Marker> GetReflectorMap(int& marker_id);

  // std::vector<Eigen::Vector2d> GetAllReflectors();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  void OnLocalSlamResult(
      const int trajectory_id,
      const ::cartographer::common::Time time,
      const ::cartographer::transform::Rigid3d local_pose,
      ::cartographer::sensor::RangeData range_data_in_local,
      std::shared_ptr<::cartographer::sensor::PointCloud> local_map,
      std::map<int, std::vector<::cartographer::sensor::LineData>> line_data_in_local,
      std::shared_ptr<::cartographer::mapping::LineMap2DList> line_map_2d_list)
    LOCKS_EXCLUDED(mutex_);

  // SaveMap 的两条路径
  bool SaveMapFromGlobalSubmap(const std::string& map_filename,
                               float occ_thresh,
                               float free_thresh);
  bool SaveMapFromPoseGraph(const std::string& map_filename,
                            float occ_thresh,
                            float free_thresh);

 private:
  mutable absl::Mutex mutex_;

  const LocalizationNodeOptions node_options_;
  std::unordered_map<int,
           std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
    local_slam_data_ GUARDED_BY(mutex_);
  std::unique_ptr<::cartographer::localization::LocalizationUpdater> localization_updater_;
  tf2_ros::Buffer* const tf_buffer_;  // 继续外部传入

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;

  // 地图加载器
  MapLoader map_loader_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_UPDATER_BRIDGE_H
