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

#ifndef CARTOGRAPHER_LOCALIZATION_LOCALIZATION_UPDATER_H_
#define CARTOGRAPHER_LOCALIZATION_LOCALIZATION_UPDATER_H_

#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/localization/proto/localization_updater_options.pb.h"
#include "cartographer/localization/internal/2d/pose_estimator_2d.h"
#include "cartographer/sensor/collator_interface.h"
#include "cartographer/sensor/line_data.h"

namespace cartographer {
namespace localization {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class LocalizationUpdater
{
public:
  using LocalSlamResultCallback =
    mapping::TrajectoryBuilderInterface::LocalSlamResultCallback;

  using SensorId = mapping::TrajectoryBuilderInterface::SensorId;

  explicit LocalizationUpdater(const proto::LocalizationUpdaterOptions &options);
  ~LocalizationUpdater() {}

  LocalizationUpdater(const LocalizationUpdater &) = delete;
  LocalizationUpdater &operator=(const LocalizationUpdater &) = delete;

  // 从全局静态submap中获取地图数据（仅在加载pbstream后可用）
  std::shared_ptr<GridMap2D> GetGlobalGridMap() const;
  std::shared_ptr<CloudMap2D> GetGlobalCloudMap() const;
  std::shared_ptr<CloudMap2D> GetGlobalHighlighCloudMap() const;
  std::shared_ptr<sensor::LineMapData> GetGlobalLineMap() const;

  // 进行重定位
  bool SetInitialPose(const transform::Rigid3d &initial_pose);
  std::optional<LocalizationState> GetLocalizationState();

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const mapping::proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback);

  int AddTrajectoryForDeserialization(
      const mapping::proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto);

  void FinishTrajectory(int trajectory_id);

  std::string SubmapToProto(const mapping::SubmapId &submap_id,
                            mapping::proto::SubmapQuery::Response *response);

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer);

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename);

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state);

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state);


  // 全局一致的反光板地图
  const std::shared_ptr<std::vector<Eigen::Vector2d>>
  GetReflectorMap() const {
    return global_reflector_map_;
  }

  mapping::PoseGraphInterface *pose_graph() {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const {
    return all_trajectory_builder_options_;
  }
  
  // 获取 PoseEstimator2D (用于获取动态地图)
  localization::PoseEstimator2D* GetPoseEstimator2D() const {
    return pose_estimator_2d_;
  }

private:
  const proto::LocalizationUpdaterOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping::PoseGraph> pose_graph_;

  // ValueConversionTables用于global_static_submap_的概率值转换
  // 必须是持久的成员变量，因为grid保存了指向它内部查找表的指针
  mapping::ValueConversionTables conversion_tables_;
  
  std::shared_ptr<mapping::Submap2D> global_static_submap_;

  std::shared_ptr<std::vector<Eigen::Vector2d>> global_reflector_map_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  // 保存全局轨迹的构建器
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
      
  // 保存 PoseEstimator2D 指针 (用于访问动态地图)
  localization::PoseEstimator2D* pose_estimator_2d_ = nullptr;
};

std::unique_ptr<LocalizationUpdater> CreateLocalizationUpdater(
    const proto::LocalizationUpdaterOptions& options);

proto::LocalizationUpdaterOptions CreateLocalizationUpdaterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);
}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_LOCALIZATION_UPDATER_H_
