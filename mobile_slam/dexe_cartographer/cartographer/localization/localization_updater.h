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
#include <tuple>
#include <vector>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/localization/proto/localization_updater_options.pb.h"
#include "cartographer/localization/internal/2d/pose_estimator_2d.h"
#include "cartographer/localization/2d/chunk_map_2d.h"
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
  void TriggerGlobalRelocalization();
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

  // 将所有全局位姿（轨迹节点、submap、路标）平移/旋转到以机器人当前位置为原点的新坐标系。
  // T 为从旧坐标系到新坐标系的变换（即 T_old_in_new），通常为机器人位姿的逆。
  // 同时更新 global_static_submap_ 的 local_pose。
  void ApplyOriginTransform(const transform::Rigid3d& T);

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

  // 获取 chunk map（可视化/调试用；未启用 chunk 时为 nullptr）。
  std::shared_ptr<ChunkMap2D> chunk_map() const { return chunk_map_; }

  // 注入当前地图目录（= 加载的 map.pbstream 所在目录），供 LocalMapUpdater2D 落 debug data。
  // 由 bridge / offline 在 LoadState 前调用；AddTrajectoryBuilder 时透传给 LocalMapUpdater2D。
  void SetMapDir(const std::string& map_dir) { loaded_map_dir_ = map_dir; }

private:
  // 从 pose_graph_ 的所有轨迹节点重新生成 global_static_submap_
  // 当旧格式 pbstream（不含 kGlobalStaticSubmap 字段）被加载时，作为 fallback
  std::shared_ptr<mapping::Submap2D> GenerateGlobalStaticSubmap(double resolution);

  // 创建/加载 chunk map（在 LoadState 调用，与其他 map 元素一致）：
  // pbstream 含 chunk 且 chunk_length 匹配 → LoadFromChunkSubmaps 复用；
  // 否则从 global_static_submap_ BuildFromSubmap 重建。幂等（chunk_map_ 已建则跳过）。
  void CreateOrLoadChunkMap();

  // 保存时（SerializeState）从给定 submap 临时构建 chunk map（方案 B：与保存的 global
  // submap 同源一致）；返回的 chunk map 交给 WritePbStream 在最后统一序列化（chunk 最后写）。
  std::unique_ptr<ChunkMap2D> BuildChunkForSave(
      const std::shared_ptr<mapping::Submap2D>& submap);
  const proto::LocalizationUpdaterOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping::PoseGraph> pose_graph_;

  // ValueConversionTables用于global_static_submap_的概率值转换
  // 必须是持久的成员变量，因为grid保存了指向它内部查找表的指针
  mapping::ValueConversionTables conversion_tables_;
  
  std::shared_ptr<mapping::Submap2D> global_static_submap_;

  // chunk map（chunk 完全归定位侧：LoadState 收集 pbstream chunk，AddTrajectoryBuilder
  // 创建/加载 chunk_map_ 并经 SetChunkMap 注入 LocalMapUpdater2D）。
  bool has_loaded_chunks_ = false;
  double loaded_chunk_length_ = 0.0;
  // LoadState 反序列化收集的 pbstream chunk submap：(gx, gy, 完整 Submap2D)。
  std::vector<std::tuple<int, int, std::shared_ptr<mapping::Submap2D>>>
      loaded_chunk_submaps_;
  // LoadState/AddTrajectoryBuilder 创建的 chunk map，传给 LocalMapUpdater2D 消费。
  std::shared_ptr<ChunkMap2D> chunk_map_;

  // 当前地图目录（SetMapDir 注入 / LoadStateFromFile 推导）：AddTrajectoryBuilder 时
  // 经 LocalMapUpdater2D::SetDebugDir 透传，debug data 落到该目录下。
  std::string loaded_map_dir_;

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
