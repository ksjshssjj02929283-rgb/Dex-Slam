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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <functional>
#include <memory>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/sensor/collator_interface.h"

namespace cartographer {
namespace mapping {

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a PoseGraph for loop closure.
class MapBuilder : public MapBuilderInterface {
 public:
  explicit MapBuilder(const proto::MapBuilderOptions &options);
  ~MapBuilder() override {}

  MapBuilder(const MapBuilder &) = delete;
  MapBuilder &operator=(const MapBuilder &) = delete;

  int AddTrajectoryBuilder(
      const std::set<SensorId> &expected_sensor_ids,
      const proto::TrajectoryBuilderOptions &trajectory_options,
      LocalSlamResultCallback local_slam_result_callback) override;

  int AddTrajectoryForDeserialization(
      const proto::TrajectoryBuilderOptionsWithSensorIds
          &options_with_sensor_ids_proto) override;

  void FinishTrajectory(int trajectory_id) override;

  std::string SubmapToProto(const SubmapId &submap_id,
                            proto::SubmapQuery::Response *response) override;

  void SerializeState(bool include_unfinished_submaps,
                      io::ProtoStreamWriterInterface *writer) override;

  bool SerializeStateToFile(bool include_unfinished_submaps,
                            const std::string &filename) override;

  std::map<int, int> LoadState(io::ProtoStreamReaderInterface *reader,
                               bool load_frozen_state) override;

  std::map<int, int> LoadStateFromFile(const std::string &filename,
                                       const bool load_frozen_state) override;

  // // 全局一致的二维码地图
  // const std::shared_ptr<std::vector<sensor::CodeData>> GetCodeMap() const override
  // {
  //   return code_map_;
  // }
  // // 全局一致的Tag地图
  // const std::shared_ptr<std::vector<sensor::CodeData>> GetTagMap() const override
  // {
  //   return tag_map_;
  // }
  // 全局一致的反光板地图
  const std::shared_ptr<std::vector<Eigen::Vector2d>>
  GetReflectorMap() const override
  {
    return reflector_map_;
  }
  // 全局一致的线特征地图
  const std::shared_ptr<sensor::LineMapData>
  GetLineMap()const override
  {
    return line_map_;
  }

  mapping::PoseGraphInterface *pose_graph() override {
    return pose_graph_.get();
  }

  int num_trajectory_builders() const override {
    return trajectory_builders_.size();
  }

  mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
      int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get();
  }

  const std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      &GetAllTrajectoryBuilderOptions() const override {
    return all_trajectory_builder_options_;
  }

  // Generate global static submap from all trajectory nodes
  std::shared_ptr<Submap2D> GenerateGlobalStaticSubmap(double resolution);

  // 返回已缓存的全局静态 Submap（SerializeStateToFile 后可用），否则返回 nullptr
  void SetProgressCallback(std::function<void(size_t, size_t)> callback) override {
    progress_callback_ = std::move(callback);
  }

  std::shared_ptr<Submap2D> GetGlobalStaticSubmap() const override {
    return global_static_submap_;
  }

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<PoseGraph> pose_graph_;

  // 全局统一的二维码地图`
  // std::shared_ptr<std::vector<sensor::CodeData>> code_map_;
  // // 全局统一的TAG地图
  // std::shared_ptr<std::vector<sensor::CodeData>> tag_map_;
  // 全局统一的反光板地图
  std::shared_ptr<std::vector<Eigen::Vector2d>> reflector_map_;
  // 全局统一的反光板地图
  std::shared_ptr<sensor::LineMapData> line_map_;

  std::unique_ptr<sensor::CollatorInterface> sensor_collator_;
  // 保存全局轨迹的构建器
  std::vector<std::unique_ptr<mapping::TrajectoryBuilderInterface>>
      trajectory_builders_;
  std::vector<proto::TrajectoryBuilderOptionsWithSensorIds>
      all_trajectory_builder_options_;
  
  // Global static submap generated from all trajectory nodes
  std::shared_ptr<Submap2D> global_static_submap_;
  // 逐 node 处理进度回调（可为空）
  std::function<void(size_t, size_t)> progress_callback_;
  
  // Conversion tables for probability grid - must be persistent for the lifetime
  // of global_static_submap_
  ValueConversionTables conversion_tables_;
};

std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
