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

#include "cartographer/io/internal/mapping_state_serialization.h"

#include "cartographer/localization/2d/chunk_map_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace io {
namespace {
using mapping::MapById;
using mapping::NodeId;
using mapping::PoseGraphInterface;
using mapping::SubmapId;
using mapping::TrajectoryNode;
using mapping::proto::SerializedData;

mapping::proto::AllTrajectoryBuilderOptions
CreateAllTrajectoryBuilderOptionsProto(
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        all_options_with_sensor_ids,
    const std::vector<int>& trajectory_ids_to_serialize) {
  mapping::proto::AllTrajectoryBuilderOptions all_options_proto;
  for (auto id : trajectory_ids_to_serialize) {
    *all_options_proto.add_options_with_sensor_ids() =
        all_options_with_sensor_ids[id];
  }
  return all_options_proto;
}

// Will return all trajectory ids, that have `state != DELETED`.
std::vector<int> GetValidTrajectoryIds(
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectory_states) {
  std::vector<int> valid_trajectories;
  for (const auto& t : trajectory_states) {
    if (t.second != PoseGraphInterface::TrajectoryState::DELETED) {
      valid_trajectories.push_back(t.first);
    }
  }
  return valid_trajectories;
}

mapping::proto::SerializationHeader CreateHeader() {
  mapping::proto::SerializationHeader header;
  header.set_format_version(kMappingStateSerializationFormatVersion);
  return header;
}

SerializedData SerializePoseGraph(const mapping::PoseGraph& pose_graph,
                                  bool include_unfinished_submaps) {
  SerializedData proto;
  *proto.mutable_pose_graph() = pose_graph.ToProto(include_unfinished_submaps);
  return proto;
}

SerializedData SerializeTrajectoryBuilderOptions(
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    const std::vector<int>& trajectory_ids_to_serialize) {
  SerializedData proto;
  *proto.mutable_all_trajectory_builder_options() =
      CreateAllTrajectoryBuilderOptionsProto(trajectory_builder_options,
                                             trajectory_ids_to_serialize);
  return proto;
}

void SerializeSubmaps(
    const MapById<SubmapId, PoseGraphInterface::SubmapData>& submap_data,
    bool include_unfinished_submaps, ProtoStreamWriterInterface* const writer) {
  // Next serialize all submaps.
  for (const auto& submap_id_data : submap_data) {
    if (!include_unfinished_submaps &&
        !submap_id_data.data.submap->insertion_finished()) {
      continue;
    }
    SerializedData proto;
    auto* const submap_proto = proto.mutable_submap();
    *submap_proto = submap_id_data.data.submap->ToProto(
        /*include_probability_grid_data=*/true);
    submap_proto->mutable_submap_id()->set_trajectory_id(
        submap_id_data.id.trajectory_id);
    submap_proto->mutable_submap_id()->set_submap_index(
        submap_id_data.id.submap_index);
    writer->WriteProto(proto);
  }
}

void SerializeTrajectoryNodes(
    const MapById<NodeId, TrajectoryNode>& trajectory_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node_id_data : trajectory_nodes) {
    SerializedData proto;
    auto* const node_proto = proto.mutable_node();
    node_proto->mutable_node_id()->set_trajectory_id(
        node_id_data.id.trajectory_id);
    node_proto->mutable_node_id()->set_node_index(node_id_data.id.node_index);
    *node_proto->mutable_node_data() =
        ToProto(*node_id_data.data.constant_data);
    writer->WriteProto(proto);
  }
}

void SerializeTrajectoryData(
    const std::map<int, PoseGraphInterface::TrajectoryData>&
        all_trajectory_data,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& trajectory_data : all_trajectory_data) {
    SerializedData proto;
    auto* const trajectory_data_proto = proto.mutable_trajectory_data();
    trajectory_data_proto->set_trajectory_id(trajectory_data.first);
    trajectory_data_proto->set_gravity_constant(
        trajectory_data.second.gravity_constant);
    *trajectory_data_proto->mutable_imu_calibration() = transform::ToProto(
        Eigen::Quaterniond(trajectory_data.second.imu_calibration[0],
                           trajectory_data.second.imu_calibration[1],
                           trajectory_data.second.imu_calibration[2],
                           trajectory_data.second.imu_calibration[3]));
    if (trajectory_data.second.fixed_frame_origin_in_map.has_value()) {
      *trajectory_data_proto->mutable_fixed_frame_origin_in_map() =
          transform::ToProto(
              trajectory_data.second.fixed_frame_origin_in_map.value());
    }
    writer->WriteProto(proto);
  }
}

void SerializeImuData(const sensor::MapByTime<sensor::ImuData>& all_imu_data,
                      ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_imu_data.trajectory_ids()) {
    for (const auto& imu_data : all_imu_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const imu_data_proto = proto.mutable_imu_data();
      imu_data_proto->set_trajectory_id(trajectory_id);
      *imu_data_proto->mutable_imu_data() = sensor::ToProto(imu_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeOdometryData(
    const sensor::MapByTime<sensor::OdometryData>& all_odometry_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_odometry_data.trajectory_ids()) {
    for (const auto& odometry_data :
         all_odometry_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const odometry_data_proto = proto.mutable_odometry_data();
      odometry_data_proto->set_trajectory_id(trajectory_id);
      *odometry_data_proto->mutable_odometry_data() =
          sensor::ToProto(odometry_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeFixedFramePoseData(
    const sensor::MapByTime<sensor::FixedFramePoseData>&
        all_fixed_frame_pose_data,
    ProtoStreamWriterInterface* const writer) {
  for (const int trajectory_id : all_fixed_frame_pose_data.trajectory_ids()) {
    for (const auto& fixed_frame_pose_data :
         all_fixed_frame_pose_data.trajectory(trajectory_id)) {
      SerializedData proto;
      auto* const fixed_frame_pose_data_proto =
          proto.mutable_fixed_frame_pose_data();
      fixed_frame_pose_data_proto->set_trajectory_id(trajectory_id);
      *fixed_frame_pose_data_proto->mutable_fixed_frame_pose_data() =
          sensor::ToProto(fixed_frame_pose_data);
      writer->WriteProto(proto);
    }
  }
}

void SerializeLandmarkNodes(
    const std::map<std::string, PoseGraphInterface::LandmarkNode>&
        all_landmark_nodes,
    ProtoStreamWriterInterface* const writer) {
  for (const auto& node : all_landmark_nodes) {
    for (const auto& observation : node.second.landmark_observations) {
      SerializedData proto;
      auto* landmark_data_proto = proto.mutable_landmark_data();
      landmark_data_proto->set_trajectory_id(observation.trajectory_id);
      landmark_data_proto->mutable_landmark_data()->set_timestamp(
          common::ToUniversal(observation.time));
      auto* observation_proto = landmark_data_proto->mutable_landmark_data()
                                    ->add_landmark_observations();
      observation_proto->set_id(node.first);
      *observation_proto->mutable_landmark_to_tracking_transform() =
          transform::ToProto(observation.landmark_to_tracking_transform);
      observation_proto->set_translation_weight(observation.translation_weight);
      observation_proto->set_rotation_weight(observation.rotation_weight);
      writer->WriteProto(proto);
    }
  }
}

void SerializeGlobalStaticSubmap(
    const mapping::Submap2D* global_static_submap,
    ProtoStreamWriterInterface* const writer) {
  if (global_static_submap == nullptr) {
    return;
  }

  LOG(INFO) << "Serializing global static submap to pbstream";
  SerializedData proto;
  auto submap_proto = global_static_submap->ToProto(
      /*include_probability_grid_data=*/true);
  proto.mutable_global_static_submap()->CopyFrom(submap_proto.submap_2d());
  writer->WriteProto(proto);
  LOG(INFO) << "Global static submap serialized successfully";
}

// chunk map 最后写：把 ChunkMap2D 产出的 proto 包装成 SerializedData 写入——
// 先写 ChunkMapInfo，再写各 ChunkSubmap2D。chunk map 由调用方
// （LocalizationUpdater::SerializeState）先 BuildFromSubmap 构建好再传入。
// 风格对齐 SerializeGlobalStaticSubmap：ChunkMap2D 只产 proto（ToProto 形式），
// 本函数负责包装 + WriteProto；空指针/空 chunk 直接返回。
void SerializeChunkMap(const localization::ChunkMap2D* chunk_map,
                       ProtoStreamWriterInterface* const writer) {
  if (chunk_map == nullptr || chunk_map->empty()) {
    return;
  }

  LOG(INFO) << "Serializing chunk map to pbstream";
  // 先写 ChunkMapInfo（chunk_length + num_chunks）。
  SerializedData info_data;
  info_data.mutable_chunk_map_info()->CopyFrom(chunk_map->ToInfoProto());
  writer->WriteProto(info_data);
  // 再写各 ChunkSubmap2D（gx/gy + 自包含 Submap2D）。
  const auto chunk_protos = chunk_map->ToChunkProtos();
  for (const auto& chunk_proto : chunk_protos) {
    SerializedData data;
    data.mutable_chunk_submap()->CopyFrom(chunk_proto);
    writer->WriteProto(data);
  }
  LOG(INFO) << "Chunk map serialized: " << chunk_protos.size() << " chunks";
}

}  // namespace

void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        trajectory_builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps,
    const mapping::Submap2D* global_static_submap,
    const localization::ChunkMap2D* chunk_map) {
  writer->WriteProto(CreateHeader());
  writer->WriteProto(
      SerializePoseGraph(pose_graph, include_unfinished_submaps));
  writer->WriteProto(SerializeTrajectoryBuilderOptions(
      trajectory_builder_options,
      GetValidTrajectoryIds(pose_graph.GetTrajectoryStates())));

  SerializeSubmaps(pose_graph.GetAllSubmapData(), include_unfinished_submaps,
                   writer);
  SerializeTrajectoryNodes(pose_graph.GetTrajectoryNodes(), writer);
  SerializeTrajectoryData(pose_graph.GetTrajectoryData(), writer);
  // IMU/Odometry raw sensor data is not needed for localization or re-mapping,
  // skip serialization to reduce pbstream file size.
  // SerializeImuData(pose_graph.GetImuData(), writer);
  // SerializeOdometryData(pose_graph.GetOdometryData(), writer);
  SerializeFixedFramePoseData(pose_graph.GetFixedFramePoseData(), writer);
  SerializeLandmarkNodes(pose_graph.GetLandmarkNodes(), writer);
  
  // Serialize global static submap if available
  SerializeGlobalStaticSubmap(global_static_submap, writer);

  // chunk map 最后写：在所有 map 元素（含 global_static_submap）之后追加。
  SerializeChunkMap(chunk_map, writer);
}

}  // namespace io
}  // namespace cartographer
