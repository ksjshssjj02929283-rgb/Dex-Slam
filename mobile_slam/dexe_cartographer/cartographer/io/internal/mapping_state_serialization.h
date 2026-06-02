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

#ifndef CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
#define CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_

#include "cartographer/io/proto_stream_interface.h"
#include "cartographer/mapping/pose_graph.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer {
namespace localization {
class ChunkMap2D;  // 前向声明：WritePbStream 末尾序列化（chunk 最后写），避免强依赖。
}  // namespace localization
namespace io {

// The current serialization format version.
static constexpr int kMappingStateSerializationFormatVersion = 2;
static constexpr int kFormatVersionWithoutSubmapHistograms = 1;

// Serialize mapping state to a pbstream.
// chunk_map 非空时，在所有 map 元素（含 global_static_submap）之后追加 chunk map
// （ChunkMapInfo + 各 ChunkSubmap2D），即 chunk 最后写。chunk map 由调用方
// （LocalizationUpdater::SerializeState）先 BuildFromSubmap 构建好再传入。
void WritePbStream(
    const mapping::PoseGraph& pose_graph,
    const std::vector<mapping::proto::TrajectoryBuilderOptionsWithSensorIds>&
        builder_options,
    ProtoStreamWriterInterface* const writer, bool include_unfinished_submaps,
    const mapping::Submap2D* global_static_submap = nullptr,
    const localization::ChunkMap2D* chunk_map = nullptr);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_MAPPING_STATE_SERIALIZATION_H_
