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

#include "cartographer/io/internal/pbstream_info.h"

#include <map>
#include <sstream>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_bool(all_debug_strings, false,
            "Print debug strings of all serialized data.");

using cartographer::mapping::proto::SerializedData;

namespace cartographer {
namespace io {
namespace {

void Run(const std::string& pbstream_filename, bool all_debug_strings) {
  LOG(INFO) << "Reading pbstream file from '" << pbstream_filename << "'...";
  io::ProtoStreamReader reader(pbstream_filename);
  io::ProtoStreamDeserializer deserializer(&reader);
  const auto header = deserializer.header();
  LOG(INFO) << "Header: " << header.DebugString();
  for (const mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
           trajectory_options : deserializer.all_trajectory_builder_options()
                                    .options_with_sensor_ids()) {
    LOG(INFO) << "Trajectory options: " << trajectory_options.DebugString();
  }
  const mapping::proto::PoseGraph pose_graph = deserializer.pose_graph();

  // ---- Trajectory summary ----
  for (const mapping::proto::Trajectory& trajectory : pose_graph.trajectory()) {
    LOG(INFO) << "Trajectory id: " << trajectory.trajectory_id()
              << " has #nodes " << trajectory.node_size() << " has #submaps "
              << trajectory.submap_size();
  }

  // ---- Constraint summary ----
  {
    int intra_constraints = 0;
    int inter_constraints = 0;
    // Count per (submap_trajectory, node_trajectory) pair for inter-constraints
    std::map<std::pair<int,int>, int> inter_constraint_pairs;
    for (const auto& constraint : pose_graph.constraint()) {
      if (constraint.tag() ==
          mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        ++intra_constraints;
      } else {
        ++inter_constraints;
        auto key = std::make_pair(constraint.submap_id().trajectory_id(),
                                  constraint.node_id().trajectory_id());
        ++inter_constraint_pairs[key];
      }
    }
    LOG(INFO) << "Constraints: #intra_submap=" << intra_constraints
              << " #inter_submap=" << inter_constraints
              << " (total=" << (intra_constraints + inter_constraints) << ")";
    for (const auto& kv : inter_constraint_pairs) {
      LOG(INFO) << "  Inter-submap: submap_traj=" << kv.first.first
                << " <-> node_traj=" << kv.first.second
                << " count=" << kv.second;
    }
  }

  if (all_debug_strings) {
    LOG(INFO) << "Pose graph: " << pose_graph.DebugString();
  }

  const std::map<SerializedData::DataCase, std::string> data_case_to_name = {
      {SerializedData::kSubmap, "submap"},
      {SerializedData::kNode, "node"},
      {SerializedData::kTrajectoryData, "trajectory_data"},
      {SerializedData::kImuData, "imu_data"},
      {SerializedData::kOdometryData, "odometry_data"},
      {SerializedData::kFixedFramePoseData, "fixed_frame_pose_data"},
      {SerializedData::kLandmarkData, "landmark_data"},
      {SerializedData::kGlobalStaticSubmap, "global_static_submap"},
  };
  // Initialize so zero counts of these are also reported.
  std::map<std::string, int> data_counts = {
      {"submap_2d", 0},
      {"submap_2d_grid", 0},
      {"submap_3d", 0},
      {"submap_3d_high_resolution_hybrid_grid", 0},
      {"global_static_submap", 0},
  };

  // Extra info for global_static_submap
  bool has_global_static_submap = false;
  int global_submap_num_range_data = 0;
  int global_submap_grid_cells_x = 0;
  int global_submap_grid_cells_y = 0;
  bool global_submap_has_cloud = false;
  int global_submap_cloud_size = 0;
  int global_submap_line_segments = 0;

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    if (all_debug_strings) {
      LOG(INFO) << "Serialized data: " << proto.DebugString();
    }
    auto it = data_case_to_name.find(proto.data_case());
    if (it == data_case_to_name.end()) {
      LOG(WARNING) << "Skipping unknown message type in stream: "
                   << proto.GetTypeName();
      continue;
    }
    const std::string& data_name = it->second;
    ++data_counts[data_name];
    if (proto.data_case() == SerializedData::kSubmap) {
      if (proto.mutable_submap()->has_submap_2d()) {
        ++data_counts["submap_2d"];
        if (proto.mutable_submap()->mutable_submap_2d()->has_grid()) {
          ++data_counts["submap_2d_grid"];
        }
      }
      if (proto.mutable_submap()->has_submap_3d()) {
        ++data_counts["submap_3d"];
        if (proto.mutable_submap()
                ->mutable_submap_3d()
                ->has_high_resolution_hybrid_grid()) {
          ++data_counts["submap_3d_high_resolution_hybrid_grid"];
        }
      }
    }
    if (proto.data_case() == SerializedData::kGlobalStaticSubmap) {
      has_global_static_submap = true;
      const auto& s2d = proto.global_static_submap();
      global_submap_num_range_data = s2d.num_range_data();
      if (s2d.has_grid()) {
        const auto& grid = s2d.grid();
        global_submap_grid_cells_x = grid.limits().cell_limits().num_x_cells();
        global_submap_grid_cells_y = grid.limits().cell_limits().num_y_cells();
      }
      if (s2d.has_cloud()) {
        global_submap_has_cloud = true;
        global_submap_cloud_size = s2d.cloud().points_size();
      }
      global_submap_line_segments = s2d.has_line_map_2d_list()
                                        ? s2d.line_map_2d_list().line_map_data_size()
                                        : 0;
    }
  }

  // ---- Serialized data counts ----
  for (const auto& entry : data_counts) {
    LOG(INFO) << "SerializedData package contains #" << entry.first << ": "
              << entry.second;
  }

  // ---- Global static submap detail ----
  if (has_global_static_submap) {
    LOG(INFO) << "GlobalStaticSubmap detail:";
    LOG(INFO) << "  num_range_data   = " << global_submap_num_range_data;
    LOG(INFO) << "  grid size (cells)= " << global_submap_grid_cells_x
              << " x " << global_submap_grid_cells_y;
    LOG(INFO) << "  has_cloud        = " << (global_submap_has_cloud ? "yes" : "no")
              << " (" << global_submap_cloud_size << " points)";
    LOG(INFO) << "  line_segments    = " << global_submap_line_segments;
  } else {
    LOG(WARNING) << "GlobalStaticSubmap: NOT FOUND in pbstream";
  }
}
}  // namespace

int pbstream_info(int argc, char* argv[]) {
  std::stringstream ss;
  ss << "\n\n"
     << "Reads a pbstream file and summarizes its contents.\n\n"
     << "Usage: " << argv[0] << " " << argv[1]
     << " <pbstream_filename> [flags]\n";
  google::SetUsageMessage(ss.str());
  if (argc < 3) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_info");
    return EXIT_FAILURE;
  }
  Run(argv[2], FLAGS_all_debug_strings);
  return EXIT_SUCCESS;
}

}  // namespace io
}  // namespace cartographer
