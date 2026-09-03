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

#include "cartographer/io/internal/pbstream_check.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <google/protobuf/repeated_field.h>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/structure_alignment.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(json, "", "Write machine readable check result to this path.");
DEFINE_int32(min_cloud_points, 1,
             "Minimum point count in the global static submap cloud.");
DEFINE_double(align_tolerance_deg, 1.0,
              "ALIGN-01: residual rotation not exceeding this value (deg) "
              "counts as aligned to the structural environment.");

using cartographer::mapping::proto::SerializedData;

namespace cartographer {
namespace io {

namespace pbstream_check_internal {

bool SurvivesCulling(double length, int inliers, const CullingCriteria& c) {
  return length >= c.min_length && length <= c.max_length &&
         inliers >= kMinLineInliers;
}

void CollectLineStats(const mapping::proto::LineMap2DList& proto,
                      const CullingCriteria& pbstream_criteria,
                      LineStats* stats) {
  stats->count = proto.line_map_data_size();
  stats->lengths.reserve(stats->count);
  stats->inliers.reserve(stats->count);
  const CullingCriteria defaults{kDefaultMinLineLength, kDefaultMaxLineLength,
                                 kDefaultLineCullingObs};
  for (const auto& map_line : proto.line_map_data()) {
    const auto& line = map_line.line_data();
    const double dx = line.end_x() - line.begin_x();
    const double dy = line.end_y() - line.begin_y();
    const double length = std::hypot(dx, dy);
    stats->lengths.push_back(length);
    stats->inliers.push_back(map_line.inliers());
    if (SurvivesCulling(length, map_line.inliers(), pbstream_criteria)) {
      ++stats->survive_pbstream;
    }
    if (SurvivesCulling(length, map_line.inliers(), defaults)) {
      ++stats->survive_defaults;
    }
  }
}

CullingCriteria ResolveCriteria(
    const ::google::protobuf::RepeatedPtrField<
        mapping::proto::TrajectoryBuilderOptionsWithSensorIds>& all_options,
    bool* from_pbstream) {
  *from_pbstream = false;
  for (const auto& options_with_sensor_ids : all_options) {
    const auto& builder_options =
        options_with_sensor_ids.trajectory_builder_options();
    if (!builder_options.has_trajectory_builder_2d_options()) continue;
    const auto& submaps_options =
        builder_options.trajectory_builder_2d_options().submaps_options();
    // max_line_length 未设置（proto3 默认 0）说明这份 pbstream 的建图配置里
    // 根本没有线特征参数，此时回退默认比用 0 更接近实际加载行为。
    if (submaps_options.max_line_length() <= 0.0) continue;
    *from_pbstream = true;
    return CullingCriteria{submaps_options.min_line_length(),
                           submaps_options.max_line_length(),
                           submaps_options.line_culling_obs()};
  }
  return CullingCriteria{kDefaultMinLineLength, kDefaultMaxLineLength,
                         kDefaultLineCullingObs};
}

AlignmentStats EvaluateStructureAlignment(
    const mapping::proto::LineMap2DList& proto) {
  // proto → 运行时 LineMap2DList 的轻量重建（submap_2d.cc FromProto 的子集）：
  // 只重建主线段数据、不重放 line_data_list 观测历史 —— 检查的正是保存时的
  // 角度，重放反而会触发重拟合。
  std::map<int, mapping::MapLine2D> line_map_data;
  for (const auto& map_line_proto : proto.line_map_data()) {
    const auto& line_data_proto = map_line_proto.line_data();
    const sensor::LineData line_data(
        Eigen::Vector2d(line_data_proto.begin_x(), line_data_proto.begin_y()),
        Eigen::Vector2d(line_data_proto.end_x(), line_data_proto.end_y()),
        line_data_proto.weight());
    mapping::MapLine2D map_line(mapping::MapLine2D::static_line,
                                map_line_proto.global_id(), line_data);
    map_line.SetLocalId(map_line_proto.local_id());
    map_line.SetInliers(map_line_proto.inliers());
    line_map_data.insert({map_line_proto.local_id(), std::move(map_line)});
  }
  const mapping::LineMap2DList line_map(proto.latest_local_id(),
                                        std::move(line_map_data));

  // 默认阈值即可：保存来的线地图已做过 culling，此处仅做结构方向估计。
  const mapping::StructureAlignmentOptions options;
  const auto yaw = mapping::EstimateAlignmentYaw(line_map, options);
  if (!yaw.has_value()) return AlignmentStats{};
  return AlignmentStats{true, std::abs(*yaw) * 180.0 / M_PI};
}

}  // namespace pbstream_check_internal

namespace {

using pbstream_check_internal::AlignmentStats;
using pbstream_check_internal::CollectLineStats;
using pbstream_check_internal::CullingCriteria;
using pbstream_check_internal::EvaluateStructureAlignment;
using pbstream_check_internal::kDefaultMaxLineLength;
using pbstream_check_internal::kDefaultMinLineLength;
using pbstream_check_internal::kMinLineInliers;
using pbstream_check_internal::LineStats;
using pbstream_check_internal::ResolveCriteria;

enum class Level { kPass, kWarn, kFail };

const char* LevelName(Level level) {
  switch (level) {
    case Level::kPass: return "PASS";
    case Level::kWarn: return "WARN";
    case Level::kFail: return "FAIL";
  }
  return "?";
}

struct CheckItem {
  Level level;
  std::string id;
  std::string message;
  std::string hint;  // 可选，附在下一行缩进显示
};

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t mid = values.size() / 2;
  if (values.size() % 2 == 1) return values[mid];
  return 0.5 * (values[mid - 1] + values[mid]);
}

std::string JsonEscape(const std::string& in) {
  std::string out;
  for (const char c : in) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else {
      out += c;
    }
  }
  return out;
}

struct GlobalSubmapStats {
  bool present = false;
  bool has_grid = false;
  bool has_line_field = false;  // line_map_2d_list 字段是否存在（区别于存在但为空）
  int num_range_data = 0;
  int grid_cells_x = 0;
  int grid_cells_y = 0;
  double resolution = 0.0;
  int64_t grid_cells_total = 0;
  int64_t grid_cells_known = 0;
  bool has_cloud = false;
  int cloud_points = 0;
  bool has_pyramid = false;
  int pyramid_levels = 0;
  LineStats lines;
  AlignmentStats alignment;
};

void WriteJson(const std::string& path, const std::string& pbstream_filename,
               uint32_t format_version, const std::map<std::string, int>& counts,
               int trajectory_nodes, int trajectory_submaps,
               const GlobalSubmapStats& gs, const CullingCriteria& criteria,
               bool criteria_from_pbstream,
               const std::vector<CheckItem>& checks, const char* verdict,
               int exit_code) {
  std::ofstream out(path);
  if (!out) {
    LOG(ERROR) << "Failed to open json output path: " << path;
    return;
  }
  out << std::boolalpha;
  out << "{\n";
  out << "  \"file\": \"" << JsonEscape(pbstream_filename) << "\",\n";
  out << "  \"format_version\": " << format_version << ",\n";
  out << "  \"trajectory\": {\"nodes\": " << trajectory_nodes
      << ", \"submaps\": " << trajectory_submaps << "},\n";
  out << "  \"serialized_data_counts\": {";
  bool first = true;
  for (const auto& entry : counts) {
    if (!first) out << ", ";
    first = false;
    out << "\"" << entry.first << "\": " << entry.second;
  }
  out << "},\n";
  out << "  \"criteria\": {\"source\": \""
      << (criteria_from_pbstream ? "pbstream" : "builtin_default")
      << "\", \"min_line_length\": " << criteria.min_length
      << ", \"max_line_length\": " << criteria.max_length
      << ", \"line_culling_obs\": " << criteria.culling_obs
      << ", \"min_line_inliers\": " << kMinLineInliers << "},\n";
  out << "  \"global_static_submap\": {\n";
  out << "    \"present\": " << gs.present << ",\n";
  out << "    \"num_range_data\": " << gs.num_range_data << ",\n";
  out << "    \"grid\": {\"present\": " << gs.has_grid << ", \"cells_x\": "
      << gs.grid_cells_x << ", \"cells_y\": " << gs.grid_cells_y
      << ", \"resolution\": " << gs.resolution
      << ", \"cells_total\": " << gs.grid_cells_total
      << ", \"cells_known\": " << gs.grid_cells_known << "},\n";
  out << "    \"cloud\": {\"present\": " << gs.has_cloud << ", \"points\": "
      << gs.cloud_points << "},\n";
  out << "    \"pyramid\": {\"present\": " << gs.has_pyramid
      << ", \"levels\": " << gs.pyramid_levels << "},\n";
  out << "    \"lines\": {\"field_present\": " << gs.has_line_field
      << ", \"count\": " << gs.lines.count
      << ", \"survive_pbstream_criteria\": " << gs.lines.survive_pbstream
      << ", \"survive_dynamic_defaults\": " << gs.lines.survive_defaults << "},\n";
  out << std::boolalpha
      << "    \"alignment\": {\"evaluated\": " << gs.alignment.evaluated
      << ", \"residual_yaw_deg\": " << gs.alignment.residual_yaw_deg
      << ", \"tolerance_deg\": " << FLAGS_align_tolerance_deg << "}\n";
  out << "  },\n";
  out << "  \"checks\": [\n";
  for (size_t i = 0; i < checks.size(); ++i) {
    out << "    {\"id\": \"" << checks[i].id << "\", \"level\": \""
        << LevelName(checks[i].level) << "\", \"message\": \""
        << JsonEscape(checks[i].message) << "\"}";
    out << (i + 1 == checks.size() ? "\n" : ",\n");
  }
  out << "  ],\n";
  out << "  \"verdict\": \"" << verdict << "\",\n";
  out << "  \"exit_code\": " << exit_code << "\n";
  out << "}\n";
}

int Run(const std::string& pbstream_filename, const std::string& json_path,
        int min_cloud_points) {
  // 文件预检。更深层的结构损坏由 ProtoStreamDeserializer 的 CHECK 直接终止
  // 进程（上游行为），本工具不做伪装处理。
  {
    std::ifstream probe(pbstream_filename, std::ios::binary | std::ios::ate);
    if (!probe) {
      std::cerr << "ERROR: cannot open '" << pbstream_filename << "'\n";
      return 2;
    }
    if (probe.tellg() <= 0) {
      std::cerr << "ERROR: '" << pbstream_filename << "' is empty\n";
      return 2;
    }
  }

  io::ProtoStreamReader reader(pbstream_filename);
  io::ProtoStreamDeserializer deserializer(&reader);
  const uint32_t format_version = deserializer.header().format_version();

  bool criteria_from_pbstream = false;
  const CullingCriteria criteria = ResolveCriteria(
      deserializer.all_trajectory_builder_options().options_with_sensor_ids(),
      &criteria_from_pbstream);

  const mapping::proto::PoseGraph pose_graph = deserializer.pose_graph();
  int trajectory_nodes = 0;
  int trajectory_submaps = 0;
  for (const mapping::proto::Trajectory& trajectory : pose_graph.trajectory()) {
    trajectory_nodes += trajectory.node_size();
    trajectory_submaps += trajectory.submap_size();
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
  std::map<std::string, int> counts = {
      {"submap_2d", 0}, {"submap_3d", 0}, {"global_static_submap", 0},
      {"node", 0},
  };

  GlobalSubmapStats gs;
  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    auto it = data_case_to_name.find(proto.data_case());
    if (it == data_case_to_name.end()) continue;
    ++counts[it->second];
    if (proto.data_case() == SerializedData::kSubmap) {
      if (proto.submap().has_submap_2d()) ++counts["submap_2d"];
      if (proto.submap().has_submap_3d()) ++counts["submap_3d"];
    }
    if (proto.data_case() == SerializedData::kGlobalStaticSubmap) {
      const auto& s2d = proto.global_static_submap();
      gs.present = true;
      gs.num_range_data = s2d.num_range_data();
      if (s2d.has_grid()) {
        gs.has_grid = true;
        const auto& grid = s2d.grid();
        gs.grid_cells_x = grid.limits().cell_limits().num_x_cells();
        gs.grid_cells_y = grid.limits().cell_limits().num_y_cells();
        gs.resolution = grid.limits().resolution();
        gs.grid_cells_total = grid.cells_size();
        for (const int cell : grid.cells()) {
          if (cell != 0) ++gs.grid_cells_known;  // 0 = unknown
        }
      }
      if (s2d.has_cloud()) {
        gs.has_cloud = true;
        gs.cloud_points = s2d.cloud().points_size();
      }
      if (s2d.has_precomputation_grid_stack()) {
        gs.has_pyramid = true;
        gs.pyramid_levels = s2d.precomputation_grid_stack().grids_size();
      }
      if (s2d.has_line_map_2d_list()) {
        gs.has_line_field = true;
        CollectLineStats(s2d.line_map_2d_list(), criteria, &gs.lines);
        gs.alignment = EvaluateStructureAlignment(s2d.line_map_2d_list());
      }
    }
  }

  // ---------------- 判定 ----------------
  std::vector<CheckItem> checks;
  auto add = [&checks](Level level, const std::string& id,
                       const std::string& message,
                       const std::string& hint = "") {
    checks.push_back(CheckItem{level, id, message, hint});
  };

  // 结构完整性
  if (trajectory_nodes > 0) {
    add(Level::kPass, "STRUCT-01",
        "trajectory nodes = " + std::to_string(trajectory_nodes));
  } else {
    add(Level::kFail, "STRUCT-01", "no trajectory node in pbstream");
  }

  if (gs.present) {
    add(Level::kPass, "STRUCT-02", "global_static_submap present");
  } else if (trajectory_nodes > 0) {
    add(Level::kWarn, "STRUCT-02",
        "global_static_submap missing (legacy pbstream format)",
        "localization falls back to rebuilding it from nodes on load; "
        "re-save or migrate to avoid the startup cost");
  } else {
    add(Level::kFail, "STRUCT-02",
        "global_static_submap missing and no node to rebuild it from");
  }

  if (gs.present) {
    if (!gs.has_grid) {
      add(Level::kFail, "GRID-01", "global_static_submap has no grid");
    } else if (gs.grid_cells_known == 0) {
      add(Level::kFail, "GRID-01",
          "grid has no known cell (all unknown), map is empty");
    } else {
      std::ostringstream os;
      os << "grid " << gs.grid_cells_x << " x " << gs.grid_cells_y << " @ "
         << gs.resolution << " m, known cells " << gs.grid_cells_known << "/"
         << gs.grid_cells_total << " (" << std::fixed << std::setprecision(1)
         << (100.0 * gs.grid_cells_known / std::max<int64_t>(gs.grid_cells_total, 1))
         << "%)";
      add(Level::kPass, "GRID-01", os.str());
    }

    if (gs.cloud_points >= min_cloud_points) {
      add(Level::kPass, "CLOUD-01",
          "cloud " + std::to_string(gs.cloud_points) + " points");
    } else {
      add(Level::kWarn, "CLOUD-01",
          "cloud has " + std::to_string(gs.cloud_points) + " points (< " +
              std::to_string(min_cloud_points) + ")",
          "point-cloud based matching degrades to grid only");
    }

    // 线特征
    if (!gs.has_line_field) {
      add(Level::kWarn, "LINE-01",
          "line_map_2d_list field absent from global_static_submap",
          "Submap2D::ToProto() skips the whole field when zero line remains, "
          "so an absent field means every line was culled before the save");
    } else if (gs.lines.count == 0) {
      add(Level::kWarn, "LINE-01", "line_map_2d_list present but empty");
    } else {
      std::ostringstream os;
      os << "lines " << gs.lines.count << " segments, length min/med/max "
         << std::fixed << std::setprecision(2)
         << *std::min_element(gs.lines.lengths.begin(), gs.lines.lengths.end())
         << " / " << Median(gs.lines.lengths) << " / "
         << *std::max_element(gs.lines.lengths.begin(), gs.lines.lengths.end())
         << " m, inliers min "
         << *std::min_element(gs.lines.inliers.begin(), gs.lines.inliers.end());
      add(Level::kPass, "LINE-01", os.str());

      // 加载后剔除预测：pbstream 判据（static submap 走这套）
      const int culled_pbstream = gs.lines.count - gs.lines.survive_pbstream;
      if (culled_pbstream > 0) {
        add(Level::kWarn, "LINE-02",
            std::to_string(culled_pbstream) + "/" +
                std::to_string(gs.lines.count) +
                " lines fail the pbstream criteria and will be culled on load",
            "saved with criteria different from the ones embedded in this file");
      } else {
        add(Level::kPass, "LINE-02",
            "all lines pass the pbstream culling criteria");
      }

      // dynamic submap 走类内默认（LocalMapUpdater2D 从不调用 SetLineCullingParam）
      const int culled_defaults = gs.lines.count - gs.lines.survive_defaults;
      if (culled_defaults > culled_pbstream) {
        std::ostringstream os2;
        os2 << (culled_defaults - culled_pbstream)
            << " extra lines will be culled by the dynamic submap, which uses "
               "the built-in defaults (min "
            << kDefaultMinLineLength << " / max " << kDefaultMaxLineLength
            << " m)";
        add(Level::kWarn, "LINE-03", os2.str(),
            "LocalMapUpdater2D never calls SetLineCullingParam on the dynamic "
            "submap; under the PERSISTENT policy the loss is written back to "
            "disk on the next save");
      } else {
        add(Level::kPass, "LINE-03",
            "dynamic submap defaults cull no extra line");
      }
    }

    // 结构化对齐（从线特征角度判断；未对齐只 WARN：不影响定位可用性，
    // 只影响栅格/结构化适配）
    if (!gs.has_line_field || gs.lines.count == 0) {
      add(Level::kWarn, "ALIGN-01",
          "no line features; structural alignment not evaluable");
    } else if (!gs.alignment.evaluated) {
      add(Level::kWarn, "ALIGN-01",
          "lines present but no reliable orthogonal structure; structural "
          "alignment not evaluable");
    } else if (gs.alignment.residual_yaw_deg <= FLAGS_align_tolerance_deg) {
      std::ostringstream os;
      os << "map aligned to structure (residual rotation " << std::fixed
         << std::setprecision(2) << gs.alignment.residual_yaw_deg
         << " deg <= " << FLAGS_align_tolerance_deg << " deg)";
      add(Level::kPass, "ALIGN-01", os.str());
    } else {
      std::ostringstream os;
      os << "map NOT aligned to structure: residual rotation " << std::fixed
         << std::setprecision(2) << gs.alignment.residual_yaw_deg
         << " deg > " << FLAGS_align_tolerance_deg << " deg";
      add(Level::kWarn, "ALIGN-01", os.str(),
          "re-save with reset_map_origin_to_align_structure = true to align "
          "the map to the structural environment");
    }

    // 全局定位金字塔缓存
    if (gs.has_pyramid) {
      add(Level::kPass, "PYRAMID-01",
          "precomputation grid stack cached (" +
              std::to_string(gs.pyramid_levels) + " levels)");
    } else {
      add(Level::kWarn, "PYRAMID-01",
          "no precomputation grid stack in pbstream",
          "the first global relocalization has to rebuild it, which costs "
          "seconds instead of hundreds of milliseconds");
    }
  }

  // 兼容性
  if (counts["submap_3d"] > 0) {
    add(Level::kWarn, "COMPAT-01",
        std::to_string(counts["submap_3d"]) +
            " submap_3d found in a 2D localization map");
  }
  if (format_version < 2) {
    add(Level::kWarn, "COMPAT-02",
        "serialization format version " + std::to_string(format_version) +
            " is older than the current one",
        "run `cartographer_pbstream migrate` first");
  }

  // ---------------- 输出 ----------------
  std::cout << "\n[COUNT]  trajectory_nodes = " << trajectory_nodes
            << "   submaps = " << trajectory_submaps << "\n";
  std::cout << "[COUNT]  serialized:";
  for (const auto& entry : counts) {
    std::cout << "  " << entry.first << "=" << entry.second;
  }
  std::cout << "\n";
  if (gs.present) {
    std::cout << "[COUNT]  global_static_submap:\n";
    std::cout << "           grid        = " << gs.grid_cells_x << " x "
              << gs.grid_cells_y << " @ " << gs.resolution << " m\n";
    std::cout << "           cloud       = " << gs.cloud_points << " pts\n";
    std::cout << "           lines       = " << gs.lines.count << " segments\n";
    std::cout << "           pyramid     = " << gs.pyramid_levels
              << " levels\n";
    std::cout << "           range_data  = " << gs.num_range_data << "\n";
  }
  std::cout << "[CRIT ]  culling criteria from "
            << (criteria_from_pbstream ? "pbstream" : "built-in defaults")
            << ": min " << criteria.min_length << " / max "
            << criteria.max_length << " m, obs " << criteria.culling_obs
            << ", inliers >= " << kMinLineInliers << "\n\n";

  int warnings = 0;
  int failures = 0;
  for (const CheckItem& item : checks) {
    if (item.level == Level::kWarn) ++warnings;
    if (item.level == Level::kFail) ++failures;
    std::cout << "[CHECK]  " << LevelName(item.level) << "  " << item.id << "  "
              << item.message << "\n";
    if (!item.hint.empty()) {
      std::cout << "                 -> " << item.hint << "\n";
    }
  }

  const char* verdict = failures > 0 ? "FAIL" : "PASS";
  const int exit_code = failures > 0 ? 1 : 0;
  std::cout << "\nVERDICT: " << verdict << " (" << failures << " failure(s), "
            << warnings << " warning(s))\n\n";

  if (!json_path.empty()) {
    WriteJson(json_path, pbstream_filename, format_version, counts,
              trajectory_nodes, trajectory_submaps, gs, criteria,
              criteria_from_pbstream, checks, verdict, exit_code);
  }
  return exit_code;
}

}  // namespace

int pbstream_check(int argc, char* argv[]) {
  std::stringstream ss;
  ss << "\n\n"
     << "Checks whether a pbstream satisfies the localization requirements.\n\n"
     << "Usage: " << argv[0] << " " << argv[1]
     << " <pbstream_filename> [--json=<path>] [--min_cloud_points=N]\n"
     << "       [--align_tolerance_deg=D]\n\n"
     << "Exit code: 0 = ok (warnings allowed), 1 = requirements not met,\n"
     << "           2 = file missing / empty / unreadable\n";
  google::SetUsageMessage(ss.str());
  if (argc < 3) {
    google::ShowUsageWithFlagsRestrict(argv[0], "pbstream_check");
    return 2;
  }
  return Run(argv[2], FLAGS_json, FLAGS_min_cloud_points);
}

}  // namespace io
}  // namespace cartographer
