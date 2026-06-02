/*
 * pbstream 快速检查工具
 * 用法：ros2 run cartographer_ros pbstream_inspector <map.pbstream>
 *
 * 输出：pbstream 内所有 SerializedData 元素类型与数量、是否含 chunk map、是否符合定位要求。
 * 返回码：0=符合定位要求；1=参数错误；2=文件无法读取；3=不符合定位要求。
 */

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "cartographer/io/proto_stream.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "glog/logging.h"

namespace {

using SerializedData = ::cartographer::mapping::proto::SerializedData;

const char* CaseName(SerializedData::DataCase c) {
  switch (c) {
    case SerializedData::kPoseGraph: return "pose_graph";
    case SerializedData::kAllTrajectoryBuilderOptions:
      return "all_trajectory_builder_options";
    case SerializedData::kSubmap: return "submap";
    case SerializedData::kNode: return "node";
    case SerializedData::kTrajectoryData: return "trajectory_data";
    case SerializedData::kImuData: return "imu_data";
    case SerializedData::kOdometryData: return "odometry_data";
    case SerializedData::kFixedFramePoseData: return "fixed_frame_pose_data";
    case SerializedData::kLandmarkData: return "landmark_data";
    case SerializedData::kGlobalStaticSubmap: return "global_static_submap";
    case SerializedData::kChunkMapInfo: return "chunk_map_info";
    case SerializedData::kChunkSubmap: return "chunk_submap";
    default: return "(unknown/not_set)";
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  if (argc < 2) {
    std::cerr << "用法: ros2 run cartographer_ros pbstream_inspector "
                 "<map.pbstream>\n";
    return 1;
  }
  const std::string path = argv[1];
  {
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) {
      std::cerr << "无法打开文件: " << path << "\n";
      return 2;
    }
  }

  ::cartographer::io::ProtoStreamReader reader(path);

  // pbstream 首条是 SerializationHeader，其余均为 SerializedData。
  ::cartographer::mapping::proto::SerializationHeader header;
  if (!reader.ReadProto(&header)) {
    std::cerr << path << " 不是有效 pbstream（读不到 header）。\n";
    return 2;
  }

  std::map<int, int> counts;  // data_case -> 数量
  float chunk_length = 0.f;
  int chunk_info_num = 0;
  int total = 0;
  SerializedData data;
  while (reader.ReadProto(&data)) {
    const auto c = data.data_case();
    ++counts[c];
    if (c == SerializedData::kChunkMapInfo) {
      chunk_length = data.chunk_map_info().chunk_length();
      chunk_info_num = data.chunk_map_info().num_chunks();
    }
    ++total;
    data.Clear();
  }

  auto get = [&](SerializedData::DataCase c) {
    const auto it = counts.find(c);
    return it == counts.end() ? 0 : it->second;
  };

  std::cout << "\n========== pbstream 检查 ==========\n";
  std::cout << "文件            : " << path << "\n";
  std::cout << "format_version  : " << header.format_version() << "\n";
  std::cout << "SerializedData 总条目: " << total << "\n\n";
  std::cout << "元素类型与数量:\n";
  for (const auto& kv : counts) {
    std::cout << "  " << std::left << std::setw(34)
              << CaseName(static_cast<SerializedData::DataCase>(kv.first))
              << " : " << kv.second << "\n";
  }

  const int pg = get(SerializedData::kPoseGraph);
  const int opt = get(SerializedData::kAllTrajectoryBuilderOptions);
  const int submap = get(SerializedData::kSubmap);
  const int node = get(SerializedData::kNode);
  const int gss = get(SerializedData::kGlobalStaticSubmap);
  const int cinfo = get(SerializedData::kChunkMapInfo);
  const int csub = get(SerializedData::kChunkSubmap);

  std::cout << "\n--- chunk map ---\n";
  const bool has_chunk = csub > 0;
  std::cout << "  是否含 chunk map : " << (has_chunk ? "是" : "否");
  if (has_chunk) {
    std::cout << "（chunk_submap=" << csub << ", chunk_map_info=" << cinfo
              << ", chunk_length=" << chunk_length << "m, info.num_chunks="
              << chunk_info_num << "）";
  }
  std::cout << "\n";
  if (has_chunk && cinfo == 0) {
    std::cout << "  ⚠️ 有 chunk_submap 但缺 chunk_map_info\n";
  }
  if (has_chunk && cinfo > 0 && chunk_info_num != csub) {
    std::cout << "  ⚠️ chunk_map_info.num_chunks(" << chunk_info_num
              << ") ≠ 实际 chunk_submap(" << csub << ")\n";
  }

  std::cout << "\n--- 定位要求 ---\n";
  auto chk = [](bool ok) { return ok ? "✓" : "✗"; };
  const bool has_pg = pg >= 1;
  const bool has_opt = opt >= 1;
  // 定位用 global_static_submap；缺失时 LoadState 从 node 回退生成（GenerateGlobalStaticSubmap）。
  const bool has_map = gss >= 1 || node >= 1;
  std::cout << "  pose_graph                 : " << chk(has_pg) << " (" << pg << ")\n";
  std::cout << "  trajectory_builder_options : " << chk(has_opt) << " (" << opt << ")\n";
  std::cout << "  全局地图(static 或可回退)  : " << chk(has_map)
            << " (global_static_submap=" << gss << ", submap=" << submap
            << ", node=" << node << ")\n";

  const bool loc_ok = has_pg && has_opt && has_map;
  std::cout << "\n是否符合定位要求: " << (loc_ok ? "✅ 是" : "❌ 否") << "\n";
  if (!loc_ok) {
    std::cout << "  缺少: ";
    if (!has_pg) std::cout << "pose_graph ";
    if (!has_opt) std::cout << "trajectory_builder_options ";
    if (!has_map) std::cout << "全局地图(global_static_submap 或 node) ";
    std::cout << "\n";
  } else if (gss == 0) {
    std::cout << "  注: 无 global_static_submap，定位时将从 node 回退生成"
                 "（GenerateGlobalStaticSubmap，较慢）。\n";
  }
  std::cout << "==================================\n";
  return loc_ok ? 0 : 3;
}
