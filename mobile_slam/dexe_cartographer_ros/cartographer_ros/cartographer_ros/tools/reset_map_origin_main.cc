/*
 * reset_map_origin_main.cc
 *
 * 离线工具：变更地图原点。
 *
 * 用法：
 *   ros2 run cartographer_ros reset_map_origin_tool \
 *     --map_dir <地图名称 或 地图目录路径> \
 *     --new_x 1.5 --new_y -2.0 --new_yaw 0.785
 *
 * 说明：
 *   传入地图目录（包含 map.pbstream、map.yaml、map.png）以及新原点在旧地图坐标系下的
 *   位姿（x, y, yaw）。工具将：
 *     1. 加载 map.pbstream，反序列化 submap 切片
 *     2. 对所有 submap 的全局位姿应用坐标变换（新原点的逆变换）
 *     3. 重新渲染并保存 map.png + map.yaml（覆盖原文件）
 *     4. 将变换后的 submap pose 也写回 map.pbstream（使多次调用的变换可累积）
 *
 * 注意：
 *   - 输入 yaw 单位为弧度
 *   - --map_dir 可传地图名称（在默认路径 ~/.dexe/data/maps 下查找）或完整路径
 *   - 原文件会被覆盖，请提前备份
 */

#include <cstdio>
#include <string>
#include <map>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <cmath>

#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_dir, "", "地图目录路径（包含 map.pbstream, map.yaml, map.png）");
DEFINE_string(pbstream_filename, "", "pbstream 文件路径（可选，默认为 map_dir/map.pbstream）");
DEFINE_string(map_filestem, "", "输出文件名 stem（可选，默认为 map_dir/map）");
DEFINE_double(new_x, 0.0, "新原点在旧地图坐标系中的 X 坐标（米）");
DEFINE_double(new_y, 0.0, "新原点在旧地图坐标系中的 Y 坐标（米）");
DEFINE_double(new_yaw, 0.0, "新原点在旧地图坐标系中的旋转角度（弧度）");
DEFINE_double(resolution, 0.05, "输出地图分辨率（米/格）");
DEFINE_double(occ_thresh, 0.65, "占用阈值（>此值为障碍物）");
DEFINE_double(free_thresh, 0.196, "空闲阈值（<此值为自由空间）");

namespace cartographer_ros {
namespace {

void Run(const std::string& pbstream_file,
         const std::string& map_filestem,
         const double new_x,
         const double new_y,
         const double new_yaw,
         const double resolution,
         const double occ_thresh,
         const double free_thresh) {
  // 1. 加载 pbstream，反序列化 submap 切片
  LOG(INFO) << "Loading pbstream: " << pbstream_file;
  ::cartographer::io::ProtoStreamReader reader(pbstream_file);
  ::cartographer::io::ProtoStreamDeserializer deserializer(&reader);

  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;
  ::cartographer::io::DeserializeAndFillSubmapSlices(
      &deserializer, &submap_slices, &conversion_tables);
  CHECK(reader.eof()) << "Failed to fully read pbstream";
  LOG(INFO) << "Loaded " << submap_slices.size() << " submap slices";

  // 2. 构造变换：新原点在旧坐标系下的位姿，取其逆即为坐标变换
  //    T_new_origin: 从旧坐标系到新坐标系（新原点的位姿）
  //    T_inv: 将所有旧坐标系下的坐标变换到新坐标系
  const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()));
  const ::cartographer::transform::Rigid3d T_new_origin(
      Eigen::Vector3d(new_x, new_y, 0.0), rotation);
  const ::cartographer::transform::Rigid3d T_inv = T_new_origin.inverse();

  LOG(INFO) << "New origin in old map frame: x=" << new_x
            << " y=" << new_y << " yaw=" << new_yaw << " rad";
  LOG(INFO) << "Transform (T_inv): translation=("
            << T_inv.translation().x() << ", " << T_inv.translation().y()
            << "), rotation=" << T_inv.rotation().coeffs().transpose();

  // 3. 对所有 submap 切片的全局位姿应用变换
  for (auto& [id, slice] : submap_slices) {
    slice.pose = T_inv * slice.pose;
  }

  // 4. 渲染地图图像
  LOG(INFO) << "Rendering map image...";
  auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);

  ::cartographer::io::Image image(std::move(result.surface));

  const Eigen::Vector2d origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  // 5. 保存 PNG + YAML
  const std::string png_file = map_filestem + ".png";
  const std::string yaml_file = map_filestem + ".yaml";

  ::cartographer::io::StreamFileWriter png_writer(png_file);
  WritePng(image, png_writer.GetFilename(), occ_thresh, free_thresh);

  ::cartographer::io::StreamFileWriter yaml_writer(yaml_file);
  WriteYaml(resolution, origin, png_writer.GetFilename(), &yaml_writer,
            occ_thresh, free_thresh);

  LOG(INFO) << "Saved map to: " << map_filestem << ".png/.yaml";
  LOG(INFO) << "Map size: " << image.width() << "x" << image.height() << " pixels";
  LOG(INFO) << "Map origin: (" << origin.x() << ", " << origin.y() << ")";

  // 6. 将变换后的 submap pose 写回 pbstream，使下次调用时的变换可累积
  //    重新读取 pbstream，修改 pose_graph 中的 submap pose，写入临时文件后 rename
  LOG(INFO) << "Updating pbstream submap poses...";
  {
    ::cartographer::io::ProtoStreamReader reader2(pbstream_file);
    ::cartographer::io::ProtoStreamDeserializer deserializer2(&reader2);

    // 修改 pose_graph 中每个 submap 的 pose
    auto& pose_graph = deserializer2.pose_graph();
    for (auto& trajectory : *pose_graph.mutable_trajectory()) {
      for (auto& submap : *trajectory.mutable_submap()) {
        const ::cartographer::mapping::SubmapId id{
            trajectory.trajectory_id(), submap.submap_index()};
        const auto old_pose =
            ::cartographer::transform::ToRigid3(submap.pose());
        const auto new_pose = T_inv * old_pose;
        *submap.mutable_pose() =
            ::cartographer::transform::ToProto(new_pose);
      }
    }

    // 缓存所有 SerializedData
    std::vector<::cartographer::mapping::proto::SerializedData> all_data;
    ::cartographer::mapping::proto::SerializedData data;
    while (deserializer2.ReadNextSerializedData(&data)) {
      all_data.push_back(std::move(data));
    }

    // 写入临时文件
    const std::string tmp_pbstream = pbstream_file + ".tmp";
    {
      ::cartographer::io::ProtoStreamWriter writer(tmp_pbstream);
      writer.WriteProto(deserializer2.header());
      // 写 pose_graph（包含修改后的 submap poses）
      ::cartographer::mapping::proto::SerializedData pg_data;
      *pg_data.mutable_pose_graph() = pose_graph;
      writer.WriteProto(pg_data);
      // 写 all_trajectory_builder_options
      ::cartographer::mapping::proto::SerializedData opts_data;
      *opts_data.mutable_all_trajectory_builder_options() =
          deserializer2.all_trajectory_builder_options();
      writer.WriteProto(opts_data);
      // 写原始 submap 纹理数据
      for (const auto& d : all_data) {
        writer.WriteProto(d);
      }
      CHECK(writer.Close()) << "Failed to close pbstream writer";
    }
    // 原子替换
    CHECK_EQ(std::rename(tmp_pbstream.c_str(), pbstream_file.c_str()), 0)
        << "Failed to rename tmp pbstream to " << pbstream_file;
  }
  LOG(INFO) << "Pbstream updated: " << pbstream_file;
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_dir.empty() || !FLAGS_pbstream_filename.empty())
      << "Either --map_dir or --pbstream_filename must be specified";

  // 解析输入路径
  // --map_dir 既接受地图目录的完整路径，也接受地图名称（在默认路径下查找）
  // 优先使用默认地图路径 ~/.dexe/data/maps/<name>
  std::string map_dir = FLAGS_map_dir;
  if (!map_dir.empty() && FLAGS_pbstream_filename.empty()) {
    const char* home = std::getenv("HOME");
    CHECK(home != nullptr) << "HOME environment variable not set";
    const std::string default_maps_root =
        std::string(home) + "/.dexe/data/maps";
    const std::string candidate_default = default_maps_root + "/" + map_dir;
    const std::string candidate_direct = map_dir;

    const bool default_exists =
        (access((candidate_default + "/map.pbstream").c_str(), F_OK) == 0);
    const bool direct_exists =
        (access((candidate_direct + "/map.pbstream").c_str(), F_OK) == 0);

    if (default_exists) {
      // 优先使用默认地图路径下的地图（即使 map_dir 也是完整路径）
      map_dir = candidate_default;
      LOG(INFO) << "Found map in default path: " << map_dir;
    } else if (direct_exists) {
      map_dir = candidate_direct;
      LOG(INFO) << "Found map at given path: " << map_dir;
    } else {
      LOG(FATAL) << "Map not found.\n"
                 << "  Tried default path: " << candidate_default << "\n"
                 << "  Tried given path:   " << candidate_direct << "\n"
                 << "Please check --map_dir or --pbstream_filename.";
    }
  }

  std::string pbstream_file = FLAGS_pbstream_filename;
  std::string map_filestem = FLAGS_map_filestem;

  if (pbstream_file.empty()) {
    pbstream_file = map_dir + "/map.pbstream";
  }
  if (map_filestem.empty()) {
    if (!map_dir.empty()) {
      map_filestem = map_dir + "/map";
    } else {
      // 从 pbstream 路径推断
      const std::string& p = pbstream_file;
      const size_t dot = p.rfind(".pbstream");
      map_filestem = (dot != std::string::npos) ? p.substr(0, dot) : p;
    }
  }

  ::cartographer_ros::Run(
      pbstream_file, map_filestem,
      FLAGS_new_x, FLAGS_new_y, FLAGS_new_yaw,
      FLAGS_resolution, FLAGS_occ_thresh, FLAGS_free_thresh);

  rclcpp::shutdown();
  return 0;
}
