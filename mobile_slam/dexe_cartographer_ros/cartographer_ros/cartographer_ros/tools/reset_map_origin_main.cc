/*
 * reset_map_origin_main.cc
 *
 * 离线工具：变更地图原点。
 *
 * 用法（推荐）：
 *   ros2 run cartographer_ros reset_map_origin_tool \
 *     --map_name <地图名称> \
 *     --new_x 1.5 --new_y -2.0 --new_yaw 0.785
 *
 *   会从 ~/.dexe/data/maps/<map_name> 加载地图（可用 --maps_root 改默认根目录）。
 *
 * 说明：
 *   传入地图目录（含 map.pbstream、map.yaml、map.png）以及新原点在旧地图坐标系下的
 *   位姿（x, y, yaw）。工具将：
 *     1. 加载 map.pbstream，反序列化 submap 切片
 *     2. 对所有 submap 的全局位姿应用坐标变换（新原点的逆变换）
 *     3. 重新渲染并保存 map.png + map.yaml（覆盖原文件）
 *     4. 将变换后的 submap pose 也写回 map.pbstream（使多次调用的变换可累积）
 *
 * 路径解析：
 *   --map_name <name>         → ~/.dexe/data/maps/<name>（或 --maps_root 指定的根目录）
 *   --map_dir  <name|path>    → 完整路径或相对 maps_root 的名称（向后兼容）
 *   --pbstream_filename <p>   → 直接指定 pbstream 文件（更细粒度控制）
 *
 * 注意：
 *   - 输入 yaw 单位为弧度
 *   - 原文件会被覆盖，请提前备份
 *
 * 变换/渲染/栅格重建 helper 已抽取到 tools/map_transform_common.{h,cc}，
 * 与 align_map_tool 共享。
 */

#include <cstdio>
#include <string>
#include <map>
#include <unistd.h>
#include <cstdlib>
#include <vector>
#include <cmath>

#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros/tools/map_transform_common.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_name, "",
              "地图名称（推荐用法）。最终目录解析为 <maps_root>/<map_name>，需含 "
              "map.pbstream / map.yaml / map.png");
DEFINE_string(maps_root, "",
              "默认地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(map_dir, "",
              "地图目录完整路径（与 --map_name 二选一）。"
              "若为相对名称，将相对 maps_root 解析（向后兼容旧用法）");
DEFINE_string(pbstream_filename, "", "pbstream 文件路径（可选，默认为 map_dir/map.pbstream）");
DEFINE_string(map_filestem, "", "输出文件名 stem（可选，默认为 map_dir/map）");
DEFINE_double(new_x, 0.0, "新原点在旧地图坐标系中的 X 坐标（米）");
DEFINE_double(new_y, 0.0, "新原点在旧地图坐标系中的 Y 坐标（米）");
DEFINE_double(new_yaw, 0.0, "新原点在旧地图坐标系中的旋转角度（弧度）");
DEFINE_double(resolution, 0.05, "输出地图分辨率（米/格）");
DEFINE_double(occ_thresh, 0.65, "占用阈值（>此值为障碍物）");
DEFINE_double(free_thresh, 0.196, "空闲阈值（<此值为自由空间）");
DEFINE_string(navigation_map_filter_mode, "normal",
              "导航地图噪声过滤强度（normal/high/super），与 node 的 save_map 一致");
DEFINE_bool(save_origin_map, false,
            "是否同时保存未经噪声过滤的原始地图为 *_origin.png");
DEFINE_bool(save_debug_map, false, "是否保存噪声过滤的中间调试图（debug_map/）");

namespace cartographer_ros {
namespace {

namespace mt = ::cartographer_ros::map_transform;

void Run(const std::string& pbstream_file,
         const std::string& map_filestem,
         const double new_x,
         const double new_y,
         const double new_yaw,
         const double resolution,
         const double occ_thresh,
         const double free_thresh,
         const std::string& navigation_map_filter_mode,
         const bool save_origin_map,
         const bool save_debug_map) {
  // === Step 1: 单次读盘，把 pbstream 全部数据加载到内存 ===
  mt::PbstreamContents contents = mt::LoadPbstream(pbstream_file);
  auto& pose_graph = contents.pose_graph;
  auto& traj_options = contents.traj_options;
  auto& all_records = contents.records;

  // === Step 2: 构造变换 T_inv（旧系 → 新系）===
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

  // === Step 3: 应用 T_inv 到 pose_graph 内所有全局位姿 ===
  size_t n_submap = 0, n_node = 0, n_landmark = 0;
  for (auto& trajectory : *pose_graph.mutable_trajectory()) {
    for (auto& submap : *trajectory.mutable_submap()) {
      const auto old_pose =
          ::cartographer::transform::ToRigid3(submap.pose());
      *submap.mutable_pose() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_submap;
    }
    for (auto& node : *trajectory.mutable_node()) {
      const auto old_pose = ::cartographer::transform::ToRigid3(node.pose());
      *node.mutable_pose() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_node;
    }
  }
  for (auto& landmark : *pose_graph.mutable_landmark_poses()) {
    const auto old_pose =
        ::cartographer::transform::ToRigid3(landmark.global_pose());
    *landmark.mutable_global_pose() =
        ::cartographer::transform::ToProto(T_inv * old_pose);
    ++n_landmark;
  }
  LOG(INFO) << "Pose graph transformed: " << n_submap << " submap poses, "
            << n_node << " node poses, " << n_landmark << " landmark poses";

  // === Step 4: 应用 T_inv 到 SerializedData ===
  //   - global_static_submap：local_pose + cloud + line（grid 在 Step 4b 重新生成）
  //   - trajectory_data：fixed_frame_origin_in_map
  //   - 其余记录（submap/node/imu/odometry/landmark_data/fixed_frame_pose_data）保持原值
  ::cartographer::mapping::proto::Submap2D* static_submap_proto = nullptr;
  size_t n_static = 0, n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
      mt::TransformStaticSubmapNonGrid(rec.mutable_global_static_submap(), T_inv);
      static_submap_proto = rec.mutable_global_static_submap();
      ++n_static;
    } else if (rec.has_trajectory_data() &&
               rec.trajectory_data().has_fixed_frame_origin_in_map()) {
      auto* td = rec.mutable_trajectory_data();
      const auto old_pose = ::cartographer::transform::ToRigid3(
          td->fixed_frame_origin_in_map());
      *td->mutable_fixed_frame_origin_in_map() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_traj_data;
    }
  }
  LOG(INFO) << "SerializedData transformed (non-grid): " << n_static
            << " global_static_submap (local_pose + cloud + line), "
            << n_traj_data << " trajectory_data fixed_frame_origin";

  // === Step 4b: 从 trajectory node 的点云重新生成最终栅格地图 ===
  //   理由：直接拼接各 trajectory submap 切片渲染会在重叠区出现重影；
  //         重新插入所有 node cloud 到一张干净 ProbabilityGrid，结果与
  //         MapBuilderBridge::SaveMap 内 GenerateFinalSubmap 的产物一致。
  //   仅在 pose_graph 至少有 1 个 node 时执行，否则保留旧 grid 并按
  //   重栅格化方式回退（用于纯 pose-graph、无原始 cloud 的特殊 pbstream）。
  if (static_submap_proto != nullptr) {
    bool has_nodes = false;
    for (const auto& traj : pose_graph.trajectory()) {
      if (traj.node_size() > 0) { has_nodes = true; break; }
    }
    if (has_nodes) {
      // padding 30m，覆盖典型激光最大距离；可后续暴露成 flag
      *static_submap_proto->mutable_grid() = mt::BuildFreshGridFromNodes(
          pose_graph, all_records, traj_options, resolution,
          /*lidar_range_pad=*/30.0);
    } else if (static_submap_proto->has_grid()) {
      LOG(WARNING) << "pose_graph has no nodes; falling back to "
                      "rerasterize existing global_static_submap.grid";
      mt::RerasterizeGridProto(static_submap_proto->mutable_grid(), T_inv);
    }
  }

  // === Step 5: 渲染地图（与 MapBuilderBridge::SaveMap 一致）===
  //   优先用变换后的 global_static_submap（cached merged map，从 range_data 累积出来），
  //   缺失则回退到把 trajectory submap 切片拼接（旧行为）。
  cv::Mat origin_mat;
  Eigen::Vector2d map_origin;
  double render_resolution = resolution;

  if (static_submap_proto != nullptr) {
    LOG(INFO) << "Rendering from cached global_static_submap (matches "
                 "MapBuilderBridge::SaveMap behavior)";
    mt::RenderSingleStaticSubmap(*static_submap_proto, occ_thresh, free_thresh,
                                 &origin_mat, &map_origin, &render_resolution);
  } else {
    LOG(WARNING) << "pbstream has no global_static_submap, falling back to "
                    "stitching trajectory submap slices";
    std::map<::cartographer::mapping::SubmapId,
             ::cartographer::io::SubmapSlice>
        submap_slices;
    ::cartographer::mapping::ValueConversionTables conv;
    // 用变换后的 submap.pose 重新填充 slices
    std::map<::cartographer::mapping::SubmapId,
             ::cartographer::transform::Rigid3d>
        submap_poses;
    for (const auto& trajectory : pose_graph.trajectory()) {
      for (const auto& sm : trajectory.submap()) {
        submap_poses[{trajectory.trajectory_id(), sm.submap_index()}] =
            ::cartographer::transform::ToRigid3(sm.pose());
      }
    }
    for (const auto& rec : all_records) {
      if (!rec.has_submap()) continue;
      const auto& sm = rec.submap();
      const ::cartographer::mapping::SubmapId id{
          sm.submap_id().trajectory_id(), sm.submap_id().submap_index()};
      auto it = submap_poses.find(id);
      if (it == submap_poses.end()) continue;
      ::cartographer::io::FillSubmapSlice(it->second, sm,
                                           &submap_slices[id], &conv);
    }
    auto paint =
        ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
    ::cartographer::io::Image image(std::move(paint.surface));
    origin_mat = ::cartographer_ros::GeneratePngMat(image, occ_thresh, free_thresh);
    map_origin = Eigen::Vector2d(
        -paint.origin.x() * resolution,
        (paint.origin.y() - image.height()) * resolution);
    render_resolution = resolution;
  }

  // === Step 6: 写出 png/yaml（统一走 WriteNavigationMap，与 node 端 save_map 一致）===
  const std::string png_file = map_filestem + ".png";
  const std::string yaml_file = map_filestem + ".yaml";

  std::string map_dir;
  {
    const size_t last_slash = map_filestem.find_last_of("/");
    map_dir = (last_slash != std::string::npos)
                  ? map_filestem.substr(0, last_slash)
                  : ".";
  }

  WriteNavigationMap(origin_mat, png_file,
                     navigation_map_filter_mode, save_debug_map, map_dir);

  ::cartographer::io::StreamFileWriter yaml_writer(yaml_file);
  WriteYaml(render_resolution, map_origin, map_filestem, &yaml_writer,
            occ_thresh, free_thresh);

  if (save_origin_map) {
    const std::string origin_png = map_filestem + "_origin.png";
    cv::imwrite(origin_png, origin_mat);
    LOG(INFO) << "Saved origin map to: " << origin_png;
  }

  LOG(INFO) << "Saved map to: " << map_filestem << ".png/.yaml";

  // === Step 7: 写回 pbstream（含全部已变换的 pose_graph + records）===
  mt::WritePbstream(pbstream_file, contents);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_name.empty() || !FLAGS_map_dir.empty() ||
        !FLAGS_pbstream_filename.empty())
      << "Need one of: --map_name, --map_dir, or --pbstream_filename";

  // 默认地图根目录：用户传 --maps_root 优先，否则 ~/.dexe/data/maps
  std::string maps_root = FLAGS_maps_root;
  if (maps_root.empty()) {
    const char* home = std::getenv("HOME");
    CHECK(home != nullptr) << "HOME environment variable not set";
    maps_root = std::string(home) + "/.dexe/data/maps";
  }

  // 解析地图目录优先级：
  //   1. --map_name        → <maps_root>/<map_name>（推荐）
  //   2. --map_dir         → 完整路径或相对 maps_root 的名称（向后兼容旧用法）
  //   3. --pbstream_filename 单独使用，map_dir 留空
  std::string map_dir;
  if (!FLAGS_map_name.empty()) {
    map_dir = maps_root + "/" + FLAGS_map_name;
    if (access((map_dir + "/map.pbstream").c_str(), F_OK) != 0) {
      LOG(FATAL) << "Map not found: " << map_dir << "/map.pbstream\n"
                 << "  --map_name = " << FLAGS_map_name << "\n"
                 << "  --maps_root = " << maps_root;
    }
    LOG(INFO) << "Resolved map directory: " << map_dir;
  } else if (!FLAGS_map_dir.empty() && FLAGS_pbstream_filename.empty()) {
    // --map_dir 同时支持完整路径和相对 maps_root 的名称
    const std::string candidate_default = maps_root + "/" + FLAGS_map_dir;
    const std::string candidate_direct = FLAGS_map_dir;

    const bool default_exists =
        (access((candidate_default + "/map.pbstream").c_str(), F_OK) == 0);
    const bool direct_exists =
        (access((candidate_direct + "/map.pbstream").c_str(), F_OK) == 0);

    if (default_exists) {
      map_dir = candidate_default;
      LOG(INFO) << "Found map in default path: " << map_dir;
    } else if (direct_exists) {
      map_dir = candidate_direct;
      LOG(INFO) << "Found map at given path: " << map_dir;
    } else {
      LOG(FATAL) << "Map not found.\n"
                 << "  Tried default path: " << candidate_default << "\n"
                 << "  Tried given path:   " << candidate_direct << "\n"
                 << "Please check --map_name / --map_dir / --pbstream_filename.";
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
      FLAGS_resolution, FLAGS_occ_thresh, FLAGS_free_thresh,
      FLAGS_navigation_map_filter_mode,
      FLAGS_save_origin_map,
      FLAGS_save_debug_map);

  rclcpp::shutdown();
  return 0;
}
