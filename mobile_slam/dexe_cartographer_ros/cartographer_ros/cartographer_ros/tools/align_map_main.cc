/*
 * align_map_main.cc
 *
 * 离线工具：把已保存的地图旋转到对齐结构化环境（基于线特征）。
 *
 * 与在线建图的 Node::ResetMapOriginToAlignStructure 功能相同，但作用于
 * 已落盘的地图文件夹：从 pbstream 的 global_static_submap 读取聚合后的
 * 全局线特征，用 EstimateAlignmentYaw() 自动估计对齐角，然后把绕原点的
 * 纯旋转施加到所有地图数据并重新生成地图文件。
 *
 * 用法（推荐，直接指定地图文件夹）：
 *   ros2 run cartographer_ros align_map_tool \
 *     --map_dir /home/dex/.dexe/data/maps/map_dynamic
 *
 *   自动探测 <map_dir>/slam/map.pbstream（分层布局），否则 <map_dir>/map.pbstream。
 *   也可用 --map_name <名称>（相对 ~/.dexe/data/maps，同样自动补 slam/）。
 *
 * 工具将：
 *   1. 加载 map.pbstream，从 global_static_submap.line_map_2d_list 估计对齐角
 *      （--dry_run 只打印角度，不修改任何文件）
 *   2. 备份原地图文件到 <slam>/backups/backup_<时间戳>/（子文件保持原名）
 *   3. 对 pose graph / global_static_submap / trajectory_data 施加旋转
 *   4. 重建栅格（默认从 trajectory node 点云重建干净栅格；
 *      --norebuild_grid_from_nodes 改为重栅格化原 grid）并重渲染
 *      map.png + map.yaml
 *   5. 变换 map.pcd / map_highlight.pcd（若存在）
 *   6. 原子写回 map.pbstream，并对变换后的线特征复检残余角
 *
 * 失败路径：无 global_static_submap / 无线特征 / 估计不出可靠正交结构时
 * 直接报错退出，不修改任何文件。
 *
 * 注意：依赖 feature/add_align_map 分支的 pbstream 格式（global_static_submap
 * + 线特征）；旧格式地图无法处理。
 */

#include <unistd.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/structure_alignment.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros/tools/map_transform_common.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_dir, "",
              "地图文件夹完整路径（推荐用法），如 "
              "/home/dex/.dexe/data/maps/map_dynamic。"
              "自动探测 <map_dir>/slam/map.pbstream，否则 <map_dir>/map.pbstream");
DEFINE_string(map_name, "",
              "地图名称（与 --map_dir 二选一）。解析为 <maps_root>/<map_name>，"
              "同样自动补 slam/ 子目录");
DEFINE_string(maps_root, "", "默认地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(pbstream_filename, "", "pbstream 文件路径（可选，覆盖目录探测）");
DEFINE_string(map_filestem, "", "输出文件名 stem（可选，默认为 <slam目录>/map）");

DEFINE_bool(dry_run, false, "只估计并打印对齐角，不修改任何文件");
DEFINE_bool(rebuild_grid_from_nodes, true,
            "栅格重建策略：true（默认）= 从 trajectory node 点云重建干净栅格"
            "（与 reset_map_origin_tool 一致，但 misses 为空、free space 丢失）；"
            "--norebuild_grid_from_nodes = 重栅格化原 grid（保留概率值与 "
            "free space，观感最接近保存时）");

// 结构对齐参数（默认值与 dexe_slam_2d.lua 的 reset_map_origin_* 一致）
DEFINE_double(min_line_length, 0.3, "结构线段最小长度（米）");
DEFINE_int32(max_candidate_lines, 20, "按长度降序取前 N 条候选线段");
DEFINE_double(orthogonality_tolerance_deg, 10.0,
              "正交容差（度）：次方向须落在主方向 +90° ± 该容差内");
DEFINE_double(min_secondary_support_ratio, 0.1,
              "次方向支撑量 / 主方向支撑量 的最小比值");

// 渲染参数：默认 0 = 沿用原 map.yaml 中的值（缺 yaml 时回退括号内默认）
DEFINE_double(resolution, 0.0, "输出地图分辨率（米/格），0 = 沿用原 yaml（0.05）");
DEFINE_double(occ_thresh, 0.0, "占用阈值，0 = 沿用原 yaml（0.65）");
DEFINE_double(free_thresh, 0.0, "空闲阈值，0 = 沿用原 yaml（0.196）");
DEFINE_string(navigation_map_filter_mode, "normal",
              "导航地图噪声过滤强度（normal/high/super），与 node 的 save_map 一致");
DEFINE_bool(save_origin_map, false,
            "是否同时保存未经噪声过滤的原始地图为 *_origin.png"
            "（原目录已存在 *_origin.png 时会自动保存，无需此 flag）");
DEFINE_bool(save_debug_map, false, "是否保存噪声过滤的中间调试图（debug_map/）");

namespace cartographer_ros {
namespace {

namespace fs = std::filesystem;
namespace mt = ::cartographer_ros::map_transform;

// 从原 map.yaml 读取 resolution / occupied_thresh / free_thresh 作为默认值。
// 极简逐行解析（"key: value" 格式），键缺失时保留传入的当前值。
void ReadYamlDefaults(const std::string& yaml_file, double* resolution,
                      double* occ_thresh, double* free_thresh) {
  std::ifstream in(yaml_file);
  if (!in.is_open()) {
    LOG(WARNING) << "Cannot open " << yaml_file
                 << "; falling back to built-in defaults";
    return;
  }
  std::string line;
  const auto try_read = [&line](const char* key, double* out) {
    const std::string prefix = std::string(key) + ":";
    if (line.compare(0, prefix.size(), prefix) != 0) return false;
    const std::string value = line.substr(prefix.size());
    try {
      *out = std::stod(value);
    } catch (const std::exception&) {
      return false;
    }
    return true;
  };
  while (std::getline(in, line)) {
    // 去掉行首空白
    const size_t b = line.find_first_not_of(" \t");
    if (b == std::string::npos) continue;
    line = line.substr(b);
    try_read("resolution", resolution);
    try_read("occupied_thresh", occ_thresh);
    try_read("free_thresh", free_thresh);
  }
}

void Run(const std::string& pbstream_file, const std::string& map_filestem,
         const bool dry_run) {
  // === Step 1: 加载 pbstream ===
  mt::PbstreamContents contents = mt::LoadPbstream(pbstream_file);
  auto& pose_graph = contents.pose_graph;
  auto& traj_options = contents.traj_options;
  auto& all_records = contents.records;

  // 渲染参数默认值：flag 优先，其次原 map.yaml，最后内置默认
  double resolution = 0.05, occ_thresh = 0.65, free_thresh = 0.196;
  ReadYamlDefaults(map_filestem + ".yaml", &resolution, &occ_thresh,
                   &free_thresh);
  if (FLAGS_resolution > 0.0) resolution = FLAGS_resolution;
  if (FLAGS_occ_thresh > 0.0) occ_thresh = FLAGS_occ_thresh;
  if (FLAGS_free_thresh > 0.0) free_thresh = FLAGS_free_thresh;
  LOG(INFO) << "Render params: resolution=" << resolution
            << " occ_thresh=" << occ_thresh << " free_thresh=" << free_thresh;

  // === Step 2: 定位 global_static_submap，提取全局线特征 ===
  ::cartographer::mapping::proto::Submap2D* static_submap_proto = nullptr;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
      static_submap_proto = rec.mutable_global_static_submap();
      break;
    }
  }
  if (static_submap_proto == nullptr) {
    LOG(FATAL) << "pbstream has no global_static_submap record.\n"
               << "  This tool requires maps saved by the add_align_map branch "
                  "(global_static_submap + line features).\n"
               << "  Run `cartographer_pbstream check " << pbstream_file
               << "` to inspect the file (see STRUCT-02).";
  }
  const bool has_lines = static_submap_proto->has_line_map_2d_list() &&
                         static_submap_proto->line_map_2d_list()
                                 .line_map_data_size() > 0;
  if (!has_lines) {
    LOG(FATAL) << "global_static_submap has no line features "
                  "(line_map_2d_list absent or empty).\n"
               << "  Submap2D::ToProto() skips the field when every line was "
                  "culled before the save, so the map was likely built with "
                  "line features disabled or by an older binary (see "
                  "pbstream_check LINE-01).\n"
               << "  Structural alignment is not possible without line "
                  "features; files were NOT modified.";
  }

  // === Step 3: 估计对齐角 ===
  const ::cartographer::mapping::LineMap2DList line_map =
      mt::LineMap2DListFromProto(static_submap_proto->line_map_2d_list());

  ::cartographer::mapping::StructureAlignmentOptions align_opts;
  align_opts.min_line_length = FLAGS_min_line_length;
  align_opts.max_candidate_lines = FLAGS_max_candidate_lines;
  align_opts.orthogonality_tolerance_deg = FLAGS_orthogonality_tolerance_deg;
  align_opts.min_secondary_support_ratio = FLAGS_min_secondary_support_ratio;

  const auto yaw_opt =
      ::cartographer::mapping::EstimateAlignmentYaw(line_map, align_opts);
  if (!yaw_opt.has_value()) {
    LOG(FATAL) << "EstimateAlignmentYaw found no reliable orthogonal "
                  "structure; files were NOT modified.\n"
               << "  Try adjusting --min_line_length / "
                  "--orthogonality_tolerance_deg / "
                  "--min_secondary_support_ratio.";
  }
  const double new_yaw = *yaw_opt;
  LOG(INFO) << "=== Estimated alignment yaw: " << new_yaw << " rad ("
            << new_yaw * 180.0 / M_PI << " deg) ===";

  if (dry_run) {
    LOG(INFO) << "Dry run: no file modified. Re-run without --dry_run to "
                 "apply the rotation.";
    return;
  }

  // === Step 4: 备份原地图文件（修改任何文件之前）===
  const std::vector<std::string> kBackupExtensions = {
      ".pbstream", ".png",   ".yaml", ".pcd",
      "_highlight.pcd", "_origin.png", "_line_map.json"};
  const std::string backup_dir =
      mt::BackupMapFiles(map_filestem, kBackupExtensions);
  LOG(INFO) << "Backup directory: " << backup_dir;
  if (fs::exists(map_filestem + "_line_map.json")) {
    LOG(WARNING) << "map_line_map.json exists but will NOT be transformed "
                    "(only backed up); regenerate it from the aligned map if "
                    "needed.";
  }

  // === Step 5: 构造变换（绕原点纯旋转，与在线实现一致）===
  // T_new_origin = 新原点（原点不动）在旧地图系下的朝向；对其取逆得到
  // 旧系 → 新系 的变换，即所有全局数据要乘的 T_inv。
  const Eigen::Quaterniond rotation(
      Eigen::AngleAxisd(new_yaw, Eigen::Vector3d::UnitZ()));
  const ::cartographer::transform::Rigid3d T_new_origin(
      Eigen::Vector3d::Zero(), rotation);
  const ::cartographer::transform::Rigid3d T_inv = T_new_origin.inverse();
  LOG(INFO) << "Applying pure rotation about origin: T_inv rotation="
            << T_inv.rotation().coeffs().transpose();

  // === Step 6: 变换 pose graph 内所有全局位姿 ===
  size_t n_submap = 0, n_node = 0, n_landmark = 0;
  for (auto& trajectory : *pose_graph.mutable_trajectory()) {
    for (auto& submap : *trajectory.mutable_submap()) {
      const auto old_pose = ::cartographer::transform::ToRigid3(submap.pose());
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

  // === Step 7: 变换 SerializedData（非 grid 部分）===
  size_t n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_global_static_submap()) {
      mt::TransformStaticSubmapNonGrid(rec.mutable_global_static_submap(),
                                       T_inv);
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
  LOG(INFO) << "SerializedData transformed (non-grid): global_static_submap "
               "(local_pose + cloud + line), "
            << n_traj_data << " trajectory_data fixed_frame_origin";

  // === Step 8: 栅格重建 ===
  bool has_nodes = false;
  for (const auto& traj : pose_graph.trajectory()) {
    if (traj.node_size() > 0) { has_nodes = true; break; }
  }
  if (FLAGS_rebuild_grid_from_nodes) {
    CHECK(has_nodes)
        << "--rebuild_grid_from_nodes requires trajectory nodes in pose graph";
    // padding 30m，覆盖典型激光最大距离（与 reset_map_origin_tool 一致）
    *static_submap_proto->mutable_grid() = mt::BuildFreshGridFromNodes(
        pose_graph, all_records, traj_options, resolution,
        /*lidar_range_pad=*/30.0);
  } else if (static_submap_proto->has_grid()) {
    mt::RerasterizeGridProto(static_submap_proto->mutable_grid(), T_inv);
  } else if (has_nodes) {
    LOG(WARNING) << "global_static_submap has no grid; falling back to "
                    "rebuilding from node clouds";
    *static_submap_proto->mutable_grid() = mt::BuildFreshGridFromNodes(
        pose_graph, all_records, traj_options, resolution,
        /*lidar_range_pad=*/30.0);
  } else {
    LOG(FATAL) << "global_static_submap has no grid and pose graph has no "
                  "nodes; cannot rebuild the grid. Backups are kept in "
                  "backups/, no output file has been written yet.";
  }

  // === Step 9: 重渲染 map.png + map.yaml（与 node 端 SaveMap 一致）===
  cv::Mat origin_mat;
  Eigen::Vector2d map_origin;
  double render_resolution = resolution;
  mt::RenderSingleStaticSubmap(*static_submap_proto, occ_thresh, free_thresh,
                               &origin_mat, &map_origin, &render_resolution);

  const std::string map_dir = fs::path(map_filestem).parent_path().string();
  WriteNavigationMap(origin_mat, map_filestem + ".png",
                     FLAGS_navigation_map_filter_mode, FLAGS_save_debug_map,
                     map_dir);

  {
    ::cartographer::io::StreamFileWriter yaml_writer(map_filestem + ".yaml");
    WriteYaml(render_resolution, map_origin, map_filestem, &yaml_writer,
              occ_thresh, free_thresh);
  }

  // 原目录存在 *_origin.png（已备份）或显式要求时，同步输出未过滤图
  if (FLAGS_save_origin_map || fs::exists(map_filestem + "_origin.png")) {
    cv::imwrite(map_filestem + "_origin.png", origin_mat);
    LOG(INFO) << "Saved unfiltered map to: " << map_filestem << "_origin.png";
  }
  LOG(INFO) << "Saved map to: " << map_filestem << ".png/.yaml";

  // === Step 10: 变换 PCD 点云文件（若存在；点云在全局系）===
  mt::TransformPcdFile(map_filestem + ".pcd", T_inv);
  mt::TransformPcdFile(map_filestem + "_highlight.pcd", T_inv);

  // === Step 11: 写回 pbstream ===
  mt::WritePbstream(pbstream_file, contents);

  // === Step 12: 复检 —— 对变换后的线特征重跑估计，残余角应 ≈ 0 ===
  const ::cartographer::mapping::LineMap2DList line_map_after =
      mt::LineMap2DListFromProto(static_submap_proto->line_map_2d_list());
  const auto residual_opt = ::cartographer::mapping::EstimateAlignmentYaw(
      line_map_after, align_opts);
  if (residual_opt.has_value()) {
    LOG(INFO) << "=== Post-transform residual yaw: " << *residual_opt
              << " rad (" << *residual_opt * 180.0 / M_PI
              << " deg), expected < 1 deg ===";
  } else {
    LOG(WARNING) << "Post-transform re-estimation returned nullopt (structure "
                    "no longer evaluable after rotation — unexpected)";
  }
  LOG(INFO) << "align_map_tool done: " << map_dir << " (backup: "
            << backup_dir << ")";
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_dir.empty() || !FLAGS_map_name.empty() ||
        !FLAGS_pbstream_filename.empty())
      << "Need one of: --map_dir, --map_name, or --pbstream_filename";

  // 默认地图根目录：用户传 --maps_root 优先，否则 ~/.dexe/data/maps
  std::string maps_root = FLAGS_maps_root;
  if (maps_root.empty()) {
    const char* home = std::getenv("HOME");
    CHECK(home != nullptr) << "HOME environment variable not set";
    maps_root = std::string(home) + "/.dexe/data/maps";
  }

  // 解析地图根目录（--map_dir 优先；--map_name 相对 maps_root）
  std::string map_base;
  if (!FLAGS_map_dir.empty()) {
    map_base = FLAGS_map_dir;
    if (!std::filesystem::exists(map_base) &&
        std::filesystem::exists(maps_root + "/" + FLAGS_map_dir)) {
      map_base = maps_root + "/" + FLAGS_map_dir;  // 兼容相对名称
    }
  } else if (!FLAGS_map_name.empty()) {
    map_base = maps_root + "/" + FLAGS_map_name;
  }

  // 自动探测 slam/ 子目录（分层布局：底图在 <map>/slam/，资产在 <map>/assets/）
  std::string map_dir;
  if (!map_base.empty()) {
    if (std::filesystem::exists(map_base + "/slam/map.pbstream")) {
      map_dir = map_base + "/slam";
    } else if (std::filesystem::exists(map_base + "/map.pbstream")) {
      map_dir = map_base;
    } else {
      LOG(FATAL) << "Map not found.\n"
                 << "  Tried: " << map_base << "/slam/map.pbstream\n"
                 << "  Tried: " << map_base << "/map.pbstream\n"
                 << "Please check --map_dir / --map_name / --maps_root.";
    }
    LOG(INFO) << "Resolved map directory: " << map_dir;
  }

  std::string pbstream_file = FLAGS_pbstream_filename;
  std::string map_filestem = FLAGS_map_filestem;
  if (pbstream_file.empty()) {
    CHECK(!map_dir.empty())
        << "Need --map_dir/--map_name or --pbstream_filename";
    pbstream_file = map_dir + "/map.pbstream";
  }
  if (map_filestem.empty()) {
    if (!map_dir.empty()) {
      map_filestem = map_dir + "/map";
    } else {
      // 从 pbstream 路径推断
      const size_t dot = pbstream_file.rfind(".pbstream");
      map_filestem =
          (dot != std::string::npos) ? pbstream_file.substr(0, dot)
                                     : pbstream_file;
    }
  }

  ::cartographer_ros::Run(pbstream_file, map_filestem, FLAGS_dry_run);

  rclcpp::shutdown();
  return 0;
}
