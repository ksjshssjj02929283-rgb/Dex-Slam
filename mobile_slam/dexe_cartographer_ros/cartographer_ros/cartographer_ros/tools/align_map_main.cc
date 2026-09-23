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
 *   4. 从 trajectory node 点云**完全重建**栅格 / 点云 / 线特征（无退路开关：
 *      重栅格化旧 grid 会产出内部不自洽的地图）并重渲染 map.png + map.yaml
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

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/2d/structure_alignment.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros/options/mapping_node_options.h"
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

// 结构对齐参数：默认**从 lua 的 reset_map_origin_* 读**，与建图节点
// （Node::ResetMapOriginToAlignStructure → CreateStructureAlignmentOptions）
// 同源。下面这些 flag 只在显式指定时覆盖 lua —— 否则 flag 的默认值会静默盖掉
// 配置文件，两条路径又会各走各的。
DEFINE_string(configuration_directory, "",
              "lua 配置目录，留空则自动解析 cartographer_ros 的 "
              "configuration_files");
DEFINE_string(configuration_basename, "dexe_slam_2d.lua",
              "lua 配置文件名，从中读取 reset_map_origin_* 估角参数");
DEFINE_double(min_line_length, 0.3, "结构线段最小长度（米）；显式指定才覆盖 lua");
DEFINE_int32(max_candidate_lines, 20,
             "按长度降序取前 N 条候选线段；显式指定才覆盖 lua");
DEFINE_double(orthogonality_tolerance_deg, 10.0,
              "正交容差（度）：次方向须落在主方向 +90° ± 该容差内；"
              "显式指定才覆盖 lua");
DEFINE_double(min_secondary_support_ratio, 0.1,
              "次方向支撑量 / 主方向支撑量 的最小比值；显式指定才覆盖 lua");

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


// ⭐ 资产失效确认门：本工具只旋转 <map>/slam/ 下的底图，
// 不碰 <map>/assets/（点位/路网/禁区）与 <map>/versions/（manifest）。
// 对已挂资产的地图直接跑 = 资产全部错位且 backups/ 不覆盖它们、无回滚路径。
// 正确路径是走 command_board 的 ~/align_map（它随后会调 transform_assets 补上）。
DEFINE_bool(i_know_assets_will_break, false,
            "地图目录下存在 assets/ 或 versions/ 时，必须显式加此 flag 才继续"
            "（本工具不变换它们；请改用 command_board 的 ~/align_map 编排）");

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

// 机器可读结果行。恒输出到 stdout，前缀是固定标记：
//
//   [ALIGN_RESULT] {"ok":true,"reason":"applied",...}
//
// 为什么用 stdout 标记行而不是写临时文件：command_board 用 popen 调本工具，
// 本来就在收 stdout，多走一个 /tmp 文件只是白增三个失败面（路径不可写 /
// 残留 / 读不到）。为什么不直接让编排解析日志文本：glog 的 alsologtostderr
// 会把时间戳、线程号混进同一个流，且日志措辞一改解析就断——固定标记 + JSON
// 才是稳定契约。编排侧取**最后一条**标记行即可，不受前面日志干扰。
//
// 字段与 nav_interfaces/srv/AlignMap.srv 的响应一一对应，编排侧直接映射。
// ⚠️ 必须在任何 LOG(FATAL) 之前调用：FATAL 会 abort，std::endl 先 flush 才保得住。
void PrintResultLine(
    bool ok, const std::string& reason,
    const ::cartographer::mapping::StructureAlignmentDiagnostics& diag,
    double yaw_rad, bool applied, double residual_rad);

// 重算后的地图面积，随结果行带出供编排方使用（写盘发生在 PrintResultLine 之前，
// 未走到写盘的失败路径保持 0）。
double g_total_area_m2 = 0.0;
double g_effective_area_m2 = 0.0;

// 估角参数来源：lua 的 reset_map_origin_*（与建图节点 CreateStructureAlignmentOptions
// 同一条路径），命令行 flag 仅在显式指定时覆盖。
// 这样「建图时对齐」与「离线工具对齐」用的是同一组判据，不会因为有人只改了 lua
// 就让两边估出不同的角。
::cartographer::mapping::StructureAlignmentOptions LoadAlignmentOptions() {
  ::cartographer::mapping::StructureAlignmentOptions opts;
  std::string config_dir = FLAGS_configuration_directory;
  if (config_dir.empty()) {
    try {
      config_dir = ament_index_cpp::get_package_share_directory(
                       "cartographer_ros") +
                   "/configuration_files";
    } catch (const std::exception& e) {
      LOG(ERROR) << "Cannot auto-resolve cartographer_ros share dir: "
                 << e.what();
      config_dir.clear();
    }
  }
  // ⚠️ ConfigurationFileResolver 找不到文件走 LOG(FATAL) 不抛异常，try/catch
  // 拦不住且来不及打结果行 —— 必须先自己确认文件可读。
  const std::string full = config_dir.empty()
                               ? std::string()
                               : config_dir + "/" +
                                     FLAGS_configuration_basename;
  if (full.empty() || access(full.c_str(), R_OK) != 0) {
    PrintResultLine(false, "lua_config_not_found", {}, 0.0, false, 0.0);
    LOG(FATAL) << "Alignment thresholds must come from the same lua config the "
                  "mapping node uses (reset_map_origin_*). Not readable: "
               << (full.empty() ? "<unresolved>" : full)
               << ". Pass --configuration_directory / "
                  "--configuration_basename.";
  }
  const auto node_options = ::cartographer_ros::LoadMappingOptions(
      config_dir, FLAGS_configuration_basename);
  opts = ::cartographer_ros::CreateStructureAlignmentOptions(
      std::get<0>(node_options));
  const std::string source = full;
  // flag 只在显式指定时覆盖 —— 用 gflags 的 is_default 区分「用户给了」和
  // 「只是默认值」，避免默认值静默盖掉 lua。
  const auto override_if_set = [&](const char* name, auto&& apply) {
    if (!google::GetCommandLineFlagInfoOrDie(name).is_default) {
      apply();
      LOG(INFO) << "Alignment option overridden by flag: --" << name;
    }
  };
  override_if_set("min_line_length",
                  [&] { opts.min_line_length = FLAGS_min_line_length; });
  override_if_set("max_candidate_lines",
                  [&] { opts.max_candidate_lines = FLAGS_max_candidate_lines; });
  override_if_set("orthogonality_tolerance_deg", [&] {
    opts.orthogonality_tolerance_deg = FLAGS_orthogonality_tolerance_deg;
  });
  override_if_set("min_secondary_support_ratio", [&] {
    opts.min_secondary_support_ratio = FLAGS_min_secondary_support_ratio;
  });
  LOG(INFO) << "Alignment params source: " << source
            << " | min_line_length=" << opts.min_line_length
            << " max_candidate_lines=" << opts.max_candidate_lines
            << " orthogonality_tolerance_deg="
            << opts.orthogonality_tolerance_deg
            << " min_secondary_support_ratio="
            << opts.min_secondary_support_ratio;
  return opts;
}

void PrintResultLine(
    const bool ok, const std::string& reason,
    const ::cartographer::mapping::StructureAlignmentDiagnostics& diag,
    const double yaw_rad, const bool applied, const double residual_rad) {
  constexpr double kRadToDeg = 180.0 / M_PI;
  std::cout << std::boolalpha << "[ALIGN_RESULT] {"
            << "\"ok\":" << ok
            << ",\"reason\":\"" << reason << "\""
            << ",\"yaw_rad\":" << yaw_rad
            << ",\"yaw_deg\":" << yaw_rad * kRadToDeg
            << ",\"orthogonality_error_deg\":" << diag.orthogonality_error * kRadToDeg
            << ",\"rms_axis_residual_deg\":" << diag.rms_axis_residual * kRadToDeg
            << ",\"primary_support_m\":" << diag.primary_support
            << ",\"secondary_support_m\":" << diag.secondary_support
            << ",\"candidate_lines\":" << diag.candidate_lines
            << ",\"applied\":" << applied
            << ",\"residual_yaw_deg\":" << residual_rad * kRadToDeg
            << ",\"total_area_m2\":" << g_total_area_m2
            << ",\"effective_area_m2\":" << g_effective_area_m2
            << "}" << std::endl;
}

// 资产失效确认门：找出本工具**不会**变换、但会因底图旋转而失效的东西。
// 宁可多报不可少报——漏报的后果是现场标注的点位/路网静默错位且无回滚路径。
//
// 覆盖三种布局：
//   ① 分层（当前布局）：map_filestem = <map>/slam/map，资产在 <map>/assets/、
//      manifest 在 <map>/versions/；
//   ② 遗留单数目录 <map>/asset/（早期数据里出现过，迁移逻辑并不认它，
//      所以更要拦——没人会帮它迁走）；
//   ③ 旧扁平布局（分层之前）：map_filestem = <map>/map，
//      map_assets.json 直接躺在底图旁边。此前这一路完全没有门。
std::vector<std::string> DetectAssetDirs(const std::string& map_filestem) {
  std::vector<std::string> found;
  std::error_code ec;
  const fs::path base_dir = fs::path(map_filestem).parent_path();

  const auto add_if_nonempty_dir = [&](const fs::path& p) {
    if (fs::is_directory(p, ec) && !ec && !fs::is_empty(p, ec) && !ec) {
      found.push_back(p.string());
    }
  };

  if (base_dir.filename() == "slam") {
    // 布局 ①②：资产目录在 slam/ 的同级
    const fs::path map_dir = base_dir.parent_path();
    for (const char* sub : {"assets", "asset", "versions"}) {
      add_if_nonempty_dir(map_dir / sub);
    }
  } else {
    // 布局 ③：扁平，资产文件与底图同级
    if (fs::is_regular_file(base_dir / "map_assets.json", ec) && !ec) {
      found.push_back((base_dir / "map_assets.json").string());
    }
    // 扁平布局下也可能已经手工建了资产子目录
    for (const char* sub : {"assets", "asset", "versions"}) {
      add_if_nonempty_dir(base_dir / sub);
    }
  }
  return found;
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
    PrintResultLine(false, "no_global_static_submap", {}, 0.0, false, 0.0);
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
    PrintResultLine(false, "no_line_features", {}, 0.0, false, 0.0);
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

  const ::cartographer::mapping::StructureAlignmentOptions align_opts =
      LoadAlignmentOptions();

  ::cartographer::mapping::StructureAlignmentDiagnostics diag;
  const auto yaw_opt =
      ::cartographer::mapping::EstimateAlignmentYaw(line_map, align_opts, &diag);
  if (!yaw_opt.has_value()) {
    PrintResultLine(false, "no_reliable_orthogonal_structure", diag, 0.0, false, 0.0);
    LOG(FATAL) << "EstimateAlignmentYaw found no reliable orthogonal "
                  "structure; files were NOT modified.\n"
               << "  Try adjusting --min_line_length / "
                  "--orthogonality_tolerance_deg / "
                  "--min_secondary_support_ratio.";
  }
  const double new_yaw = *yaw_opt;
  LOG(INFO) << "=== Estimated alignment yaw: " << new_yaw << " rad ("
            << new_yaw * 180.0 / M_PI << " deg) ===";

  LOG(INFO) << "    orthogonality error delta=" << diag.orthogonality_error * 180.0 / M_PI
            << " deg, weighted RMS axis residual="
            << diag.rms_axis_residual * 180.0 / M_PI << " deg, candidates="
            << diag.candidate_lines;

  if (dry_run) {
    // 估角成功但 applied=false；residual 填 estimated（尚未施加）
    PrintResultLine(true, "dry_run", diag, new_yaw, /*applied=*/false, new_yaw);
    LOG(INFO) << "Dry run: no file modified. Re-run without --dry_run to "
                 "apply the rotation.";
    return;
  }

  // === Step 3b: 资产失效确认门 ===
  // 放在估角之后、备份之前：估角失败时连这层检查都不必做；
  // 一旦走到这里就意味着真的要改文件了。
  const auto asset_dirs = DetectAssetDirs(map_filestem);
  if (!asset_dirs.empty() && !FLAGS_i_know_assets_will_break) {
    std::string dirs;
    for (const auto& d : asset_dirs) dirs += "\n    " + d;
    PrintResultLine(false, "assets_present_without_confirmation", diag, new_yaw, false,
             new_yaw);
    LOG(FATAL)
        << "Map has assets that this tool does NOT transform:" << dirs << "\n"
        << "  Rotating the base map by " << new_yaw * 180.0 / M_PI
        << " deg would leave every navigation point / route / forbidden area\n"
        << "  at its old coordinates, and backups/ does NOT cover them.\n"
        << "  Use the orchestrated entry instead:\n"
        << "    ros2 service call /command_board/align_map "
           "nav_interfaces/srv/AlignMap \"{map_name: '...'}\"\n"
        << "  which rotates the base map and the assets together and rolls "
           "back on failure.\n"
        << "  To proceed anyway (assets WILL break), pass "
           "--i_know_assets_will_break.\n"
        << "  Files were NOT modified.";
  }
  if (!asset_dirs.empty()) {
    LOG(WARNING) << "Proceeding with --i_know_assets_will_break: "
                 << asset_dirs.size()
                 << " asset director(y|ies) will be left in the OLD frame.";
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

  // === Step 7: 变换 trajectory_data 的 fixed_frame_origin ===
  // （global_static_submap 不在这里变换——它整体重建，见 Step 8）
  size_t n_traj_data = 0;
  for (auto& rec : all_records) {
    if (rec.has_trajectory_data() &&
        rec.trajectory_data().has_fixed_frame_origin_in_map()) {
      auto* td = rec.mutable_trajectory_data();
      const auto old_pose = ::cartographer::transform::ToRigid3(
          td->fixed_frame_origin_in_map());
      *td->mutable_fixed_frame_origin_in_map() =
          ::cartographer::transform::ToProto(T_inv * old_pose);
      ++n_traj_data;
    }
  }
  LOG(INFO) << "SerializedData transformed: " << n_traj_data
            << " trajectory_data fixed_frame_origin";

  // === Step 8: 重建 global_static_submap（栅格 + 点云 + 线特征）===
  // ⭐ 旋转改变的不只是位姿——栅格 cell、点云体素格、线段拟合都锚在
  // 坐标系上，所以三类地图必须在新系从 node 重建，而不是把旧产物刚体搬过去。
  // 只有「无 node 可重建」的退化情形才退回「刚体变换 + 重栅格化」旧路径。
  bool has_nodes = false;
  for (const auto& traj : pose_graph.trajectory()) {
    if (traj.node_size() > 0) { has_nodes = true; break; }
  }
  // 恒定完全重建：不再提供「重栅格化旧 grid」的退路。
  // 退路的唯一好处是保住 miss 射线的 free space，代价却是点云 / 线特征只能做
  // 刚体变换，仍带旧系体素量化与旧栅格的 culling 结论 —— 产物内部不自洽
  // （DEC-003）。留一个会产出不自洽地图的开关，不如不留。
  if (has_nodes) {
    // padding 30m，覆盖典型激光最大距离（与 reset_map_origin_tool 一致）
    *static_submap_proto = mt::RebuildGlobalStaticSubmapFromNodes(
        pose_graph, all_records, traj_options, resolution,
        /*lidar_range_pad=*/30.0);
  } else {
    PrintResultLine(false, "no_nodes_to_rebuild", diag, new_yaw, false, new_yaw);
    LOG(FATAL) << "pose graph has no trajectory nodes; cannot rebuild the map. "
                  "Backups are kept in backups/, no output file has been "
                  "written yet.";
  }
  // === Step 9: 重渲染 map.png + map.yaml（与 node 端 SaveMap 一致）===
  cv::Mat origin_mat;
  Eigen::Vector2d map_origin;
  double render_resolution = resolution;
  mt::RenderSingleStaticSubmap(*static_submap_proto, occ_thresh, free_thresh,
                               &origin_mat, &map_origin, &render_resolution);

  const std::string map_dir = fs::path(map_filestem).parent_path().string();
  // 地图重新生成了，地图信息也要一起重算：面积是 map_manager get_maps 对外暴露的
  // 字段，不重算就会退回 0。口径与建图侧 SaveGridMap 一致 —— effective 必须数
  // **滤波后最终图**的自由像素，所以要把 filtered_out 接出来。
  cv::Mat filtered_mat;
  WriteNavigationMap(origin_mat, map_filestem + ".png",
                     FLAGS_navigation_map_filter_mode, FLAGS_save_debug_map,
                     map_dir, &filtered_mat);

  const std::string yaml_path = map_filestem + ".yaml";
  const GridMapStats map_stats = ComputeGridMapStats(
      origin_mat, filtered_mat, render_resolution, free_thresh);
  // 全量重写 yaml 会丢掉建图侧写过、本工具不认识的键；宁可吵也别静默丢。
  for (const auto& key : FindDroppedYamlKeys(yaml_path, true)) {
    LOG(WARNING) << "map.yaml key dropped by rewrite: '" << key
                 << "' — this tool regenerates map.yaml from scratch and does "
                    "not know this key.";
  }
  // 对齐只是把整张图转了个角，内容没换 ⇒ created_at（内容采集时刻）原样保留。
  const std::string created_at = ReadMapCreatedAt(yaml_path);
  {
    ::cartographer::io::StreamFileWriter yaml_writer(yaml_path);
    WriteYaml(render_resolution, map_origin, map_filestem, &yaml_writer,
              occ_thresh, free_thresh, &map_stats, created_at);
  }
  LOG(INFO) << "Map area recomputed: total " << map_stats.TotalAreaM2()
            << " m2, effective (free) " << map_stats.EffectiveAreaM2()
            << " m2 (" << map_stats.free_cells << "/" << map_stats.total_cells
            << " free pixels)";
  g_total_area_m2 = map_stats.TotalAreaM2();
  g_effective_area_m2 = map_stats.EffectiveAreaM2();

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
  ::cartographer::mapping::StructureAlignmentDiagnostics diag_after;
  const auto residual_opt = ::cartographer::mapping::EstimateAlignmentYaw(
      line_map_after, align_opts, &diag_after);
  double residual = 0.0;
  if (residual_opt.has_value()) {
    residual = *residual_opt;
    LOG(INFO) << "=== Post-transform residual yaw: " << residual
              << " rad (" << residual * 180.0 / M_PI
              << " deg), expected < 1 deg ===";
    // 线特征已在新系重拟合（Step 8），残余角理论下界是 |δ|/2；
    // 超出该下界较多说明重建或变换有问题，值得告警而不是静默。
    const double lower_bound_deg =
        std::abs(diag_after.orthogonality_error) * 0.5 * 180.0 / M_PI;
    const double residual_deg = std::abs(residual) * 180.0 / M_PI;
    if (residual_deg > std::max(1.0, lower_bound_deg + 0.5)) {
      LOG(WARNING) << "Residual " << residual_deg
                   << " deg exceeds the expected bound (max(1.0, |delta|/2 + "
                      "0.5) = "
                   << std::max(1.0, lower_bound_deg + 0.5)
                   << " deg); check the rebuild.";
    }
  } else {
    LOG(WARNING) << "Post-transform re-estimation returned nullopt (structure "
                    "no longer evaluable after rotation — unexpected)";
  }
  // 成功路径：诊断量用施加前的 diag（描述这次对齐做了什么），
  // residual 用施加后的复检值。
  PrintResultLine(true, "applied", diag, new_yaw, /*applied=*/true, residual);
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
