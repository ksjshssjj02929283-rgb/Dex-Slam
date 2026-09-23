/*
 * replace_map_main.cc
 *
 * 离线工具：用地图 src 替换 地图 tgt，同时保持 tgt 的坐标系不变，以保持和 tgt
 * 的 map_assets一致
 *
 * 用法：
 *   ros2 run cartographer_ros replace_map_tool \
 *     --map_target <tgt 地图名称或路径> \
 *     --map_source <src 地图名称或路径> \
 *     [--output_dir <输出目录，默认覆盖 tgt 目录>] \
 *     [--transform_mode auto|manual]
 *
 * 变换获取方式（--transform_mode）：
 *   auto（默认）：三阶段匹配管线自动求
 * T_src_in_tgt，要求两图有重叠区域：
 *     1) 从 src 轨迹筛选关键帧（间距 --keyframe_distance /
 * --keyframe_angle，至少 11 帧，不足自动收紧阈值兜底）；
 *     2) BatchFCSM（加速分支定界）对最远点采样的 --initial_frames 帧做全局
 *        匹配 + Ceres 精化，鲁棒聚合（中位数 + 一致性剔除）得初始 delta_T；
 *     3) 按时间序每轮加 --frame_batch 帧，用当前 delta_T 预测位姿做局部
 *        匹配 + Ceres 精化，重新聚合，直到 delta_T 变化小于
 *        --converge_xy / --converge_yaw（或达到帧数/迭代/时间预算上限）。
 *     outlier 四层拒绝：匹配分数门（--min_score / --local_min_score）；
 *     Ceres 收敛性 + 位姿打分宽松下限（--refine_min_score，绝对分数尺度随
 *     地图对变化，仅作健全性门）；L3 聚合一致性剔除 + 相对分数剔除
 *     （--score_relative_margin，防别名帧，尺度自适应）；
 *     内点数护栏（--min_success_frames）。失败时打印观测分数分布与调参建议。
 *   manual：由人工给定 src 原点在 tgt
 * 坐标系中的位姿 （--src_in_tgt_x / _y / _yaw）。默认 0/0/0 即单位阵，
 *     适配"两图从同一物理起点建图且已做结构化对齐"的场景。
 *
 * 地图保存/使用参数（--resolution / --occ_thresh / --free_thresh /
 * --navigation_map_filter_mode）： 这些参数都是"SLAM
 * 保存/使用地图"的配置（与展示资产无关），默认继承 src 地图：
 *     - resolution ：继承 src 的全局栅格分辨率（或 src 的
 * map.yaml）；--resolution 可覆盖；
 *     - occ/free   ：继承 src 的 map.yaml 的 occupied_thresh / free_thresh；
 *                    --occ_thresh / --free_thresh 可覆盖；
 *     - filter_mode：该值是建图节点的运行时 lua 配置，不落盘到地图文件，
 *                    无法从 src 继承，只能手动指定（默认 normal）。
 *
 * 覆盖与安全：
 *   - 输出会覆盖输出目录（默认即 tgt 目录）下的地图文件；
 *   - --backup 默认开启：覆盖前把所有将被替换的文件（pbstream/png/yaml/pcd)
 * 复制到 backup_时间戳 子目录，保留原文件名，便于对比新旧地图；
 * 是 copy 不是 move —— 中途失败时原地图仍在原位可用；
 *   - --save_map_pcd 默认开启：一并重新生成 map.pcd / map_highlight.pcd，
 *     避免残留旧点云与新的栅格/坐标系不一致。
 */

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "absl/memory/memory.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/localization/internal/2d/pose_initializer_options_2d.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros/tools/map_ops/map_ops.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(map_target, "",
              "target 地图名称或目录路径（提供目标坐标系 + 默认输出位置）");
DEFINE_string(map_source, "", "source 地图名称或目录路径（提供替换内容）");
DEFINE_string(maps_root, "", "地图根目录，留空则使用 ~/.dexe/data/maps");
// 匹配阶段的算子参数不自定义，直接读定位节点用的那份 lua —— 本工具的全局重定位
// 与 PoseInitializer 是同一件事，参数必须同源（见 map_ops.h FindTransformOptions）。
DEFINE_string(configuration_directory, "",
              "lua 配置目录，留空则自动解析 cartographer_ros 的 "
              "configuration_files");
DEFINE_string(configuration_basename, "dexe_localization_2d.lua",
              "lua 配置文件名，从中读取 POSE_INITIALIZER_2D 的匹配器参数");
DEFINE_string(output_dir, "", "输出目录（可选，默认覆盖 target 地图目录）");
DEFINE_string(
    transform_mode, "auto",
    "变换获取方式：auto=全局匹配自动求；manual=用 --src_in_tgt_* 手动给定");
DEFINE_double(src_in_tgt_x, 0.0, "[manual] src 原点在 tgt 坐标系中的 X（米）");
DEFINE_double(src_in_tgt_y, 0.0, "[manual] src 原点在 tgt 坐标系中的 Y（米）");
DEFINE_double(src_in_tgt_yaw, 0.0, "[manual] src 相对 tgt 的旋转角（弧度）");
DEFINE_double(min_score, 0.45, "[auto] 全局 BBS 匹配最低接受分数（0~1）");
DEFINE_int32(initial_frames, 10, "[auto] 全局初对齐使用的帧数");
DEFINE_int32(min_success_frames, 5,
             "[auto] 成功匹配帧数低于该值则判定失败");
DEFINE_int32(frame_batch, 5, "[auto] 增量迭代每轮加入的帧数");
DEFINE_int32(max_frames, 100, "[auto] 全程参与匹配的最大帧数");
DEFINE_int32(max_iterations, 20, "[auto] 增量迭代最大轮数");
DEFINE_double(converge_xy, 0.01,
              "[auto] delta_T 收敛阈值：平移变化（米）");
DEFINE_double(converge_yaw, 0.005,
              "[auto] delta_T 收敛阈值：旋转变化（弧度）");
DEFINE_double(local_min_score, 0.30,
              "[auto] 局部匹配最低接受分数（0~1；跨会话分数普遍偏低）");
DEFINE_double(refine_min_score, 0.35,
              "[auto] L2 宽松健全性下限（平均占据概率；绝对尺度随地图对变化，"
              "离群主要交给 L3 一致性/相对分数剔除）");
DEFINE_double(score_relative_margin, 0.25,
              "[auto] L3 相对分数剔除裕量：refine_score 低于（T-内点最高分-该值）"
              "的帧被剔除；<=0 关闭");
DEFINE_double(outlier_translation, 0.5,
              "[auto] 单帧变换偏离中位数上限（米），超出即剔除（outlier L3）");
DEFINE_double(outlier_yaw, 0.1,
              "[auto] 单帧变换偏航偏离中位数上限（弧度），超出即剔除（outlier L3）");
DEFINE_double(keyframe_distance, 2.0, "[auto] 关键帧筛选间距：平移（米）");
DEFINE_double(keyframe_angle, 0.35, "[auto] 关键帧筛选间距：旋转（弧度）");
DEFINE_double(match_budget_seconds, 25.0,
              "[auto] 匹配阶段墙钟预算（秒），保证整体 30s 内完成");
DEFINE_bool(bbs_angular_branching, true,
            "[auto] BatchFCSM 角度金字塔分支（关闭则回退保守的 V1 行为）");
DEFINE_double(resolution, 0.0,
              "输出地图分辨率（米/格）。<=0 表示自动：继承 src 地图的分辨率");
DEFINE_double(
    occ_thresh, 0.0,
    "占用阈值。<=0 表示自动：继承 src 地图 map.yaml 的 occupied_thresh");
DEFINE_double(free_thresh, 0.0,
              "空闲阈值。<=0 表示自动：继承 src 地图 map.yaml 的 free_thresh");
DEFINE_string(
    navigation_map_filter_mode, "normal",
    "导航地图噪声过滤强度（normal/high/super）。注意：该值是建图节点的运行时 "
    "lua 配置，不会落盘到地图文件，因此无法从 src 地图继承，只能在此手动指定");
DEFINE_bool(save_origin_map, false,
            "是否同时保存未过滤的原始地图为 *_origin.png");
DEFINE_bool(save_debug_map, false, "是否保存噪声过滤的中间调试图");
DEFINE_bool(save_map_pcd, true,
            "是否额外输出 map.pcd / map_highlight.pcd（来自变换后的全局点云）。"
            "默认开启，避免残留旧点云");
DEFINE_bool(backup, true,
            "覆盖前把所有将被替换的地图文件复制到 backup_<时间戳>/ 保留。默认开启");

namespace cartographer_ros {
namespace {

std::string MakeTimestamp() {
  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  localtime_r(&now, &tm_buf);
  char ts[32];
  std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm_buf);
  return std::string(ts);
}

std::string Basename(const std::string& path) {
  const size_t pos = path.find_last_of('/');
  return (pos == std::string::npos) ? path : path.substr(pos + 1);
}

// ──────────────────────────────────────────────────
// lua 配置读取
// ──────────────────────────────────────────────────
// 与 pose_initializer_benchmark_main.cc / bbs_benchmark_main.cc 同一套惯例：
// 读 POSE_INITIALIZER_2D，因为本工具的全局重定位与定位节点的 PoseInitializer
// 是同一件事。自定义一份参数只会让两边随时间漂移开。
// ⚠️ ConfigurationFileResolver 找不到文件时走 LOG(FATAL) 而不是抛异常，
// try/catch 拦不住，进程会直接 core dump 且来不及打结果行。所以必须**先自己
// 确认文件存在**，把「配置缺失」变成一个有结果行的、可解析的失败。
// 返回解析好的配置目录；文件不存在时返回空串（调用方负责报错退出）。
std::string ResolveLuaConfigDir() {
  std::string config_dir = FLAGS_configuration_directory;
  if (config_dir.empty()) {
    try {
      config_dir = ament_index_cpp::get_package_share_directory(
                       "cartographer_ros") +
                   "/configuration_files";
    } catch (const std::exception& e) {
      LOG(ERROR) << "Cannot auto-resolve cartographer_ros share dir: "
                 << e.what();
      return "";
    }
  }
  const std::string full = config_dir + "/" + FLAGS_configuration_basename;
  if (access(full.c_str(), R_OK) != 0) {
    LOG(ERROR) << "Lua configuration not readable: " << full;
    return "";
  }
  return config_dir;
}

::cartographer::localization::proto::PoseInitializerOptions2D
LoadPoseInitializerOptions(const std::string& config_dir) {
  auto resolver =
      absl::make_unique<::cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{config_dir});
  const std::string code = "include \"" + FLAGS_configuration_basename +
                           "\"\nreturn POSE_INITIALIZER_2D";
  auto lua =
      ::cartographer::common::LuaParameterDictionary::NonReferenceCounted(
          code, std::move(resolver));
  const auto options =
      ::cartographer::localization::CreatePoseInitializerOptions2D(lua.get());
  LOG(INFO) << "Loaded matcher params from lua: " << config_dir << "/"
            << FLAGS_configuration_basename << " (POSE_INITIALIZER_2D)";
  return options;
}

// ──────────────────────────────────────────────────
// 结构化结果行
// ──────────────────────────────────────────────────
// 与 align_map_tool 的 [ALIGN_RESULT] 同款：往 stdout 打一行 JSON，供
// command_board 通过 popen 解析。**任何退出路径都必须先打这一行**，包括
// LOG(FATAL) 之前 —— 否则编排方只能看到进程非零退出，拿不到失败原因。
struct ReplaceResult {
  bool ok = false;
  std::string reason = "not_run";
  double x = 0.0, y = 0.0, yaw_rad = 0.0;
  std::string transform_mode;
  int global_matched = 0, local_matched = 0, inliers = 0;
  int rejected_l1 = 0, rejected_l2 = 0, rejected_l3 = 0;
  bool converged = false, degraded = false;
  std::string stop_reason;
  double rms_translation_m = 0.0, rms_yaw_rad = 0.0;
  double elapsed_seconds = 0.0;
  double resolution = 0.0;
  bool align_evaluated = false;
  double align_residual_yaw_deg = 0.0;
  double total_area_m2 = 0.0, effective_area_m2 = 0.0;  // 重算后写进 map.yaml
  double src_known_area_m2 = 0.0, tgt_known_area_m2 = 0.0;
  bool coverage_warning = false;
  std::string backup_dir;
  std::string matcher_params_source = "n/a";  // lua / builtin_fallback / n/a
  bool applied = false;
};

ReplaceResult g_result;

void PrintResultLine() {
  constexpr double kRadToDeg = 180.0 / M_PI;
  const auto& r = g_result;
  std::cout << std::boolalpha << "[REPLACE_RESULT] {"
            << "\"ok\":" << r.ok
            << ",\"reason\":\"" << r.reason << "\""
            << ",\"transform_mode\":\"" << r.transform_mode << "\""
            << ",\"x\":" << r.x << ",\"y\":" << r.y
            << ",\"yaw_rad\":" << r.yaw_rad
            << ",\"yaw_deg\":" << r.yaw_rad * kRadToDeg
            << ",\"global_matched\":" << r.global_matched
            << ",\"local_matched\":" << r.local_matched
            << ",\"inliers\":" << r.inliers
            << ",\"rejected_l1\":" << r.rejected_l1
            << ",\"rejected_l2\":" << r.rejected_l2
            << ",\"rejected_l3\":" << r.rejected_l3
            << ",\"converged\":" << r.converged
            << ",\"degraded\":" << r.degraded
            << ",\"stop_reason\":\"" << r.stop_reason << "\""
            << ",\"inlier_rms_xy_m\":" << r.rms_translation_m
            << ",\"inlier_rms_yaw_deg\":" << r.rms_yaw_rad * kRadToDeg
            << ",\"match_seconds\":" << r.elapsed_seconds
            << ",\"resolution\":" << r.resolution
            << ",\"align_evaluated\":" << r.align_evaluated
            << ",\"align_residual_yaw_deg\":" << r.align_residual_yaw_deg
            << ",\"total_area_m2\":" << r.total_area_m2
            << ",\"effective_area_m2\":" << r.effective_area_m2
            << ",\"src_known_area_m2\":" << r.src_known_area_m2
            << ",\"tgt_known_area_m2\":" << r.tgt_known_area_m2
            << ",\"coverage_warning\":" << r.coverage_warning
            << ",\"backup_dir\":\"" << r.backup_dir << "\""
            << ",\"matcher_params_source\":\"" << r.matcher_params_source << "\""
            << ",\"applied\":" << r.applied
            << "}" << std::endl;
}

// 在 out_dir 下创建带时间戳的备份子目录；同一秒多次运行时追加序号避免冲突。
std::string CreateBackupDir(const std::string& out_dir) {
  const std::string stamp = MakeTimestamp();
  std::string dir = out_dir + "/backup_" + stamp;
  int suffix = 1;
  while (::mkdir(dir.c_str(), 0755) != 0) {
    CHECK(errno == EEXIST) << "Failed to create backup dir: " << dir;
    dir = out_dir + "/backup_" + stamp + "_" + std::to_string(++suffix);
  }
  return dir;
}

// 覆盖前，把将被替换的文件【保留原文件名】**复制**到时间戳备份子目录
// （如 <out_dir>/backup_20260827_163045/map.pbstream）。
// 这样替换前后文件名一致、分别位于新旧两个目录，便于直接对比图片/文档。
// 只备份"本次会写出/覆盖"的文件，避免把不会被覆盖的文件误备份。
//
// ⚠️ 必须是 copy 而不是 move：备份发生在变换与重写之前，而后者有大量
// CHECK 退出点。若用 move，任何一处中途失败都会让目标目录**一个地图文件
// 都不剩**（全在 backup 子目录里），需要人工搬回才能恢复可用状态。
// copy 的代价是一次磁盘拷贝，换取「失败时原地图原封不动」。
// 与 align 链路的 map_transform::BackupMapFiles 语义保持一致。
void BackupOverwrittenFiles(const std::string& out_dir,
                            const std::string& out_pbstream,
                            const std::string& out_filestem, bool save_map_pcd,
                            bool save_origin_map) {
  std::vector<std::string> files = {
      out_pbstream,
      out_filestem + ".png",
      out_filestem + ".yaml",
  };
  if (save_map_pcd) {
    files.push_back(out_filestem + ".pcd");
    files.push_back(out_filestem + "_highlight.pcd");
  }
  if (save_origin_map) {
    files.push_back(out_filestem + "_origin.png");
  }

  // 仅保留实际存在的文件。
  std::vector<std::string> existing;
  for (const auto& f : files) {
    if (access(f.c_str(), F_OK) == 0) existing.push_back(f);
  }
  if (existing.empty()) {
    LOG(INFO) << "No existing map files to backup in " << out_dir;
    return;
  }

  const std::string backup_dir = CreateBackupDir(out_dir);
  g_result.backup_dir = backup_dir;
  namespace fs = std::filesystem;
  for (const auto& f : existing) {
    const std::string dst = backup_dir + "/" + Basename(f);
    std::error_code ec;
    fs::copy_file(f, dst, fs::copy_options::overwrite_existing, ec);
    CHECK(!ec) << "Failed to backup " << f << " -> " << dst << ": "
               << ec.message();
    LOG(INFO) << "Backed up: " << f << " -> " << dst;
  }
  LOG(INFO) << "Backup saved to: " << backup_dir << "  (compare new '"
            << out_filestem << ".png' with old '" << backup_dir << "/map.png')";
}

void Run(const std::string& dir_tgt, const std::string& dir_src,
         const std::string& out_dir) {
  const std::string pbstream_tgt = dir_tgt + "/map.pbstream";
  const std::string pbstream_src = dir_src + "/map.pbstream";
  const std::string out_pbstream = out_dir + "/map.pbstream";
  const std::string out_filestem = out_dir + "/map";

  // Step 1: 加载 src（完整，待变换）与（auto 模式）tgt（提供匹配用全局地图）。
  LOG(INFO) << "Loading source map: " << pbstream_src;
  map_ops::MapData src_data = map_ops::LoadMap(pbstream_src,
                                               /*build_slices=*/false);

  ::cartographer::transform::Rigid3d T_src_in_tgt;
  g_result.transform_mode = FLAGS_transform_mode;
  if (FLAGS_transform_mode == "auto") {
    LOG(INFO) << "Loading target map: " << pbstream_tgt;
    map_ops::MapData tgt_data = map_ops::LoadMap(pbstream_tgt,
                                                 /*build_slices=*/false);

    // 覆盖范围护栏（DEC-012）：替换的语义是「整场重走 → 全量覆盖」，所以 src
    // 的实际覆盖不应明显小于 tgt。小了通常意味着**这一趟没走完**，而全量覆盖
    // 会把没走到的区域直接丢掉。只告警不阻断 —— 走廊类场地本就可能差异较大，
    // 拿它当硬门会误杀。
    if (src_data.global_submap && src_data.global_submap->grid() &&
        tgt_data.global_submap && tgt_data.global_submap->grid()) {
      g_result.src_known_area_m2 =
          map_ops::EstimateKnownAreaM2(*src_data.global_submap->grid());
      g_result.tgt_known_area_m2 =
          map_ops::EstimateKnownAreaM2(*tgt_data.global_submap->grid());
      constexpr double kCoverageWarnRatio = 0.8;
      const double ratio = g_result.tgt_known_area_m2 > 0.0
                               ? g_result.src_known_area_m2 /
                                     g_result.tgt_known_area_m2
                               : 1.0;
      LOG(INFO) << "Coverage: src " << g_result.src_known_area_m2
                << " m2 vs tgt " << g_result.tgt_known_area_m2
                << " m2 (ratio " << ratio << ")";
      if (ratio < kCoverageWarnRatio) {
        g_result.coverage_warning = true;
        LOG(WARNING) << "Source map covers only " << ratio * 100
                     << "% of the target map's known area. Replacement is a "
                        "FULL overwrite — any area missing from the source "
                        "will be lost. Did the re-survey finish?";
      }
    }

    map_ops::KeyframeSelectionOptions sel_options;
    sel_options.max_distance_meters = FLAGS_keyframe_distance;
    sel_options.max_angle_radians = FLAGS_keyframe_angle;
    const auto keyframes = map_ops::SelectKeyframes(src_data, sel_options);
    const auto src_clouds = map_ops::ExtractNodeClouds(src_data, keyframes);
    if (src_clouds.empty()) {
      g_result.reason = "src_no_node_clouds";
      PrintResultLine();
      LOG(FATAL) << "source map has no node point clouds";
    }

    map_ops::FindTransformOptions align_options;

    // 匹配器参数从 lua 继承（与定位节点同源）；命令行 flag 只在**显式指定**时
    // 覆盖，未指定则以 lua 为准 —— 否则 flag 的默认值会静默盖掉配置文件。
    // 配置缺失直接失败，**不静默回退到内置默认值** —— 那组值与生产配置差
    // 1~2 个数量级且不含 improved line cost，用它算出来的 T 看着合理却与定位
    // 侧不是同一套算法，比明确失败危险得多。
    const std::string config_dir = ResolveLuaConfigDir();
    if (config_dir.empty()) {
      g_result.reason = "lua_config_not_found";
      PrintResultLine();
      LOG(FATAL) << "Matcher parameters must come from the same lua config the "
                    "localization node uses. Pass --configuration_directory / "
                    "--configuration_basename (current basename: "
                 << FLAGS_configuration_basename << ").";
    }
    const auto pose_init = LoadPoseInitializerOptions(config_dir);
    align_options.has_matcher_options = true;
    align_options.ceres_options = pose_init.ceres_scan_matcher_options();
    align_options.line_matcher_options = pose_init.line_matcher_options();
    align_options.branch_and_bound_depth =
        pose_init.fast_correlative_scan_matcher_options()
            .branch_and_bound_depth();
    g_result.matcher_params_source =
        config_dir + "/" + FLAGS_configuration_basename;
    if (google::GetCommandLineFlagInfoOrDie("min_score").is_default) {
      align_options.min_score = pose_init.global_localization_min_score();
      LOG(INFO) << "min_score inherited from lua: " << align_options.min_score;
    } else {
      align_options.min_score = FLAGS_min_score;
      LOG(INFO) << "min_score overridden by flag: " << align_options.min_score;
    }
    align_options.initial_frames = FLAGS_initial_frames;
    align_options.min_success_frames = FLAGS_min_success_frames;
    align_options.frames_per_step = FLAGS_frame_batch;
    align_options.max_iterations = FLAGS_max_iterations;
    align_options.max_frames = FLAGS_max_frames;
    align_options.local_min_score = FLAGS_local_min_score;
    align_options.converge_translation = FLAGS_converge_xy;
    align_options.converge_yaw = FLAGS_converge_yaw;
    align_options.refine_min_score = FLAGS_refine_min_score;
    align_options.outlier_translation = FLAGS_outlier_translation;
    align_options.outlier_yaw = FLAGS_outlier_yaw;
    align_options.score_relative_margin = FLAGS_score_relative_margin;
    align_options.budget_seconds = FLAGS_match_budget_seconds;
    align_options.bbs_angular_branching = FLAGS_bbs_angular_branching;

    const auto result =
        map_ops::FindTransformBinA(tgt_data, src_clouds, align_options);
    g_result.global_matched = result.global_matched;
    g_result.local_matched = result.local_matched;
    g_result.inliers = result.inliers;
    g_result.rejected_l1 = result.rejected_l1;
    g_result.rejected_l2 = result.rejected_l2;
    g_result.rejected_l3 = result.rejected_l3;
    g_result.converged = result.converged;
    g_result.degraded = result.degraded;
    g_result.stop_reason = result.stop_reason;
    g_result.rms_translation_m = result.rms_translation_m;
    g_result.rms_yaw_rad = result.rms_yaw_rad;
    g_result.elapsed_seconds = result.elapsed_seconds;
    if (!result.success) {
      g_result.reason = "match_failed_" + result.stop_reason;
      PrintResultLine();
      LOG(FATAL) << "Failed to locate source map in target map ("
                 << result.stop_reason << "). Try lowering --min_score / "
                    "--refine_min_score, ensuring the maps overlap, or use "
                    "--transform_mode=manual with a known transform.";
    }
    // success 表示「内点够、T 可用」，不表示「已收敛」。未收敛时 T 仍然写出，
    // 但必须让调用方看见，由它决定接受还是重跑 —— 不把两件事混成一个布尔。
    if (result.degraded) {
      LOG(WARNING) << "Transform is USABLE BUT NOT CONVERGED (stop_reason="
                   << result.stop_reason << ", inlier RMS "
                   << result.rms_translation_m << " m / "
                   << result.rms_yaw_rad << " rad). Review before trusting it.";
    }
    T_src_in_tgt = result.T_src_in_tgt;
  } else {  // manual
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(FLAGS_src_in_tgt_yaw, Eigen::Vector3d::UnitZ()));
    T_src_in_tgt = ::cartographer::transform::Rigid3d(
        Eigen::Vector3d(FLAGS_src_in_tgt_x, FLAGS_src_in_tgt_y, 0.0), rotation);
    LOG(INFO) << "Using manual transform (src in tgt frame).";
  }

  LOG(INFO) << "T_src_in_tgt: translation=(" << T_src_in_tgt.translation().x()
            << ", " << T_src_in_tgt.translation().y()
            << "), yaw=" << ::cartographer::transform::GetYaw(T_src_in_tgt);
  g_result.x = T_src_in_tgt.translation().x();
  g_result.y = T_src_in_tgt.translation().y();
  g_result.yaw_rad = ::cartographer::transform::GetYaw(T_src_in_tgt);

  // Step 2: 确定地图保存/使用参数。这些都属于"SLAM 保存/使用地图"的配置，
  // 内容来自 src，未显式指定时默认继承 src 地图（可用对应 --flag 手动覆盖）。
  const auto src_yaml = map_ops::ReadMapYamlConfig(dir_src + "/map.yaml");

  double resolution = FLAGS_resolution;
  if (resolution <= 0.0) {
    // 这些参数都属于"SLAM 保存/使用地图"的配置，内容来自 src → 默认继承 src。
    if (src_data.global_submap && src_data.global_submap->grid()) {
      resolution = src_data.global_submap->grid()->limits().resolution();
      LOG(INFO) << "Inherited resolution from src global grid: " << resolution;
    } else if (src_yaml.has_resolution) {
      resolution = src_yaml.resolution;
      LOG(INFO) << "Inherited resolution from src map.yaml: " << resolution;
    } else {
      resolution = 0.05;
      LOG(WARNING) << "No resolution found in src, using default 0.05";
    }
  }
  g_result.resolution = resolution;

  // 分辨率跟 src 走（内容来自 src），但坐标系是 tgt 的 —— 于是「同一张地图」
  // 的分辨率可能在替换后静默改变。下游按新写出的 map.yaml 走没问题，但缓存过
  // 该图的组件需要重新加载，所以这里必须明确说出来，而不是只在 INFO 里一闪而过。
  {
    const auto tgt_yaml = map_ops::ReadMapYamlConfig(dir_tgt + "/map.yaml");
    if (tgt_yaml.has_resolution &&
        std::abs(tgt_yaml.resolution - resolution) > 1e-9) {
      LOG(WARNING) << "Map resolution will CHANGE from " << tgt_yaml.resolution
                   << " (target) to " << resolution
                   << " (inherited from source). Consumers that cached this "
                      "map must reload it. Pass --resolution to override.";
    }
  }

  double occ_thresh = FLAGS_occ_thresh;
  if (occ_thresh <= 0.0) {
    if (src_yaml.has_occ_thresh) {
      occ_thresh = src_yaml.occ_thresh;
      LOG(INFO) << "Inherited occupied_thresh from src map.yaml: "
                << occ_thresh;
    } else {
      occ_thresh = 0.65;
      LOG(WARNING) << "No occupied_thresh in src, using default 0.65";
    }
  }

  double free_thresh = FLAGS_free_thresh;
  if (free_thresh <= 0.0) {
    if (src_yaml.has_free_thresh) {
      free_thresh = src_yaml.free_thresh;
      LOG(INFO) << "Inherited free_thresh from src map.yaml: " << free_thresh;
    } else {
      free_thresh = 0.196;
      LOG(WARNING) << "No free_thresh in src, using default 0.196";
    }
  }

  // Step 3（默认开启）: 覆盖前备份将被替换的文件。
  if (FLAGS_backup) {
    BackupOverwrittenFiles(out_dir, out_pbstream, out_filestem,
                           FLAGS_save_map_pcd, FLAGS_save_origin_map);
  }

  // Step 4: 施加变换并写出（覆盖输出目录）。
  map_ops::RewriteOptions opts;
  opts.resolution = resolution;
  opts.occ_thresh = occ_thresh;
  opts.free_thresh = free_thresh;
  opts.navigation_map_filter_mode = FLAGS_navigation_map_filter_mode;
  opts.save_origin_map = FLAGS_save_origin_map;
  opts.save_debug_map = FLAGS_save_debug_map;
  opts.save_map_pcd = FLAGS_save_map_pcd;
  // 替换 = 整场重走后的全量覆盖：内容整体来自 src ⇒ created_at（内容采集时刻）
  // 取 **src 的**。不取 tgt 的（那是被换掉的老内容），也不取"现在"（那是文件
  // 修改时刻，已由 map.yaml 的 mtime / MapInfo.updated_at 表达）。
  opts.created_at = ReadMapCreatedAt(dir_src + "/map.yaml");
  if (!opts.created_at.empty()) {
    LOG(INFO) << "created_at inherited from source map: " << opts.created_at;
  }

  map_ops::RewriteStats rewrite_stats;
  map_ops::TransformAndRewriteMap(std::move(src_data), T_src_in_tgt,
                                  out_pbstream, out_filestem, opts,
                                  &rewrite_stats);

  constexpr double kRadToDeg = 180.0 / M_PI;
  g_result.total_area_m2 = rewrite_stats.total_area_m2;
  g_result.effective_area_m2 = rewrite_stats.effective_area_m2;
  g_result.align_evaluated = rewrite_stats.align_evaluated;
  g_result.align_residual_yaw_deg =
      rewrite_stats.align_residual_yaw_rad * kRadToDeg;
  g_result.applied = true;
  g_result.ok = true;
  g_result.reason = "applied";

  // DEC-013：替换后只检查对齐度，不做对齐。残余大说明 target 坐标系本身没对齐，
  // 需要的话单独跑一次 align_map_tool（它会把底图与资产一起转），而不是把对齐
  // 塞进替换流程 —— 那会破坏「替换不动资产坐标」这个不变量。
  constexpr double kAlignResidualWarnDeg = 1.0;
  if (rewrite_stats.align_evaluated &&
      std::abs(g_result.align_residual_yaw_deg) > kAlignResidualWarnDeg) {
    LOG(WARNING) << "Replaced map has structure-alignment residual "
                 << g_result.align_residual_yaw_deg << " deg (> "
                 << kAlignResidualWarnDeg
                 << "). The target coordinate frame is likely not aligned. "
                    "Run align_map_tool separately if you want it aligned — "
                    "replacement deliberately does not realign.";
  }

  LOG(INFO) << "Done. Target map replaced with source content in the target "
               "coordinate frame.";
  PrintResultLine();
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  FLAGS_alsologtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_map_target.empty()) << "--map_target is required";
  CHECK(!FLAGS_map_source.empty()) << "--map_source is required";
  CHECK(FLAGS_transform_mode == "auto" || FLAGS_transform_mode == "manual")
      << "--transform_mode must be 'auto' or 'manual', got: "
      << FLAGS_transform_mode;

  const std::string dir_tgt = ::cartographer_ros::map_ops::ResolveMapDir(
      FLAGS_map_target, FLAGS_maps_root);
  const std::string dir_src = ::cartographer_ros::map_ops::ResolveMapDir(
      FLAGS_map_source, FLAGS_maps_root);

  const std::string out_dir =
      FLAGS_output_dir.empty() ? dir_tgt : FLAGS_output_dir;

  LOG(INFO) << "Replace map: source=" << dir_src << " -> target=" << dir_tgt
            << " (output=" << out_dir << ", mode=" << FLAGS_transform_mode
            << ", backup=" << (FLAGS_backup ? "on" : "off") << ")";

  ::cartographer_ros::Run(dir_tgt, dir_src, out_dir);

  rclcpp::shutdown();
  return 0;
}
