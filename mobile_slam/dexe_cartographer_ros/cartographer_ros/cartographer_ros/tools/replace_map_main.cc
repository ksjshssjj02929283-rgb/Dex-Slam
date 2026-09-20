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
 * 移入到 backup_时间戳 子目录， 保留原文件名，便于对比新旧地图；
 *   - --save_map_pcd 默认开启：一并重新生成 map.pcd / map_highlight.pcd，
 *     避免残留旧点云与新的栅格/坐标系不一致。
 */

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/tools/map_ops/map_ops.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(map_target, "",
              "target 地图名称或目录路径（提供目标坐标系 + 默认输出位置）");
DEFINE_string(map_source, "", "source 地图名称或目录路径（提供替换内容）");
DEFINE_string(maps_root, "", "地图根目录，留空则使用 ~/.dexe/data/maps");
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
            "覆盖前把所有将被替换的地图文件改名为 *.bak 保留。默认开启");

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

// 覆盖前，把将被替换的文件【保留原文件名】移入时间戳备份子目录
// （如 <out_dir>/backup_20260827_163045/map.pbstream）。
// 这样替换前后文件名一致、分别位于新旧两个目录，便于直接对比图片/文档。
// 只备份"本次会写出/覆盖"的文件，避免把不会被覆盖的文件误移走。
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
  for (const auto& f : existing) {
    const std::string dst = backup_dir + "/" + Basename(f);
    CHECK_EQ(std::rename(f.c_str(), dst.c_str()), 0)
        << "Failed to backup " << f << " -> " << dst;
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
  if (FLAGS_transform_mode == "auto") {
    LOG(INFO) << "Loading target map: " << pbstream_tgt;
    map_ops::MapData tgt_data = map_ops::LoadMap(pbstream_tgt,
                                                 /*build_slices=*/false);

    map_ops::KeyframeSelectionOptions sel_options;
    sel_options.max_distance_meters = FLAGS_keyframe_distance;
    sel_options.max_angle_radians = FLAGS_keyframe_angle;
    const auto keyframes = map_ops::SelectKeyframes(src_data, sel_options);
    const auto src_clouds = map_ops::ExtractNodeClouds(src_data, keyframes);
    CHECK(!src_clouds.empty()) << "source map has no node point clouds";

    map_ops::FindTransformOptions align_options;
    align_options.min_score = FLAGS_min_score;
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
    CHECK(result.success)
        << "Failed to locate source map in target map ("
        << result.stop_reason << "). Try lowering --min_score / "
           "--refine_min_score, ensuring the maps overlap, or use "
           "--transform_mode=manual with a known transform.";
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

  map_ops::TransformAndRewriteMap(std::move(src_data), T_src_in_tgt,
                                  out_pbstream, out_filestem, opts);

  LOG(INFO) << "Done. Target map replaced with source content in the target "
               "coordinate frame.";
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
