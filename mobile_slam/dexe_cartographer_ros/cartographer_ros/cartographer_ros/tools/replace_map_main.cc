/*
 * replace_map_main.cc
 *
 * 离线工具：用地图 src 的内容「替换」地图 tgt，替换结果保持 tgt 的坐标系。
 *
 * 与 merge 的区别：
 *   - merge 是把 src 追加为 tgt 的一条新轨迹（两图并存）；
 *   - replace 是整体替换——输出里只有 src 的数据，但被刚体变换到 tgt 坐标系，
 *     然后覆盖写入 tgt 目录。适合"用一张新地图刷新旧地图、但保留旧地图坐标系
 *     （其上已有的导航数据/锚点仍按旧坐标系引用）"的场景。
 *
 * 用法：
 *   ros2 run cartographer_ros replace_map_tool \
 *     --map_target <tgt 地图名称或路径> \
 *     --map_source <src 地图名称或路径> \
 *     [--output_dir <输出目录，默认覆盖 tgt 目录>] \
 *     [--transform_mode auto|manual]
 *
 * 变换获取方式（--transform_mode）：
 *   auto（默认）：复用与 merge 相同的全局重定位（在 tgt 的 global_static_submap
 *     上对 src 节点做 FCSM + Ceres），自动求出 T_src_in_tgt。要求两图有重叠区域。
 *   manual：由人工给定 src 原点在 tgt 坐标系中的位姿
 *     （--src_in_tgt_x / _y / _yaw）。默认 0/0/0 即单位阵，
 *     适配"两图从同一物理起点建图且已做结构化对齐"的场景。
 *
 * 渲染参数（--resolution / --occ_thresh / --free_thresh / --navigation_map_filter_mode）：
 *   默认 0/0/0/"(沿用)" 表示自动继承，不显式指定时按"参数是否与 map_assets 相关"继承：
 *     - resolution  ：决定重建的 pbstream 全局栅格的栅格化分辨率，属【内容参数】，
 *                     内容来自 src → 默认继承 src 地图自身的分辨率；
 *     - occ/free    ：只影响 png 渲染阈值与 yaml 阈值字段，不改变 pbstream，
 *                     属【map_assets 参数】 → 默认继承 tgt 的 map.yaml（缺失时退回 src）；
 *     - filter_mode ：只影响 png 后处理，属 map_assets 参数，但 tgt 的 yaml 不落盘该值，
 *                     无法继承，保持命令行默认（可用 --navigation_map_filter_mode 覆盖）。
 *
 * 覆盖与安全：
 *   - 输出会覆盖输出目录（默认即 tgt 目录）下的地图文件；
 *   - --backup 默认开启：覆盖前把所有将被替换的文件（pbstream/png/yaml，
 *     以及开启时的 pcd/highlight.pcd）改名为 *.bak 保留；--backup=false 关闭。
 *   - --save_map_pcd 默认开启：一并重新生成 map.pcd / map_highlight.pcd，
 *     避免残留旧点云与新的栅格/坐标系不一致。
 */

#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <vector>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/tools/map_ops/map_ops.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <rclcpp/rclcpp.hpp>

DEFINE_string(map_target, "",
              "target 地图名称或目录路径（提供目标坐标系 + 默认输出位置）");
DEFINE_string(map_source, "",
              "source 地图名称或目录路径（提供替换内容）");
DEFINE_string(maps_root, "",
              "地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(output_dir, "",
              "输出目录（可选，默认覆盖 target 地图目录）");
DEFINE_string(transform_mode, "auto",
              "变换获取方式：auto=全局匹配自动求；manual=用 --src_in_tgt_* 手动给定");
DEFINE_double(src_in_tgt_x, 0.0,
              "[manual] src 原点在 tgt 坐标系中的 X（米）");
DEFINE_double(src_in_tgt_y, 0.0,
              "[manual] src 原点在 tgt 坐标系中的 Y（米）");
DEFINE_double(src_in_tgt_yaw, 0.0,
              "[manual] src 相对 tgt 的旋转角（弧度）");
DEFINE_double(min_score, 0.45,
              "[auto] scan match 最低接受分数（0~1）");
DEFINE_int32(min_votes, 10,
             "[auto] 连续 scan match 成功帧数达到 N 后触发 Ceres 精配准取均值");
DEFINE_double(resolution, 0.0,
              "输出地图分辨率（米/格）。<=0 表示自动：继承 src 地图的分辨率");
DEFINE_double(occ_thresh, 0.0,
              "占用阈值。<=0 表示自动：继承 tgt 的 map.yaml（缺失时退回 src）");
DEFINE_double(free_thresh, 0.0,
              "空闲阈值。<=0 表示自动：继承 tgt 的 map.yaml（缺失时退回 src）");
DEFINE_string(navigation_map_filter_mode, "normal",
              "导航地图噪声过滤强度（normal/high/super）。tgt yaml 不落盘该值，无法继承");
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

// 覆盖前，把将被替换的文件改名为 *.bak。只备份"本次会写出/覆盖"的文件，
// 避免把不会被覆盖的文件误移走。
void BackupOverwrittenFiles(const std::string& out_pbstream,
                            const std::string& out_filestem,
                            bool save_map_pcd, bool save_origin_map) {
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
  for (const auto& f : files) {
    if (access(f.c_str(), F_OK) != 0) continue;  // 不存在则无需备份
    const std::string bak = f + ".bak";
    CHECK_EQ(std::rename(f.c_str(), bak.c_str()), 0)
        << "Failed to backup " << f << " -> " << bak;
    LOG(INFO) << "Backed up: " << f << " -> " << bak;
  }
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

    const auto src_clouds = map_ops::ExtractNodeClouds(src_data);
    CHECK(!src_clouds.empty()) << "source map has no node point clouds";

    auto [ok, T] = map_ops::FindTransformBinA(tgt_data, src_clouds,
                                              FLAGS_min_score, FLAGS_min_votes);
    CHECK(ok) << "Failed to locate source map in target map. "
                 "Try lowering --min_score, ensuring the maps overlap, or "
                 "use --transform_mode=manual with a known transform.";
    T_src_in_tgt = T;
  } else {  // manual
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(FLAGS_src_in_tgt_yaw, Eigen::Vector3d::UnitZ()));
    T_src_in_tgt = ::cartographer::transform::Rigid3d(
        Eigen::Vector3d(FLAGS_src_in_tgt_x, FLAGS_src_in_tgt_y, 0.0), rotation);
    LOG(INFO) << "Using manual transform (src in tgt frame).";
  }

  LOG(INFO) << "T_src_in_tgt: translation=("
            << T_src_in_tgt.translation().x() << ", "
            << T_src_in_tgt.translation().y() << "), yaw="
            << ::cartographer::transform::GetYaw(T_src_in_tgt);

  // Step 2: 确定渲染参数（未显式指定时按"内容/资产"归属继承）。
  const auto src_yaml = map_ops::ReadMapYamlConfig(dir_src + "/map.yaml");
  const auto tgt_yaml = map_ops::ReadMapYamlConfig(dir_tgt + "/map.yaml");

  double resolution = FLAGS_resolution;
  if (resolution <= 0.0) {
    // resolution 属内容参数：重建栅格的分辨率跟随内容来源（src）。
    if (src_data.global_submap && src_data.global_submap->grid()) {
      resolution = src_data.global_submap->grid()->limits().resolution();
      LOG(INFO) << "Inherited resolution from src global grid: " << resolution;
    } else if (src_yaml.has_resolution) {
      resolution = src_yaml.resolution;
      LOG(INFO) << "Inherited resolution from src map.yaml: " << resolution;
    } else if (tgt_yaml.has_resolution) {
      resolution = tgt_yaml.resolution;
      LOG(INFO) << "Inherited resolution from tgt map.yaml: " << resolution;
    } else {
      resolution = 0.05;
      LOG(WARNING) << "No resolution found, using default 0.05";
    }
  }

  double occ_thresh = FLAGS_occ_thresh;
  if (occ_thresh <= 0.0) {
    // occ/free 属 map_assets 参数：优先继承 tgt 的 yaml，缺失时退回 src。
    if (tgt_yaml.has_occ_thresh) {
      occ_thresh = tgt_yaml.occ_thresh;
    } else if (src_yaml.has_occ_thresh) {
      occ_thresh = src_yaml.occ_thresh;
    } else {
      occ_thresh = 0.65;
      LOG(WARNING) << "No occupied_thresh found, using default 0.65";
    }
    LOG(INFO) << "Using occupied_thresh=" << occ_thresh;
  }

  double free_thresh = FLAGS_free_thresh;
  if (free_thresh <= 0.0) {
    if (tgt_yaml.has_free_thresh) {
      free_thresh = tgt_yaml.free_thresh;
    } else if (src_yaml.has_free_thresh) {
      free_thresh = src_yaml.free_thresh;
    } else {
      free_thresh = 0.196;
      LOG(WARNING) << "No free_thresh found, using default 0.196";
    }
    LOG(INFO) << "Using free_thresh=" << free_thresh;
  }

  // Step 3（默认开启）: 覆盖前备份将被替换的文件。
  if (FLAGS_backup) {
    BackupOverwrittenFiles(out_pbstream, out_filestem, FLAGS_save_map_pcd,
                           FLAGS_save_origin_map);
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

  const std::string dir_tgt =
      ::cartographer_ros::map_ops::ResolveMapDir(FLAGS_map_target, FLAGS_maps_root);
  const std::string dir_src =
      ::cartographer_ros::map_ops::ResolveMapDir(FLAGS_map_source, FLAGS_maps_root);

  const std::string out_dir = FLAGS_output_dir.empty() ? dir_tgt : FLAGS_output_dir;

  LOG(INFO) << "Replace map: source=" << dir_src << " -> target=" << dir_tgt
            << " (output=" << out_dir << ", mode=" << FLAGS_transform_mode
            << ", backup=" << (FLAGS_backup ? "on" : "off") << ")";

  ::cartographer_ros::Run(dir_tgt, dir_src, out_dir);

  rclcpp::shutdown();
  return 0;
}
