/*
 * pose_initializer_benchmark_main.cc
 *
 * PoseInitializer 端到端全局重定位离线基准测试工具。
 *
 * 测的是整条全局重定位流水线（global matching → online correlative refine →
 * line tracking → ceres refine → validation）的耗时，而非单独的全图匹配阶段
 *（后者见 bbs_benchmark_tool）。复用与线上一致的地图、lua 配置与预处理。
 *
 * 三版本对比：original / cpu / gpu（global_matcher_type），各跑同一批帧。
 * 双数据源：synth（地图自由空间合成 scan，带真值）+ bag（真实回放）。
 * 计时边界：地图与匹配器准备（含 ikd-tree 重建、map cache 构建）在计时外，
 *           只计端到端 5 个阶段——与线上语义一致（submap/matcher 预先就绪）。
 *
 * 输出：~/.dexe/localization/poseinit_baseline/<地图名>_<时间戳>_<类型>_<模式>.txt
 *
 * 用法示例：
 *   ros2 run cartographer_ros pose_initializer_benchmark_tool \
 *     --map_name <地图名> --mode synth --num_poses 30
 *   ros2 run cartographer_ros pose_initializer_benchmark_tool \
 *     --map_name <地图名> --mode bag --bag_filename <bag目录> --bag_random_samples 15
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/localization/internal/2d/pose_initializer_2d.h"
#include "cartographer/localization/internal/2d/pose_initializer_options_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/preprocess/data_preprocessor_options.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cartographer_ros/tools/bbs_benchmark/benchmark_common.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

// --- 地图与配置 ---
DEFINE_string(map_name, "", "地图名称，解析为 <maps_root>/<map_name>/map.pbstream");
DEFINE_string(maps_root, "", "地图根目录，留空则使用 ~/.dexe/data/maps");
DEFINE_string(pbstream_filename, "", "直接指定 pbstream 文件路径（优先于 map_name）");
DEFINE_string(configuration_directory, "",
              "lua 配置目录覆盖，默认自动解析本包 share/configuration_files");
DEFINE_string(configuration_basename, "dexe_localization_2d.lua",
              "线上定位 lua 文件名");

// --- 版本与模式 ---
DEFINE_string(matcher_types, "original,cpu,gpu",
              "对比的 global_matcher_type，逗号分隔：original|cpu|gpu"
              "（gpu 需 BUILD_GPU_MATCHER=ON，否则自动跳过）");
DEFINE_string(mode, "synth", "synth | bag | both");

// --- 合成模式参数 ---
DEFINE_int32(num_poses, 30, "合成模式：随机撒位姿数量");
DEFINE_uint64(seed, 42, "随机种子（保证可复现）");
DEFINE_int32(beams, 2400, "合成 scan 束数");
DEFINE_double(fov_deg, 360.0, "合成 scan 视场角（度）");
DEFINE_double(max_range, 20.0, "合成 scan 最大量程（米）");
DEFINE_double(range_noise_sigma, 0.01, "合成 range 高斯噪声 σ（米）");
DEFINE_double(min_valid_beam_ratio, 0.3, "有效回波束数下限比例");
DEFINE_double(success_trans_tol, 0.2, "成功判定：平移误差阈值（米）");
DEFINE_double(success_yaw_tol_deg, 3.0, "成功判定：角度误差阈值（度）");

// --- bag 模式参数 ---
DEFINE_string(bag_filename, "", "rosbag2 目录路径");
DEFINE_string(scan_topic, "/scan", "单雷达 LaserScan 话题名（scan_topics 置空时使用）");
DEFINE_string(scan_topics, "/laser_scan/front,/laser_scan/rear",
              "多雷达话题，逗号分隔（默认 W1 双雷达）。第一个为主雷达");
DEFINE_string(tracking_frame, "base_footprint", "多雷达合并目标坐标系");
DEFINE_string(scan_extrinsics, "",
              "手动外参覆盖 \"x,y,z,roll,pitch,yaw;...\"（米/度），"
              "留空则从 /tf_static 解析");
DEFINE_double(pair_tolerance, 0.1, "副雷达与主雷达帧时间配对最大容差（秒）");
DEFINE_int32(bag_stride, 20, "每 N 帧取 1 帧（bag_random_samples>0 时无效）");
DEFINE_int32(max_bag_scans, 30, "stride 抽帧模式最多测试帧数");
DEFINE_int32(bag_random_samples, 0, ">0 时启用时间随机抽帧");

// --- 输出 ---
DEFINE_string(output_dir, "",
              "报告输出目录，留空则使用 ~/.dexe/localization/poseinit_baseline/");

namespace {

namespace carto = ::cartographer;
namespace bc = ::cartographer_ros::benchmark_common;
namespace loc = ::cartographer::localization;

// 一帧基准输入。
struct Frame {
  carto::sensor::PointCloud cloud;
  std::vector<carto::sensor::LineData> line_data;
  bool has_gt = false;
  carto::transform::Rigid2d gt_pose;
};

// 单次端到端测量记录。
struct Rec {
  std::string type;
  std::string mode;
  int id = 0;
  loc::GlobalRelocInfo profile;
  bool success = false;       // synth：matched 且误差在阈值内；bag：=matched
  double trans_err = NAN, yaw_err = NAN;
};

std::string ResolvePbstreamPath() {
  if (!FLAGS_pbstream_filename.empty()) return FLAGS_pbstream_filename;
  CHECK(!FLAGS_map_name.empty())
      << "需要 --map_name 或 --pbstream_filename 指定地图";
  std::string root = FLAGS_maps_root;
  if (root.empty()) {
    const char* home = std::getenv("HOME");
    root = std::string(home ? home : "/tmp") + "/.dexe/data/maps";
  }
  return root + "/" + FLAGS_map_name + "/map.pbstream";
}

// 从线上 lua 加载完整 PoseInitializerOptions2D + 预处理参数。
struct LuaConfig {
  carto::preprocess::proto::DataPreprocessorOptions dp_options;
  loc::proto::PoseInitializerOptions2D pose_init_options;
  double min_score = 0.;
};

LuaConfig LoadLuaConfig() {
  std::string config_dir = FLAGS_configuration_directory;
  if (config_dir.empty()) {
    config_dir =
        ament_index_cpp::get_package_share_directory("cartographer_ros") +
        "/configuration_files";
    LOG(INFO) << "lua config dir (auto-resolved): " << config_dir;
  }
  auto resolver = absl::make_unique<carto::common::ConfigurationFileResolver>(
      std::vector<std::string>{config_dir});
  const std::string code =
      "include \"" + FLAGS_configuration_basename + "\"\n" +
      "return { data_preprocessor = POSE_ESTIMATOR_2D.data_preprocessor, " +
      "pose_initializer = POSE_INITIALIZER_2D }";
  auto lua = carto::common::LuaParameterDictionary::NonReferenceCounted(
      code, std::move(resolver));
  LuaConfig c;
  c.dp_options = carto::preprocess::CreateDataPreprocessorOptions(
      lua->GetDictionary("data_preprocessor").get());
  c.pose_init_options = loc::CreatePoseInitializerOptions2D(
      lua->GetDictionary("pose_initializer").get());
  c.min_score = c.pose_init_options.global_localization_min_score();
  // 离线基准约束：无里程计 → 关去畸变；line 特征不参与（synth 无线特征，
  // bag 亦统一关闭以保证两模式预处理一致、可复现）。
  c.dp_options.set_use_odometry(false);
  c.dp_options.set_use_deskew(false);
  c.dp_options.set_enable_feature_detection(false);
  LOG(INFO) << "lua loaded: min_score=" << c.min_score
            << " preprocess min/max_range=" << c.dp_options.min_range() << "/"
            << c.dp_options.max_range();
  return c;
}

loc::proto::GlobalMatcherType ParseMatcherType(const std::string& s) {
  if (s == "original") return loc::proto::GLOBAL_MATCHER_ORIGINAL;
  if (s == "cpu") return loc::proto::GLOBAL_MATCHER_CPU_ANGULAR;
  if (s == "gpu") return loc::proto::GLOBAL_MATCHER_GPU_ANGULAR;
  LOG(FATAL) << "unknown matcher type: " << s
             << " (expected original | cpu | gpu)";
  return loc::proto::GLOBAL_MATCHER_ORIGINAL;
}

// 准备 synth 帧（带真值）。
std::vector<Frame> PrepareSynthFrames(const carto::mapping::ProbabilityGrid& grid,
                                      const LuaConfig& lua) {
  bc::SynthConfig synth_cfg;
  synth_cfg.beams = FLAGS_beams;
  synth_cfg.fov_deg = FLAGS_fov_deg;
  synth_cfg.max_range = FLAGS_max_range;
  synth_cfg.range_noise_sigma = FLAGS_range_noise_sigma;

  std::vector<Frame> frames;
  std::mt19937 rng(FLAGS_seed);
  int generated = 0, attempts = 0;
  while (generated < FLAGS_num_poses && attempts < FLAGS_num_poses * 20) {
    ++attempts;
    carto::transform::Rigid2d gt_pose;
    if (!bc::SampleFreePose(grid, rng, &gt_pose)) break;
    carto::sensor::PointCloud cloud =
        bc::SynthesizeScan(grid, gt_pose, rng, synth_cfg);
    if (static_cast<int>(cloud.size()) <
        FLAGS_beams * FLAGS_min_valid_beam_ratio) {
      continue;
    }
    // 与线上 merged_filtered_cloud 同样的滤波，保证输入点密度一致。
    cloud = bc::ApplyMergedCloudFilters(cloud, lua.dp_options);
    Frame f;
    f.cloud = std::move(cloud);
    f.has_gt = true;
    f.gt_pose = gt_pose;
    frames.push_back(std::move(f));
    ++generated;
  }
  CHECK_GT(generated, 0) << "未能在自由空间采到有效位姿，检查地图与阈值";
  LOG(INFO) << "prepared " << frames.size() << " synth frames";
  return frames;
}

// 准备 bag 帧（无真值，line_data 取自预处理输出）。
std::vector<Frame> PrepareBagFrames(carto::preprocess::DataPreprocessor* prep) {
  CHECK(!FLAGS_bag_filename.empty()) << "bag 模式需要 --bag_filename";
  std::vector<std::string> topics = bc::SplitString(FLAGS_scan_topics, ',');
  if (topics.empty()) topics.push_back(FLAGS_scan_topic);

  bc::BagPrepConfig cfg;
  cfg.bag_filename = FLAGS_bag_filename;
  cfg.topics = topics;
  cfg.scan_extrinsics = FLAGS_scan_extrinsics;
  cfg.tracking_frame = FLAGS_tracking_frame;
  cfg.pair_tolerance = FLAGS_pair_tolerance;
  cfg.bag_stride = FLAGS_bag_stride;
  cfg.max_bag_scans = FLAGS_max_bag_scans;
  cfg.bag_random_samples = FLAGS_bag_random_samples;
  cfg.seed = FLAGS_seed;

  const auto bag_frames = bc::PrepareBagFrames(cfg, prep);
  std::vector<Frame> frames;
  frames.reserve(bag_frames.size());
  for (const auto& bf : bag_frames) {
    Frame f;
    f.cloud = bf.cloud;
    f.line_data = bf.line_data;
    frames.push_back(std::move(f));
  }
  LOG(INFO) << "prepared " << frames.size() << " bag frames";
  return frames;
}

// 对一个版本跑完一批帧（计时仅 RunGlobalRelocalizationBenchmark）。
void RunVersion(const std::string& type_label,
                loc::proto::GlobalMatcherType type,
                const loc::proto::PoseInitializerOptions2D& base_options,
                const std::shared_ptr<carto::mapping::Submap2D>& submap,
                const std::vector<Frame>& synth_frames,
                const std::vector<Frame>& bag_frames, const float min_score,
                const double yaw_tol, std::vector<Rec>* out) {
  loc::proto::PoseInitializerOptions2D options = base_options;
  options.set_global_matcher_type(type);

  loc::PoseInitializer initializer(options);
  initializer.Initialize();
  initializer.InitGlobalSubmap(submap, submap);

  auto drive_once = [&](const Frame& f, loc::GlobalRelocInfo* profile) {
    initializer.UpdateSensorData(f.cloud, f.line_data);
    initializer.TriggerGlobalRelocalization();
    // 统一接口：跑生产同一个方法，结果从 getter 取（不再透传 profile 指针）。
    initializer.GlobalRelocalzationOnce();
    *profile = initializer.GetLastGlobalRelocInfo();
  };

  // Warm-up：首帧丢弃，让 matcher 懒构造 map cache（不计入统计）。
  const Frame* warm =
      !synth_frames.empty()
          ? &synth_frames.front()
          : (!bag_frames.empty() ? &bag_frames.front() : nullptr);
  if (warm != nullptr) {
    loc::GlobalRelocInfo discard;
    drive_once(*warm, &discard);
    LOG(INFO) << "[" << type_label << "] warm-up done ("
              << discard.t_total_ms << " ms)";
  }

  auto run_mode = [&](const std::vector<Frame>& frames, const std::string& mode) {
    int id = 0;
    for (const auto& f : frames) {
      loc::GlobalRelocInfo profile;
      drive_once(f, &profile);
      Rec rec;
      rec.type = type_label;
      rec.mode = mode;
      rec.id = id++;
      rec.profile = profile;
      if (f.has_gt && profile.matched) {
        const double dx = profile.pose_estimate.translation().x() -
                          f.gt_pose.translation().x();
        const double dy = profile.pose_estimate.translation().y() -
                          f.gt_pose.translation().y();
        rec.trans_err = std::hypot(dx, dy);
        rec.yaw_err = std::abs(carto::common::NormalizeAngleDifference(
            profile.pose_estimate.rotation().angle() -
            f.gt_pose.rotation().angle()));
        rec.success = rec.trans_err < FLAGS_success_trans_tol &&
                      rec.yaw_err < yaw_tol;
      } else {
        rec.success = profile.matched;
      }
      out->push_back(rec);
      LOG(INFO) << "[" << type_label << " " << mode << " " << rec.id << "] "
                << "total=" << profile.t_total_ms << " ms"
                << " (bbs=" << profile.t_bbs_ms << " ceres=" << profile.t_ceres_ms
                << ") score=" << profile.bbs_score
                << " matched=" << profile.matched;
    }
    (void)min_score;
  };

  if (!synth_frames.empty()) run_mode(synth_frames, "synth");
  if (!bag_frames.empty()) run_mode(bag_frames, "bag");
}

// (type, mode) 单组汇总。
std::string BuildSummary(const std::vector<Rec>& recs, const std::string& type,
                         const std::string& mode) {
  std::vector<const Rec*> sel;
  for (const auto& r : recs) {
    if (r.type == type && r.mode == mode) sel.push_back(&r);
  }
  if (sel.empty()) return "";

  auto collect = [&](auto getter) {
    std::vector<double> v;
    v.reserve(sel.size());
    for (const auto* r : sel) v.push_back(getter(*r));
    return v;
  };
  const auto t_total = collect([](const Rec& r) { return r.profile.t_total_ms; });
  const auto t_bbs = collect([](const Rec& r) { return r.profile.t_bbs_ms; });
  const auto t_rtcsm = collect([](const Rec& r) { return r.profile.t_rtcsm_ms; });
  const auto t_line = collect([](const Rec& r) { return r.profile.t_line_ms; });
  const auto t_ceres = collect([](const Rec& r) { return r.profile.t_ceres_ms; });
  const auto t_val = collect([](const Rec& r) { return r.profile.t_validate_ms; });

  const double mean_total = bc::Mean(t_total);
  auto pct = [&](const std::vector<double>& v) {
    return bc::Mean(v) / mean_total * 100.;
  };

  std::ostringstream oss;
  oss << "===== [" << type << " | " << mode << "] 端到端汇总（" << sel.size()
      << " 次）=====\n";
  oss << "  全局重定位总耗时 ms: mean=" << mean_total
      << " median=" << bc::Percentile(t_total, 50)
      << " p95=" << bc::Percentile(t_total, 95)
      << " min=" << bc::Percentile(t_total, 0)
      << " max=" << bc::Percentile(t_total, 100) << "\n";
  oss << "  阶段均值 ms（占比）:\n";
  oss << "    BBS 全图匹配 : " << bc::Mean(t_bbs) << " (" << pct(t_bbs) << "%)\n";
  oss << "    在线相关精化 : " << bc::Mean(t_rtcsm) << " (" << pct(t_rtcsm) << "%)\n";
  oss << "    线特征跟踪   : " << bc::Mean(t_line) << " (" << pct(t_line) << "%)\n";
  oss << "    Ceres 精匹配 : " << bc::Mean(t_ceres) << " (" << pct(t_ceres) << "%)\n";
  oss << "    验证         : " << bc::Mean(t_val) << " (" << pct(t_val) << "%)\n";

  int matched = 0, success = 0;
  std::vector<double> terr, yerr, scores;
  for (const auto* r : sel) {
    if (r->profile.matched) ++matched;
    if (r->success) ++success;
    scores.push_back(r->profile.bbs_score);
    if (r->success && std::isfinite(r->trans_err)) {
      terr.push_back(r->trans_err);
      yerr.push_back(r->yaw_err);
    }
  }
  oss << "  匹配率(score>min): " << matched << "/" << sel.size() << "\n";
  oss << "  bbs_score: mean=" << bc::Mean(scores)
      << " min=" << bc::Percentile(scores, 0) << "\n";
  if (mode == "synth") {
    oss << "  成功率(误差阈值内): " << success << "/" << sel.size() << "\n";
    oss << "  成功样本误差: 平移 mean=" << bc::Mean(terr)
        << " m / 角度 mean=" << bc::Mean(yerr) * 180. / M_PI << " deg\n";
  }
  return oss.str();
}

// 三版本横向对比表（每模式一张）。
std::string BuildComparison(const std::vector<Rec>& recs,
                            const std::vector<std::string>& types,
                            const std::string& mode) {
  std::ostringstream oss;
  bool any = false;
  oss << "===== [" << mode << "] 版本横向对比（端到端总耗时 ms）=====\n";
  oss << "  type      | n   | mean   | median | p95    | matched\n";
  for (const auto& type : types) {
    std::vector<double> t_total;
    int matched = 0;
    for (const auto& r : recs) {
      if (r.type == type && r.mode == mode) {
        t_total.push_back(r.profile.t_total_ms);
        if (r.profile.matched) ++matched;
      }
    }
    if (t_total.empty()) continue;
    any = true;
    char line[256];
    std::snprintf(line, sizeof(line),
                  "  %-9s | %-3zu | %6.1f | %6.1f | %6.1f | %d/%zu\n",
                  type.c_str(), t_total.size(), bc::Mean(t_total),
                  bc::Percentile(t_total, 50), bc::Percentile(t_total, 95),
                  matched, t_total.size());
    oss << line;
  }
  return any ? oss.str() : "";
}

void WriteReport(const std::string& path, const std::string& summary,
                 const std::vector<Rec>& recs, const std::string& type,
                 const std::string& mode) {
  std::ofstream out(path);
  CHECK(out.is_open()) << "无法写入报告: " << path;
  out << summary << "\n";
  out << "===== 每次测试明细（CSV）=====\n";
  out << "type,mode,id,t_total_ms,t_bbs_ms,t_rtcsm_ms,t_line_ms,t_ceres_ms,"
         "t_validate_ms,bbs_score,matched,success,trans_err,yaw_err\n";
  for (const auto& r : recs) {
    if (r.type != type || r.mode != mode) continue;
    out << r.type << "," << r.mode << "," << r.id << "," << r.profile.t_total_ms
        << "," << r.profile.t_bbs_ms << "," << r.profile.t_rtcsm_ms << ","
        << r.profile.t_line_ms << "," << r.profile.t_ceres_ms << ","
        << r.profile.t_validate_ms << "," << r.profile.bbs_score << ","
        << r.profile.matched << "," << r.success << "," << r.trans_err << ","
        << r.yaw_err << "\n";
  }
  LOG(INFO) << "report written: " << path;
}

void Run() {
  const std::string pbstream_path = ResolvePbstreamPath();
  carto::mapping::ValueConversionTables conversion_tables;
  auto submap = bc::LoadGlobalSubmap(pbstream_path, &conversion_tables);
  const auto* grid =
      dynamic_cast<const carto::mapping::ProbabilityGrid*>(submap->grid());
  CHECK(grid != nullptr) << "global_static_submap grid is not a ProbabilityGrid";

  // 端到端 ceres 阶段需要 ikd-tree——pbstream 不含，从 cloud 重建（计时外）。
  submap->RebuildIkdTrees();

  const auto& cells = grid->limits().cell_limits();
  const bc::GridStats stats = bc::ComputeGridStats(*grid);
  LOG(INFO) << "===== map info =====";
  LOG(INFO) << "  grid: " << cells.num_x_cells << " x " << cells.num_y_cells
            << " @ " << grid->limits().resolution() << " m/cell";
  LOG(INFO) << "  cell stats: total=" << stats.total << " known=" << stats.known
            << " free=" << stats.free << " occupied=" << stats.occupied;

  const LuaConfig lua = LoadLuaConfig();
  const float min_score = lua.min_score;
  const double yaw_tol = FLAGS_success_yaw_tol_deg * M_PI / 180.;
  carto::preprocess::DataPreprocessor preprocessor(lua.dp_options);

  // 准备帧（计时外，三版本共用同一批，保证公平对比）。
  std::vector<Frame> synth_frames, bag_frames;
  if (FLAGS_mode == "synth" || FLAGS_mode == "both") {
    synth_frames = PrepareSynthFrames(*grid, lua);
  }
  if (FLAGS_mode == "bag" || FLAGS_mode == "both") {
    bag_frames = PrepareBagFrames(&preprocessor);
  }

  // 解析版本列表（gpu 在未编译 GPU 时跳过）。
  std::vector<std::string> requested = bc::SplitString(FLAGS_matcher_types, ',');
  std::vector<std::string> types;
  for (const auto& t : requested) {
    if (t == "gpu") {
#ifndef BUILD_GPU_MATCHER
      LOG(WARNING) << "gpu matcher requested but BUILD_GPU_MATCHER is off, skipping";
      continue;
#endif
    }
    types.push_back(t);
  }
  CHECK(!types.empty()) << "没有可运行的 matcher 版本";

  std::vector<Rec> recs;
  for (const auto& type_label : types) {
    LOG(INFO) << "===== running version: " << type_label << " =====";
    RunVersion(type_label, ParseMatcherType(type_label), lua.pose_init_options,
               submap, synth_frames, bag_frames, min_score, yaw_tol, &recs);
  }

  // ===== 输出 =====
  std::time_t now = std::time(nullptr);
  char timestamp[32];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S",
                std::localtime(&now));
  std::string out_dir = FLAGS_output_dir;
  if (out_dir.empty()) {
    const char* home = std::getenv("HOME");
    out_dir = std::string(home ? home : "/tmp") +
              "/.dexe/localization/poseinit_baseline";
  }
  std::filesystem::create_directories(out_dir);
  const std::string map_tag = FLAGS_map_name.empty() ? "map" : FLAGS_map_name;

  LOG(INFO) << "";
  LOG(INFO) << "########## PoseInitializer end-to-end summary ##########";
  for (const std::string& mode : {"synth", "bag"}) {
    const std::string cmp = BuildComparison(recs, types, mode);
    if (!cmp.empty()) LOG(INFO) << "\n" << cmp;
    for (const auto& type : types) {
      const std::string summary = BuildSummary(recs, type, mode);
      if (summary.empty()) continue;
      LOG(INFO) << "\n" << summary;
      const std::string path = out_dir + "/" + map_tag + "_" + timestamp + "_" +
                               type + "_" + mode + ".txt";
      WriteReport(path, summary + "\n" + cmp, recs, type, mode);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  google::SetUsageMessage(
      "PoseInitializer 端到端全局重定位离线基准测试工具，详见文件头注释。");
  google::ParseCommandLineFlags(&argc, &argv, true);
  Run();
  return 0;
}
