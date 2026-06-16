/*
 * benchmark_common.h
 *
 * 离线定位基准工具共享单元（bbs_benchmark 与 pose_initializer_benchmark 共用）。
 * 内容：pbstream 地图加载、合成 scan 生成、bag 双雷达取帧与预处理、通用统计。
 * 全部不依赖具体工具的 gflags——参数经显式 config 结构体传入。
 */

#ifndef CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_BENCHMARK_COMMON_H_
#define CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_BENCHMARK_COMMON_H_

#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/preprocess/data_preprocessor_options.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cartographer_ros {
namespace benchmark_common {

namespace carto = ::cartographer;

// 占据/自由概率阈值（与线上栅格语义一致）。
constexpr float kOccupiedProbability = 0.65f;
constexpr float kFreeProbability = 0.196f;

// ===== 地图加载 =====

// 从 pbstream 读取 global_static_submap 字段，构建与线上一致的 Submap2D。
// 不含 ikd-tree——端到端定位前需对返回的 submap 调 RebuildIkdTrees()。
std::shared_ptr<carto::mapping::Submap2D> LoadGlobalSubmap(
    const std::string& pbstream_path,
    carto::mapping::ValueConversionTables* conversion_tables);

struct GridStats {
  int known = 0, free = 0, occupied = 0, total = 0;
};

GridStats ComputeGridStats(const carto::mapping::ProbabilityGrid& grid);

// ===== 合成 scan 生成 =====

struct SynthConfig {
  int beams = 2400;
  double fov_deg = 360.0;
  double max_range = 20.0;
  double range_noise_sigma = 0.01;
};

// 沿世界系方向 ray-march，命中占据格返回距离，否则返回 -1（无回波）。
double RayCast(const carto::mapping::ProbabilityGrid& grid,
               const Eigen::Vector2d& origin, const Eigen::Vector2d& dir,
               double max_range);

// 在地图自由空间随机采样一个位姿（作为真值）。
bool SampleFreePose(const carto::mapping::ProbabilityGrid& grid,
                    std::mt19937& rng, carto::transform::Rigid2d* pose);

// 给定真值位姿，ray-cast 合成传感器系 scan。
carto::sensor::PointCloud SynthesizeScan(
    const carto::mapping::ProbabilityGrid& grid,
    const carto::transform::Rigid2d& gt_pose, std::mt19937& rng,
    const SynthConfig& config);

// 复刻 GetMergedData 对 merged_filtered_cloud 的滤波（用于合成模式对齐线上输入）。
carto::sensor::PointCloud ApplyMergedCloudFilters(
    const carto::sensor::PointCloud& cloud,
    const carto::preprocess::proto::DataPreprocessorOptions& dp_options);

// ===== 小工具 =====

std::vector<std::string> SplitString(const std::string& s, char delim);

// "x,y,z,roll,pitch,yaw"（米/度）→ Rigid3d
carto::transform::Rigid3d ParseExtrinsic(const std::string& s);

// ===== bag 双雷达取帧 + 预处理 =====

struct BagPrepConfig {
  std::string bag_filename;
  std::vector<std::string> topics;  // 第一个为主雷达（抽帧基准）
  std::string scan_extrinsics;      // 手动外参覆盖，留空则从 /tf_static 解析
  std::string tracking_frame;
  double pair_tolerance = 0.1;
  int bag_stride = 20;
  int max_bag_scans = 30;
  int bag_random_samples = 0;  // >0 启用时间随机抽帧
  uint64_t seed = 42;
};

// 一帧准备好的输入：各雷达经外参合并 + 预处理后的点云（及线特征，若启用）。
struct BagFrame {
  carto::sensor::PointCloud cloud;
  std::vector<carto::sensor::LineData> line_data;
  size_t primary_frame_index = 0;  // 主话题原始帧序号
  double t_rel = 0.0;              // 相对 bag 起始秒
};

// 两遍读取 bag：第一遍收集 /tf_static 外参 + 各话题时间戳；抽帧 + 副话题配对；
// 第二遍只反序列化选中帧，按外参合并并经预处理产出 BagFrame。
// preprocessor != nullptr 时走线上 DataPreprocessor（与 pose_estimator 同一份代码，
// 取 merged_filtered_cloud + merged_line_data）；nullptr 时简化合并（不去畸变/不滤波）。
std::vector<BagFrame> PrepareBagFrames(
    const BagPrepConfig& config,
    carto::preprocess::DataPreprocessor* preprocessor);

// ===== 通用统计 =====

double Percentile(std::vector<double> v, double p);
double Mean(const std::vector<double>& v);

}  // namespace benchmark_common
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_TOOLS_BBS_BENCHMARK_BENCHMARK_COMMON_H_
