/*
 * map_ops.h
 *
 * 地图离线工具共享库：把此前散落在 merge_maps_main.cc / reset_map_origin_main.cc
 * 中几乎相同的公共逻辑（加载 pbstream、提取节点点云、全局重定位、刚体变换重写、
 * 渲染输出）集中到此处，供新的离线工具（如 replace_map）复用。
 *
 * 注意：为了不改动现有工具，本库内的实现是从那两个工具"抽取 + 泛化"的副本，
 * 放在 `cartographer_ros::map_ops` 命名空间；现有工具仍使用各自匿名命名空间里的
 * 原实现，互不影响。
 *
 * 主要能力：
 *   - LoadMap / ExtractNodeClouds / FindTransformBinA：加载地图并在目标地图上对
 *     源地图做全局重定位，得到 T_src_in_tgt（与 merge_maps 相同）。
 *   - TransformAndRewriteMap：对一个已加载的地图施加刚体变换（旧系 → 新系），
 *     重建全局栅格、重新渲染并原子地写出 pbstream + png + yaml（可选 pcd）。
 *     这是 reset_map_origin 的"变换 + 重写"核心的参数化版本。
 *   - ResolveMapDir：地图名称/路径解析（同现有工具）。
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_OPS_MAP_OPS_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_OPS_MAP_OPS_H_

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/structure_alignment.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/line_matcher_options.pb.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/value_conversion_tables.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "opencv2/core.hpp"

namespace cartographer_ros {
namespace map_ops {

// ──────────────────────────────────────────────────
// 加载结果
// ──────────────────────────────────────────────────

// 一张地图 pbstream 反序列化后的全部数据。
// 与 merge_maps_main.cc 中的 MapData 等价；字段声明顺序保证
// global_submap_conv_tables 先于 global_submap 析构。
struct MapData {
  ::cartographer::mapping::proto::PoseGraph pose_graph;
  ::cartographer::mapping::proto::SerializationHeader header;
  ::cartographer::mapping::proto::AllTrajectoryBuilderOptions traj_options;

  // 所有 SerializedData 记录（submap 纹理、node、imu、global_static_submap 等）。
  std::vector<::cartographer::mapping::proto::SerializedData> all_records;

  // submap 切片（仅渲染拼接送用；加载时可选构建）。
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>
      submap_slices;
  ::cartographer::mapping::ValueConversionTables conversion_tables;

  // 全局静态 submap（kGlobalStaticSubmap，整张地图合并的单一 grid），用于全局匹配。
  std::unique_ptr<::cartographer::mapping::ValueConversionTables>
      global_submap_conv_tables;
  std::unique_ptr<::cartographer::mapping::Submap2D> global_submap;
};

// 单个轨迹节点的可匹配数据（点云 + 线特征 + 该节点在源地图坐标系下的全局位姿）。
struct NodeCloudEntry {
  int trajectory_id;
  int node_index;
  int64_t timestamp = 0;                             // common::ToUniversal 刻度
  ::cartographer::transform::Rigid3d global_pose_b;  // 在源地图坐标系下
  ::cartographer::sensor::PointCloud point_cloud;    // gravity-aligned 点云
  std::vector<::cartographer::sensor::LineData> line_data_list;
};

// ──────────────────────────────────────────────────
// 加载 / 提取 / 筛选 / 匹配
// ──────────────────────────────────────────────────

// 加载一张地图的 pbstream。
// build_slices=false 时跳过 submap 切片构建（不做渲染拼接时更省 CPU/内存）。
MapData LoadMap(const std::string& pbstream_file, bool build_slices = true);

// 从地图节点中提取用于匹配的点云 / 线特征（全部节点，按记录序）。
std::vector<NodeCloudEntry> ExtractNodeClouds(const MapData& data);

// 按给定 (trajectory_id, node_index) 集合提取，结果按节点时间戳稳定排序。
std::vector<NodeCloudEntry> ExtractNodeClouds(
    const MapData& data,
    const std::vector<std::pair<int, int>>& selected);

// 关键帧筛选参数（对轨迹按距离/角度抽稀，只用 pose_graph 的位姿与时间戳，
// 不解压点云）。
struct KeyframeSelectionOptions {
  double max_distance_meters = 2.0;
  double max_angle_radians = 0.35;
  int min_frames = 11;    // 文档要求 >10 帧；不足时自动收紧阈值兜底
  int max_selected = 200; // 候选池上限，超出按时间等间隔抽稀
};

// 从源地图轨迹中筛选用于匹配的关键帧，返回 (trajectory_id, node_index)，
// 按节点时间戳稳定排序。实现：每条轨迹各用一个纯距离/角度 MotionFilter
// （max_time_seconds=1e9）；只保留带点云的节点；不足 min_frames 时阈值减半
// 重试（下限 0.25m），仍不足则取全部带点云节点。
std::vector<std::pair<int, int>> SelectKeyframes(
    const MapData& data, const KeyframeSelectionOptions& opts);

// 变换求解参数（三阶段管线：全局 BBS 初对齐 → 增量加帧迭代 → 收敛）。
struct FindTransformOptions {
  // Stage B：全局初对齐
  double min_score = 0.45;    // 全局 BBS 接受分数
  int initial_frames = 10;    // 初始对齐帧数 K
  int min_success_frames = 5; // 成功帧数低于此值 → 失败
  // Stage C：增量迭代
  int frames_per_step = 5;
  int max_iterations = 20;
  int max_frames = 100;         // 全程匹配帧数上限
  double local_min_score = 0.30;  // 局部匹配接受门（预测窗口已强约束；跨会话分数普遍偏低）
  double local_linear_window = 3.0;   // m
  double local_angular_window = 0.3;  // rad
  // 收敛判定（文档：位移/旋转 <1e-3 或合理值；0.05m 分辨率下默认放宽）
  double converge_translation = 0.01;  // m
  double converge_yaw = 0.005;         // rad
  // outlier 拒绝（L2/L3 阈值）。注意 refine_score（平均占据概率）的绝对尺度
  // 随地图对变化（自匹配 0.5–0.84，跨会话正确帧可低至 0.36–0.52），故 L2 只做
  // 宽松健全性下限，离群拒绝主要交给尺度不变的 L3 一致性与相对分数剔除。
  double refine_min_score = 0.35;    // L2 宽松健全性下限
  double outlier_translation = 0.5;  // m，单帧 T 偏离中位数上限
  double outlier_yaw = 0.1;          // rad
  // L3 相对分数剔除：refine_score 低于（T-内点最高分 − 该裕量）的帧视为
  // 别名/漂移帧剔除；<=0 关闭。跨会话整体偏低时（分数跨度小）不误伤。
  double score_relative_margin = 0.25;
  // 预算与 BBS 引擎
  double budget_seconds = 25.0;
  int bbs_num_threads = 4;
  double bbs_timeout_ms = 2000.0;
  bool bbs_angular_branching = true;

  // ── 匹配器参数：来自 lua 的 POSE_INITIALIZER_2D ──────────────────────
  // 本函数做的事（对 global_static_submap 做全局重定位）与定位节点的
  // PoseInitializer 完全同构，**必须用同一份配置**，否则定位侧这些年调过的
  // 参数（Ceres 权重量级、improved line cost、huber 尺度…）不会传导到离线工具，
  // 两边会静默跑成两套算法。
  // has_matcher_options=false 时回退到内置默认值 —— 只为兼容未传 lua 的调用方，
  // 并会打 WARNING；正常路径都应该由调用方从 lua 填好。
  bool has_matcher_options = false;
  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D
      ceres_options;
  ::cartographer::mapping::scan_matching::proto::LineMatcherOptions
      line_matcher_options;
  int branch_and_bound_depth = 7;
};

struct FindTransformResult {
  bool success = false;
  ::cartographer::transform::Rigid3d T_src_in_tgt =
      ::cartographer::transform::Rigid3d::Identity();
  int global_matched = 0;   // Stage B 成功帧数
  int local_matched = 0;    // Stage C 成功帧数
  int inliers = 0;          // 参与最终聚合的内点数
  int rejected_l1 = 0;      // 匹配分数门拒绝
  int rejected_l2 = 0;      // Ceres 未收敛 / 精化打分不足
  int rejected_l3 = 0;      // 聚合一致性剔除
  int iterations = 0;
  double elapsed_seconds = 0.0;
  bool converged = false;
  std::string stop_reason;  // converged / max_frames / max_iterations / budget
  // 内点相对最终 delta_T 的残差 RMS —— 「这次匹配值不值得信」的定量依据。
  // 只有 success 时有意义；inliers==0 时为 0。
  double rms_translation_m = 0.0;
  double rms_yaw_rad = 0.0;
  // success 但未收敛（stop_reason != "converged"）：内点数够、T 可用，但
  // delta_T 还没稳定下来。与 success 分开表达，让调用方自己决定是否接受，
  // 而不是把「够了」和「稳了」混成一个布尔。
  bool degraded = false;
};

// 栅格中已知（已被观测过）格子的面积（m²）= 已知格子数 × resolution²。
// 用于比较两张图的实际覆盖范围；比包围盒面积稳健（U 形场地包围盒会虚高）。
double EstimateKnownAreaM2(const ::cartographer::mapping::Grid2D& grid);

// 在目标地图 `data_a` 上对源地图关键帧 `frames_b`（按时间戳排序）求
// T_src_in_tgt。三阶段：BatchFCSM 全局初对齐（最远点采样 K 帧）→
// 局部 FCSM + Ceres 增量加帧 → delta_T 收敛即停。
// 四层 outlier 拒绝：L1 匹配分数门；L2 Ceres 收敛性 + 位姿打分（宽松下限）；
// L3 中位数 + 一致性剔除 + 相对分数剔除；L4 内点数护栏。
FindTransformResult FindTransformBinA(
    const MapData& data_a,
    const std::vector<NodeCloudEntry>& frames_b,
    const FindTransformOptions& opts);

// ──────────────────────────────────────────────────
// 刚体变换辅助（作用于 proto 数据）
// ──────────────────────────────────────────────────

// 对 LineData proto 施加 2D 刚体变换（begin/end 刚体，normal 仅旋转）。
void TransformLineDataProto(::cartographer::mapping::proto::LineData* line,
                            const ::cartographer::transform::Rigid2d& T_2d);

// 对 global_static_submap 的非栅格数据施加变换：
// local_pose、cloud、highlight_cloud、line_map_2d_list。栅格由调用方重建。
void TransformStaticSubmapNonGrid(
    ::cartographer::mapping::proto::Submap2D* submap_2d,
    const ::cartographer::transform::Rigid3d& T_old_to_new);

// 重栅格化 Grid2D（无节点可重建栅格时的回退路径）。
void RerasterizeGridProto(::cartographer::mapping::proto::Grid2D* grid_proto,
                          const ::cartographer::transform::Rigid3d& T_old_to_new);

// 把已变换位姿的所有节点点云重新插入一张干净的 ProbabilityGrid。
::cartographer::mapping::proto::Grid2D BuildFreshGridFromNodes(
    const ::cartographer::mapping::proto::PoseGraph& pose_graph,
    const std::vector<::cartographer::mapping::proto::SerializedData>& records,
    const ::cartographer::mapping::proto::AllTrajectoryBuilderOptions&
        traj_options,
    double resolution, double lidar_range_pad);

// 用单个变换后的 global_static_submap 渲染，输出 cv::Mat / origin / resolution。
void RenderSingleStaticSubmap(
    const ::cartographer::mapping::proto::Submap2D& submap_2d_proto,
    double occ_thresh, double free_thresh, cv::Mat* out_mat,
    Eigen::Vector2d* out_origin, double* out_resolution);

// ──────────────────────────────────────────────────
// 变换 + 重写（高层封装）
// ──────────────────────────────────────────────────

struct RewriteOptions {
  double resolution = 0.05;
  double occ_thresh = 0.65;
  double free_thresh = 0.196;
  std::string navigation_map_filter_mode = "normal";
  bool save_origin_map = false;
  bool save_debug_map = false;
  // 是否额外输出 map.pcd / map_highlight.pcd（来自变换后的全局点云）。
  bool save_map_pcd = false;
  // 产物自检时估算结构化对齐残余用的参数。应与建图节点的
  // reset_map_origin_* （lua）保持同一份，否则「工具报的残余」与「建图报的残余」
  // 不可比。调用方不填则用结构体默认值。
  ::cartographer::mapping::StructureAlignmentOptions align_options;
  // 写进 map.yaml 的 created_at（内容采集时刻）。替换语义下内容整体来自 src，
  // 所以调用方应传 **src 的** created_at，而不是目标图的、更不是"现在"
  // （"现在"由 map.yaml 的 mtime 表达）。src 也没有时留空，写入当前时间。
  std::string created_at;
  // map.yaml 的合并底：本次不负责的键（display_yaw 等）从这里原样带过来。
  // 留空则以输出路径上的旧 yaml 为底。
  std::string base_yaml_path;
};

// 重写后的产物自检量。变换本身没有"对不对"的内建判据，只能靠产物侧的
// 可观测量交叉验证，故在重建完成、写盘之前就地采集。
struct RewriteStats {
  // 重建后的全局栅格尺寸与点云规模（对照替换前可看出产物是否合理）。
  int grid_num_x_cells = 0;
  int grid_num_y_cells = 0;
  int cloud_points = 0;
  int line_count = 0;
  // 产物的结构化对齐残余（对重建后的线特征跑一次 EstimateAlignmentYaw）。
  // 这是 DEC-013 定的「替换后只做检查、不做对齐」：残余大说明 target 坐标系
  // 本身没对齐（或替换引入的 yaw 误差把它推大了），由人决定是否单独补一次 align。
  bool align_evaluated = false;
  double align_residual_yaw_rad = 0.0;
  // 重算后的地图面积（同时已写进 map.yaml）。地图重新生成 ⇒ 地图信息必须重算，
  // 否则 map_manager 的 get_maps 会读到旧值或 0。
  double total_area_m2 = 0.0;
  double effective_area_m2 = 0.0;
};

// 对已加载的地图 `data` 施加 T_old_to_new（旧系 → 新系），重建全局栅格，
// 重新渲染，并把结果写到：
//   - output_pbstream ：合并/替换后的 map.pbstream（先写 .tmp 再 rename，原子）
//   - output_filestem ：map.png / map.yaml（以及可选 .pcd）的文件名前缀
// 会移动消费 `data`。`stats` 可为 nullptr。
void TransformAndRewriteMap(
    MapData data, const ::cartographer::transform::Rigid3d& T_old_to_new,
    const std::string& output_pbstream, const std::string& output_filestem,
    const RewriteOptions& opts, RewriteStats* stats = nullptr);

// ──────────────────────────────────────────────────
// map.yaml 配置读取
// ──────────────────────────────────────────────────

// 从 map.yaml 读取 resolution / occupied_thresh / free_thresh。
// 这些字段决定渲染产物（png 阈值 + yaml 记录），不影响 pbstream 内容。
// 文件不存在或字段缺失时对应标志保持 false。
struct MapYamlConfig {
  bool has_resolution = false;
  double resolution = 0.0;
  bool has_occ_thresh = false;
  double occ_thresh = 0.0;
  bool has_free_thresh = false;
  double free_thresh = 0.0;
};
MapYamlConfig ReadMapYamlConfig(const std::string& yaml_path);

// ──────────────────────────────────────────────────
// 路径解析
// ──────────────────────────────────────────────────

// 传入名称或路径，解析出**含 map.pbstream 的那一级目录**。
// 依次尝试四种组合（分层布局优先，与 align_map_tool 的探测口径一致）：
//   <maps_root>/<name>/slam/  →  <maps_root>/<name>/  →
//   <name>/slam/             →  <name>/
// 分层布局（<map>/slam/ + <map>/assets/ + <map>/versions/）下，传地图名即可，
// 不必手写到 slam/ 那一级。maps_root 留空则用 ~/.dexe/data/maps。
// 四种都没有 map.pbstream 时 LOG(FATAL) 并列出尝试过的路径。
std::string ResolveMapDir(const std::string& name_or_path,
                          const std::string& maps_root = "");

}  // namespace map_ops
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_OPS_MAP_OPS_H_
