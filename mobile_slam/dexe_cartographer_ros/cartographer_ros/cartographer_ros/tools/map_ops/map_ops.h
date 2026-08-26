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

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/id.h"
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
  ::cartographer::transform::Rigid3d global_pose_b;  // 在源地图坐标系下
  ::cartographer::sensor::PointCloud point_cloud;    // gravity-aligned 点云
  std::vector<::cartographer::sensor::LineData> line_data_list;
};

// ──────────────────────────────────────────────────
// 加载 / 提取 / 匹配
// ──────────────────────────────────────────────────

// 加载一张地图的 pbstream。
// build_slices=false 时跳过 submap 切片构建（不做渲染拼接时更省 CPU/内存）。
MapData LoadMap(const std::string& pbstream_file, bool build_slices = true);

// 从地图节点中提取用于匹配的点云 / 线特征。
std::vector<NodeCloudEntry> ExtractNodeClouds(const MapData& data);

// 在目标地图 `data_a` 上对源地图节点 `node_clouds_b` 做全局重定位，
// 成功时返回 {true, T_src_in_tgt}（源地图原点 → 目标地图坐标系）。
// 与 merge_maps_main.cc 的 FindTransformBinA 相同。
std::pair<bool, ::cartographer::transform::Rigid3d> FindTransformBinA(
    const MapData& data_a,
    const std::vector<NodeCloudEntry>& node_clouds_b,
    double min_score, int consecutive_needed);

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
};

// 对已加载的地图 `data` 施加 T_old_to_new（旧系 → 新系），重建全局栅格，
// 重新渲染，并把结果写到：
//   - output_pbstream ：合并/替换后的 map.pbstream（先写 .tmp 再 rename，原子）
//   - output_filestem ：map.png / map.yaml（以及可选 .pcd）的文件名前缀
// 会移动消费 `data`。
void TransformAndRewriteMap(
    MapData data, const ::cartographer::transform::Rigid3d& T_old_to_new,
    const std::string& output_pbstream, const std::string& output_filestem,
    const RewriteOptions& opts);

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

// 传入名称或路径：先查 <maps_root>/<name_or_path>，再直接当作路径查。
// maps_root 留空则用 ~/.dexe/data/maps。找不到则 LOG(FATAL)。
std::string ResolveMapDir(const std::string& name_or_path,
                          const std::string& maps_root = "");

}  // namespace map_ops
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_OPS_MAP_OPS_H_
