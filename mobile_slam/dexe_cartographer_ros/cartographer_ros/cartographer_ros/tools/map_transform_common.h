/*
 * map_transform_common.h
 *
 * 离线地图工具（reset_map_origin_tool / align_map_tool）共享的变换 helper：
 *   - pbstream 整体加载 / 原子写回
 *   - pose graph / global_static_submap / 线特征 / 栅格的刚体变换
 *   - 栅格重建（重栅格化 或 从 node 点云重建）
 *   - global_static_submap 渲染为 cv::Mat
 *   - 地图文件备份、PCD 变换
 *
 * 这些函数原先内联在 reset_map_origin_main.cc 中，抽取到此处供多个工具复用
 * （与 bbs_benchmark/benchmark_common.h 的组织方式一致）。
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_TRANSFORM_COMMON_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_TRANSFORM_COMMON_H_

#include <string>
#include <vector>

#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/proto/serialization.pb.h"
#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "opencv2/core.hpp"

namespace cartographer_ros {
namespace map_transform {

// pbstream 全量内容（加载后常驻内存，变换完再整体写回）
struct PbstreamContents {
  ::cartographer::mapping::proto::SerializationHeader header;
  ::cartographer::mapping::proto::PoseGraph pose_graph;
  ::cartographer::mapping::proto::AllTrajectoryBuilderOptions traj_options;
  std::vector<::cartographer::mapping::proto::SerializedData> records;
};

// 读取整个 pbstream（header + pose_graph + traj_options + 全部 SerializedData）。
// 读取失败时 CHECK 终止。
PbstreamContents LoadPbstream(const std::string& pbstream_file);

// 原子写回 pbstream：先写 <file>.tmp 再 rename 覆盖。
void WritePbstream(const std::string& pbstream_file,
                   const PbstreamContents& contents);

// 对 LineData proto 应用 2D 刚体变换（begin/end 用刚体，normal 仅旋转）
void TransformLineDataProto(::cartographer::mapping::proto::LineData* line,
                            const ::cartographer::transform::Rigid2d& T_2d);

// 对 global_static_submap 的非 grid 数据应用 T_inv：
//   local_pose、cloud、highlight_cloud、line_map_2d_list
// grid 不在这里处理 —— 由调用方决定走"从 node 重新生成"还是"重栅格化"路径
void TransformStaticSubmapNonGrid(
    ::cartographer::mapping::proto::Submap2D* submap_2d,
    const ::cartographer::transform::Rigid3d& T_inv);

// 重新栅格化 Grid2D：把每个已知 cell 的中心位置经 T_inv 变换后写入新栅格。
// 保留原概率值（含 free space），仅支持 ProbabilityGrid。
void RerasterizeGridProto(::cartographer::mapping::proto::Grid2D* grid_proto,
                          const ::cartographer::transform::Rigid3d& T_inv);

// 在新坐标系中从 trajectory node 重建**整个** global_static_submap：
// 栅格 + 点云 + 高亮点云 + 线特征，最后 Finish()（含 line / cloud culling）。
//
// 为什么必须重建而不能只对旧产物做刚体变换：
//   - 点云入 ikd-tree 时按栅格分辨率做**轴对齐体素降采样**，刚体变换后
//     点云仍带着旧坐标系的体素量化，与新栅格 cell 边界系统性错位；
//   - cloud culling 依据是**当时那张栅格**的占据概率，栅格重建后旧的保留集
//     失效，且此前被剔除的点已不在数据里，刚体变换救不回来；
//   - 线特征的关联与拟合同理，是旧系下的结论。
// 与 MapBuilder::GenerateGlobalStaticSubmap 的步骤 7a/7b/7c 一一对应，
// 区别只在这里是 proto 级输入（pbstream 已落盘，没有内存位姿图对象）。
//
// 已知限制：pbstream 的 TrajectoryNodeData 不存 range_datas，只有
// filtered_gravity_aligned_point_cloud，因此插栅格时 misses 为空 ——
// 命中射线沿途的 free space 由 CastRays 重建，但**无回波 / 超量程光束**
// 带来的 free space 补不回来。这是 pbstream 格式边界，不是实现缺陷。
//
// 另注：重建出的 submap 不含 precomputation_grid_stack 缓存（旧缓存属于
// 旧栅格，本就该失效），首次全局重定位会重建它，是一次性开销。
//
// 'pose_graph' 的位姿必须已变换到新坐标系。
::cartographer::mapping::proto::Submap2D RebuildGlobalStaticSubmapFromNodes(
    const ::cartographer::mapping::proto::PoseGraph& pose_graph,
    const std::vector<::cartographer::mapping::proto::SerializedData>& records,
    const ::cartographer::mapping::proto::AllTrajectoryBuilderOptions&
        traj_options,
    const double resolution, const double lidar_range_pad);

// 把所有 trajectory node 的 filtered_gravity_aligned_point_cloud 在新坐标系中
// 重新插入到一张全新的 ProbabilityGrid（干净、无重影；但 pbstream 不存原始
// range_data，misses 为空，free space 信息会丢失）。
// ⚠️ 仅重建栅格，不重建点云 / 线特征 —— 新代码请用
// RebuildGlobalStaticSubmapFromNodes；本函数保留给只需要栅格的旧调用方。
::cartographer::mapping::proto::Grid2D BuildFreshGridFromNodes(
    const ::cartographer::mapping::proto::PoseGraph& pose_graph,
    const std::vector<::cartographer::mapping::proto::SerializedData>& records,
    const ::cartographer::mapping::proto::AllTrajectoryBuilderOptions&
        traj_options,
    const double resolution, const double lidar_range_pad);

// 用单个变换后的 global_static_submap 渲染地图（与
// MapBuilderBridge::SaveGridMapFromRange 一致）。
// submap_2d_proto 的 grid/local_pose 需在调用前已经过变换。
void RenderSingleStaticSubmap(
    const ::cartographer::mapping::proto::Submap2D& submap_2d_proto,
    const double occ_thresh, const double free_thresh, cv::Mat* out_mat,
    Eigen::Vector2d* out_origin, double* out_resolution);

// proto → 运行时 LineMap2DList 的轻量重建（submap_2d.cc FromProto 的子集）：
// 只重建主线段数据、不重放 line_data_list 观测历史。
// 与 cartographer/io/internal/pbstream_check.cc 的 ALIGN-01 检查同源。
::cartographer::mapping::LineMap2DList LineMap2DListFromProto(
    const ::cartographer::mapping::proto::LineMap2DList& proto);

// 把 map_filestem 对应的所有现存地图文件整体备份到一个带时间戳的目录：
//   <slam目录>/backups/backup_<YYYYMMDD_HHMMSS>/<原文件名>
// 目录内子文件保持原名（map.pbstream / map.png / ...），便于直接整目录恢复。
// candidate_extensions 为待检查的后缀列表（如 ".pbstream"、"_highlight.pcd"）。
// 返回备份目录路径；没有任何候选文件存在时返回空串且不创建目录。
// 注意：建图节点自己的归档用 backup.<ts>/（点分隔）命名，AtomicDirSwap 轮换
// 只认该前缀；本工具的 backup_<ts>/（下划线）目录不参与节点轮换，两者共存。
std::string BackupMapFiles(
    const std::string& map_filestem,
    const std::vector<std::string>& candidate_extensions);

// 生成当前本地时间的时间戳字符串（YYYYMMDD_HHMMSS）。
std::string CurrentTimestamp();

// 对 PCD 文件（pcl::PointXYZ）施加 T_inv 并写回。文件不存在时返回 false。
bool TransformPcdFile(const std::string& pcd_file,
                      const ::cartographer::transform::Rigid3d& T_inv);

}  // namespace map_transform
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TOOLS_MAP_TRANSFORM_COMMON_H_
