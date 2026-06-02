#ifndef CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_

#include <atomic>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/localization/proto/local_map_updater_options_2d.pb.h"
#include "cartographer/localization/2d/chunk_map_2d.h"
#include "cartographer/sensor/internal/box_filter.h"

namespace cartographer {
namespace localization {

proto::LocalMapUpdaterOptions2D CreateLocalMapUpdaterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class LocalMapUpdater2D
{
public:
  LocalMapUpdater2D(const proto::LocalMapUpdaterOptions2D& options,
                    const mapping::proto::SubmapsOptions2D& submap_options);
  ~LocalMapUpdater2D();

  void SetGlobalSubmap(std::shared_ptr<mapping::Submap2D> global_submap);

  void UpdateRobotPose(const transform::Rigid2d& robot_pose);
  
  // 更新 global_dynamic_submap_
  void UpdateDynamicMap(const std::vector<sensor::RangeData>& range_data_vector,
                        const sensor::PointCloud& cloud,
                        const sensor::PointCloud& highlight_cloud,
                        const std::map<int, std::vector<sensor::LineData>>& line_data);

  bool IsLocalMapUpdated() const { return is_local_map_updated_; }

  std::shared_ptr<mapping::Submap2D> local_submap();

  // 根据策略获取 global submap
  // - NONE: 返回 global_static_submap_
  // - SHORT/LONG/PERSISTENT: 返回 global_dynamic_submap_
  std::shared_ptr<mapping::Submap2D> GetGlobalSubmap() const;

  // 获取漂移免疫的静态底图（SetGlobalSubmap 后从不插入新数据）。
  // 与 GetGlobalSubmap() 不同：后者在 SHORT/LONG/PERSISTENT 策略下返回被持续更新的
  // global_dynamic_submap_（会随定位漂移一起污染）。漂移门控必须用这张不漂的底图做独立锚点。
  std::shared_ptr<mapping::Submap2D> GetStaticSubmap() const;

  // 同步地以给定位姿为中心，从 global_dynamic_submap_ 裁剪并构建新的 local submap，
  // 并直接替换内部的 local_submap_（同时更新 local_map_origin_）。
  // 用于 Recovery 场景：先将 local map 对准估计位姿，再调用 local_submap() 进行搜索。
  // global_dynamic_submap_ 未初始化时为 no-op。
  void RebuildLocalSubmapAt(const transform::Rigid2d& pose);

  // 与 RebuildLocalSubmapAt 相同，但从【漂移免疫的 global_static_submap_】裁剪，
  // 得到干净的 local submap（含 grid + cloud → ikdtree，可供 correlative + ceres 精修）。
  // 用于漂移恢复：在干净底图上精修校正位姿，避免被已漂的 global_dynamic_submap_ 拉回漂移方向。
  // global_static_submap_ 未初始化时为 no-op。
  void RebuildLocalSubmapFromStaticAt(const transform::Rigid2d& pose);

  // reset_map_origin 时同步更新内部地图和状态到新坐标系。
  void ApplyOriginTransform(const transform::Rigid3d& T);
  
  // 获取动态地图策略
  proto::DynamicMapPolicy GetDynamicMapPolicy() const {
    return options_.dynamic_map_policy();
  }

  std::shared_ptr<transform::Rigid2d> origin_pose();
  std::shared_ptr<transform::Rigid2d> robot_pose();

  // 传入由 LocalizationUpdater::LoadState 创建/加载好的 chunk map（本类不再自建 chunk map，
  // 也无独立 use_chunk_map 开关——注入即启用）。chunk_map_ 非空时 BuildLocalSubmap 用它拼装；
  // 若 save_debug_data 开启，注入时把**全部** chunk 一次性落到 <map_dir>/debug/all_chunks/。
  void SetChunkMap(std::shared_ptr<ChunkMap2D> chunk_map);

  // 注入当前地图目录（= 加载的 map.pbstream 所在目录，由 LocalizationUpdater 传入）。
  // save_debug_data 开启时所有 debug data 落到此目录下：
  //   - 不用 chunk：local submap 的栅格图/点云 → <map_dir>/debug/（local_map.png + local_submap.pcd）
  //   - 用 chunk：全部 chunk → <map_dir>/debug/all_chunks/（SetChunkMap 时）；
  //              当前 active chunk + 拼装栅格图/点云 → <map_dir>/debug/（每次 BuildLocalSubmap）
  void SetDebugDir(const std::string& map_dir) { map_dir_ = map_dir; }

  // 定位侧持久化：把 chunk map 落盘到 chunks_dir/（pbstream 格式，透传到 ChunkMap2D::SaveToDir）。
  // 仅 chunk_map_ 非空时有效，否则返回 false（非致命，调用方按"无 chunk map"处理）。
  bool SaveChunkMap(const std::string& chunks_dir) const;

private:
  // 判断是否需要触发局部地图重建（机器人接近当前 local map 边界）
  bool NeedRebuildLocalMap() const;

  // 纯计算函数：根据快照构建 local submap，可在后台线程安全调用
  std::shared_ptr<mapping::Submap2D> BuildLocalSubmap(
      std::shared_ptr<mapping::Submap2D> source_snapshot,
      transform::Rigid2d origin_snapshot);

  // 异步启动 BuildLocalSubmap，完成后替换 local_submap_
  void TriggerAsyncRebuild();

  // 实际执行插入（在低优先级线程池中调用，不持 data_mutex_）
  struct DynamicUpdateBatch {
    std::vector<sensor::RangeData> range_data_vector;
    sensor::PointCloud cloud;
    sensor::PointCloud highlight_cloud;
    std::map<int, std::vector<sensor::LineData>> line_data;
  };
  void ApplyDynamicUpdate(const DynamicUpdateBatch& batch);

private:
  const proto::LocalMapUpdaterOptions2D options_;
  const mapping::proto::SubmapsOptions2D submap_options_;

  std::shared_ptr<mapping::Submap2D> global_static_submap_;
  std::shared_ptr<mapping::Submap2D> global_dynamic_submap_;

  std::shared_ptr<mapping::Submap2D> local_submap_;

  std::shared_ptr<transform::Rigid2d> robot_pose_;
  std::shared_ptr<transform::Rigid2d> local_map_origin_;  // 当前 local_submap_ 的中心位姿

  sensor::CropBox::Box box_;

  bool is_local_map_updated_ = false;;

  // ValueConversionTables用于ProbabilityGrid
  mapping::ValueConversionTables conversion_tables_;
  
  // RangeDataInserter用于插入range data到global_dynamic_submap_
  std::unique_ptr<mapping::RangeDataInserterInterface> range_data_inserter_;

  // data_mutex_ 保护元数据（robot_pose_, local_map_origin_, local_submap_, 等）
  // submap_mutex_ 保护 global_dynamic_submap_ 的内部数据（grid/ikdtree/line_map）
  // 两者独立，避免插入操作阻塞元数据读写
  mutable std::mutex data_mutex_;
  mutable std::shared_mutex submap_mutex_;  // 读写锁：writer=InsertWorker，reader=BuildLocalSubmap

  std::future<std::shared_ptr<mapping::Submap2D>> build_future_;
  std::atomic<bool> is_building_{false};

  // chunk 模式重建触发：跨 chunk（LoadOnPose 返回 true）置位，实际重建受 is_building_ 节流；
  // 用标记而非一次性条件，自愈"建图过程中又跨格被跳过"的情况（标记保留到下次能建为止）。
  bool pending_chunk_rebuild_ = false;

  // ===== Chunk 化 =====
  // chunk_map_ 非空（被注入）时，local submap 改由 ChunkMap2D 从覆盖窗口的 chunk 拼装，
  // 替代对全局点云的 O(N) 全量 BoxFilter；为空则退回原 BoxFilter 路径。
  // A/B 由 LocalizationUpdater 侧 chunk_map_options.use_chunk_map 控制：关→不建/不注入→走 BoxFilter。
  // chunk_map_ 由 LocalizationUpdater::LoadState 创建后经 SetChunkMap 传入（本类不自建、无独立开关）。
  std::shared_ptr<ChunkMap2D> chunk_map_;

  // 当前地图目录（SetDebugDir 注入）：debug data 落到 <map_dir_>/debug/（及用 chunk 时
  // <map_dir_>/debug/all_chunks/）。空（未注入）时跳过 debug 落盘。
  std::string map_dir_;

  // insert_thread_pool_ 必须声明在最后：C++ 逆序析构保证线程池先于其他成员销毁，
  // 即所有待执行的插入任务完成后，range_data_inserter_、global_dynamic_submap_ 等才析构。
  // ThreadPool(1) 使用 nice(10) 低优先级，确保地图更新让出 CPU 给定位主线程。
  common::ThreadPool insert_thread_pool_{1};
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
