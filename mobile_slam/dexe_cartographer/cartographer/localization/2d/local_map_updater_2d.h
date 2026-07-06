#ifndef CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_

#include <atomic>
#include <future>
#include <memory>
#include <mutex>

#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/proto/submaps_options_2d.pb.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/localization/proto/local_map_updater_options_2d.pb.h"
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

private:
  // 判断是否需要触发局部地图重建（机器人接近当前 local map 边界）
  bool NeedRebuildLocalMap() const;

  // 纯计算函数：根据快照构建 local submap，可在后台线程安全调用
  std::shared_ptr<mapping::Submap2D> BuildLocalSubmap(
      std::shared_ptr<mapping::Submap2D> source_snapshot,
      transform::Rigid2d origin_snapshot);

  // 异步启动 BuildLocalSubmap，完成后替换 local_submap_
  void TriggerAsyncRebuild();

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

  mutable std::mutex data_mutex_;

  std::future<std::shared_ptr<mapping::Submap2D>> build_future_;
  std::atomic<bool> is_building_{false};
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
