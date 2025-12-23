#ifndef CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_

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
  ~LocalMapUpdater2D() = default;

  void SetGlobalSubmap(std::shared_ptr<mapping::Submap2D> global_submap);

  void UpdateRobotPose(const transform::Rigid2d& robot_pose);
  
  // 更新 global_dynamic_submap_
  void UpdateDynamicMap(const std::vector<sensor::RangeData>& range_data_vector,
                        const sensor::PointCloud& cloud,
                        const sensor::PointCloud& highlight_cloud,
                        const std::map<int, std::vector<sensor::LineData>>& line_data);

  bool IsLocalMapUpdated() const { return is_local_map_updated_; }

  std::shared_ptr<mapping::Submap2D> local_submap();

  std::shared_ptr<transform::Rigid2d> origin_pose();
  std::shared_ptr<transform::Rigid2d> robot_pose();

private:
  bool NeedUpdateLocalMap();

  void UpdateLocalMap();

private:
  const proto::LocalMapUpdaterOptions2D options_;
  const mapping::proto::SubmapsOptions2D submap_options_;

  std::shared_ptr<mapping::Submap2D> global_static_submap_;
  std::shared_ptr<mapping::Submap2D> global_dynamic_submap_;

  std::shared_ptr<mapping::Submap2D> local_submap_;

  std::shared_ptr<transform::Rigid2d> robot_pose_;
  std::shared_ptr<transform::Rigid2d> origin_pose_;

  sensor::CropBox::Box box_;
  std::unique_ptr<sensor::BoxFilter> box_filter_;

  bool is_local_map_updated_ = false;;

  // ValueConversionTables用于ProbabilityGrid
  mapping::ValueConversionTables conversion_tables_;
  
  // RangeDataInserter用于插入range data到global_dynamic_submap_
  std::unique_ptr<mapping::RangeDataInserterInterface> range_data_inserter_;

  std::mutex data_mutex_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
