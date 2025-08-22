#ifndef CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_

#include <memory>
#include <mutex>

#include "cartographer/localization/2d/grid_map_2d.h"
#include "cartographer/localization/2d/cloud_map_2d.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/localization/proto/local_map_updater_options_2d.pb.h"
#include "cartographer/sensor/internal/box_filter.h"
#include "cartographer/ikd_tree/ikd_tree.h"

namespace cartographer {
namespace localization {

proto::LocalMapUpdaterOptions2D CreateLocalMapUpdaterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

class LocalMapUpdater2D
{
public:
  LocalMapUpdater2D(const proto::LocalMapUpdaterOptions2D& options);
  ~LocalMapUpdater2D() = default;

  void SetGlobalGridMap(std::shared_ptr<GridMap2D> grid_map);
  void SetGlobalCloudMap(std::shared_ptr<CloudMap2D> cloud_map);

  void UpdateRobotPose(const transform::Rigid2d& robot_pose);

  bool IsLocalMapUpdated() const { return is_local_map_updated_; }

  std::shared_ptr<GridMap2D> global_grid_map();
  std::shared_ptr<CloudMap2D> global_cloud_map();

  std::shared_ptr<GridMap2D> local_grid_map();
  std::shared_ptr<CloudMap2D> local_cloud_map();
  std::shared_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> local_cloud_map_ikdtree();

  std::shared_ptr<transform::Rigid2d> origin_pose();
  std::shared_ptr<transform::Rigid2d> robot_pose();

private:
  bool NeedUpdateLocalMap();

  void UpdateLocalMap();
  void UpdateLocalGridMap();
  void UpdateLocalCloudMap();

private:
  const proto::LocalMapUpdaterOptions2D options_;

  std::shared_ptr<GridMap2D> global_grid_map_;
  std::shared_ptr<CloudMap2D> global_cloud_map_;

  std::shared_ptr<GridMap2D> local_grid_map_;
  std::shared_ptr<CloudMap2D> local_cloud_map_;

  std::shared_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> local_cloud_ikdtree_ = nullptr;

  std::shared_ptr<transform::Rigid2d> robot_pose_;
  std::shared_ptr<transform::Rigid2d> origin_pose_;

  sensor::CropBox::Box box_;
  std::unique_ptr<sensor::BoxFilter> box_filter_;

  bool is_local_map_updated_ = false;;

  std::mutex data_mutex_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_LOCAL_MAP_UPDATER_2D_H_
