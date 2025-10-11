#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "cartographer/localization/2d/local_map_updater_2d.h"
#include "cartographer/common/tic_toc.h"

namespace cartographer {
namespace localization {

proto::LocalMapUpdaterOptions2D CreateLocalMapUpdaterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary)
{
  proto::LocalMapUpdaterOptions2D options;
  options.set_min_x(parameter_dictionary->GetDouble("min_x"));
  options.set_min_y(parameter_dictionary->GetDouble("min_y"));
  options.set_min_z(parameter_dictionary->GetDouble("min_z"));
  options.set_max_x(parameter_dictionary->GetDouble("max_x"));
  options.set_max_y(parameter_dictionary->GetDouble("max_y"));
  options.set_max_z(parameter_dictionary->GetDouble("max_z"));
  options.set_update_threshold(parameter_dictionary->GetDouble("update_threshold"));
  return options;
}

LocalMapUpdater2D::LocalMapUpdater2D(
        const proto::LocalMapUpdaterOptions2D& options)
: options_(options)
{
  box_ = {options_.min_x(), options_.min_y(), options_.min_z(),
          options_.max_x(), options_.max_y(), options_.max_z()};
  box_filter_ = std::make_unique<sensor::BoxFilter>(box_);
}

void LocalMapUpdater2D::SetGlobalGridMap(std::shared_ptr<GridMap2D> grid_map)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_grid_map_ = grid_map;
}

void LocalMapUpdater2D::SetGlobalCloudMap(std::shared_ptr<CloudMap2D> cloud_map)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_cloud_map_ = cloud_map;
}

void LocalMapUpdater2D::UpdateRobotPose(const transform::Rigid2d& robot_pose)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_ = std::make_shared<transform::Rigid2d>(robot_pose);
  }

  if (NeedUpdateLocalMap()) {
    common::TicToc tic_toc;
    UpdateLocalMap();
    LOG(INFO) << "Update local map cost: " << tic_toc.toc() << " ms";
    is_local_map_updated_ = true;
  } else {
    is_local_map_updated_ = false;
  }
}

bool LocalMapUpdater2D::NeedUpdateLocalMap()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  CHECK(robot_pose_ != nullptr);

  // 第一次更新
  if (origin_pose_ == nullptr) {
    return true;
  }

  std::vector<float> edge = box_filter_->GetEdge();

  if (fabs(robot_pose_->translation().x() - edge.at(0)) < options_.update_threshold() ||
      fabs(robot_pose_->translation().x() - edge.at(1)) < options_.update_threshold() ||
      fabs(robot_pose_->translation().y() - edge.at(2)) < options_.update_threshold() ||
      fabs(robot_pose_->translation().y() - edge.at(3)) < options_.update_threshold())
  {
    return true;
  }

  return false;
}

void LocalMapUpdater2D::UpdateLocalMap()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (global_grid_map_ == nullptr) {
    LOG(ERROR) << "Global grid map is not initialized, please set global grid map first.";
    return;
  }

  if (global_cloud_map_ == nullptr) {
    LOG(ERROR) << "Global cloud map is not initialized, please set global cloud map first.";
    return;
  }

  if (robot_pose_ == nullptr) {
    LOG(ERROR) << "Robot pose is not set, please set robot pose first.";
    return;
  }
  origin_pose_ = robot_pose_;

  UpdateLocalGridMap();
  UpdateLocalCloudMap();

  LOG(WARNING) << "Update local map at " << robot_pose_->translation().x() << ", " << robot_pose_->translation().y();
}

void LocalMapUpdater2D::UpdateLocalGridMap()
{
  CHECK(global_grid_map_ != nullptr);
  CHECK(origin_pose_ != nullptr);
  // 1. 构建MapLimits
  double origin_x = origin_pose_->translation().x();
  double origin_y = origin_pose_->translation().y();
  double resolution = global_grid_map_->limits().resolution();
  int num_x_cells = (box_.max_x - box_.min_x) / resolution;
  int num_y_cells = (box_.max_y - box_.min_y) / resolution;
  
  // 以机器人位置为中心构建局部地图边界
  Eigen::Vector2d max = Eigen::Vector2d(origin_x + 0.5 * num_x_cells * resolution,
                                        origin_y + 0.5 * num_y_cells * resolution);

  // 注意：CellLimits构造函数的参数顺序是(num_x_cells, num_y_cells)
  // 但内部索引约定是 cell_index[0]=行(y方向), cell_index[1]=列(x方向)
  mapping::CellLimits cell_limits(num_x_cells, num_y_cells);
  mapping::MapLimits map_limits(resolution, max, cell_limits);

  // 2. create submap
  local_grid_map_.reset(new GridMap2D(map_limits));

  const mapping::MapLimits& global_map_limits =
                                global_grid_map_->limits();

  // 给submap赋值
  // 注意：Cartographer的索引约定是 Array2i[0]=行(y), Array2i[1]=列(x)
  // 使用更清晰的变量名：row对应y方向，col对应x方向  
  for (int row = 0; row < num_y_cells; ++row) {      // row对应y方向(Array2i[0])
    for (int col = 0; col < num_x_cells; ++col) {    // col对应x方向(Array2i[1])
      Eigen::Array2i local_cell_index(row, col);  // [0]=row(y), [1]=col(x)
      Eigen::Vector2f sub_cell_center = map_limits.GetCellCenter(local_cell_index);
      Eigen::Array2i global_cell_index = global_map_limits.GetCellIndex(sub_cell_center);
      if (global_map_limits.Contains(global_cell_index)) {
        float prob = global_grid_map_->GetProbability(global_cell_index);
        local_grid_map_->SetProbability(local_cell_index, prob);
      } else {
        local_grid_map_->SetProbability(local_cell_index, mapping::kMinProbability);
      }
    }
  }

  // 4. 设置submap的known_cells_box的边界（这个操作在SetProbability已包含）
  // sub_grid_map->mutable_known_cells_box()->extend(
  //   Eigen::Array2i(num_x_cells, num_y_cells).matrix());
  // sub_grid_map->mutable_known_cells_box()->extend(
  //   Eigen::Array2i(0, 0).matrix());

  // 5. 将submap的grid map保存为图片，用于调试
  // global_grid_map_->DrawAsImage("/home/fengbro/dexforce/0_slam/data/", "global_map.png");
  // local_grid_map_->DrawAsImage("/home/fengbro/dexforce/0_slam/data/", "loc_map.png");
}

void LocalMapUpdater2D::UpdateLocalCloudMap()
{
  CHECK(global_cloud_map_ != nullptr);
  CHECK(origin_pose_ != nullptr);

  box_filter_->SetOrigin(Eigen::Vector3f(origin_pose_->translation().x(),
                                         origin_pose_->translation().y(),
                                         0.0));
  std::shared_ptr<sensor::PointCloud> sub_cloud(new sensor::PointCloud());
  box_filter_->Filter(*(global_cloud_map_->cloud()), *sub_cloud);
  local_cloud_map_.reset(new CloudMap2D(*sub_cloud));

  pcl::PointCloud<pcl::PointXYZ>::Ptr sub_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : local_cloud_map_->cloud()->points()) {
    pcl::PointXYZ p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = 0.0;
    sub_cloud_pcl->push_back(p);
  }

  float resolution = global_grid_map_->limits().resolution();
  local_cloud_ikdtree_ = std::make_shared<ikd_tree::iKdTree<pcl::PointXYZ>>();

  local_cloud_ikdtree_->set_downsample_param(resolution);
  local_cloud_ikdtree_->Build(sub_cloud_pcl->points);

  // pcl::io::savePCDFile("/home/fengbro/dexforce/0_slam/data/loc_map.pcd", *sub_cloud_pcl);
}

std::shared_ptr<GridMap2D> LocalMapUpdater2D::global_grid_map()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return global_grid_map_;
}

std::shared_ptr<CloudMap2D> LocalMapUpdater2D::global_cloud_map()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return global_cloud_map_;
}

std::shared_ptr<GridMap2D> LocalMapUpdater2D::local_grid_map()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_grid_map_;
}

std::shared_ptr<CloudMap2D> LocalMapUpdater2D::local_cloud_map()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_cloud_map_;
}

std::shared_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> LocalMapUpdater2D::local_cloud_map_ikdtree()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_cloud_ikdtree_;
}

std::shared_ptr<transform::Rigid2d> LocalMapUpdater2D::origin_pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return origin_pose_;
}

std::shared_ptr<transform::Rigid2d> LocalMapUpdater2D::robot_pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return robot_pose_;
}


}  // namespace localization
}  // namespace cartographer
