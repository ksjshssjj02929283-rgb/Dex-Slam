#include "cartographer/localization/2d/local_map_updater_2d.h"
#include "cartographer/common/tic_toc.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"

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
        const proto::LocalMapUpdaterOptions2D& options,
        const mapping::proto::SubmapsOptions2D& submap_options)
: options_(options),
  submap_options_(submap_options)
{
  box_ = {options_.min_x(), options_.min_y(), options_.min_z(),
          options_.max_x(), options_.max_y(), options_.max_z()};
  box_filter_ = std::make_unique<sensor::BoxFilter>(box_);
  
  // 根据submap_options创建range_data_inserter
  const auto& inserter_options = submap_options_.range_data_inserter_options();
  switch (inserter_options.range_data_inserter_type()) {
    case mapping::proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
      range_data_inserter_ = 
          std::make_unique<mapping::ProbabilityGridRangeDataInserter2D>(
              inserter_options.probability_grid_range_data_inserter_options_2d());
      break;
    case mapping::proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
      range_data_inserter_ = 
          std::make_unique<mapping::TSDFRangeDataInserter2D>(
              inserter_options.tsdf_range_data_inserter_options_2d());
      break;
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType for LocalMapUpdater2D.";
  }
  LOG(INFO) << "Created LocalMapUpdater2D with range_data_inserter";
}

void LocalMapUpdater2D::SetGlobalSubmap(std::shared_ptr<mapping::Submap2D> global_submap)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_static_submap_ = global_submap;
  
  // 使用 global_static_submap_ 初始化 global_dynamic_submap_
  if (global_static_submap_) {
    // 获取 global_static_submap_ 的数据
    auto global_grid = global_static_submap_->grid();
    auto global_cloud = global_static_submap_->cloud();
    auto global_line_map = global_static_submap_->line_map_2d_list();
    
    // 深拷贝 grid
    std::unique_ptr<mapping::Grid2D> grid_copy;
    if (global_grid) {
      const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(global_grid);
      if (prob_grid) {
        grid_copy = std::make_unique<mapping::ProbabilityGrid>(*prob_grid);
      }
    }
    
    // 深拷贝 cloud
    std::unique_ptr<sensor::PointCloud> cloud_copy;
    if (global_cloud) {
      cloud_copy = std::make_unique<sensor::PointCloud>(*global_cloud);
    } else {
      cloud_copy = std::make_unique<sensor::PointCloud>();
    }
    
    // 创建空的 highlight_cloud
    std::unique_ptr<sensor::PointCloud> highlight_cloud_copy = 
        std::make_unique<sensor::PointCloud>();
    
    // 深拷贝 line_map
    int latest_line_id = 0;
    std::map<int, mapping::MapLine2D> active_line_data;
    if (global_line_map) {
      latest_line_id = global_line_map->latest_local_id();
      active_line_data = global_line_map->GetLineMapData2DList();
    }
    
    // 创建 global_dynamic_submap_
    // 将 Eigen::Vector2d 转换为 Eigen::Vector2f
    Eigen::Vector2f origin = global_static_submap_->local_pose().translation().head<2>().cast<float>();
    global_dynamic_submap_ = std::make_shared<mapping::Submap2D>(
        origin,
        std::move(grid_copy),
        &conversion_tables_,  // 使用 conversion_tables_
        std::move(cloud_copy),
        std::move(highlight_cloud_copy),
        latest_line_id,
        active_line_data);
        
    // 重建ikd-trees
    global_dynamic_submap_->RebuildIkdTrees();
  }
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
  if (!global_dynamic_submap_) {
    LOG(ERROR) << "Global dynamic submap is not initialized, please set global submap first.";
    return;
  }

  if (robot_pose_ == nullptr) {
    LOG(ERROR) << "Robot pose is not set, please set robot pose first.";
    return;
  }
  origin_pose_ = robot_pose_;

  // ============ 1. 更新 Local Grid Map ============
  auto global_grid = global_dynamic_submap_->grid();
  CHECK(global_grid != nullptr);
  const auto* global_probability_grid = dynamic_cast<const mapping::ProbabilityGrid*>(global_grid);
  CHECK(global_probability_grid != nullptr);
  
  // 构建MapLimits
  double origin_x = origin_pose_->translation().x();
  double origin_y = origin_pose_->translation().y();
  double resolution = global_probability_grid->limits().resolution();
  int num_x_cells = (box_.max_x - box_.min_x) / resolution;
  int num_y_cells = (box_.max_y - box_.min_y) / resolution;
  
  // 以机器人位置为中心构建局部地图边界
  Eigen::Vector2d max = Eigen::Vector2d(origin_x + 0.5 * num_x_cells * resolution,
                                        origin_y + 0.5 * num_y_cells * resolution);

  mapping::CellLimits cell_limits(num_x_cells, num_y_cells);
  mapping::MapLimits map_limits(resolution, max, cell_limits);

  // 创建local probability grid
  std::unique_ptr<mapping::ProbabilityGrid> local_grid = 
      std::make_unique<mapping::ProbabilityGrid>(map_limits, &conversion_tables_);

  const mapping::MapLimits& global_map_limits = global_probability_grid->limits();

  // 给local grid赋值
  for (int row = 0; row < num_y_cells; ++row) {
    for (int col = 0; col < num_x_cells; ++col) {
      Eigen::Array2i local_cell_index(row, col);
      Eigen::Vector2f sub_cell_center = map_limits.GetCellCenter(local_cell_index);
      Eigen::Array2i global_cell_index = global_map_limits.GetCellIndex(sub_cell_center);
      if (global_map_limits.Contains(global_cell_index)) {
        if (!global_probability_grid->IsKnown(global_cell_index)) {
          continue;
        }
        float prob = global_probability_grid->GetProbability(global_cell_index);
        local_grid->SetProbability(local_cell_index, prob);
      } else {
        local_grid->SetProbability(local_cell_index, mapping::kMinProbability);
      }
    }
  }

  // ============ 2. 更新 Local Cloud Map ============
  auto global_cloud = global_dynamic_submap_->cloud();
  CHECK(global_cloud != nullptr);

  box_filter_->SetOrigin(Eigen::Vector3f(origin_pose_->translation().x(),
                                         origin_pose_->translation().y(),
                                         0.0));
  std::unique_ptr<sensor::PointCloud> local_cloud(new sensor::PointCloud());
  box_filter_->Filter(*global_cloud, *local_cloud);

  // ============ 3. 更新 Local Line Map ============
  auto global_line_map = global_dynamic_submap_->line_map_2d_list();
  int latest_line_id = 0;
  std::map<int, mapping::MapLine2D> active_line_data;
  if (global_line_map) {
    latest_line_id = global_line_map->latest_local_id();
    active_line_data = global_line_map->GetLineMapData2DList();
  }

  // ============ 4. 创建 Local Submap ============
  // 创建空的 highlight_cloud
  std::unique_ptr<sensor::PointCloud> local_highlight_cloud = 
      std::make_unique<sensor::PointCloud>();

  local_submap_ = std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(static_cast<float>(origin_x), static_cast<float>(origin_y)),
      std::move(local_grid),
      nullptr,  // local_submap只用于匹配，不需要conversion_tables
      std::move(local_cloud),
      std::move(local_highlight_cloud),
      latest_line_id,
      active_line_data);
      
  // 重建ikd-trees
  local_submap_->RebuildIkdTrees();

  // 调试：将submap的grid map保存为图片
  // save debug submap
  std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  std::string data_dir = home_dir + "/.dexe/data/";
  global_static_submap_->grid()->DrawAsImage(data_dir, "global_static_map.png");
  global_dynamic_submap_->grid()->DrawAsImage(data_dir, "global_dynamic_map.png");
  local_submap_->grid()->DrawAsImage(data_dir, "local_map.png");
  global_static_submap_->SaveSubmapCloud(data_dir, "global_static_submap.pcd");
  global_dynamic_submap_->SaveSubmapCloud(data_dir, "global_dynamic_submap.pcd");
  local_submap_->SaveSubmapCloud(data_dir, "local_submap.pcd");

  LOG(WARNING) << "Update local map at " << robot_pose_->translation().x() 
               << ", " << robot_pose_->translation().y();
}

void LocalMapUpdater2D::UpdateDynamicMap(
    const std::vector<sensor::RangeData>& range_data_vector,
    const sensor::PointCloud& cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!global_dynamic_submap_) {
    LOG(ERROR) << "Global dynamic submap is not initialized, cannot update dynamic map.";
    return;
  }
  
  // 确保global_dynamic_submap_允许插入数据
  if (global_dynamic_submap_->insertion_finished()) {
    global_dynamic_submap_->set_insertion_finished(false);
    LOG(INFO) << "Reopened global_dynamic_submap_ for data insertion";
  }
  
  // 1. 插入所有range data到global_dynamic_submap_（更新栅格地图）
  CHECK(range_data_inserter_ != nullptr);
  for (const auto& range_data : range_data_vector) {
    global_dynamic_submap_->InsertRangeData(range_data, range_data_inserter_.get());
  }
  
  // 2. 插入点云数据到global_dynamic_submap_
  // InsertCloudData内部会自动增量更新ikd-tree，不需要手动rebuild
  global_dynamic_submap_->InsertCloudData(cloud, highlight_cloud);
  
  // 3. 插入线数据到global_dynamic_submap_
  global_dynamic_submap_->InsertLineData(line_data);
  
  // LOG(INFO) << "UpdateDynamicMap: inserted " << range_data_vector.size() 
  //           << " range_datas, " << cloud.size() << " cloud points, "
  //           << line_data.size() << " line groups";
}

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::local_submap()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_submap_;
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
