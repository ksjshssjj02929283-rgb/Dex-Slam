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
  
  // 读取动态地图策略参数
  if (parameter_dictionary->HasKey("dynamic_map_policy")) {
    int policy_value = parameter_dictionary->GetInt("dynamic_map_policy");
    options.set_dynamic_map_policy(static_cast<proto::DynamicMapPolicy>(policy_value));
  } else {
    // 默认使用 LONG 策略
    options.set_dynamic_map_policy(proto::DynamicMapPolicy::LONG);
  }
  
  // 读取调试数据保存选项
  if (parameter_dictionary->HasKey("save_debug_data")) {
    options.set_save_debug_data(parameter_dictionary->GetBool("save_debug_data"));
  } else {
    // 默认不保存调试数据
    options.set_save_debug_data(false);
  }
  
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

LocalMapUpdater2D::~LocalMapUpdater2D() {
  if (build_future_.valid()) build_future_.wait();
}

void LocalMapUpdater2D::SetGlobalSubmap(std::shared_ptr<mapping::Submap2D> global_submap)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_static_submap_ = global_submap;
  
  // 根据动态地图策略初始化 global_dynamic_submap_
  if (!global_static_submap_) {
    return;
  }
  
  // 策略1: NONE - 不使用dynamic，也不更新dynamic map
  if (options_.dynamic_map_policy() == proto::DynamicMapPolicy::NONE) {
    LOG(INFO) << "DynamicMapPolicy is NONE, not initializing dynamic submap";
    global_dynamic_submap_ = nullptr;
    return;
  }
  
  // 策略2: SHORT - 暂未实现
  // 策略3: LONG - 使用dynamic，但不保存到static map
  // 策略4: PERSISTENT - 使用dynamic，并保存到static map
  // 对于策略2/3/4，都需要从static map初始化dynamic map
  
  LOG(INFO) << "Initializing global_dynamic_submap from global_static_submap, policy: " 
            << static_cast<int>(options_.dynamic_map_policy());
  
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
  // TODO: 后续如果需要使用 highlight_cloud，可以从 global_static_submap_ 中深拷贝
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
      
  // 继承静态底图的线特征 culling 判据（其判据来自 pbstream 内嵌的建图期
  // submaps_options）。必须在 Finish() 之前 —— Finish() 会立即触发一次
  // LineDataCulling()，此时若还是类内默认值，建图判据与默认值的差集会被剔除。
  global_dynamic_submap_->InheritLineCullingParamFrom(*global_static_submap_);

  // 重建ikd-trees
  global_dynamic_submap_->RebuildIkdTrees();
  if (options_.dynamic_map_policy() == proto::DynamicMapPolicy::NONE) {
    global_dynamic_submap_->Finish();
    LOG(INFO) << "Dynamic map policy is NONE, global_dynamic_submap_ is a read-only copy of global_static_submap_";
  } else {
    LOG(INFO) << "Dynamic map initialized from static map, ready for updates";
  }

  // 同步构建初始 local_submap_（以全局地图原点为中心），保证之后 local_submap() 永不返回空
  transform::Rigid2d init_origin(origin.cast<double>(), Eigen::Rotation2Dd(0.0));
  local_submap_     = BuildLocalSubmap(global_dynamic_submap_, init_origin);
  local_map_origin_ = std::make_shared<transform::Rigid2d>(init_origin);
  LOG(INFO) << "Initial local_submap_ built synchronously at origin ("
            << origin.x() << ", " << origin.y() << ")";
}

void LocalMapUpdater2D::UpdateRobotPose(const transform::Rigid2d& robot_pose)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    robot_pose_ = std::make_shared<transform::Rigid2d>(robot_pose);
  }

  is_local_map_updated_ = false;

  // 后台任务完成 → 替换 local_submap_
  if (build_future_.valid() &&
      build_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    if (auto new_submap = build_future_.get()) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      local_submap_         = std::move(new_submap);
      is_local_map_updated_ = true;
      LOG(INFO) << "local_submap_ replaced at ("
                << robot_pose.translation().transpose() << ")";
    }
    is_building_ = false;
  }

  if (NeedRebuildLocalMap()) {
    TriggerAsyncRebuild();
  }
}

// ============================================================
// NeedRebuildLocalMap：机器人接近当前 local map 边界时返回 true
// ============================================================

bool LocalMapUpdater2D::NeedRebuildLocalMap() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!robot_pose_ || !local_map_origin_) return false;

  const double dx = robot_pose_->translation().x() - local_map_origin_->translation().x();
  const double dy = robot_pose_->translation().y() - local_map_origin_->translation().y();
  const double half_x = (box_.max_x - box_.min_x) * 0.5;
  const double half_y = (box_.max_y - box_.min_y) * 0.5;
  const double thresh  = options_.update_threshold();

  return (std::abs(dx) > half_x - thresh || std::abs(dy) > half_y - thresh);
}

// ============================================================
// TriggerAsyncRebuild：启动后台线程，完成后在下次 UpdateRobotPose 中替换
// ============================================================

void LocalMapUpdater2D::TriggerAsyncRebuild()
{
  if (is_building_.exchange(true)) return;  // 已有后台任务，跳过

  std::shared_ptr<mapping::Submap2D> source_snapshot;
  transform::Rigid2d origin_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_dynamic_submap_ || !robot_pose_) {
      is_building_ = false;
      return;
    }
    source_snapshot   = global_dynamic_submap_;
    origin_snapshot   = *robot_pose_;
    local_map_origin_ = robot_pose_;  // 预更新原点，防止在后台完成前重复触发
  }

  build_future_ = std::async(
      std::launch::async,
      [this, source_snapshot, origin_snapshot]() {
        common::TicToc t;
        // source_snapshot is only a shared_ptr copy, not an immutable map
        // snapshot. UpdateDynamicMap() mutates the same Submap2D under
        // data_mutex_, so rebuilding without the lock races with grid/cloud/
        // line-map insertion and can intermittently crash.
        std::shared_ptr<mapping::Submap2D> result;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          result = BuildLocalSubmap(source_snapshot, origin_snapshot);
        }
        LOG(INFO) << "Async BuildLocalSubmap done in " << t.toc() << " ms";
        return result;
      });
}

// ============================================================
// BuildLocalSubmap：纯计算；调用方必须保证 source_submap 在构建期间不被写入
// ============================================================

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::BuildLocalSubmap(
    std::shared_ptr<mapping::Submap2D> source_submap,
    transform::Rigid2d origin)
{
  const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(
      source_submap->grid());
  if (!prob_grid) {
    LOG(ERROR) << "BuildLocalSubmap: source grid is not ProbabilityGrid.";
    return nullptr;
  }
  auto global_cloud = source_submap->cloud();
  if (!global_cloud) {
    LOG(ERROR) << "BuildLocalSubmap: source cloud is null.";
    return nullptr;
  }

  const double ox  = origin.translation().x();
  const double oy  = origin.translation().y();
  const double res = prob_grid->limits().resolution();
  const int    nx  = static_cast<int>((box_.max_x - box_.min_x) / res);
  const int    ny  = static_cast<int>((box_.max_y - box_.min_y) / res);

  // ============ 1. 裁剪栅格 ============
  mapping::MapLimits map_limits(
      res,
      Eigen::Vector2d(ox + 0.5 * nx * res, oy + 0.5 * ny * res),
      mapping::CellLimits(nx, ny));
  auto local_grid = std::make_unique<mapping::ProbabilityGrid>(
      map_limits, &conversion_tables_);
  const auto& gl = prob_grid->limits();
  for (int row = 0; row < ny; ++row) {
    for (int col = 0; col < nx; ++col) {
      Eigen::Array2i li(row, col);
      Eigen::Array2i gi = gl.GetCellIndex(map_limits.GetCellCenter(li));
      if (!gl.Contains(gi)) {
        local_grid->SetProbability(li, mapping::kMinProbability);
      } else if (prob_grid->IsKnown(gi)) {
        local_grid->SetProbability(li, prob_grid->GetProbability(gi));
      }
    }
  }

  // ============ 2. 裁剪点云（临时 BoxFilter，线程安全）============
  sensor::BoxFilter tmp_filter(box_);
  tmp_filter.SetOrigin(Eigen::Vector3f(static_cast<float>(ox),
                                       static_cast<float>(oy), 0.0f));
  auto local_cloud = std::make_unique<sensor::PointCloud>();
  tmp_filter.Filter(*global_cloud, *local_cloud);

  // ============ 3. 线地图 ============
  int latest_line_id = 0;
  std::map<int, mapping::MapLine2D> active_line_data;
  if (auto* lm = source_submap->line_map_2d_list()) {
    latest_line_id   = lm->latest_local_id();
    active_line_data = lm->GetLineMapData2DList();
  }

  // ============ 4. 组装并构建 ikd-tree ============
  auto new_submap = std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(static_cast<float>(ox), static_cast<float>(oy)),
      std::move(local_grid),
      nullptr,  // local_submap 只用于匹配，不需要 conversion_tables
      std::move(local_cloud), std::make_unique<sensor::PointCloud>(),
      latest_line_id, active_line_data);
  // 同 SetGlobalSubmap：深拷贝不带 culling 判据，需从源 submap 继承后再 Finish()。
  new_submap->InheritLineCullingParamFrom(*source_submap);
  new_submap->RebuildIkdTrees();
  new_submap->Finish();

  // 调试：如果启用，将 submap 的 grid map 保存为图片和点云文件
  if (options_.save_debug_data()) {
    const std::string dir =
        std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") + "/.dexe/data/";
    new_submap->grid()->DrawAsImage(dir, "local_map.png");
    new_submap->SaveSubmapCloud(dir, "local_submap.pcd");
  }
  return new_submap;
}

void LocalMapUpdater2D::UpdateDynamicMap(
    const std::vector<sensor::RangeData>& range_data_vector,
    const sensor::PointCloud& cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 策略1: NONE - 不更新dynamic map
  if (options_.dynamic_map_policy() == proto::DynamicMapPolicy::NONE) {
    return;
  }
  
  if (!global_dynamic_submap_) {
    LOG(ERROR) << "Global dynamic submap is not initialized, cannot update dynamic map.";
    return;
  }
  
  // 策略2/3/4: 更新dynamic map
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

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::GetGlobalSubmap() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  // 统一返回 global_dynamic_submap_（不论策略如何）
  // 所有匹配计算都使用 dynamic map，static map 仅作为底图数据源
  return global_dynamic_submap_;
}

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::GetStaticSubmap() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // 漂移免疫底图：SetGlobalSubmap 后从不再插入数据，供漂移门控做独立锚点
  return global_static_submap_;
}

void LocalMapUpdater2D::ApplyOriginTransform(const transform::Rigid3d& T)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // dynamic map 参与保存和定位匹配，需要同步到新坐标系。
  if (global_dynamic_submap_) {
    global_dynamic_submap_->set_local_pose(T * global_dynamic_submap_->local_pose());
  }

  // local submap 在定位过程中也会被使用，避免 reset 后短时间内坐标不一致。
  if (local_submap_) {
    local_submap_->set_local_pose(T * local_submap_->local_pose());
  }

  // 运行时状态为 2D 位姿，同步应用 2D 变换。
  const transform::Rigid2d t_2d = transform::Project2D(T);
  if (robot_pose_) {
    *robot_pose_ = t_2d * (*robot_pose_);
  }
  if (local_map_origin_) {
    *local_map_origin_ = t_2d * (*local_map_origin_);
  }
}

void LocalMapUpdater2D::RebuildLocalSubmapAt(const transform::Rigid2d& pose)
{
  std::shared_ptr<mapping::Submap2D> source_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_dynamic_submap_) {
      LOG(ERROR) << "RebuildLocalSubmapAt: global_dynamic_submap_ is not initialized.";
      return;
    }
    source_snapshot = global_dynamic_submap_;
  }

  // 纯计算，无需持锁
  common::TicToc t;
  auto new_submap = BuildLocalSubmap(source_snapshot, pose);
  if (!new_submap) {
    LOG(ERROR) << "RebuildLocalSubmapAt: BuildLocalSubmap returned nullptr.";
    return;
  }
  LOG(INFO) << "RebuildLocalSubmapAt done in " << t.toc() << " ms at ("
            << pose.translation().x() << ", " << pose.translation().y() << ")";

  // 就地替换内部 local_submap_ 和 local_map_origin_
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_submap_ = std::move(new_submap);
    local_map_origin_ = std::make_shared<transform::Rigid2d>(pose);
  }
}

void LocalMapUpdater2D::RebuildLocalSubmapFromStaticAt(const transform::Rigid2d& pose)
{
  std::shared_ptr<mapping::Submap2D> source_snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_static_submap_) {
      LOG(ERROR) << "RebuildLocalSubmapFromStaticAt: global_static_submap_ is not initialized.";
      return;
    }
    // 从【漂移免疫】的静态底图裁剪，而非会随定位漂移污染的 global_dynamic_submap_。
    source_snapshot = global_static_submap_;
  }

  // 纯计算，无需持锁
  common::TicToc t;
  auto new_submap = BuildLocalSubmap(source_snapshot, pose);
  if (!new_submap) {
    LOG(ERROR) << "RebuildLocalSubmapFromStaticAt: BuildLocalSubmap returned nullptr.";
    return;
  }
  LOG(INFO) << "RebuildLocalSubmapFromStaticAt done in " << t.toc() << " ms at ("
            << pose.translation().x() << ", " << pose.translation().y() << ")";

  // 就地替换内部 local_submap_ 和 local_map_origin_
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_submap_ = std::move(new_submap);
    local_map_origin_ = std::make_shared<transform::Rigid2d>(pose);
  }
}

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::local_submap()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_submap_;
}

std::shared_ptr<transform::Rigid2d> LocalMapUpdater2D::origin_pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return local_map_origin_;
}

std::shared_ptr<transform::Rigid2d> LocalMapUpdater2D::robot_pose()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return robot_pose_;
}


}  // namespace localization
}  // namespace cartographer
