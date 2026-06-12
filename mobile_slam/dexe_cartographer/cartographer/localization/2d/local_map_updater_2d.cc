#include "cartographer/localization/2d/local_map_updater_2d.h"

#include <cmath>
#include <filesystem>

#include "cartographer/common/perf_stat.h"  // 运行时间统计（A/B 计时与重建频率）
#include "cartographer/common/task.h"
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

  // ===== Chunk 化（Phase 4-a）=====
  // 无任何 chunk 选项：开关/尺寸归 LocalizationUpdaterOptions.chunk_map_options 单一来源，
  // LocalMapUpdater2D 以“是否被注入 chunk_map_”为门控；debug 落盘由 save_debug_data 统一开关、
  // 落到注入的当前地图目录下（SetDebugDir），无需独立 chunk_debug_dir。

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

  // ===== Chunk 化 =====
  // chunk_map_ 不在此创建，也无独立开关——由 LocalizationUpdater::LoadState 创建/加载后
  // 经 SetChunkMap 注入；注入即启用（gate = chunk_map_ 非空）。
  LOG(INFO) << "Created LocalMapUpdater2D with range_data_inserter";
}

LocalMapUpdater2D::~LocalMapUpdater2D() {
  if (build_future_.valid()) build_future_.wait();
  // 退出前打印最终运行时间统计（回放到 bag 末尾或节点关闭时触发）。
  common::PerfRegistry::Get().Dump("final@LocalMapUpdater2D::dtor");
  // insert_thread_pool_ 析构时会自动 drain 队列并 join 线程，无需手动停止
}

void LocalMapUpdater2D::SetGlobalSubmap(std::shared_ptr<mapping::Submap2D> global_submap)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  global_static_submap_ = global_submap;
  
  // 根据动态地图策略初始化 global_dynamic_submap_
  if (!global_static_submap_) {
    return;
  }

  // ===== Chunk 化（T1）：chunk 模式不需要 global_dynamic_submap_ =====
  // local map 由 chunk_map_ 拼装（BuildLocalSubmap chunk 路径），完全不用 global_dynamic_submap_。
  // 跳过这里的整图深拷贝 + RebuildIkdTrees——省下一份全图 grid/cloud + 一棵全图 ikd-tree
  // + 该树的后台自旋线程 + 每帧 UpdateDynamicMap 的插入开销（chunk 模式 CPU/内存暴涨的次因）。
  // 前提：SetChunkMap 已在本函数之前注入（见 LocalizationUpdater::AddTrajectoryBuilder 顺序，bug-590）。
  if (chunk_map_ && !chunk_map_->empty()) {
    global_dynamic_submap_ = nullptr;
    Eigen::Vector2f c_origin =
        global_static_submap_->local_pose().translation().head<2>().cast<float>();
    transform::Rigid2d init_origin(c_origin.cast<double>(), Eigen::Rotation2Dd(0.0));
    // 匹配 + 更新合一的单张 active submap（不 Finish，可插入又可 Nearest_Search 匹配）。
    local_submap_     = chunk_map_->AssembleLiveSubmap(init_origin);
    local_map_origin_ = std::make_shared<transform::Rigid2d>(init_origin);
    LOG(INFO) << "chunk mode: skip global_dynamic_submap_; initial active submap "
                 "assembled from chunk_map_ at origin (" << c_origin.x() << ", "
              << c_origin.y() << ")";
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
      
  // 重建ikd-trees
  global_dynamic_submap_->RebuildIkdTrees();
  if (options_.dynamic_map_policy() == proto::DynamicMapPolicy::NONE) {
    global_dynamic_submap_->Finish();
    LOG(INFO) << "Dynamic map policy is NONE, global_dynamic_submap_ is a read-only copy of global_static_submap_";
  } else {
    LOG(INFO) << "Dynamic map initialized from static map, ready for updates";
  }

  // ===== Chunk 化 =====
  // chunk_map_ 由 LocalizationUpdater::LoadState 创建/加载，经 SetChunkMap 在本函数之前注入；
  // 本类不再在此切分/重建 chunk。若 chunk_map_ 已就位，下方初始 local_submap_ 即走 chunk 拼装路径。

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

  // 累计机器人行程（重建频率统计用：次/米）。
  {
    const double x = robot_pose.translation().x();
    const double y = robot_pose.translation().y();
    if (has_prev_pose_for_dist_) {
      const double dx = x - prev_dist_x_, dy = y - prev_dist_y_;
      common::PerfRegistry::Get().AddScalar("robot_distance_m",
                                             std::sqrt(dx * dx + dy * dy));
    }
    prev_dist_x_ = x;
    prev_dist_y_ = y;
    has_prev_pose_for_dist_ = true;
  }

  is_local_map_updated_ = false;

  // ===== Chunk 化（DEC-025）：匹配 + 更新合一的单张 active submap，**全同步**处理 =====
  // 跨 chunk 时：把旧 active submap 的累积更新切回所有覆盖 chunk + 整合进 global_static（供重定位）→
  // 从 chunk 重拼一张新的。未跨 chunk：active submap 由每帧 UpdateDynamicMap 直接更新，此处无操作。
  if (chunk_map_ && !chunk_map_->empty()) {
    const bool crossed = chunk_map_->LoadOnPose(robot_pose);
    if (crossed) {  // 中心 chunk 变化（或首次）才返回 true
      std::lock_guard<std::mutex> lock(data_mutex_);
      const bool had_prev = static_cast<bool>(local_submap_);
      // ① 旧 active submap → 切回所有覆盖 chunk + 整合进 global_static（global_grid 非空时）。
      //    持 submap_mutex_：与 save 线程的 FlushForSave 互斥，避免 flush 读到半写状态。
      if (local_submap_) {
        std::unique_lock<std::shared_mutex> wlock(submap_mutex_);
        mapping::ProbabilityGrid* global_grid =
            global_static_submap_
                ? dynamic_cast<mapping::ProbabilityGrid*>(
                      global_static_submap_->mutable_grid())
                : nullptr;
        chunk_map_->FlushLiveSubmap(*local_submap_, global_grid);
      }
      common::PerfRegistry::Get().AddScalar("rebuild_count", 1);
      // ② 从（已 flush 的）chunk 重拼单张 active submap（不 Finish→既可匹配又可插入）。
      //    PERF Dump 在 AssembleLocalSubmap 内部触发（"每次 assemble 时打印"）。
      local_submap_     = chunk_map_->AssembleLiveSubmap(robot_pose);
      local_map_origin_ = std::make_shared<transform::Rigid2d>(robot_pose);
      is_local_map_updated_ = true;
      LOG(INFO) << "[chunk-trace CROSS] robot(" << robot_pose.translation().x() << ","
                << robot_pose.translation().y() << ") had_prev=" << had_prev
                << " local_submap " << (local_submap_ ? "OK" : "NULL")
                << " (cloud=" << (local_submap_ && local_submap_->cloud() ? local_submap_->cloud()->size() : 0)
                << ") active=" << chunk_map_->NumActiveChunks() << "/" << chunk_map_->NumChunks();
      // [临时调试] 跨格时存：更新后的匹配 submap（local_map.png）+ active chunks（已 flush 的更新）。
      // 这样可对比 live_update.png（连续累积）/ local_map.png（跨格刷新的匹配图）/ chunk_*.png（flush 回的 chunk）。
      if (options_.save_debug_data() && !map_dir_.empty()) {
        const std::string dir = map_dir_ + "/debug/";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (local_submap_ && local_submap_->grid()) {
          local_submap_->grid()->DrawAsImageOnCanvas(dir, "local_map.png",
                                                     local_submap_->grid()->limits());
          local_submap_->SaveSubmapCloud(dir, "local_submap.pcd");
        }
        chunk_map_->SaveDebugDir(dir, /*active_only=*/true);  // active chunk_*.pcd/png（含 flush 更新）
      }
    }
    return;  // chunk 路径同步处理完毕，不走 async / box-edge
  }

  // ===== 非 chunk 模式：原 async 重建（box-edge 触发）=====
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
    // chunk 模式（T1）：local map 由 chunk_map_ 拼装，source 可为空（global_dynamic_submap_ 已不建）；
    // 非 chunk 模式仍需 global_dynamic_submap_ 作为裁剪源。两种模式都需要 robot_pose_。
    const bool chunk_mode = chunk_map_ && !chunk_map_->empty();
    if (!robot_pose_ || (!chunk_mode && !global_dynamic_submap_)) {
      is_building_ = false;
      return;
    }
    source_snapshot   = global_dynamic_submap_;  // chunk 模式为 null，BuildLocalSubmap chunk 路径忽略
    origin_snapshot   = *robot_pose_;
    local_map_origin_ = robot_pose_;  // 预更新原点，防止在后台完成前重复触发
  }
  common::PerfRegistry::Get().AddScalar("rebuild_count", 1);  // 实际触发的重建计数

  build_future_ = std::async(
      std::launch::async,
      [this, source_snapshot, origin_snapshot]() {
        common::TicToc t;
        auto result = BuildLocalSubmap(source_snapshot, origin_snapshot);
        LOG(INFO) << "Async BuildLocalSubmap done in " << t.toc() << " ms";
        return result;
      });
}

// ============================================================
// SetChunkMap：接收 LocalizationUpdater 创建/加载好的 chunk map（本类不自建）。
// ============================================================
void LocalMapUpdater2D::SetChunkMap(std::shared_ptr<ChunkMap2D> chunk_map) {
  chunk_map_ = std::move(chunk_map);
  if (!chunk_map_) return;
  LOG(INFO) << "LocalMapUpdater2D: chunk map injected, "
            << chunk_map_->NumChunks() << " chunks.";
  // 调试落盘：save_debug_data 开启 + 当前地图目录已注入时，把**全部** chunk 落到
  // <map_dir>/debug/all_chunks/（注入时一次性 dump 整张 chunk map，便于离线核对）。
  // 当前已加载（active）chunk 的可视化由 BuildLocalSubmap 随栅格地图逐次落到 <map_dir>/debug/。
  if (options_.save_debug_data() && !map_dir_.empty() && !chunk_map_->empty()) {
    chunk_map_->SaveDebugDir(map_dir_ + "/debug/all_chunks/", /*active_only=*/false);
  }
}

// ============================================================
// FlushForSave（DEC-024）：保存前把当前 live submap 切回 chunk + 整合进 global_static
// ============================================================
void LocalMapUpdater2D::FlushForSave() {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!chunk_map_ || chunk_map_->empty() || !local_submap_) return;
  std::unique_lock<std::shared_mutex> wlock(submap_mutex_);
  mapping::ProbabilityGrid* global_grid =
      global_static_submap_
          ? dynamic_cast<mapping::ProbabilityGrid*>(
                global_static_submap_->mutable_grid())
          : nullptr;
  chunk_map_->FlushLiveSubmap(*local_submap_, global_grid);
  LOG(INFO) << "FlushForSave: active submap flushed back to chunks + "
               "integrated into global_static.";
}

// ============================================================
// SaveChunkMap：把 chunk map 落盘到 chunks_dir/（pbstream，仅 chunk_map_ 已注入时）
// ============================================================
bool LocalMapUpdater2D::SaveChunkMap(const std::string& chunks_dir) const {
  if (!chunk_map_ || chunk_map_->empty()) {
    LOG(WARNING) << "LocalMapUpdater2D::SaveChunkMap: chunk map not available, skip";
    return false;
  }
  return chunk_map_->SaveToDir(chunks_dir);
}

// ============================================================
// BuildLocalSubmap：纯计算，无锁，可在后台线程安全调用
// ============================================================

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::BuildLocalSubmap(
    std::shared_ptr<mapping::Submap2D> source_submap,
    transform::Rigid2d origin)
{
  // local map 单次组装总耗时（chunk on / off 两路径都计），A/B 头号指标。
  // 注：save_debug_data 开启时这里含 PNG/PCD 落盘 I/O，性能实验请关 save_debug_data。
  CARTO_PERF_SCOPE("BuildLocalSubmap.total");

  // ===== Chunk 化（DEC-025）：拼装合一的单张 active submap（RebuildLocalSubmapAt 恢复用）。
  // 不 Finish——恢复后这张就是 local_submap_，要继续被每帧 UpdateDynamicMap 插入；Finish 会让
  // InsertRangeData 因 insertion_finished() 跳过，导致恢复后动态更新静默停止。匹配走未 Finish 树。
  // PERF Dump 在 AssembleLocalSubmap 内部触发（"每次 assemble 时打印"）。
  if (chunk_map_ && !chunk_map_->empty()) {
    auto local = chunk_map_->AssembleLiveSubmap(origin);
    // 调试落盘（save_debug_data + 已注入地图目录）：把当前已加载（active）chunk 与拼装出的
    // 栅格地图/点云一起落到 <map_dir>/debug/（与 SetChunkMap 落的全量 all_chunks/ 区分）。
    if (options_.save_debug_data() && !map_dir_.empty()) {
      const std::string dir = map_dir_ + "/debug/";
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      chunk_map_->SaveDebugDir(dir, /*active_only=*/true);
      if (local && local->grid()) {
        // 用与 merged.png / chunk tile 相同的"北朝上"朝向渲染（按本 grid 自身 limits 作画布），
        // 而非 DrawAsImage 的 ROTATE_90 约定——否则 local_map 与 merged 朝向不一致，叠图看着像错位。
        local->grid()->DrawAsImageOnCanvas(dir, "local_map.png",
                                           local->grid()->limits());
        local->SaveSubmapCloud(dir, "local_submap.pcd");
      }
    }
    return local;
  }

  // off 路径整体（与 chunk on 的 AssembleLocalSubmap 对位的 A/B 基准）。
  CARTO_PERF_SCOPE("BoxFilter.assemble(off)");

  // ============ Phase 1: 持共享锁做快照（尽量短）============
  std::unique_ptr<mapping::ProbabilityGrid> grid_snapshot;
  sensor::PointCloud cloud_snapshot;
  int latest_line_id = 0;
  std::map<int, mapping::MapLine2D> active_line_data;
  {
    CARTO_PERF_SCOPE("BoxFilter.snapshot_deepcopy(off)");
    std::shared_lock<std::shared_mutex> read_lock(submap_mutex_);
    const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(
        source_submap->grid());
    if (!prob_grid) {
      LOG(ERROR) << "BuildLocalSubmap: source grid is not ProbabilityGrid.";
      return nullptr;
    }
    const auto* raw_cloud = source_submap->cloud();
    if (!raw_cloud) {
      LOG(ERROR) << "BuildLocalSubmap: source cloud is null.";
      return nullptr;
    }
    // 深拷贝 grid 和 cloud（持锁期间完成，保证一致性）
    grid_snapshot = std::make_unique<mapping::ProbabilityGrid>(*prob_grid);
    cloud_snapshot = *raw_cloud;
    if (auto* lm = source_submap->line_map_2d_list()) {
      latest_line_id   = lm->latest_local_id();
      active_line_data = lm->GetLineMapData2DList();
    }
  }  // read_lock 释放，writer 可继续插入

  // ============ Phase 2: 无锁计算（使用快照，与 worker 并发安全）============
  const double ox  = origin.translation().x();
  const double oy  = origin.translation().y();
  const double res = grid_snapshot->limits().resolution();
  const int    nx  = static_cast<int>((box_.max_x - box_.min_x) / res);
  const int    ny  = static_cast<int>((box_.max_y - box_.min_y) / res);

  // ============ 1. 裁剪栅格 ============
  mapping::MapLimits map_limits(
      res,
      Eigen::Vector2d(ox + 0.5 * nx * res, oy + 0.5 * ny * res),
      mapping::CellLimits(nx, ny));
  auto local_grid = std::make_unique<mapping::ProbabilityGrid>(
      map_limits, &conversion_tables_);
  const auto& gl = grid_snapshot->limits();
  {
    // 全图栅格逐 cell 反查（O(nx*ny)，大图随规模增长的主成本之一）。
    CARTO_PERF_SCOPE("BoxFilter.grid_crop(off)");
    for (int row = 0; row < ny; ++row) {
      for (int col = 0; col < nx; ++col) {
        Eigen::Array2i li(row, col);
        Eigen::Array2i gi = gl.GetCellIndex(map_limits.GetCellCenter(li));
        if (!gl.Contains(gi)) {
          local_grid->SetProbability(li, mapping::kMinProbability);
        } else if (grid_snapshot->IsKnown(gi)) {
          local_grid->SetProbability(li, grid_snapshot->GetProbability(gi));
        }
      }
    }
  }

  // ============ 2. 裁剪点云（临时 BoxFilter，线程安全）============
  auto local_cloud = std::make_unique<sensor::PointCloud>();
  {
    // 全图点云 BoxFilter（O(N_global)，大图随规模增长的另一主成本）。
    CARTO_PERF_SCOPE("BoxFilter.cloud_filter(off)");
    sensor::BoxFilter tmp_filter(box_);
    tmp_filter.SetOrigin(Eigen::Vector3f(static_cast<float>(ox),
                                         static_cast<float>(oy), 0.0f));
    tmp_filter.Filter(cloud_snapshot, *local_cloud);
  }

  // ============ 3. 组装并构建 ikd-tree ============
  auto new_submap = std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(static_cast<float>(ox), static_cast<float>(oy)),
      std::move(local_grid),
      nullptr,  // local_submap 只用于匹配，不需要 conversion_tables
      std::move(local_cloud), std::make_unique<sensor::PointCloud>(),
      latest_line_id, active_line_data);
  {
    CARTO_PERF_SCOPE("BoxFilter.RebuildIkd+Finish(off)");
    new_submap->RebuildIkdTrees();
    new_submap->Finish();
  }

  // 调试：如果启用，将 submap 的 grid map 保存为图片和点云文件，落到当前地图目录下 <map_dir>/debug/。
  if (options_.save_debug_data() && !map_dir_.empty()) {
    const std::string dir = map_dir_ + "/debug/";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    new_submap->grid()->DrawAsImage(dir, "local_map.png");
    new_submap->SaveSubmapCloud(dir, "local_submap.pcd");
  }
  // off 模式：每次 box 边缘重建 local map 时打印运行时间统计（与 chunk 模式在 AssembleLocalSubmap
  // 内 Dump 对称）。这是 off 模式 perf_stat.txt 的唯一写入点——chunk 不调本 off 路径，故两边各自触发。
  common::PerfRegistry::Get().Dump("off-rebuild");
  return new_submap;
}

void LocalMapUpdater2D::UpdateDynamicMap(
    const std::vector<sensor::RangeData>& range_data_vector,
    const sensor::PointCloud& cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data)
{
  // chunk 模式（DEC-025）：把本帧扫描**同步**插入合一的单张 local_submap_（匹配 + 更新同一张，
  // 不 Finish）。与 ScanMatch 同线程、顺序在其之后执行（ScanMatch 读 → 本函数写），无并发。
  // 单张 grid 做整图 ray-cast（无 chunk 边界问题，同 off 的 ApplyDynamicUpdate）；累积更新跨格时由
  // UpdateRobotPose 的 FlushLiveSubmap 切回 chunk + 整合 global_static，再从 chunk 重拼。
  // submap_mutex_ 仅与 save 线程（FlushForSave）互斥（同线程的 match/cross 不竞争）。
  if (chunk_map_ && !chunk_map_->empty()) {
    CARTO_PERF_SCOPE("ChunkMap.local_submap_insert");
    std::shared_ptr<mapping::Submap2D> s;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      s = local_submap_;
    }
    if (!s) return;
    std::unique_lock<std::shared_mutex> wlock(submap_mutex_);
    if (s->insertion_finished()) return;
    for (const auto& r : range_data_vector) {
      s->InsertRangeData(r, range_data_inserter_.get());
    }
    s->InsertCloudData(cloud, highlight_cloud);
    s->InsertLineData(line_data);
    // [临时调试] 每 30 次更新：打印 active submap 累积状态 + 存 grid/点云，看动态更新在累积。
    static int upd_n = 0;
    if ((upd_n++ % 30) == 0) {
      const auto* lg = dynamic_cast<const mapping::ProbabilityGrid*>(s->grid());
      LOG(INFO) << "[chunk-trace UPD] #" << upd_n << " sync inserted "
                << range_data_vector.size() << " range_data; active cloud="
                << (s->cloud() ? s->cloud()->size() : 0)
                << " grid_cells=" << lg->limits().cell_limits().num_x_cells
                << "x" << lg->limits().cell_limits().num_y_cells;
      if (options_.save_debug_data() && !map_dir_.empty()) {
        const std::string dir = map_dir_ + "/debug/";
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (lg) lg->DrawAsImageOnCanvas(dir, "live_update.png", lg->limits());
        s->SaveSubmapCloud(dir, "live_update.pcd");
      }
    }
    return;
  }

  // 策略1: NONE - 不更新dynamic map（无需锁，options_ 只读）
  if (options_.dynamic_map_policy() == proto::DynamicMapPolicy::NONE) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!global_dynamic_submap_) {
      LOG(ERROR) << "Global dynamic submap is not initialized, cannot update dynamic map.";
      return;
    }
  }

  // 构造 batch（拷贝数据）并提交到低优先级线程池，微秒级，不阻塞当前线程
  auto task = std::make_unique<common::Task>();
  task->SetWorkItem(
      [this, batch = DynamicUpdateBatch{range_data_vector, cloud, highlight_cloud, line_data}]() {
        ApplyDynamicUpdate(batch);
      });
  insert_thread_pool_.Schedule(std::move(task));
}

void LocalMapUpdater2D::ApplyDynamicUpdate(const DynamicUpdateBatch& batch)
{
  // 先在 data_mutex_ 下捕获 shared_ptr（避免与 SetGlobalSubmap 的指针赋值竞争）
  std::shared_ptr<mapping::Submap2D> submap;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    submap = global_dynamic_submap_;
  }
  if (!submap) return;

  // 独占锁保护 submap 内部数据（grid/ikdtree/line_map）写入
  std::unique_lock<std::shared_mutex> write_lock(submap_mutex_);

  if (submap->insertion_finished()) {
    submap->set_insertion_finished(false);
    LOG(INFO) << "Reopened global_dynamic_submap_ for data insertion";
  }

  CHECK(range_data_inserter_ != nullptr);
  for (const auto& range_data : batch.range_data_vector) {
    submap->InsertRangeData(range_data, range_data_inserter_.get());
  }

  submap->InsertCloudData(batch.cloud, batch.highlight_cloud);
  submap->InsertLineData(batch.line_data);
}

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::GetGlobalSubmap() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 优先返回 global_dynamic_submap_；为空时（chunk 模式 T1 / NONE 策略）回退到 global_static_submap_。
  // 调用方（保存 submap_to_save、pose_estimator 线特征跟踪）据此拿到非空底图，避免空指针解引用。
  return global_dynamic_submap_ ? global_dynamic_submap_ : global_static_submap_;
}

std::shared_ptr<mapping::Submap2D> LocalMapUpdater2D::GetStaticSubmap() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // 漂移免疫底图：SetGlobalSubmap 后从不再插入数据，供漂移门控做独立锚点
  return global_static_submap_;
}

void LocalMapUpdater2D::ApplyOriginTransform(const transform::Rigid3d& T)
{
  // data_mutex_ 先于 submap_mutex_（与 ApplyDynamicUpdate 加锁顺序一致，无死锁风险）
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::unique_lock<std::shared_mutex> submap_lock(submap_mutex_);

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
    // chunk 模式（T1）：local map 由 chunk_map_ 拼装，source 可为空；非 chunk 模式需 global_dynamic_submap_。
    const bool chunk_mode = chunk_map_ && !chunk_map_->empty();
    if (!chunk_mode && !global_dynamic_submap_) {
      LOG(ERROR) << "RebuildLocalSubmapAt: global_dynamic_submap_ is not initialized.";
      return;
    }
    source_snapshot = global_dynamic_submap_;  // chunk 模式为 null，chunk 路径忽略
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
