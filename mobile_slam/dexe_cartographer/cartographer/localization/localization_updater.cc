/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/localization/localization_updater.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace localization {
namespace {

using mapping::proto::SerializedData;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<LocalizationUpdater::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const LocalizationUpdater::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == LocalizationUpdater::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

/**
 * @brief 纯定位模式下，向pose graph中添加修正器
 * 
 * @param trajectory_id 
 * @param trajectory_options 
 * @param pose_graph 
 */
void MaybeAddPureLocalizationTrimmer(
    const int trajectory_id,
    const mapping::proto::TrajectoryBuilderOptions& trajectory_options,
    mapping::PoseGraph* pose_graph) {
  // 有参数 pure_localization 时，增加默认参数的trimmer（高优先级）
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<mapping::PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  // 有参数 pure_localization_trimmer 时，增加指定参数的trimmer
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<mapping::PureLocalizationTrimmer>(
        trajectory_id,
        trajectory_options.pure_localization_trimmer().max_submaps_to_keep()));
  }
}

}  // namespace

/**
 * @brief Construct a new Map Builder:: Map Builder object
 *        构建2D/3D轨迹优化器，构建修正器
 * 
 * @param options 
 */
LocalizationUpdater::LocalizationUpdater(const proto::LocalizationUpdaterOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {

  // 根据配置，创建轨迹优化器（pose graph）
  pose_graph_ = absl::make_unique<mapping::PoseGraph2D>(
      options_.pose_graph_options(),
      absl::make_unique<mapping::optimization::OptimizationProblem2D>(
          options_.pose_graph_options().optimization_problem_options()),
      &thread_pool_);

  // 构建修正器
  // TODO：修正器的作用是什么？？
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}


std::shared_ptr<GridMap2D> LocalizationUpdater::GetGlobalGridMap() const
{
  CHECK(global_static_submap_ != nullptr)
      << "Global static submap not available. Please load state from pbstream first.";
  
  // 从Submap2D中的Grid2D创建GridMap2D wrapper
  // global_static_submap_->grid() 返回 const Grid2D*
  // 需要将其转换为 shared_ptr<GridMap2D>
  auto grid = global_static_submap_->grid();
  if (!grid) {
    LOG(WARNING) << "Grid is null in global static submap";
    return nullptr;
  }
  
  // 创建一个GridMap2D对象来包装underlying grid
  // 注：这可能需要根据实际的GridMap2D构造函数进行调整
  return std::make_shared<GridMap2D>(*dynamic_cast<const GridMap2D*>(grid));
}

std::shared_ptr<CloudMap2D> LocalizationUpdater::GetGlobalCloudMap() const
{
  CHECK(global_static_submap_ != nullptr)
      << "Global static submap not available. Please load state from pbstream first.";
  
  auto cloud = global_static_submap_->cloud();
  if (!cloud) {
    LOG(WARNING) << "Cloud is null in global static submap";
    return nullptr;
  }
  
  // 创建CloudMap2D来包装点云数据
  return std::make_shared<CloudMap2D>(*cloud);
}

std::shared_ptr<CloudMap2D> LocalizationUpdater::GetGlobalHighlighCloudMap() const
{
  CHECK(global_static_submap_ != nullptr)
      << "Global static submap not available. Please load state from pbstream first.";
  
  auto highlight_cloud = global_static_submap_->highlight_cloud();
  if (!highlight_cloud) {
    LOG(WARNING) << "Highlight cloud is null in global static submap";
    return std::make_shared<CloudMap2D>();  // 返回空的CloudMap2D
  }
  
  // 创建CloudMap2D来包装高亮点云数据
  return std::make_shared<CloudMap2D>(*highlight_cloud);
}

std::shared_ptr<sensor::LineMapData> LocalizationUpdater::GetGlobalLineMap() const
{
  CHECK(global_static_submap_ != nullptr)
      << "Global static submap not available. Please load state from pbstream first.";
  
  std::shared_ptr<sensor::LineMapData> line_map_data(new sensor::LineMapData());
  
  auto line_map_2d_list = global_static_submap_->line_map_2d_list();
  if (!line_map_2d_list) {
    LOG(WARNING) << "Line map is null in global static submap";
    return line_map_data;
  }
  
  const auto line_map_2d_list_data = line_map_2d_list->GetLineMapData2DList();
  for (const auto& map_line : line_map_2d_list_data) {
    sensor::LaneDataWithNormal lane_data;
    lane_data.data.line_data.begin = map_line.second.line_data().begin();
    lane_data.data.line_data.end = map_line.second.line_data().end();
    line_map_data->push_back(lane_data);
  }
  return line_map_data;
}

bool LocalizationUpdater::SetInitialPose(const transform::Rigid3d &initial_pose)
{
  const int last_trajectory_id = num_trajectory_builders() - 1;
  return trajectory_builders_[last_trajectory_id]->SetInitialPose(initial_pose);
}

void LocalizationUpdater::TriggerGlobalRelocalization()
{
  if (!pose_estimator_2d_) { return; }
  pose_estimator_2d_->TriggerGlobalRelocalization();
}

std::optional<LocalizationState> LocalizationUpdater::GetLocalizationState()
{
  const int last_trajectory_id = num_trajectory_builders() - 1;
  return trajectory_builders_[last_trajectory_id]->GetLocalizationState();
}

double LocalizationUpdater::GetLocalizationConfidence()
{
  return pose_estimator_2d_ ? pose_estimator_2d_->GetLocalizationConfidence() : 0.0;
}

MatchLevel LocalizationUpdater::GetMatchLevel()
{
  return pose_estimator_2d_ ? pose_estimator_2d_->GetMatchLevel() : MatchLevel::UNKNOWN;
}


/**
 * @brief 根据传感器类型id和配置，创建LocalTrajectoryBuilder和CreateGlobalTrajectoryBuilder
 * 
 * @param expected_sensor_ids 传感器类型列表
 * @param trajectory_options  轨迹构建器配置
 * @param local_slam_result_callback local slam结果回调函数
 * @return int 轨迹id
 */
int LocalizationUpdater::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const mapping::proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  // 1. 构造一个motion filter，用来筛选pose_graph的关键帧
  absl::optional<mapping::MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        mapping::MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }
  std::unique_ptr<PoseEstimator2D> pose_estimator_2d;

  CHECK(trajectory_options.has_pose_estimator_2d_options());
  // 创建local trajectory builder 2d
  pose_estimator_2d = absl::make_unique<PoseEstimator2D>(
      trajectory_options.pose_estimator_2d_options(),
      SelectRangeSensorIds(expected_sensor_ids));

  CHECK(global_static_submap_ != nullptr)
      << "Global static submap not available. Please load state from pbstream first.";
  pose_estimator_2d->InitGlobalMapsFromSubmap(global_static_submap_);
  
  // 保存 PoseEstimator2D 指针
  pose_estimator_2d_ = pose_estimator_2d.get();

  // 创建全局轨迹优化器
  DCHECK(dynamic_cast<mapping::PoseGraph2D*>(pose_graph_.get()));
  trajectory_builders_.push_back(absl::make_unique<mapping::CollatedTrajectoryBuilder>(
      trajectory_options, sensor_collator_.get(), trajectory_id,
      expected_sensor_ids,
      CreateGlobalTrajectoryBuilder2D(
          std::move(pose_estimator_2d), trajectory_id,
          static_cast<mapping::PoseGraph2D*>(pose_graph_.get()),
          local_slam_result_callback, pose_graph_odometry_motion_filter)));

  // 3. 向pose_graph中添加纯定位的trimmer
  MaybeAddPureLocalizationTrimmer(trajectory_id, trajectory_options,
                                  pose_graph_.get());

  // 4. 如果有初始位姿，设置初始位姿
  if (trajectory_options.has_initial_trajectory_pose()) {
    const auto& initial_trajectory_pose =
        trajectory_options.initial_trajectory_pose();
    pose_graph_->SetInitialTrajectoryPose(
        trajectory_id, initial_trajectory_pose.to_trajectory_id(),
        transform::ToRigid3(initial_trajectory_pose.relative_pose()),
        common::FromUniversal(initial_trajectory_pose.timestamp()));
  }
  // 5. 接下来，将轨迹跟踪器的配置信息和传感器配置信息保存到容器all_trajectory_builder_options_中。
  mapping::proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
  for (const auto& sensor_id : expected_sensor_ids) {
    *options_with_sensor_ids_proto.add_sensor_id() = ToProto(sensor_id);
  }
  *options_with_sensor_ids_proto.mutable_trajectory_builder_options() =
      trajectory_options;
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  // 6. 最后检查一下轨迹跟踪器对象和对应的配置是否都构建成功，之后返回新建的跟踪器对象的索引。
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

int LocalizationUpdater::AddTrajectoryForDeserialization(
    const mapping::proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

void LocalizationUpdater::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

std::string LocalizationUpdater::SubmapToProto(
    const mapping::SubmapId& submap_id, mapping::proto::SubmapQuery::Response* const response) {
  if (submap_id.trajectory_id < 0 ||
      submap_id.trajectory_id >= num_trajectory_builders()) {
    return "Requested submap from trajectory " +
           std::to_string(submap_id.trajectory_id) + " but there are only " +
           std::to_string(num_trajectory_builders()) + " trajectories.";
  }

  const auto submap_data = pose_graph_->GetSubmapData(submap_id);
  if (submap_data.submap == nullptr) {
    return "Requested submap " + std::to_string(submap_id.submap_index) +
           " from trajectory " + std::to_string(submap_id.trajectory_id) +
           " but it does not exist: maybe it has been trimmed.";
  }
  submap_data.submap->ToResponseProto(submap_data.pose, response);
  return "";
}

void LocalizationUpdater::ApplyOriginTransform(const transform::Rigid3d& T) {
  // 1. 变换 pose graph 中所有节点、submap、路标的全局位姿
  pose_graph_->ApplyTransformToAllPoses(T);

  if (global_static_submap_) {
    const double resolution =
        global_static_submap_->grid()->limits().resolution();

    // 2. 从已变换的 pose graph 节点重建完整的 global_static_submap_
    //    （grid + cloud + line 全部从节点数据重新生成，确保三者在同一坐标系下对齐）
    auto fresh = GenerateGlobalStaticSubmap(resolution);

    if (fresh) {
      LOG(INFO) << "ApplyOriginTransform: rebuilt global_static_submap from "
                << pose_graph_->GetTrajectoryNodes().size()
                << " transformed nodes (grid + cloud + line in new frame)";
      global_static_submap_ = fresh;
    } else {
      LOG(WARNING) << "ApplyOriginTransform: GenerateGlobalStaticSubmap returned "
                      "null (no nodes?), falling back to set_local_pose only";
      global_static_submap_->set_local_pose(
          T * global_static_submap_->local_pose());
    }
  }

  // 3. 同步 LocalMapUpdater2D
  if (pose_estimator_2d_) {
    auto* local_map_updater = pose_estimator_2d_->GetLocalMapUpdater();
    if (local_map_updater) {
      if (global_static_submap_) {
        local_map_updater->SetGlobalSubmap(global_static_submap_);
        local_map_updater->UpdateRobotPose(transform::Rigid2d::Identity());
      } else {
        local_map_updater->ApplyOriginTransform(T);
      }
    }
  }
}

void LocalizationUpdater::EnsureGlobalSubmapPyramid(
    const std::shared_ptr<mapping::Submap2D>& submap) {
  if (!submap || !submap->grid()) return;
  for (const auto& opt : all_trajectory_builder_options_) {
    const auto& tbo = opt.trajectory_builder_options();
    if (!tbo.has_pose_estimator_2d_options()) continue;
    const auto& pe = tbo.pose_estimator_2d_options();
    if (!pe.has_pose_initializer_options()) continue;
    const auto& fcsm = pe.pose_initializer_options()
                           .fast_correlative_scan_matcher_options();
    if (fcsm.branch_and_bound_depth() < 1) continue;
    submap->GetOrBuildPrecomputationGridStack(fcsm);  // no-op if cache hit
    LOG(INFO) << "save: localization map cache ready (depth="
              << fcsm.branch_and_bound_depth() << ")";
    return;
  }
  LOG(WARNING)
      << "save: localization params missing, map cache will be absent";
}

void LocalizationUpdater::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  // 保存时传递 global_static_submap_，确保增量更新后的全局地图被保存
  EnsureGlobalSubmapPyramid(global_static_submap_);  // 补全局定位地图缓存
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps, global_static_submap_);
  
  if (global_static_submap_) {
    LOG(INFO) << "=== Global Static Submap Save Statistics ===";
    
    // 获取轨迹节点数量
    const auto nodes = pose_graph_->GetTrajectoryNodes();
    LOG(INFO) << "Total trajectory nodes: " << nodes.size();
    
    // 栅格地图信息
    const auto* grid = dynamic_cast<const mapping::ProbabilityGrid*>(global_static_submap_->grid());
    if (grid) {
      const auto& limits = grid->limits();
      LOG(INFO) << "Grid Map:";
      LOG(INFO) << "  - Resolution: " << limits.resolution() << " meters/cell";
      LOG(INFO) << "  - Size: " << limits.cell_limits().num_x_cells << " x " 
                << limits.cell_limits().num_y_cells << " cells";
      const Eigen::Vector2f origin = global_static_submap_->local_pose().translation().head<2>().cast<float>();
      LOG(INFO) << "  - Origin: [" << origin.x() << ", " << origin.y() << "]";
    }
    
    // 点云信息
    const auto* cloud = global_static_submap_->cloud();
    if (cloud) {
      LOG(INFO) << "Point Cloud: " << cloud->size() << " points";
    } else {
      LOG(INFO) << "Point Cloud: empty";
    }
    
    const auto* highlight_cloud = global_static_submap_->highlight_cloud();
    if (highlight_cloud) {
      LOG(INFO) << "Highlight Point Cloud: " << highlight_cloud->size() << " points";
    } else {
      LOG(INFO) << "Highlight Point Cloud: empty";
    }
    
    // 线特征信息
    const auto* line_map_2d_list = global_static_submap_->line_map_2d_list();
    if (line_map_2d_list) {
      const auto& line_data_map = line_map_2d_list->GetLineMapData2DList();
      LOG(INFO) << "Line Features: " << line_data_map.size() << " line segments";
    } else {
      LOG(INFO) << "Line Features: empty";
    }
    
    // Range data信息
    LOG(INFO) << "Range Data Entries: " << global_static_submap_->num_range_data();
    
    LOG(INFO) << "============================================";
  }
}

bool LocalizationUpdater::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  
  // 根据动态地图策略获取要保存的 submap
  // 策略判断逻辑在 LocalMapUpdater2D::GetGlobalSubmap() 中实现
  std::shared_ptr<mapping::Submap2D> submap_to_save = global_static_submap_;
  
  if (pose_estimator_2d_) {
    auto local_map_updater = pose_estimator_2d_->GetLocalMapUpdater();
    if (local_map_updater) {
      auto policy = local_map_updater->GetDynamicMapPolicy();
      
      if (policy == proto::DynamicMapPolicy::PERSISTENT) {
        // 策略4: PERSISTENT - 保存 dynamic map
        submap_to_save = local_map_updater->GetGlobalSubmap();
        if (submap_to_save) {
          LOG(INFO) << "Using PERSISTENT policy: saving global_dynamic_submap";
        } else {
          LOG(WARNING) << "PERSISTENT policy enabled but GetGlobalSubmap() returned null, using static submap";
          submap_to_save = global_static_submap_;
        }
      } else {
        // 策略1/2/3: 保存 static map
        LOG(INFO) << "Using policy " << static_cast<int>(policy) 
                  << ": saving global_static_submap";
        submap_to_save = global_static_submap_;
      }
    }
  }
  
  // 对实际要保存的 submap（可能是 dynamic submap）补全局定位地图缓存
  EnsureGlobalSubmapPyramid(submap_to_save);
  // 保存时传递选定的 submap
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps, submap_to_save);
  
  bool success = writer.Close();
  
  if (success && global_static_submap_) {
    LOG(INFO) << "Successfully saved state to: " << filename;
    LOG(INFO) << "=== Global Static Submap Save Statistics ===";
    
    // 获取轨迹节点数量
    const auto nodes = pose_graph_->GetTrajectoryNodes();
    LOG(INFO) << "Total trajectory nodes: " << nodes.size();
    
    // 栅格地图信息
    const auto* grid = dynamic_cast<const mapping::ProbabilityGrid*>(global_static_submap_->grid());
    if (grid) {
      const auto& limits = grid->limits();
      LOG(INFO) << "Grid Map:";
      LOG(INFO) << "  - Resolution: " << limits.resolution() << " meters/cell";
      LOG(INFO) << "  - Size: " << limits.cell_limits().num_x_cells << " x " 
                << limits.cell_limits().num_y_cells << " cells";
      const Eigen::Vector2f origin = global_static_submap_->local_pose().translation().head<2>().cast<float>();
      LOG(INFO) << "  - Origin: [" << origin.x() << ", " << origin.y() << "]";
    }
    
    // 点云信息
    const auto* cloud = global_static_submap_->cloud();
    if (cloud) {
      LOG(INFO) << "Point Cloud: " << cloud->size() << " points";
    } else {
      LOG(INFO) << "Point Cloud: empty";
    }
    
    const auto* highlight_cloud = global_static_submap_->highlight_cloud();
    if (highlight_cloud) {
      LOG(INFO) << "Highlight Point Cloud: " << highlight_cloud->size() << " points";
    } else {
      LOG(INFO) << "Highlight Point Cloud: empty";
    }
    
    // 线特征信息
    const auto* line_map_2d_list = global_static_submap_->line_map_2d_list();
    if (line_map_2d_list) {
      const auto& line_data_map = line_map_2d_list->GetLineMapData2DList();
      LOG(INFO) << "Line Features: " << line_data_map.size() << " line segments";
    } else {
      LOG(INFO) << "Line Features: empty";
    }
    
    // Range data信息
    LOG(INFO) << "Range Data Entries: " << global_static_submap_->num_range_data();
    
    LOG(INFO) << "============================================";
  }
  
  return success;
}

/**
 * @brief 从 pose_graph_ 的所有轨迹节点重新生成 global_static_submap_
 *        用于旧格式 pbstream（不含 kGlobalStaticSubmap 字段）的 fallback
 * 
 * @param resolution 栅格地图分辨率（米/格）
 * @return std::shared_ptr<mapping::Submap2D> 生成的全局静态 submap
 */
std::shared_ptr<mapping::Submap2D> LocalizationUpdater::GenerateGlobalStaticSubmap(double resolution) {
  namespace cmapping = ::cartographer::mapping;

  LOG(INFO) << "Generating global static submap from trajectory nodes (fallback for old pbstream)...";
  
  // 1. 获取所有 trajectory nodes
  cmapping::MapById<cmapping::NodeId, cmapping::TrajectoryNode> nodes =
      pose_graph_->GetTrajectoryNodes();
  
  if (nodes.empty()) {
    LOG(WARNING) << "No trajectory nodes found for global submap generation";
    return nullptr;
  }
  
  LOG(INFO) << "Found " << nodes.size() << " trajectory nodes for global submap generation";
  
  // 2. 计算地图原点
  const Eigen::Vector2f origin =
      nodes.begin()->data.global_pose.translation().head<2>().cast<float>();

  // 3. 创建大的 submap 来积累所有数据
  constexpr int kLargeSubmapSize = 2000;
  auto global_submap = std::make_shared<cmapping::Submap2D>(
      origin,
      std::unique_ptr<cmapping::Grid2D>(
          static_cast<cmapping::Grid2D*>(
              absl::make_unique<cmapping::ProbabilityGrid>(
                  cmapping::MapLimits(resolution,
                           origin.cast<double>(),
                           cmapping::CellLimits(kLargeSubmapSize, kLargeSubmapSize)),
                  &conversion_tables_).release())),
      &conversion_tables_,
      std::make_unique<sensor::PointCloud>(),
      std::make_unique<sensor::PointCloud>(),
      0, std::map<int, cmapping::MapLine2D>{});
  
  // 4. 获取配置参数
  const cmapping::proto::ProbabilityGridRangeDataInserterOptions2D* prob_grid_options_ptr = nullptr;
  const cmapping::proto::SubmapsOptions2D* submaps_options_ptr = nullptr;
  const cmapping::scan_matching::proto::LineMatcherOptions* line_matcher_options_ptr = nullptr;
  
  for (const auto& options_with_ids : all_trajectory_builder_options_) {
    const auto& traj_opts = options_with_ids.trajectory_builder_options();
    if (traj_opts.has_trajectory_builder_2d_options()) {
      const auto& traj_2d_opts = traj_opts.trajectory_builder_2d_options();
      submaps_options_ptr = &traj_2d_opts.submaps_options();
      line_matcher_options_ptr = &traj_2d_opts.line_matcher_options();
      const auto& rdi_opts = submaps_options_ptr->range_data_inserter_options();
      if (rdi_opts.range_data_inserter_type() ==
          cmapping::proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
        prob_grid_options_ptr =
            &rdi_opts.probability_grid_range_data_inserter_options_2d();
      }
      break;
    }
  }
  
  // 设置 cloud/line culling 参数
  if (submaps_options_ptr) {
    global_submap->SetCloudCullingProb(submaps_options_ptr->cloud_culling_prob());
    global_submap->SetLineCullingParam(
        submaps_options_ptr->min_line_length(),
        submaps_options_ptr->max_line_length(),
        submaps_options_ptr->line_culling_obs());
  }
  
  // 5. 创建 range data 插入器
  cmapping::proto::ProbabilityGridRangeDataInserterOptions2D inserter_options;
  if (prob_grid_options_ptr) {
    inserter_options.set_hit_probability(prob_grid_options_ptr->hit_probability());
    inserter_options.set_miss_probability(prob_grid_options_ptr->miss_probability());
    inserter_options.set_insert_free_space(prob_grid_options_ptr->insert_free_space());
  } else {
    LOG(WARNING) << "Using default range data inserter parameters";
    inserter_options.set_hit_probability(0.65);
    inserter_options.set_miss_probability(0.48);
    inserter_options.set_insert_free_space(true);
  }
  auto range_data_inserter =
      absl::make_unique<cmapping::ProbabilityGridRangeDataInserter2D>(inserter_options);
  
  // 6. 创建 line matcher
  std::unique_ptr<cmapping::scan_matching::LineMatcher2D> line_matcher;
  if (line_matcher_options_ptr) {
    line_matcher = std::make_unique<cmapping::scan_matching::LineMatcher2D>(
        *line_matcher_options_ptr);
  }
  
  // 7. 遍历所有节点，积累数据
  int nodes_with_real_range_data = 0;
  int nodes_with_synthetic_range_data = 0;
  
  for (const auto& node : nodes) {
    // 7a. 插入 range data（栅格地图）
    if (!node.data.constant_data->range_datas.empty()) {
      nodes_with_real_range_data++;
      for (const auto& range_data : node.data.constant_data->range_datas) {
        sensor::RangeData global_range_data;
        global_range_data.origin = (node.data.global_pose *
                                    range_data.origin.cast<double>()).cast<float>();
        global_range_data.returns = sensor::TransformPointCloud(
            range_data.returns, node.data.global_pose.cast<float>());
        global_range_data.misses = sensor::TransformPointCloud(
            range_data.misses, node.data.global_pose.cast<float>());
        global_submap->InsertRangeData(global_range_data, range_data_inserter.get());
      }
    } else {
      const auto& point_cloud =
          node.data.constant_data->filtered_gravity_aligned_point_cloud;
      if (!point_cloud.empty()) {
        nodes_with_synthetic_range_data++;
        sensor::RangeData global_range_data;
        global_range_data.origin = node.data.global_pose.cast<float>().translation();
        global_range_data.returns = sensor::TransformPointCloud(
            point_cloud, node.data.global_pose.cast<float>());
        global_submap->InsertRangeData(global_range_data, range_data_inserter.get());
      }
    }
    
    // 7b. 插入点云数据
    auto transformed_cloud = sensor::TransformPointCloud(
        node.data.constant_data->filtered_gravity_aligned_point_cloud,
        node.data.global_pose.cast<float>());
    auto transformed_highlight_cloud = sensor::TransformPointCloud(
        node.data.constant_data->highlight_point_cloud,
        node.data.global_pose.cast<float>());
    global_submap->InsertCloudData(transformed_cloud, transformed_highlight_cloud);
    
    // 7c. 插入线特征数据
    const auto& lines = node.data.constant_data->lines;
    if (!lines.empty() && line_matcher) {
      transform::Rigid2d global_pose_2d = transform::Project2D(node.data.global_pose);
      std::map<int, std::vector<sensor::LineData>> tracked_line_data;
      line_matcher->TrackLineFromMap(global_pose_2d, lines,
                                     *global_submap->line_map_2d_list(),
                                     tracked_line_data);
      std::map<int, std::vector<sensor::LineData>> global_tracked_lines;
      for (const auto& tracked_element : tracked_line_data) {
        for (const auto& line : tracked_element.second) {
          sensor::LineData global_line = sensor::TransformLineData(global_pose_2d, line);
          global_tracked_lines[tracked_element.first].push_back(global_line);
        }
      }
      if (!global_tracked_lines.empty()) {
        global_submap->InsertLineData(global_tracked_lines);
      }
    }
  }
  
  // 8. 完成 submap 构建（FreshCloudData + FinishIkdTrees 在 Finish() 内部完成）
  // Finish() 已将 ikd-tree 同步到 cloud_ 并转为 PCL KdTreeFLANN 只读后端，
  // PCL 后端支持 Nearest_Search，足够 scan matching 使用，无需再 RebuildIkdTrees。
  global_submap->Finish();

  LOG(INFO) << "=== Fallback Global Static Submap Generation Statistics ===";
  LOG(INFO) << "Trajectory nodes processed: " << nodes.size();
  LOG(INFO) << "  - Nodes with real range data: " << nodes_with_real_range_data;
  LOG(INFO) << "  - Nodes with synthetic range data: " << nodes_with_synthetic_range_data;
  
  const auto* cloud = global_submap->cloud();
  LOG(INFO) << "Point Cloud: " << (cloud ? cloud->size() : 0) << " points";
  
  const auto* line_map_2d_list = global_submap->line_map_2d_list();
  LOG(INFO) << "Line Features: " 
            << (line_map_2d_list ? line_map_2d_list->GetLineMapData2DList().size() : 0)
            << " line segments";
  LOG(INFO) << "===========================================================";
  
  // 10. 保存到成员变量
  global_static_submap_ = global_submap;
  
  return global_submap;
}

/**
 * @brief 从pbstream中读取数据
 * 
 * @param reader 
 * @param load_frozen_state 
 * @return std::map<int, int> 
 */
std::map<int, int> LocalizationUpdater::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  mapping::proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
  const auto& all_builder_options_proto =
      deserializer.all_trajectory_builder_options();

  std::map<int, int> trajectory_remapping;
  for (int i = 0; i < pose_graph_proto.trajectory_size(); ++i) {
    auto& trajectory_proto = *pose_graph_proto.mutable_trajectory(i);
    const auto& options_with_sensor_ids_proto =
        all_builder_options_proto.options_with_sensor_ids(i);
    const int new_trajectory_id =
        AddTrajectoryForDeserialization(options_with_sensor_ids_proto);
    CHECK(trajectory_remapping
              .emplace(trajectory_proto.trajectory_id(), new_trajectory_id)
              .second)
        << "Duplicate trajectory ID: " << trajectory_proto.trajectory_id();
    trajectory_proto.set_trajectory_id(new_trajectory_id);
    if (load_frozen_state) {
      pose_graph_->FreezeTrajectory(new_trajectory_id);
    }
  }

  // Apply the calculated remapping to constraints in the pose graph proto.
  for (auto& constraint_proto : *pose_graph_proto.mutable_constraint()) {
    constraint_proto.mutable_submap_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.submap_id().trajectory_id()));
    constraint_proto.mutable_node_id()->set_trajectory_id(
        trajectory_remapping.at(constraint_proto.node_id().trajectory_id()));
  }

  mapping::MapById<mapping::SubmapId, transform::Rigid3d> submap_poses;
  for (const mapping::proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const mapping::proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(mapping::SubmapId{trajectory_proto.trajectory_id(),
                                            submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  mapping::MapById<mapping::NodeId, transform::Rigid3d> node_poses;
  for (const mapping::proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const mapping::proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          mapping::NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
          transform::ToRigid3(node_proto.pose()));
    }
  }

  // 向pose_graph中，加入文件中的约束信息
  // Set global poses of landmarks.
  for (const auto& landmark : pose_graph_proto.landmark_poses()) {
    pose_graph_->SetLandmarkPose(landmark.landmark_id(),
                                 transform::ToRigid3(landmark.global_pose()),
                                 true);
  }

  SerializedData proto;
  while (deserializer.ReadNextSerializedData(&proto)) {
    switch (proto.data_case()) {
      case SerializedData::kPoseGraph:
        LOG(ERROR) << "Found multiple serialized `PoseGraph`. Serialized "
                      "stream likely corrupt!.";
        break;
      case SerializedData::kAllTrajectoryBuilderOptions:
        LOG(ERROR) << "Found multiple serialized "
                      "`AllTrajectoryBuilderOptions`. Serialized stream likely "
                      "corrupt!.";
        break;
      case SerializedData::kSubmap: {
        proto.mutable_submap()->mutable_submap_id()->set_trajectory_id(
            trajectory_remapping.at(
                proto.submap().submap_id().trajectory_id()));
        const mapping::SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        *proto.mutable_submap());
        // 清空 proto 以释放内存
        proto.clear_submap();
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const mapping::NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_->AddNodeFromProto(node_pose, *proto.mutable_node());
        // 清空 proto 以释放内存
        proto.clear_node();
        break;
      }
      case SerializedData::kTrajectoryData: {
        proto.mutable_trajectory_data()->set_trajectory_id(
            trajectory_remapping.at(proto.trajectory_data().trajectory_id()));
        pose_graph_->SetTrajectoryDataFromProto(proto.trajectory_data());
        break;
      }
      case SerializedData::kImuData: {
        if (load_frozen_state) break;
        pose_graph_->AddImuData(
            trajectory_remapping.at(proto.imu_data().trajectory_id()),
            sensor::FromProto(proto.imu_data().imu_data()));
        break;
      }
      case SerializedData::kOdometryData: {
        if (load_frozen_state) break;
        pose_graph_->AddOdometryData(
            trajectory_remapping.at(proto.odometry_data().trajectory_id()),
            sensor::FromProto(proto.odometry_data().odometry_data()));
        break;
      }
      case SerializedData::kFixedFramePoseData: {
        if (load_frozen_state) break;
        pose_graph_->AddFixedFramePoseData(
            trajectory_remapping.at(
                proto.fixed_frame_pose_data().trajectory_id()),
            sensor::FromProto(
                proto.fixed_frame_pose_data().fixed_frame_pose_data()));
        break;
      }
      case SerializedData::kLandmarkData: {
        if (load_frozen_state) break;
        pose_graph_->AddLandmarkData(
            trajectory_remapping.at(proto.landmark_data().trajectory_id()),
            sensor::FromProto(proto.landmark_data().landmark_data()));
        break;
      }
      case SerializedData::kGlobalStaticSubmap: {
        LOG(INFO) << "Loading global static submap from pbstream...";
        
        // 使用成员变量conversion_tables_而不是局部变量
        // 这很关键！ProbabilityGrid保存了指向conversion_tables内部查找表的指针
        // 如果使用局部变量，函数结束后指针就悬空了，导致概率值计算错误
        global_static_submap_ = std::make_shared<mapping::Submap2D>(
            proto.global_static_submap(), &conversion_tables_);
        
        // 从配置中获取参数并设置到global_static_submap_
        // 这些参数将用于后续的增量更新
        if (!all_trajectory_builder_options_.empty()) {
          const auto& first_trajectory_options = all_trajectory_builder_options_[0];
          if (first_trajectory_options.trajectory_builder_options().has_trajectory_builder_2d_options()) {
            const auto& trajectory_builder_2d_options = 
                first_trajectory_options.trajectory_builder_options().trajectory_builder_2d_options();
            const auto& submaps_options = trajectory_builder_2d_options.submaps_options();
            
            // 设置cloud culling参数（用于点云剔除）
            // 对于global_static_submap_，禁用culling（设置为0.1）
            // 因为grid更新是增量的，新插入点的区域概率还没提升，会被误删
            // active_submaps使用配置值，因为它的grid在持续更新
            global_static_submap_->SetCloudCullingProb(submaps_options.cloud_culling_prob());
            
            // 设置line culling参数（用于线特征剔除）
            global_static_submap_->SetLineCullingParam(
                submaps_options.min_line_length(),
                submaps_options.max_line_length(),
                submaps_options.line_culling_obs());
            LOG(INFO) << "Set line culling params: min_length=" << submaps_options.min_line_length()
                      << ", max_length=" << submaps_options.max_line_length()
                      << ", culling_obs=" << submaps_options.line_culling_obs();
            
            // 日志：range data inserter参数（这些参数将在PoseEstimator2D中使用）
            const auto& range_data_inserter_options = submaps_options.range_data_inserter_options();
            if (range_data_inserter_options.range_data_inserter_type() == 
                mapping::proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
              const auto& prob_grid_options = 
                  range_data_inserter_options.probability_grid_range_data_inserter_options_2d();
              LOG(INFO) << "Range data inserter params: hit_probability=" 
                        << prob_grid_options.hit_probability()
                        << ", miss_probability=" << prob_grid_options.miss_probability()
                        << ", insert_free_space=" << prob_grid_options.insert_free_space();
            }
          }
        }
        
        // 重建ikd-tree（使用Submap2D的方法）
        global_static_submap_->RebuildIkdTrees();
        LOG(INFO) << "Global static submap loaded and configured successfully";
        
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  // Load all constraints via AddSerializedConstraints regardless of frozen state.
  // This populates both data_.submap_data[submap_id].node_ids (for INTRA_SUBMAP
  // constraints) and data_.constraints, ensuring constraints are preserved when
  // the state is re-serialized after a localization run.
  // Note: Previously, frozen trajectories only called AddNodeToSubmap (skipping
  // AddSerializedConstraints), which left data_.constraints empty and caused
  // constraints() to return 0 entries when re-saving the pbstream.
  pose_graph_->AddSerializedConstraints(
      mapping::FromProto(pose_graph_proto.constraint()));
  CHECK(reader->eof());
  
  // 验证global static submap是否加载成功
  if (!global_static_submap_) {
    LOG(WARNING) << "No global static submap found in pbstream. "
                 << "This pbstream may be from an older version that doesn't include global submap.";
  } else {
    LOG(INFO) << "=== Global Static Submap Load Statistics ===";
    
    // 统计轨迹数量
    LOG(INFO) << "Trajectories loaded: " << trajectory_remapping.size();
    
    // 统计节点数量
    mapping::MapById<mapping::NodeId, mapping::TrajectoryNode> nodes = 
        pose_graph_->GetTrajectoryNodes();
    LOG(INFO) << "Total trajectory nodes loaded: " << nodes.size();
    
    // 统计点云数量
    const auto* cloud = global_static_submap_->cloud();
    if (cloud) {
      LOG(INFO) << "Global point cloud size: " << cloud->size() << " points";
    } else {
      LOG(INFO) << "Global point cloud: empty or not available";
    }
    
    const auto* highlight_cloud = global_static_submap_->highlight_cloud();
    if (highlight_cloud) {
      LOG(INFO) << "Global highlight point cloud size: " << highlight_cloud->size() << " points";
    } else {
      LOG(INFO) << "Global highlight point cloud: empty or not available";
    }
    
    // 统计线特征数量
    const auto* line_map_2d_list = global_static_submap_->line_map_2d_list();
    if (line_map_2d_list) {
      const auto& line_data_map = line_map_2d_list->GetLineMapData2DList();
      LOG(INFO) << "Global line features: " << line_data_map.size() << " line segments";
    } else {
      LOG(INFO) << "Global line features: empty or not available";
    }
    
    // 统计range data数量
    LOG(INFO) << "Global submap range data entries: " << global_static_submap_->num_range_data();
    
    // 统计栅格地图信息
    const auto* grid = dynamic_cast<const mapping::ProbabilityGrid*>(global_static_submap_->grid());
    if (grid) {
      const auto& limits = grid->limits();
      LOG(INFO) << "Grid map resolution: " << limits.resolution() << " meters/cell";
      LOG(INFO) << "Grid map size: " << limits.cell_limits().num_x_cells << " x " 
                << limits.cell_limits().num_y_cells << " cells";
    } else {
      LOG(INFO) << "Grid map: not available";
    }
    
    LOG(INFO) << "============================================";
  }
  
  // std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  // std::string folder = home_dir + "/.dexe/data/submaps/";
  // global_static_submap_->grid()->DrawAsImage(folder, "global_static_submap.png");

  return trajectory_remapping;
}

/**
 * @brief 从文件中加载状态，从.pbstream文件中，加载成ProtoStream数据流
 * 
 * @param state_filename 文件路径
 * @param load_frozen_state 
 * @return std::map<int, int> 老轨迹的trajectory_id, 新轨迹的new_trajectory_id
 */
std::map<int, int> LocalizationUpdater::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  
  // 加载状态
  std::map<int, int> trajectory_remapping = LoadState(&stream, load_frozen_state);
  
  // Fallback：如果 pbstream 是旧格式（不含 kGlobalStaticSubmap 字段），
  // 则从 pose_graph_ 的轨迹节点重新生成 global_static_submap_
  if (!global_static_submap_) {
    LOG(WARNING) << "global_static_submap_ is null after LoadState. "
                 << "This pbstream was likely generated by an older version of the mapping node. "
                 << "Regenerating global static submap from trajectory nodes...";
    
    // 从轨迹 options 获取 resolution
    double resolution = 0.05;  // 默认分辨率
    for (const auto& options_with_ids : all_trajectory_builder_options_) {
      const auto& traj_opts = options_with_ids.trajectory_builder_options();
      if (traj_opts.has_trajectory_builder_2d_options()) {
        const auto& traj_2d_opts = traj_opts.trajectory_builder_2d_options();
        if (traj_2d_opts.submaps_options().has_grid_options_2d()) {
          resolution = traj_2d_opts.submaps_options().grid_options_2d().resolution();
        }
        break;
      }
    }
    
    auto generated_submap = GenerateGlobalStaticSubmap(resolution);
    if (!generated_submap) {
      LOG(ERROR) << "Failed to generate global static submap from trajectory nodes! "
                 << "Localization will fail. Please rebuild the map.";
    } else {
      LOG(INFO) << "Successfully generated global static submap as fallback.";
    }
  }
  
  return trajectory_remapping;
}

/**
 * @brief Create a Map Builder object
 * 
 * @param options 
 * @return std::unique_ptr<LocalizationUpdaterInterface> 
 */
std::unique_ptr<LocalizationUpdater> CreateLocalizationUpdater(
    const proto::LocalizationUpdaterOptions& options) {
  return absl::make_unique<LocalizationUpdater>(options);
}

proto::LocalizationUpdaterOptions CreateLocalizationUpdaterOptions(
    common::LuaParameterDictionary* const parameter_dictionary)
{
  proto::LocalizationUpdaterOptions options;
  options.set_num_background_threads(
      parameter_dictionary->GetNonNegativeInt("num_background_threads"));
  options.set_collate_by_trajectory(
      parameter_dictionary->GetBool("collate_by_trajectory"));
  *options.mutable_pose_graph_options() = mapping::CreatePoseGraphOptions(
      parameter_dictionary->GetDictionary("pose_graph").get());
  return options;
}

}  // namespace localization
}  // namespace cartographer
