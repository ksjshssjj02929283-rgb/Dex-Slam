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

#include "cartographer/mapping/map_builder.h"

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/io/internal/mapping_state_serialization.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/io/serialization_format_migration.h"
#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"
#include "cartographer/mapping/internal/2d/pose_graph_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/3d/local_trajectory_builder_3d.h"
#include "cartographer/mapping/internal/3d/pose_graph_3d.h"
#include "cartographer/mapping/internal/collated_trajectory_builder.h"
#include "cartographer/mapping/internal/global_trajectory_builder.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/sensor/internal/collator.h"
#include "cartographer/sensor/internal/trajectory_collator.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {
namespace {

using mapping::proto::SerializedData;

std::vector<std::string> SelectRangeSensorIds(
    const std::set<MapBuilder::SensorId>& expected_sensor_ids) {
  std::vector<std::string> range_sensor_ids;
  for (const MapBuilder::SensorId& sensor_id : expected_sensor_ids) {
    if (sensor_id.type == MapBuilder::SensorId::SensorType::RANGE) {
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
    const proto::TrajectoryBuilderOptions& trajectory_options,
    PoseGraph* pose_graph) {
  // 有参数 pure_localization 时，增加默认参数的trimmer（高优先级）
  if (trajectory_options.pure_localization()) {
    LOG(WARNING)
        << "'TrajectoryBuilderOptions::pure_localization' field is deprecated. "
           "Use 'TrajectoryBuilderOptions::pure_localization_trimmer' instead.";
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
        trajectory_id, 3 /* max_submaps_to_keep */));
    return;
  }
  // 有参数 pure_localization_trimmer 时，增加指定参数的trimmer
  if (trajectory_options.has_pure_localization_trimmer()) {
    pose_graph->AddTrimmer(absl::make_unique<PureLocalizationTrimmer>(
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
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
  // 1. 判断是否有配置参数
  CHECK(options.use_trajectory_builder_2d() ^
        options.use_trajectory_builder_3d());
  // 2. 根据配置，创建轨迹优化器（pose graph）
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  if (options.use_trajectory_builder_3d()) {
    pose_graph_ = absl::make_unique<PoseGraph3D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem3D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }
  // 构建修正器
  // TODO：修正器的作用是什么？？
  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}
/**
 * @brief 根据传感器类型id和配置，创建LocalTrajectoryBuilder和CreateGlobalTrajectoryBuilder
 * 
 * @param expected_sensor_ids 传感器类型列表
 * @param trajectory_options  轨迹构建器配置
 * @param local_slam_result_callback local slam结果回调函数
 * @return int 轨迹id
 */
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {
  const int trajectory_id = trajectory_builders_.size();

  // 1. 构造一个motion filter，用来筛选pose_graph的关键帧
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter;
  if (trajectory_options.has_pose_graph_odometry_motion_filter()) {
    LOG(INFO) << "Using a motion filter for adding odometry to the pose graph.";
    pose_graph_odometry_motion_filter.emplace(
        MotionFilter(trajectory_options.pose_graph_odometry_motion_filter()));
  }

  // 2. 创建全局和local的位姿求解器
  if (options_.use_trajectory_builder_3d()) {
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_3d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder3D>(
          trajectory_options.trajectory_builder_3d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    DCHECK(dynamic_cast<PoseGraph3D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder3D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph3D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  } else {
    // 创建local trajectory builder 2d
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder;
    if (trajectory_options.has_trajectory_builder_2d_options()) {
      local_trajectory_builder = absl::make_unique<LocalTrajectoryBuilder2D>(
          trajectory_options.trajectory_builder_2d_options(),
          SelectRangeSensorIds(expected_sensor_ids));
    }
    // 创建全局轨迹优化器
    DCHECK(dynamic_cast<PoseGraph2D*>(pose_graph_.get()));
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, sensor_collator_.get(), trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));
  }
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
  proto::TrajectoryBuilderOptionsWithSensorIds options_with_sensor_ids_proto;
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

int MapBuilder::AddTrajectoryForDeserialization(
    const proto::TrajectoryBuilderOptionsWithSensorIds&
        options_with_sensor_ids_proto) {
  const int trajectory_id = trajectory_builders_.size();
  trajectory_builders_.emplace_back();
  all_trajectory_builder_options_.push_back(options_with_sensor_ids_proto);
  CHECK_EQ(trajectory_builders_.size(), all_trajectory_builder_options_.size());
  return trajectory_id;
}

void MapBuilder::FinishTrajectory(const int trajectory_id) {
  sensor_collator_->FinishTrajectory(trajectory_id);
  pose_graph_->FinishTrajectory(trajectory_id);
}

std::string MapBuilder::SubmapToProto(
    const SubmapId& submap_id, proto::SubmapQuery::Response* const response) {
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

void MapBuilder::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  // Generate global static submap before serialization
  if (options_.use_trajectory_builder_2d()) {
    LOG(INFO) << "Generating global static submap before serialization...";
    double resolution = 0.05; // Default resolution
    if (!all_trajectory_builder_options_.empty()) {
      const auto& first_options = all_trajectory_builder_options_[0];
      if (first_options.trajectory_builder_options().has_trajectory_builder_2d_options()) {
        resolution = first_options.trajectory_builder_options()
                      .trajectory_builder_2d_options()
                      .submaps_options()
                      .grid_options_2d()
                      .resolution();
      }
    }
    GenerateGlobalStaticSubmap(resolution);
  }
  
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps, global_static_submap_);
}

bool MapBuilder::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  // Generate global static submap before serialization
  if (options_.use_trajectory_builder_2d()) {
    LOG(INFO) << "Generating global static submap before saving to file...";
    double resolution = 0.05; // Default resolution
    if (!all_trajectory_builder_options_.empty()) {
      const auto& first_options = all_trajectory_builder_options_[0];
      if (first_options.trajectory_builder_options().has_trajectory_builder_2d_options()) {
        resolution = first_options.trajectory_builder_options()
                      .trajectory_builder_2d_options()
                      .submaps_options()
                      .grid_options_2d()
                      .resolution();
      }
    }
    GenerateGlobalStaticSubmap(resolution);
  }
  
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps, global_static_submap_);
  return (writer.Close());
}

std::shared_ptr<Submap2D> MapBuilder::GenerateGlobalStaticSubmap(double resolution) {
  LOG(INFO) << "Starting GenerateGlobalStaticSubmap with resolution: " << resolution;
  
  // 1. 获取所有trajectory nodes
  MapById<NodeId, TrajectoryNode> nodes = pose_graph_->GetTrajectoryNodes();
  
  if (nodes.empty()) {
    LOG(WARNING) << "No trajectory nodes found for global submap generation";
    return nullptr;
  }
  
  LOG(INFO) << "Found " << nodes.size() << " trajectory nodes for global submap generation";
  
  ValueConversionTables conversion_tables;

  // 2. 获取地图范围，使用第一个节点的位置作为原点
  const Eigen::Vector2f origin = nodes.begin()->data.global_pose.translation().head<2>().cast<float>();

  // 3. 创建大的submap来积累所有数据
  constexpr int kLargeSubmapSize = 2000;
  auto global_submap = std::make_shared<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(
              absl::make_unique<ProbabilityGrid>(
                  MapLimits(resolution,
                           origin.cast<double>(),
                           CellLimits(kLargeSubmapSize, kLargeSubmapSize)),
                  &conversion_tables).release())),
      &conversion_tables,
      std::make_unique<sensor::PointCloud>(),
      std::make_unique<sensor::PointCloud>(),
      0, std::map<int, MapLine2D>{});
  
  // 4. 获取配置参数 - 使用第一个轨迹的参数
  CHECK(!all_trajectory_builder_options_.empty()) 
      << "No trajectory options available for global submap generation.";
  
  const auto& first_trajectory_options = all_trajectory_builder_options_[0];
  const auto& trajectory_builder_2d_options = 
      first_trajectory_options.trajectory_builder_options().trajectory_builder_2d_options();
  const auto& submaps_options = trajectory_builder_2d_options.submaps_options();
  const auto& range_data_inserter_options = submaps_options.range_data_inserter_options();
  const auto& line_matcher_options = trajectory_builder_2d_options.line_matcher_options();
  
  // 设置cloud culling参数
  global_submap->SetCloudCullingProb(submaps_options.cloud_culling_prob());
  
  // 设置line culling参数
  global_submap->SetLineCullingParam(
      submaps_options.min_line_length(),
      submaps_options.max_line_length(),
      submaps_options.line_culling_obs());
  
  // 5. 创建line matcher用于线特征跟踪
  std::unique_ptr<scan_matching::LineMatcher2D> line_matcher =
      std::make_unique<scan_matching::LineMatcher2D>(line_matcher_options);

  // 6. 设置range data插入器
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  
  if (range_data_inserter_options.range_data_inserter_type() == 
      proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    const auto& prob_grid_options = 
        range_data_inserter_options.probability_grid_range_data_inserter_options_2d();
    options.set_hit_probability(prob_grid_options.hit_probability());
    options.set_miss_probability(prob_grid_options.miss_probability());
    options.set_insert_free_space(prob_grid_options.insert_free_space());
  } else {
    LOG(WARNING) << "Using default range data inserter parameters";
    options.set_hit_probability(0.65);
    options.set_miss_probability(0.48);
    options.set_insert_free_space(true);
  }
  
  LOG(INFO) << "Configuration loaded: cloud_culling_prob=" << submaps_options.cloud_culling_prob()
            << ", hit_probability=" << options.hit_probability();

  auto range_data_inserter = 
      absl::make_unique<ProbabilityGridRangeDataInserter2D>(options);

  // 7. 逐帧处理所有数据
  int nodes_with_real_range_data = 0;
  int nodes_with_synthetic_range_data = 0;
  
  for (const auto& node : nodes) {
    // 7a. 处理range_datas
    if (!node.data.constant_data->range_datas.empty()) {
      nodes_with_real_range_data++;
      for (const auto& range_data : node.data.constant_data->range_datas) {
        sensor::RangeData global_range_data;
        global_range_data.origin = (node.data.global_pose * 
                                    range_data.origin.cast<double>()).cast<float>();
        global_range_data.returns = sensor::TransformPointCloud(
                                      range_data.returns,
                                      node.data.global_pose.cast<float>());
        global_range_data.misses = sensor::TransformPointCloud(
                                     range_data.misses,
                                     node.data.global_pose.cast<float>());
        global_submap->InsertRangeData(global_range_data, range_data_inserter.get());
      }
    } else {
      // 从点云生成合成range data
      const auto& point_cloud = node.data.constant_data->filtered_gravity_aligned_point_cloud;
      if (!point_cloud.empty()) {
        nodes_with_synthetic_range_data++;
        sensor::RangeData synthetic_range_data;
        synthetic_range_data.origin = node.data.constant_data->local_pose.cast<float>().translation();
        synthetic_range_data.returns = point_cloud;
        
        sensor::RangeData global_range_data;
        global_range_data.origin = node.data.global_pose.cast<float>().translation();
        global_range_data.returns = sensor::TransformPointCloud(
                                      synthetic_range_data.returns,
                                      node.data.global_pose.cast<float>());
        global_submap->InsertRangeData(global_range_data, range_data_inserter.get());
      }
    }

    // 7b. 处理点云数据
    auto transformed_cloud = sensor::TransformPointCloud(
                                node.data.constant_data->filtered_gravity_aligned_point_cloud,
                                node.data.global_pose.cast<float>());
    auto transformed_highlight_cloud = sensor::TransformPointCloud(
                                        node.data.constant_data->highlight_point_cloud,
                                        node.data.global_pose.cast<float>());
    global_submap->InsertCloudData(transformed_cloud, transformed_highlight_cloud);

    // 7c. 处理线特征数据
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
          sensor::LineData global_line = 
              sensor::TransformLineData(global_pose_2d, line);
          global_tracked_lines[tracked_element.first].push_back(global_line);
        }
      }
      
      if (!global_tracked_lines.empty()) {
        global_submap->InsertLineData(global_tracked_lines);
      }
    }
  }

  // 8. Finish前统计线特征数量
  const auto* line_map_before = global_submap->line_map_2d_list();
  size_t line_count_before_finish = 0;
  if (line_map_before) {
    line_count_before_finish = line_map_before->GetLineMapData2DList().size();
  }
  LOG(INFO) << "Line features before Finish(): " << line_count_before_finish;

  // 9. 完成submap构建（会执行LineDataCulling剔除低质量线特征）
  global_submap->Finish();
  
  // 10. 输出详细的统计信息
  LOG(INFO) << "=== Global Static Submap Generation Statistics ===";
  LOG(INFO) << "Trajectory nodes processed: " << nodes.size();
  LOG(INFO) << "  - Nodes with real range data: " << nodes_with_real_range_data;
  LOG(INFO) << "  - Nodes with synthetic range data: " << nodes_with_synthetic_range_data;
  
  // 栅格地图信息
  const auto* grid = dynamic_cast<const ProbabilityGrid*>(global_submap->grid());
  if (grid) {
    const auto& limits = grid->limits();
    LOG(INFO) << "Grid Map:";
    LOG(INFO) << "  - Resolution: " << limits.resolution() << " meters/cell";
    LOG(INFO) << "  - Size: " << limits.cell_limits().num_x_cells << " x " 
              << limits.cell_limits().num_y_cells << " cells";
    LOG(INFO) << "  - Origin: [" << origin.x() << ", " << origin.y() << "]";
  }
  
  // 点云信息
  const auto* cloud = global_submap->cloud();
  if (cloud) {
    LOG(INFO) << "Point Cloud: " << cloud->size() << " points";
  } else {
    LOG(INFO) << "Point Cloud: empty";
  }
  
  const auto* highlight_cloud = global_submap->highlight_cloud();
  if (highlight_cloud) {
    LOG(INFO) << "Highlight Point Cloud: " << highlight_cloud->size() << " points";
  } else {
    LOG(INFO) << "Highlight Point Cloud: empty";
  }
  
  // 线特征信息
  const auto* line_map_2d_list = global_submap->line_map_2d_list();
  if (line_map_2d_list) {
    const auto& line_data_map = line_map_2d_list->GetLineMapData2DList();
    LOG(INFO) << "Line Features: " << line_data_map.size() << " line segments";
  } else {
    LOG(INFO) << "Line Features: empty";
  }
  
  // Range data信息
  LOG(INFO) << "Range Data Entries: " << global_submap->num_range_data();
  
  LOG(INFO) << "==================================================";
  LOG(INFO) << "Global static submap generation completed successfully";
  
  // 保存到成员变量
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
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  io::ProtoStreamDeserializer deserializer(reader);

  // Create a copy of the pose_graph_proto, such that we can re-write the
  // trajectory ids.
  proto::PoseGraph pose_graph_proto = deserializer.pose_graph();
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

  MapById<SubmapId, transform::Rigid3d> submap_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Submap& submap_proto :
         trajectory_proto.submap()) {
      submap_poses.Insert(SubmapId{trajectory_proto.trajectory_id(),
                                   submap_proto.submap_index()},
                          transform::ToRigid3(submap_proto.pose()));
    }
  }

  MapById<NodeId, transform::Rigid3d> node_poses;
  for (const proto::Trajectory& trajectory_proto :
       pose_graph_proto.trajectory()) {
    for (const proto::Trajectory::Node& node_proto : trajectory_proto.node()) {
      node_poses.Insert(
          NodeId{trajectory_proto.trajectory_id(), node_proto.node_index()},
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

  if (options_.use_trajectory_builder_3d()) {
    CHECK_NE(deserializer.header().format_version(),
             io::kFormatVersionWithoutSubmapHistograms)
        << "The pbstream file contains submaps without rotational histograms. "
           "This can be converted with the 'pbstream migrate' tool, see the "
           "Cartographer documentation for details. ";
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
        const SubmapId submap_id(proto.submap().submap_id().trajectory_id(),
                                 proto.submap().submap_id().submap_index());
        pose_graph_->AddSubmapFromProto(submap_poses.at(submap_id),
                                        proto.submap());
        break;
      }
      case SerializedData::kNode: {
        proto.mutable_node()->mutable_node_id()->set_trajectory_id(
            trajectory_remapping.at(proto.node().node_id().trajectory_id()));
        const NodeId node_id(proto.node().node_id().trajectory_id(),
                             proto.node().node_id().node_index());
        const transform::Rigid3d& node_pose = node_poses.at(node_id);
        pose_graph_->AddNodeFromProto(node_pose, proto.node());
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
        LOG(INFO) << "Loading global static submap from pbstream";
        if (options_.use_trajectory_builder_2d()) {
          ValueConversionTables conversion_tables;
          global_static_submap_ = std::make_shared<Submap2D>(
              proto.global_static_submap(), &conversion_tables);
          LOG(INFO) << "Global static submap loaded successfully";
        }
        break;
      }
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          SubmapId{constraint_proto.submap_id().trajectory_id(),
                   constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        FromProto(pose_graph_proto.constraint()));
  }
  CHECK(reader->eof());
  return trajectory_remapping;
}

/**
 * @brief 从文件中加载状态，从.pbstream文件中，加载成ProtoStream数据流
 * 
 * @param state_filename 文件路径
 * @param load_frozen_state 
 * @return std::map<int, int> 老轨迹的trajectory_id, 新轨迹的new_trajectory_id
 */
std::map<int, int> MapBuilder::LoadStateFromFile(
    const std::string& state_filename, const bool load_frozen_state) {
  const std::string suffix = ".pbstream";
  if (state_filename.substr(
          std::max<int>(state_filename.size() - suffix.size(), 0)) != suffix) {
    LOG(WARNING) << "The file containing the state should be a "
                    ".pbstream file.";
  }
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  io::ProtoStreamReader stream(state_filename);
  return LoadState(&stream, load_frozen_state);
}

/**
 * @brief Create a Map Builder object
 * 
 * @param options 
 * @return std::unique_ptr<MapBuilderInterface> 
 */
std::unique_ptr<MapBuilderInterface> CreateMapBuilder(
    const proto::MapBuilderOptions& options) {
  return absl::make_unique<MapBuilder>(options);
}

}  // namespace mapping
}  // namespace cartographer
