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

void LocalizationUpdater::InitGlobalGridMap(std::shared_ptr<GridMap2D> grid_map)
{
  CHECK(grid_map != nullptr);
  global_grid_map_ = grid_map;
}

void LocalizationUpdater::InitGlobalCloudMap(
                              std::shared_ptr<CloudMap2D> cloud_map,
                              std::shared_ptr<CloudMap2D> highlight_cloud_map)
{
  CHECK(cloud_map != nullptr);
  global_cloud_map_ = cloud_map;
  if (highlight_cloud_map != nullptr) {
    if (highlight_cloud_map->cloud()->size() > 0) {
      global_highlight_cloud_map_ = highlight_cloud_map;
    } else {
      global_highlight_cloud_map_ = std::make_shared<CloudMap2D>();
      LOG(WARNING) << "highlight_cloud_map is empty";
    }
  } else {
    global_highlight_cloud_map_ = std::make_shared<CloudMap2D>();
    LOG(WARNING) << "highlight_cloud_map is empty";
  }
}

void LocalizationUpdater::InitGlobalLineMap(const std::shared_ptr<sensor::LineMapData>& line_map_data)
{
  CHECK(line_map_data != nullptr);
  std::shared_ptr<mapping::LineMap2DList> line_map_2d(new mapping::LineMap2DList());
  for (const auto& line_data_w_normal : *line_map_data) {
    sensor::LineData line_data(line_data_w_normal.data.line_data.begin,
                               line_data_w_normal.data.line_data.end);
    line_map_2d->PushBack({line_data});
  }
  global_line_map_ = line_map_2d;
}

std::shared_ptr<GridMap2D> LocalizationUpdater::GetGlobalGridMap() const
{
  return global_grid_map_;
}

std::shared_ptr<CloudMap2D> LocalizationUpdater::GetGlobalCloudMap() const
{
  return global_cloud_map_;
}

std::shared_ptr<CloudMap2D> LocalizationUpdater::GetGlobalHighlighCloudMap() const
{
  return global_highlight_cloud_map_;
}

std::shared_ptr<sensor::LineMapData> LocalizationUpdater::GetGlobalLineMap() const
{
  std::shared_ptr<sensor::LineMapData> line_map_data(new sensor::LineMapData());
  const auto line_map_2d_list = global_line_map_->GetLineMapData2DList();
  for (const auto& map_line : line_map_2d_list) {
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

std::optional<LocalizationState> LocalizationUpdater::GetLocalizationState()
{
  const int last_trajectory_id = num_trajectory_builders() - 1;
  return trajectory_builders_[last_trajectory_id]->GetLocalizationState();
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

  pose_estimator_2d->InitGlobalGridMap(global_grid_map_);
  pose_estimator_2d->InitGlobalLineMap(global_line_map_);
  pose_estimator_2d->InitGlobalCloudMap(global_cloud_map_);
  pose_estimator_2d->InitGlobalHighlightCloudMap(global_highlight_cloud_map_);

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

void LocalizationUpdater::SerializeState(bool include_unfinished_submaps,
                                io::ProtoStreamWriterInterface* const writer) {
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, writer,
                    include_unfinished_submaps);
}

bool LocalizationUpdater::SerializeStateToFile(bool include_unfinished_submaps,
                                      const std::string& filename) {
  io::ProtoStreamWriter writer(filename);
  io::WritePbStream(*pose_graph_, all_trajectory_builder_options_, &writer,
                    include_unfinished_submaps);
  return (writer.Close());
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
      default:
        LOG(WARNING) << "Skipping unknown message type in stream: "
                     << proto.GetTypeName();
    }
  }

  if (load_frozen_state) {
    // Add information about which nodes belong to which submap.
    // This is required, even without constraints.
    for (const mapping::proto::PoseGraph::Constraint& constraint_proto :
         pose_graph_proto.constraint()) {
      if (constraint_proto.tag() !=
          mapping::proto::PoseGraph::Constraint::INTRA_SUBMAP) {
        continue;
      }
      pose_graph_->AddNodeToSubmap(
          mapping::NodeId{constraint_proto.node_id().trajectory_id(),
                 constraint_proto.node_id().node_index()},
          mapping::SubmapId{constraint_proto.submap_id().trajectory_id(),
                            constraint_proto.submap_id().submap_index()});
    }
  } else {
    // When loading unfrozen trajectories, 'AddSerializedConstraints' will
    // take care of adding information about which nodes belong to which
    // submap.
    pose_graph_->AddSerializedConstraints(
        mapping::FromProto(pose_graph_proto.constraint()));
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
  return LoadState(&stream, load_frozen_state);
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
