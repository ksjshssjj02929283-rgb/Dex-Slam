/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>
#include <optional>
#include <type_traits>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/localization/internal/2d/pose_estimator_2d.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {

namespace mapping {

namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback,
      const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback),
        pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter)
  {
    is_pose_estimator_ = std::is_same<decltype(local_trajectory_builder_),
                                  std::unique_ptr<localization::PoseEstimator2D>>::value;
  }
  ~GlobalTrajectoryBuilder() override {}

  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  bool SetInitialPose(const transform::Rigid3d &initial_pose) override
  {
    // 如果是地图扩建，则需要使用pose estimator
    if (!is_pose_estimator_) {
      LOG(WARNING) << "Mapping mode do not support initial pose right now.";
      return false;
    }

    const auto& pose_estimator = local_trajectory_builder_;
    return pose_estimator->SetInitialPose(initial_pose);;
  }

  std::optional<localization::LocalizationState> GetLocalizationState() const
  { 
    // 如果是地图扩建，则需要使用pose estimator
    if (!is_pose_estimator_) {
      LOG(WARNING) << "Mapping mode do not support get localization states right now.";
      return std::nullopt;
    }
    const auto& pose_estimator = local_trajectory_builder_;

    return pose_estimator->GetLocalizationState();
  }

  /**
   * @brief 传入激光雷达数据
   * 
   * @param sensor_id 
   * @param timed_point_cloud_data 
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";
    // 1. 出入激光数据到 local_trajectory_builder_ 中,并计算出 local_pose
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);
    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    kLocalSlamMatchingResults->Increment();  // 增加计数
    std::unique_ptr<InsertionResult> insertion_result;
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();  // 增加计数
      // 2. 将匹配结果加入到 pose_graph_ 中
      // 如果是地图扩建，则需要使用pose estimator
      NodeId node_id;
      if (!is_pose_estimator_) {
        node_id = pose_graph_->AddNode(
            matching_result->insertion_result->constant_data, trajectory_id_,
            matching_result->insertion_result->insertion_submaps);

        CHECK_EQ(node_id.trajectory_id, trajectory_id_);
      } else {
        static int index = 0;
        node_id = NodeId(1, index++);
      }

      // 3. 匹配结果保存
      insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
                            node_id, matching_result->insertion_result->constant_data,
                            std::vector<std::shared_ptr<const Submap>>(
                                matching_result->insertion_result->insertion_submaps.begin(),
                                matching_result->insertion_result->insertion_submaps.end()),
                            std::make_shared<sensor::PointCloud>(
                                *(matching_result->insertion_result->insertion_submaps.front()->cloud())),
                            std::make_shared<LineMap2DList>(
                                *(matching_result->insertion_result->insertion_submaps.front()->line_map_2d_list()))
                          });
    }
    // 4. 将匹配结果传递给 local_slam_result_callback_
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->particle_poses),
          std::move(matching_result->range_data_in_local),
          std::move(matching_result->line_data_in_local),
          std::move(insertion_result));
    }
  }

  /**
   * @brief 传入IMU数据
   * 
   * @param sensor_id 
   * @param imu_data 
   */
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    // 1. 如果有 local_trajectory_builder_ 则将 IMU 数据传入到 local_trajectory_builder_ 中
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddImuData(imu_data);
    }
    // 2. 将 IMU 数据传入到 pose_graph_ 中
    if (!is_pose_estimator_) {
      pose_graph_->AddImuData(trajectory_id_, imu_data);
    }
  }

  /**
   * @brief 传入odom数据
   * 
   * @param sensor_id 
   * @param odometry_data 
   */
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    // 1. 如果有 local_trajectory_builder_ 则将 odom 数据传入到 local_trajectory_builder_ 中
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    // TODO(MichaelGrupp): Instead of having an optional filter on this level,
    // odometry could be marginalized between nodes in the pose graph.
    // Related issue: cartographer-project/cartographer/#1768
    // 2. 如果有 pose_graph_odometry_motion_filter_,则进行odom位姿滤波
    if (pose_graph_odometry_motion_filter_.has_value() &&
        pose_graph_odometry_motion_filter_.value().IsSimilar(
            odometry_data.time, odometry_data.pose)) {
      return;
    }
    // 3. 将 odom 数据传入到 pose_graph_ 中
    if (!is_pose_estimator_) {
      pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
    }
  }

  /**
   * @brief 传入固定坐标点数据（绝对观测、GPS等）
   * 
   * @param sensor_id 
   * @param fixed_frame_pose 
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    // 向 pose_graph_ 中添加固定坐标点数据
    if (!is_pose_estimator_) {
      pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
    }
  }

  /**
   * @brief 传入路标点的观测
   * 
   * @param sensor_id 
   * @param landmark_data 
   */
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    // 向 pose_graph_ 中添加路标点数据
    if (!is_pose_estimator_) {
      pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
    }
  }

  /**
   * @brief 向pose_graph传入local slam的结果
   * 
   * @param local_slam_result_data 
   */
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;
  LocalSlamResultCallback local_slam_result_callback_;
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter_;

  bool is_pose_estimator_ = false;
};

}  // namespace

/**
 * @brief Create a Global Trajectory Builder 2 D object
 * 
 * @param local_trajectory_builder 
 * @param trajectory_id 
 * @param pose_graph 
 * @param local_slam_result_callback 
 * @param pose_graph_odometry_motion_filter 
 * @return std::unique_ptr<TrajectoryBuilderInterface> 
 */
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  // 直接返回构造结果
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}


/**
 * @brief Create a Global Trajectory Builder 3 D object
 * 
 * @param local_trajectory_builder 
 * @param trajectory_id 
 * @param pose_graph 
 * @param local_slam_result_callback 
 * @param pose_graph_odometry_motion_filter 
 * @return std::unique_ptr<TrajectoryBuilderInterface> 
 */
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}

std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<localization::PoseEstimator2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  // 直接返回构造结果
  return absl::make_unique<
      GlobalTrajectoryBuilder<localization::PoseEstimator2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
  auto* results = factory->NewCounterFamily(
      "mapping_global_trajectory_builder_local_slam_results",
      "Local SLAM results");
  kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
  kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}
 
}  // namespace mapping

}  // namespace cartographer