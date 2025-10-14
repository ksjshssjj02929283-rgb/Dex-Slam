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

#ifndef CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_ESTIMATOR_2D_H_
#define CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_ESTIMATOR_2D_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <fstream>
#include <string>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/preprocess/laser_synchronizer.h"
#include "cartographer/preprocess/data_preprocessor.h"

#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/laser_perception.h"
#include "cartographer/localization/proto/pose_estimator_options_2d.pb.h"
#include "cartographer/localization/2d/grid_map_2d.h"
#include "cartographer/localization/2d/cloud_map_2d.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/localization/internal/2d/pose_initializer_2d.h"
#include "cartographer/localization/amcl/core/amcl_core.h"
#include "cartographer/localization/2d/local_map_updater_2d.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace cartographer {
namespace localization {

enum class LocalizationState
{
  LOCALIZATION_FAILED = -2,  // 定位失败
  INITIALIZATION_FAILED = -1,  // 系统初始化中
  NOT_INITIALIZED = 0,  // 系统初始化中
  INITIALIZING = 1,  // 系统初始化中
  INITIALIZED  = 2,  // 系统初始化完成
  RELOCALIZING = 3,  // 重定位中
  LOCALIZED    = 4  // 重定位成功
};

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class PoseEstimator2D {
 public:
  struct InsertionResult {
    std::shared_ptr<const mapping::TrajectoryNode::Data> constant_data;  // 轨迹约束
    std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps; // submap
  };
  struct MatchingResult {
    common::Time time;
    transform::Rigid3d local_pose;  // 位姿(in local frame)
    std::vector<transform::Rigid2d> particle_poses;
    sensor::RangeData range_data_in_local;  // range data
    std::map<int, std::vector<sensor::LineData>>  line_data_in_local;
    // 'nullptr' if dropped by the motion filter.
    std::unique_ptr<const InsertionResult> insertion_result;    // 插入结果
  };

  explicit PoseEstimator2D(
	  const proto::PoseEstimatorOptions2D& options,
	  const std::vector<std::string>& expected_range_sensor_ids);
  ~PoseEstimator2D();

  PoseEstimator2D(const PoseEstimator2D&) = delete;
  PoseEstimator2D& operator=(const PoseEstimator2D&) = delete;

  void InitGlobalGridMap(std::shared_ptr<GridMap2D> grid_map);
  void InitGlobalCloudMap(std::shared_ptr<CloudMap2D> cloud_map);
  void InitGlobalHighlightCloudMap(std::shared_ptr<CloudMap2D> highlight_cloud_map);
  void InitGlobalLineMap(const std::shared_ptr<mapping::LineMap2DList>& line_map);

  std::shared_ptr<GridMap2D> GetGlobalGridMap() const { return global_grid_map_; }
  std::shared_ptr<CloudMap2D> GetGlobalCloudMap() const { return global_cloud_map_; }
  std::shared_ptr<CloudMap2D> GetGlobalHighlighCloudMap() const { return global_highlight_cloud_map_; }
  std::shared_ptr<mapping::LineMap2DList> GetGlobalLineMap() const { return global_line_map_; }

  bool SetInitialPose(const transform::Rigid3d &initial_pose);

  LocalizationState GetLocalizationState() const { return loc_state_; }

  // 触发全局重定位，不需要初始位姿
  void TriggerGlobalRelocalization();

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  std::unique_ptr<MatchingResult> AddRangeData(
	  const std::string& sensor_id,
	  const sensor::TimedPointCloudData& range_data);
  void AddImuData(const sensor::ImuData& imu_data);
  void AddOdometryData(const sensor::OdometryData& odometry_data);

  static void RegisterMetrics(metrics::FamilyFactory* family_factory);

 private:
  // 使用MergedData进行位姿估计和地图更新的重载版本
  std::unique_ptr<MatchingResult> ProcessDataAndEstimatePose(
    const std::string& sensor_id,
    const common::Time& time,
    const preprocess::MergedData& merged_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration,
    preprocess::DataPreprocessor* data_preprocessor);



  std::unique_ptr<MatchingResult> ProcessSynchronizedLaserData(
      const preprocess::LaserSynchronizer::SynchronizedLaserData& synchronized_data);

  transform::Rigid3d GetOdomDeltaPose(const common::Time& laser_time);
  absl::optional<sensor::OdometryData> GetOdometryAtTime(const common::Time& time) const;
  void UpdateLastOdomForDelta(const common::Time& time);

  sensor::FeatureData GetFeatureData(
    const common::Time & time,
    const std::vector<laser_perception::FeatureMsgWithOrigin> & feature_data);

  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
	  const transform::Rigid3f& transform_to_gravity_aligned_frame,
	  const sensor::RangeData& range_data) const;

  sensor::RangeData FilterRangeData(
	  const sensor::RangeData& range_data) const;

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
    const common::Time time,
    const preprocess::MergedData& merged_data,
    const std::map<int, std::vector<sensor::LineData>>& tracked_lane_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const std::vector<sensor::LineData> & line_data,
    std::map<int, std::vector<sensor::LineData>> & track_lane_data,
    Eigen::Matrix3d & cov);

  // 重定位之后，需要初始化建图相关的数据
  void ResetMapping();

  static amcl::pf_vector_t UniformPoseGenerator(void* arg);
  static std::vector<std::pair<int, int>> free_space_indices_;

private:
  const proto::PoseEstimatorOptions2D options_;
  mapping::ActiveSubmaps2D active_submaps_;

  std::shared_ptr<GridMap2D> global_grid_map_ = nullptr;
  std::shared_ptr<CloudMap2D> global_cloud_map_ = nullptr;
  std::shared_ptr<CloudMap2D> global_highlight_cloud_map_ = nullptr;
  std::unique_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> global_highlight_cloud_ikdtree_ = nullptr;
  std::shared_ptr<mapping::LineMap2DList> global_line_map_ = nullptr;
  // MotionFilter motion_filter_;
  mapping::MotionFilter keyframe_motion_filter_;
  mapping::MotionFilter map_insertion_motion_filter_;
  mapping::scan_matching::RealTimeCorrelativeScanMatcher2D
	  real_time_correlative_scan_matcher_;
  mapping::scan_matching::CeresScanMatcher2D ceres_scan_matcher_;

  LocalizationState loc_state_ = LocalizationState::NOT_INITIALIZED;

  std::unique_ptr<PoseInitializer> pose_initializer_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  mapping::RangeDataCollator range_data_collator_;
  preprocess::LaserSynchronizer laser_synchronizer_;
  std::vector<std::string> expected_range_sensor_ids_;

  std::unique_ptr<preprocess::DataPreprocessor> data_preprocessor_;
  bool is_sensor_to_tracking_set_ = false;
  transform::Rigid3d sensor_to_tracking_ = transform::Rigid3d::Identity();

  transform::Rigid3d local_pose_ = transform::Rigid3d::Identity();

  sensor::OdometryData last_laser_odom_data_;
  sensor::OdometryData curr_odom_data_;
  absl::optional<sensor::OdometryData> last_odom_for_delta_;

  std::unique_ptr<LocalMapUpdater2D> local_map_updater_;

  std::unique_ptr<laser_perception::LaserPerception> laser_perception_;

  std::unique_ptr<mapping::scan_matching::LineMatcher2D> line_matcher_;

  std::unique_ptr<amcl::AMCLCore> amcl_core_;

  std::mutex data_mutex_;
  
  bool has_tried_global_relocalization_ = false;  // 标志是否已尝试全局重定位

  std::fstream time_file_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_ESTIMATOR_2D_H_
