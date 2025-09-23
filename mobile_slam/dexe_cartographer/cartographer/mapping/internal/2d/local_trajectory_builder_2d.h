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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <deque>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/laser_synchronizer.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/mapping/proto/local_trajectory_builder_options_2d.pb.h"
#include "cartographer/localization/internal/2d/pose_estimator_2d.h"
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

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace cartographer {
namespace mapping {

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class LocalTrajectoryBuilder2D {
 public:
  struct InsertionResult {
    std::shared_ptr<const TrajectoryNode::Data> constant_data;  // 轨迹约束
    std::vector<std::shared_ptr<const Submap2D>> insertion_submaps; // submap
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

  struct SensorRawData {
    std::string sensor_id;
    sensor::RangeData range_data_in_local;
    sensor::PointCloud filtered_gravity_aligned_point_cloud;
    sensor::PointCloud highlight_point_cloud;
  };

  explicit LocalTrajectoryBuilder2D(
	  const proto::LocalTrajectoryBuilderOptions2D& options,
	  const std::vector<std::string>& expected_range_sensor_ids);
  ~LocalTrajectoryBuilder2D();

  LocalTrajectoryBuilder2D(const LocalTrajectoryBuilder2D&) = delete;
  LocalTrajectoryBuilder2D& operator=(const LocalTrajectoryBuilder2D&) = delete;

  bool SetInitialPose(const transform::Rigid3d &initial_pose) { return false; }
  std::optional<localization::LocalizationState> GetLocalizationState() const { return std::nullopt; }

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
  std::unique_ptr<MatchingResult> AddAccumulatedRangeData(
	  const std::string& sensor_id,
	  common::Time time, const sensor::RangeData& gravity_aligned_range_data,
    const std::vector<sensor::LineData> & line_data,
    const std::vector<sensor::LandmarkPointData> & reflector_data,
	  const transform::Rigid3d& gravity_alignment,
	  const absl::optional<common::Duration>& sensor_duration,
    const std::vector<SensorRawData>& sensor_raw_data_list);

  sensor::FeatureData GetFeatureData(
    const common::Time & time,
    const std::vector<laser_perception::FeatureMsgWithOrigin> & feature_data);

  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
	  const transform::Rigid3f& transform_to_gravity_aligned_frame,
	  const sensor::RangeData& range_data) const;

  sensor::RangeData FilterRangeData(
	  const sensor::RangeData& range_data) const;

  std::unique_ptr<InsertionResult> InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment,
    const std::map<int, std::vector<sensor::LineData>> & line_data_in_local,
    const std::map<int, std::vector<sensor::LineData>> & tracked_lane_data,
    const std::vector<sensor::LandmarkPointData> & reflector_data,
    const std::vector<SensorRawData>& sensor_raw_data_list);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  std::unique_ptr<transform::Rigid2d> ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const std::vector<sensor::LineData> & line_data,
    std::map<int, std::vector<sensor::LineData>> & track_lane_data,
    Eigen::Matrix3d & cov);

  // Lazily constructs a PoseExtrapolator.
  bool is_exptrapolator_initialized_ = false;
  void InitializeExtrapolator(common::Time time);

  // 获取指定sensor_id的数据预处理器
  preprocess::DataPreprocessor* GetDataPreprocessor(const std::string& sensor_id);

  // 处理同步后的激光数据
  std::unique_ptr<MatchingResult> ProcessSynchronizedLaserData(
      const LaserSynchronizer::SynchronizedLaserData& synchronized_data);

  // 处理单个传感器的激光数据，返回处理后的数据结构
  struct ProcessedSensorData {
    std::string sensor_id;
    sensor::RangeData undistorted_range_data;
    sensor::FeatureData feature_data;
    sensor::PointCloud filtered_gravity_aligned_point_cloud;
    sensor::PointCloud highlight_point_cloud;
  };

  absl::optional<ProcessedSensorData> ProcessSingleSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& laser_data,
      const common::Time& sync_time);

  // 里程计缓冲区管理方法
  void AddOdometryToBuffer(const sensor::OdometryData& odometry_data);
  absl::optional<sensor::OdometryData> GetOdometryAtTime(const common::Time& time) const;
  transform::Rigid3d GetOdomDeltaPose(const common::Time& laser_time);
  void UpdateLastOdomForDelta(const common::Time& laser_time);

  const proto::LocalTrajectoryBuilderOptions2D options_;
  ActiveSubmaps2D active_submaps_;

  // MotionFilter motion_filter_;
  MotionFilter keyframe_motion_filter_;
  MotionFilter map_insertion_motion_filter_;
  scan_matching::RealTimeCorrelativeScanMatcher2D
	  real_time_correlative_scan_matcher_;
  scan_matching::CeresScanMatcher2D ceres_scan_matcher_;
	
	std::mutex extrapolator_mutex_;
  std::shared_ptr<PoseExtrapolator> extrapolator_;

  int num_accumulated_ = 0;
  sensor::RangeData accumulated_range_data_;

  absl::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  absl::optional<double> last_thread_cpu_time_seconds_;
  absl::optional<common::Time> last_sensor_time_;

  LaserSynchronizer laser_synchronizer_;

  // 多雷达数据预处理器管理，每个sensor_id对应一个预处理器
  const std::vector<std::string> expected_range_sensor_ids_;  // 预期的传感器ID列表
  std::map<std::string, std::unique_ptr<preprocess::DataPreprocessor>> data_preprocessors_;
  bool is_sensor_to_tracking_set_ = false;
  transform::Rigid3d sensor_to_tracking_ = transform::Rigid3d::Identity();

  transform::Rigid3d local_pose_ = transform::Rigid3d::Identity();
  transform::Rigid3d temp_pose_ = transform::Rigid3d::Identity();
  sensor::OdometryData curr_odom_data_;

  // 里程计数据缓冲区
  mutable std::mutex odom_buffer_mutex_;
  std::deque<sensor::OdometryData> odom_buffer_;
  absl::optional<sensor::OdometryData> last_odom_for_delta_;  // 用于计算delta pose的上次里程计数据
  static constexpr size_t kMaxOdomBufferSize = 1000;  // 最大缓冲区大小
  static constexpr double kMaxOdomBufferDuration = 10.0;  // 最大缓冲时间(秒)

  std::unique_ptr<laser_perception::LaserPerception> laser_perception_;

  std::unique_ptr<scan_matching::LineMatcher2D> line_matcher_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_2D_LOCAL_TRAJECTORY_BUILDER_2D_H_
