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
#include <optional>

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
#include "cartographer/localization/internal/2d/consistency_checker.h"
#include "cartographer/localization/internal/2d/consistency_debug_dumper.h"
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

// 对外匹配质量分档（LocalizationState.match_level 的产生源）。
// 不是对 confidence 分数直接比阈值——confidence 是环境依赖的平均占用概率，
// 绝对值在空旷区/走廊差别很大；分档一律取自已有的两套判定：
// ConsistencyCheckResult.is_consistent（三路信号）+ DriftCheckResult.drift_suspected（static map 独立锚点）。
enum class MatchLevel
{
  UNKNOWN  = 0,  // 定位未运行 / 未收敛（loc_state_ != LOCALIZED）
  GOOD     = 1,  // 一致性通过，无自洽漂移嫌疑
  DEGRADED = 2,  // 检测到自洽漂移，但已在 static map 上校正成功（位姿可用，地图/环境存疑）
  POOR     = 3   // 一致性未通过，或漂移校正失败导致地图更新冻结（位姿不可信）
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

  void InitGlobalMapsFromSubmap(std::shared_ptr<mapping::Submap2D> global_submap);

  std::shared_ptr<mapping::Submap2D> GetGlobalStaticSubmap() const { return global_static_submap_; }
  
  // 获取 LocalMapUpdater2D (用于访问动态地图策略和动态 submap)
  LocalMapUpdater2D* GetLocalMapUpdater() const { return local_map_updater_.get(); }

  bool SetInitialPose(const transform::Rigid3d &initial_pose);

  LocalizationState GetLocalizationState() const { return loc_state_; }

  // 获取当前定位置信度（ScanMatch 后的地图重合率 [0,1]）
  double GetLocalizationConfidence() const;

  // 获取当前匹配质量分档（见 MatchLevel 注释：取自一致性检查 + 漂移门控，非分数比阈值）
  MatchLevel GetMatchLevel() const { return match_level_; }

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

  // 清空 active submap 并重置与之绑定的状态（关键帧过滤器、一致性检查的速度历史）。
  // 只 clear() 会留下三处仍记着重置前轨迹的状态，见 .cc 内说明。
  void ResetActiveSubmaps(const char* reason);

  sensor::FeatureData GetFeatureData(
    const common::Time & time,
    const std::vector<laser_perception::FeatureMsgWithOrigin> & feature_data);

  sensor::RangeData TransformToGravityAlignedFrameAndFilter(
	  const transform::Rigid3f& transform_to_gravity_aligned_frame,
	  const sensor::RangeData& range_data) const;

  sensor::RangeData FilterRangeData(
	  const sensor::RangeData& range_data) const;

  // 线特征存在两套互不相通的 id 编号，插入时必须各用各的：
  //   tracked_lane_data        —— active_submaps_ 的局部编号，写回 active_submaps_
  //   global_tracked_lane_data —— 全局地图编号（继承自 pbstream），写回全局动态图
  // 把局部编号写进全局图会让观测被合并进 id 相同但物理上无关的线段。
  std::unique_ptr<InsertionResult> InsertIntoSubmap(
    const common::Time time,
    const preprocess::MergedData& merged_data,
    const std::map<int, std::vector<sensor::LineData>>& tracked_lane_data,
    const std::map<int, std::vector<sensor::LineData>>& global_tracked_lane_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment);

  // Scan matches 'filtered_gravity_aligned_point_cloud' and returns the
  // observed pose, or nullptr on failure.
  // 两个跟踪结果都要输出：局部编号供 active_submaps_ 使用，全局编号供全局动态图使用。
  std::unique_ptr<transform::Rigid2d> ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const std::vector<sensor::LineData> & line_data,
    std::map<int, std::vector<sensor::LineData>> & track_lane_data,
    std::map<int, std::vector<sensor::LineData>> & global_tracked_lane_data);

  // 重定位之后，需要初始化建图相关的数据
  void ResetMapping();

private:
  const proto::PoseEstimatorOptions2D options_;
  mapping::ActiveSubmaps2D active_submaps_;

  // 全局静态地图submap，包含所有全局地图数据(包括cloud和highlight_cloud的ikd-tree)
  std::shared_ptr<mapping::Submap2D> global_static_submap_ = nullptr;
  
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

  // 一致性检查器，用于检测定位与地图的一致性
  std::unique_ptr<ConsistencyChecker> consistency_checker_;

  // 一致性/恢复现场快照（排障用，默认关闭）
  ConsistencyDebugDumper consistency_debug_dumper_;

  MatchLevel match_level_ = MatchLevel::UNKNOWN;  // 每帧由一致性/漂移判定更新

  std::mutex data_mutex_;
  
  bool has_tried_global_relocalization_ = false;  // 标志是否已尝试全局重定位

  std::fstream time_file_;
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_ESTIMATOR_2D_H_
