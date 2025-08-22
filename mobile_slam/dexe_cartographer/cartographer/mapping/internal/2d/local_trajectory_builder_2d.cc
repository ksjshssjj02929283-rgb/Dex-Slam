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

#include "cartographer/mapping/internal/2d/local_trajectory_builder_2d.h"

#include <limits>
#include <memory>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "absl/memory/memory.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/common/tic_toc.h"

namespace cartographer {
namespace mapping {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

LocalTrajectoryBuilder2D::LocalTrajectoryBuilder2D(
    const proto::LocalTrajectoryBuilderOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      keyframe_motion_filter_(options_.keyframe_motion_filter_options()),
      map_insertion_motion_filter_(options_.map_insertion_motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids)
{
  CHECK(!options_.pose_extrapolator_options().use_imu_based());
  // TODO(gaschler): Consider using InitializeWithImu as 3D does.
  extrapolator_ = std::make_shared<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()),
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant());
  data_preprocessor_ =
    absl::make_unique<preprocess::DataPreprocessor>(
      options_.data_preprocessor_options());

  laser_perception_ =
      std::make_unique<laser_perception::LaserPerception>(
          options_.laser_perception_options());

  line_matcher_ =
      std::make_unique<scan_matching::LineMatcher2D>(
            options_.line_matcher_options());
}

LocalTrajectoryBuilder2D::~LocalTrajectoryBuilder2D() {}

sensor::RangeData
LocalTrajectoryBuilder2D::TransformToGravityAlignedFrameAndFilter(
    const transform::Rigid3f& transform_to_gravity_aligned_frame,
    const sensor::RangeData& range_data) const {
  const sensor::RangeData cropped =
      sensor::CropRangeData(sensor::TransformRangeData(
                                range_data, transform_to_gravity_aligned_frame),
                            options_.min_z(), options_.max_z());
  return sensor::RangeData{
      cropped.origin,
      sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
}

sensor::RangeData
LocalTrajectoryBuilder2D::FilterRangeData(
    const sensor::RangeData& range_data) const {
  return sensor::RangeData{
      range_data.origin,
      sensor::VoxelFilter(range_data.returns, options_.voxel_filter_size()),
      sensor::VoxelFilter(range_data.misses, options_.voxel_filter_size())};
}

/**
 * @brief ScanMatch进行位姿估计
 * 
 * @param time 时间
 * @param pose_prediction 预测的位姿 
 * @param filtered_gravity_aligned_point_cloud 重力对齐的激光点云
 * @return std::unique_ptr<transform::Rigid2d> 位姿估计结果
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const std::vector<sensor::LineData> & line_data,
    std::map<int, std::vector<sensor::LineData>> & tracked_lane_data,
    Eigen::Matrix3d & cov)
{
  // 1. 如果active_submaps_中没有submap，则直接返回pose_prediction
  //    一般情况下，第一帧激光数据进来时，active_submaps_中没有submap
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 2. 读取active_submaps_中的第一个submap，作为匹配的地图
  std::shared_ptr<const Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.

  // LOG(INFO) << "prediction: "<< pose_prediction;

  transform::Rigid2d initial_ceres_pose = pose_prediction;
  // 3. 如果使用correlative方法，则先用correlative方法进行位姿估计
  //    相关性扫描匹配器，速度快，但是精度低
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &initial_ceres_pose);
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
  }

  if (active_submaps_.submaps().size() != 0) {
    line_matcher_->TrackLineFromMap(initial_ceres_pose,
                                    line_data,
                                    *active_submaps_.line_map_2d_list(),
                                    tracked_lane_data);
  }

  // 4. 使用ceres进行位姿估计，得到pose_observation
  //    使用点到地图的误差作为cost function
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
                            filtered_gravity_aligned_point_cloud,
                            *laser_perception_->getHighlightCloud(),
                            tracked_lane_data,
                            *matching_submap->grid(), *matching_submap->line_map_2d_list(),
                            *matching_submap->cloud_ikdtree(),
                            *matching_submap->highlight_cloud_ikdtree(),
                            pose_observation.get(),
                            &summary);

  // LOG(INFO) << "scan match: " << *pose_observation;

  // 5. 如果匹配成功，向Metric中增加观测信息
  if (pose_observation) {
    kCeresScanMatcherCostMetric->Observe(summary.final_cost);
    const double residual_distance =
        (pose_observation->translation() - pose_prediction.translation())
            .norm();
    kScanMatcherResidualDistanceMetric->Observe(residual_distance);
    const double residual_angle =
        std::abs(pose_observation->rotation().angle() -
                 pose_prediction.rotation().angle());
    kScanMatcherResidualAngleMetric->Observe(residual_angle);
  }
  // 6. 返回位姿估计结果（Rigid2d）
  return pose_observation;
}

/**
 * @brief 向LocalTrajectoryBuilder2D中加入激光雷达数据，同时进行位姿估计
 * 
 * @param sensor_id 
 * @param unsynchronized_data 
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 激光匹配的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddRangeData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& unsynchronized_data) {
  if (!is_sensor_to_tracking_set_) {
    sensor_to_tracking_ = unsynchronized_data.sensor_to_tracking;
    is_sensor_to_tracking_set_ = true;
  }

  // 1. 雷达数据预处理，包括雷达数据从range转换为TimedPointCloud
  auto synchronized_data =
      range_data_collator_.AddRangeData(sensor_id, unsynchronized_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return nullptr;
  }

  CHECK(!synchronized_data.ranges.empty());
  // TODO(gaschler): Check if this can strictly be 0.
  CHECK_LE(synchronized_data.ranges.back().point_time.time, 0.f);

  // 2. 不使用IMU，则在AddRangeData中初始化位姿插值器
  const common::Time& time = synchronized_data.time;

  // 关键帧策略
  if (keyframe_motion_filter_.IsSimilar(time, curr_odom_data_.pose)) {
    return nullptr;
  }

  data_preprocessor_->AddSyncedPointCloud(synchronized_data); // 雷达去畸变
  auto undistort_range = data_preprocessor_->GetProcessedRangeData();
  if (!undistort_range.has_value()) {
    return nullptr;
  }

  // 去畸变后的雷达数据是在激光坐标系下
  // 检测特征
  sensor::SyncPointCloudWithOdom sync_point_cloud;
  sync_point_cloud.stamp = undistort_range->time;
  sync_point_cloud.odom = curr_odom_data_;
  sensor::PointCloudWithOrigin cloud;
  cloud.origin = undistort_range->data.origin.cast<double>();
  for (size_t i = 0; i < undistort_range->data.returns.points().size(); i++) {
    cloud.sync_points_base_link.push_back(undistort_range->data.returns.points()[i].position);
  }
  cloud.intensities = undistort_range->data.returns.intensities();
  sync_point_cloud.sync_point_clouds.push_back(cloud);

  std::vector<laser_perception::FeatureMsgWithOrigin>
  feature_data_origin = laser_perception_->laserUpdate(sync_point_cloud);

  auto feature_data = GetFeatureData(time, feature_data_origin);

  // std::cout << "feature_data.line_data: " << feature_data.line_data.size() << std::endl;
  // std::cout << "feature_data.reflector_data: " << feature_data.reflector_data.size() << std::endl;

  const common::Time current_sensor_time = undistort_range.value().time;
  absl::optional<common::Duration> sensor_duration;
  if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
  }
  last_sensor_time_ = current_sensor_time;

  return AddAccumulatedRangeData(
      current_sensor_time,
      FilterRangeData(undistort_range.value().data),
      feature_data.line_data,
      feature_data.reflector_data,
      transform::Rigid3d::Identity(), sensor_duration);
}

/**
 * @brief 对积累的激光数据进行位姿估计
 * 
 * @param time 
 * @param gravity_aligned_range_data 
 * @param gravity_alignment 
 * @param sensor_duration 
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const std::vector<sensor::LineData> & line_data,
    const std::vector<sensor::LandmarkPointData> & reflector_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration) {
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }

  const transform::Rigid3f odom_delta_pose = data_preprocessor_->GetOdomDeltaPose();

  const transform::Rigid2d pose_prediction = transform::Project2D(
    local_pose_ * odom_delta_pose.cast<double>());
  // 3. 对积累的激光数据进行滤波
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns,
                                  options_.adaptive_voxel_filter_options());
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // 4. ScanMatch计算当前帧位姿：对齐重力坐标系在local坐标系中的位姿
  // local map frame <- gravity-aligned frame
  Eigen::Matrix3d loc_cov;
  std::map<int, std::vector<sensor::LineData>> tracked_lane_data;
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction,
                filtered_gravity_aligned_point_cloud,
                line_data,
                tracked_lane_data,
                loc_cov);
  // std::cout << "tracked_lane_data: " << tracked_lane_data.size() << std::endl;
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  local_pose_ = pose_estimate;

  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // 6. 将当前帧的激光数据，按照估计的位姿插入到submap中

  std::map<int, std::vector<sensor::LineData>> lines_in_local;
  for (const auto& data_vec: tracked_lane_data) {
    for (const auto& line: data_vec.second) {
      lines_in_local[data_vec.first].push_back(sensor::TransformLineData(*pose_estimate_2d, line));
    }
  }

  std::unique_ptr<InsertionResult>
    insertion_result = InsertIntoSubmap(
      time, range_data_in_local,
      *laser_perception_->getLessFlatCloud(),
      *laser_perception_->getHighlightCloud(),
      pose_estimate, gravity_alignment.rotation(),
      lines_in_local, tracked_lane_data, reflector_data);

  // 7. 计算local slam的频率
  const auto wall_time = std::chrono::steady_clock::now();
  if (last_wall_time_.has_value()) {
    const auto wall_time_duration = wall_time - last_wall_time_.value();
    kLocalSlamLatencyMetric->Set(common::ToSeconds(wall_time_duration));
    if (sensor_duration.has_value()) {
      kLocalSlamRealTimeRatio->Set(common::ToSeconds(sensor_duration.value()) /
                                   common::ToSeconds(wall_time_duration));
    }
  }

  const double thread_cpu_time_seconds = common::GetThreadCpuTimeSeconds();
  if (last_thread_cpu_time_seconds_.has_value()) {
    const double thread_cpu_duration_seconds =
        thread_cpu_time_seconds - last_thread_cpu_time_seconds_.value();
    if (sensor_duration.has_value()) {
      kLocalSlamCpuRealTimeRatio->Set(
          common::ToSeconds(sensor_duration.value()) /
          thread_cpu_duration_seconds);
    }
  }
  last_wall_time_ = wall_time;
  last_thread_cpu_time_seconds_ = thread_cpu_time_seconds;
  // 8. 返回匹配结果

  std::vector<transform::Rigid2d> particle_poses;
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, pose_estimate,
                     std::move(particle_poses),
                     std::move(range_data_in_local),
                     std::move(lines_in_local),
                     std::move(insertion_result)});
}

sensor::FeatureData LocalTrajectoryBuilder2D::GetFeatureData(
  const common::Time & time,
  const std::vector<laser_perception::FeatureMsgWithOrigin> & feature_data)
{
  sensor::FeatureData res;
  // std::cout << "feature_data.size(): " << feature_data.size() << std::endl;
  for (size_t i = 0; i < feature_data.size(); i++) {
    // LineData(feature_data[i].)
    transform::Rigid2d origin = transform::Rigid2d::FromVector(feature_data[i].origin);
    for (auto it : feature_data[i].MsgVec) {
      if (it.target_name == "line") {
        Eigen::Vector2d begin = origin * it.line.first.cast<double>().head<2>();
        Eigen::Vector2d end = origin * it.line.second.cast<double>().head<2>();
        res.line_data.push_back(sensor::LineData(begin, end));
      } else if (it.target_name == "reflector") {
        Eigen::Vector2d reflector = origin * Eigen::Vector2d{it.x, it.y};
        res.reflector_data.push_back(
          sensor::LandmarkPointData{time, transform::Rigid3d(), reflector});
      }
    }
  }
  return res;
}

/**
 * @brief 将雷达数据插入到submap中
 * 
 * @param time 时间
 * @param range_data_in_local local系下的激光数据
 * @param filtered_gravity_aligned_point_cloud  重力对齐的激光点云
 * @param pose_estimate 估计的位姿
 * @param gravity_alignment 重力对齐的旋转
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> 插入的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time, const sensor::RangeData& range_data_in_local,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment,
    const std::map<int, std::vector<sensor::LineData>> & line_data_in_local,
    const std::map<int, std::vector<sensor::LineData>> & tracked_lane_data,
    const std::vector<sensor::LandmarkPointData> & reflector_data)
{
  // 1. 判断是否是关键帧
  if (map_insertion_motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }
  sensor::PointCloud cloud_in_local =
            sensor::TransformPointCloud(
                        filtered_gravity_aligned_point_cloud,
                        pose_estimate.cast<float>());

  sensor::PointCloud highlight_cloud_in_local =
            sensor::TransformPointCloud(
                        highlight_point_cloud,
                        pose_estimate.cast<float>());

  // 2. 将激光数据插入到submap中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  active_submaps_.InsertCloudData(cloud_in_local, highlight_cloud_in_local);
  active_submaps_.InsertLineData(line_data_in_local);
  active_submaps_.InsertReflectorData(reflector_data);

  // 3. 返回插入结果
  // TODO: 后面再加上TrajectoryNode中的line和reflector信息
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          filtered_gravity_aligned_point_cloud,
          highlight_point_cloud,
          tracked_lane_data,
          reflector_data,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

/**
 * @brief 向LocalTrajectoryBuilder2D中加入IMU数据，同时初始化位姿插值器
 * 
 * @param imu_data ImuData格式的数据
 */
void LocalTrajectoryBuilder2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
  // 当使用IMU数据的时候，在第一次加入IMU数据时，初始化位姿插值器
  InitializeExtrapolator(imu_data.time);
  // 向位姿插值器中加入IMU数据
}

/**
 * @brief 向LocalTrajectoryBuilder2D中加入odom数据
 * @param imu_data OdometryData格式的数据
 */
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data)
{
  curr_odom_data_ = odometry_data;
  // curr_odom_data_.time = odometry_data.time;
  // double yaw = transform::GetYaw(odometry_data.pose.rotation()) * 1.05;
  // auto rotation = transform::AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(0, 0, yaw));
  // curr_odom_data_.pose = transform::Rigid3d(odometry_data.pose.translation(), rotation);
  data_preprocessor_->AddOdometryData(curr_odom_data_);
}

void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  if (extrapolator_ == nullptr) {
    return;
  }

  // 设定初始位姿为单位阵
  extrapolator_->AddPose(time, local_pose_);
  is_exptrapolator_initialized_ = true;
  std::cout << "init extrapolator at : " << time << std::endl;
}

void LocalTrajectoryBuilder2D::RegisterMetrics(
    metrics::FamilyFactory* family_factory) {
  auto* latency = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_latency",
      "Duration from first incoming point cloud in accumulation to local slam "
      "result");
  kLocalSlamLatencyMetric = latency->Add({});
  auto* real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_real_time_ratio",
      "sensor duration / wall clock duration.");
  kLocalSlamRealTimeRatio = real_time_ratio->Add({});

  auto* cpu_real_time_ratio = family_factory->NewGaugeFamily(
      "mapping_2d_local_trajectory_builder_cpu_real_time_ratio",
      "sensor duration / cpu duration.");
  kLocalSlamCpuRealTimeRatio = cpu_real_time_ratio->Add({});
  auto score_boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_scores", "Local scan matcher scores",
      score_boundaries);
  kRealTimeCorrelativeScanMatcherScoreMetric =
      scores->Add({{"scan_matcher", "real_time_correlative"}});
  auto cost_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 100);
  auto* costs = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_costs", "Local scan matcher costs",
      cost_boundaries);
  kCeresScanMatcherCostMetric = costs->Add({{"scan_matcher", "ceres"}});
  auto distance_boundaries = metrics::Histogram::ScaledPowersOf(2, 0.01, 10);
  auto* residuals = family_factory->NewHistogramFamily(
      "mapping_2d_local_trajectory_builder_residuals",
      "Local scan matcher residuals", distance_boundaries);
  kScanMatcherResidualDistanceMetric =
      residuals->Add({{"component", "distance"}});
  kScanMatcherResidualAngleMetric = residuals->Add({{"component", "angle"}});
}

}  // namespace mapping
}  // namespace cartographer
