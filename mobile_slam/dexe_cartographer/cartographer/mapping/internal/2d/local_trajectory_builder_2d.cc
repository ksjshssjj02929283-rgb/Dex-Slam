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
      laser_synchronizer_(expected_range_sensor_ids),
      expected_range_sensor_ids_(expected_range_sensor_ids)
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

  // 根据expected_range_sensor_ids预先创建所有数据预处理器
  for (const std::string& sensor_id : expected_range_sensor_ids) {
    LOG(INFO) << "Pre-creating data preprocessor for expected sensor: " << sensor_id;
    data_preprocessors_[sensor_id] = absl::make_unique<preprocess::DataPreprocessor>(
        options_.data_preprocessor_options());
  }

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
 * @brief 向LocalTrajectoryBuilder2D中加入激光雷达数据，使用激光同步器进行同步
 * 
 * @param sensor_id 
 * @param unsynchronized_data base系下的点云
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
  
  std::cout << "sensor_id: " << sensor_id << std::endl;
  
  // 1. 将激光雷达数据添加到同步器
  if (!laser_synchronizer_.AddLaserData(sensor_id, unsynchronized_data)) {
    LOG(ERROR) << "Failed to add laser data for sensor: " << sensor_id;
    return nullptr;
  }

  // 2. 尝试获取同步后的激光数据
  auto synchronized_data = laser_synchronizer_.GetSynchronizedData();
  if (!synchronized_data.is_valid) {
    LOG(INFO) << "Laser synchronizer filling buffer, waiting for more data.";
    return nullptr;
  }

  // 3. 处理同步后的激光数据
  return ProcessSynchronizedLaserData(synchronized_data);
}

/**
 * @brief 对积累的激光数据进行位姿估计
 * 
 * @param sensor_id 传感器ID
 * @param time 
 * @param gravity_aligned_range_data 
 * @param gravity_alignment 
 * @param sensor_duration 
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(
    const std::string& sensor_id,
    const common::Time time,
    const sensor::RangeData& gravity_aligned_range_data,
    const std::vector<sensor::LineData> & line_data,
    const std::vector<sensor::LandmarkPointData> & reflector_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration,
    const std::vector<SensorRawData>& sensor_raw_data_list) {
  if (gravity_aligned_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty horizontal range data.";
    return nullptr;
  }
  std::cout << "sensor_id: " << sensor_id << std::endl;
  // 根据激光雷达时间戳从里程计缓冲区获取delta pose
  const transform::Rigid3d odom_delta_pose = GetOdomDeltaPose(time);

  const transform::Rigid2d pose_prediction = transform::Project2D(
    local_pose_ * odom_delta_pose);
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

  // 位姿估计成功后，更新里程计基准位姿
  UpdateLastOdomForDelta(time);

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

  // 将传感器原始数据转换到local坐标系
  std::vector<SensorRawData> sensor_raw_data_in_local = sensor_raw_data_list;
  for (auto& raw_data : sensor_raw_data_in_local) {
    raw_data.range_data_in_local = TransformRangeData(raw_data.range_data_in_local,
                                                      transform::Embed3D(pose_estimate_2d->cast<float>()));
  }

  std::unique_ptr<InsertionResult>
    insertion_result = InsertIntoSubmap(
      time, range_data_in_local,
      *laser_perception_->getLessFlatCloud(),
      *laser_perception_->getHighlightCloud(),
      pose_estimate, gravity_alignment.rotation(),
      lines_in_local, tracked_lane_data, reflector_data, sensor_raw_data_in_local);

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

/**
 * @brief 处理同步后的激光数据
 * 
 * @param synchronized_data 同步后的激光数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 处理结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::ProcessSynchronizedLaserData(
    const LaserSynchronizer::SynchronizedLaserData& synchronized_data) {
  
  const common::Time& sync_time = synchronized_data.sync_time;
  std::cout << "Processing synchronized data at time: " << sync_time << std::endl;
  
  // 从里程计缓冲区获取同步时间对应的里程计数据
  auto odom_at_sync_time = GetOdometryAtTime(sync_time);
  if (!odom_at_sync_time.has_value()) {
    LOG(WARNING) << "No odometry data available for sync time: " << sync_time;
    return nullptr;
  }
  
  // 关键帧策略检查 - 移到同步数据处理之前
  if (keyframe_motion_filter_.IsSimilar(sync_time, odom_at_sync_time->pose)) {
    std::cout << "Skipping frame due to keyframe motion filter at time: " << sync_time << std::endl;
    return nullptr;
  }
  
  // 存储处理后的传感器数据
  std::vector<ProcessedSensorData> processed_sensors;
  
  // 对每个雷达数据进行单独的预处理和特征检测
  for (const auto& sensor_pair : synchronized_data.laser_data_map) {
    const std::string& sensor_id = sensor_pair.first;
    const sensor::TimedPointCloudData& laser_data = sensor_pair.second;
    
    std::cout << "Processing sensor: " << sensor_id << std::endl;
    
    // 处理单个传感器数据
    auto processed_data = ProcessSingleSensorData(sensor_id, laser_data, sync_time);
    if (processed_data.has_value()) {
      processed_sensors.push_back(processed_data.value());
    }
  }
  
  if (processed_sensors.empty()) {
    LOG(WARNING) << "No valid sensor data processed";
    return nullptr;
  }
  
  // 合并所有传感器的点云数据
  sensor::RangeData merged_range_data;
  sensor::PointCloud merged_filtered_cloud;
  sensor::PointCloud merged_highlight_cloud;
  std::vector<sensor::LineData> merged_line_data;
  std::vector<sensor::LandmarkPointData> merged_reflector_data;
  std::vector<SensorRawData> sensor_raw_data_list;
  
  // 初始化合并的range data origin为第一个传感器的origin
  if (!processed_sensors.empty()) {
    merged_range_data.origin = processed_sensors[0].undistorted_range_data.origin;
  }
  
  for (const auto& sensor_data : processed_sensors) {
    // 合并点云数据
    const auto& range_data = sensor_data.undistorted_range_data;
    
    // 合并returns点云
    for (const auto& point : range_data.returns) {
      merged_range_data.returns.push_back(point);
    }
    
    // 合并misses点云
    for (const auto& point : range_data.misses) {
      merged_range_data.misses.push_back(point);
    }
    
    // 合并滤波后的点云
    for (const auto& point : sensor_data.filtered_gravity_aligned_point_cloud) {
      merged_filtered_cloud.push_back(point);
    }
    
    // 合并highlight点云
    for (const auto& point : sensor_data.highlight_point_cloud) {
      merged_highlight_cloud.push_back(point);
    }
    
    // 合并特征数据
    merged_line_data.insert(merged_line_data.end(),
                           sensor_data.feature_data.line_data.begin(),
                           sensor_data.feature_data.line_data.end());
    merged_reflector_data.insert(merged_reflector_data.end(),
                                sensor_data.feature_data.reflector_data.begin(),
                                sensor_data.feature_data.reflector_data.end());
    
    // 保存每个传感器的原始数据，用于后续插入
    SensorRawData raw_data;
    raw_data.sensor_id = sensor_data.sensor_id;
    raw_data.range_data_in_local = range_data;  // 这里暂时用undistorted数据，后面会在local坐标系中更新
    raw_data.filtered_gravity_aligned_point_cloud = sensor_data.filtered_gravity_aligned_point_cloud;
    raw_data.highlight_point_cloud = sensor_data.highlight_point_cloud;
    sensor_raw_data_list.push_back(raw_data);
  }
  
  std::cout << "Merged data - points: " << merged_range_data.returns.size()
            << ", lines: " << merged_line_data.size()
            << ", reflectors: " << merged_reflector_data.size() << std::endl;
  
  // 更新传感器时间信息
  const common::Time current_sensor_time = sync_time;
  absl::optional<common::Duration> sensor_duration;
  if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
  }
  last_sensor_time_ = current_sensor_time;
  
  // 使用合并后的数据调用 AddAccumulatedRangeData
  return AddAccumulatedRangeData(
      "merged_sensors",  // 使用一个标识来表示这是合并的传感器数据
      current_sensor_time,
      FilterRangeData(merged_range_data),
      merged_line_data,
      merged_reflector_data,
      transform::Rigid3d::Identity(), 
      sensor_duration,
      sensor_raw_data_list);
}

/**
 * @brief 处理单个传感器的激光数据
 * 
 * @param sensor_id 传感器ID
 * @param laser_data 激光数据
 * @param sync_time 同步时间
 * @return absl::optional<ProcessedSensorData> 处理后的数据结构
 */
absl::optional<LocalTrajectoryBuilder2D::ProcessedSensorData>
LocalTrajectoryBuilder2D::ProcessSingleSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& laser_data,
    const common::Time& sync_time) {
  
  // 获取对应的数据预处理器
  auto* data_preprocessor = GetDataPreprocessor(sensor_id);
  if (!data_preprocessor) {
    LOG(ERROR) << "Cannot process data for unknown sensor: " << sensor_id;
    return absl::nullopt;
  }
  
  // 将时间点云数据转换为同步格式
  sensor::TimedPointCloudOriginData sync_laser_data;
  sync_laser_data.time = laser_data.time;
  sync_laser_data.origins.push_back(laser_data.origin);
  sync_laser_data.sensor_to_trackings.push_back(laser_data.sensor_to_tracking);
  
  // 转换点云数据格式
  for (const auto& point : laser_data.ranges) {
    sensor::TimedPointCloudOriginData::RangeMeasurement range_measurement;
    range_measurement.point_time = point;
    range_measurement.intensity = 0.0f;  // 默认强度
    range_measurement.origin_index = 0;
    sync_laser_data.ranges.push_back(range_measurement);
  }
  
  // 数据预处理（去畸变）
  data_preprocessor->AddSyncedPointCloud(sync_laser_data);
  auto undistort_range = data_preprocessor->GetProcessedRangeData();
  if (!undistort_range.has_value()) {
    LOG(WARNING) << "Failed to get processed range data for sensor: " << sensor_id;
    return absl::nullopt;
  }
  
  // 特征检测
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
  
  std::cout << "Sensor " << sensor_id 
            << " undistorted points: " << undistort_range->data.returns.points().size() << std::endl;
  
  // 单独进行特征检测
  std::vector<laser_perception::FeatureMsgWithOrigin> feature_data_origin = 
      laser_perception_->laserUpdate(sync_point_cloud);
  auto feature_data = GetFeatureData(sync_time, feature_data_origin);
  
  std::cout << "Sensor " << sensor_id 
            << " - lines: " << feature_data.line_data.size()
            << ", reflectors: " << feature_data.reflector_data.size() << std::endl;
  
  // 对点云进行滤波
  const sensor::PointCloud& filtered_gravity_aligned_point_cloud =
      sensor::AdaptiveVoxelFilter(undistort_range->data.returns,
                                  options_.adaptive_voxel_filter_options());
  
  // 构造返回的处理数据结构
  ProcessedSensorData processed_data;
  processed_data.sensor_id = sensor_id;
  processed_data.undistorted_range_data = undistort_range->data;
  processed_data.feature_data = feature_data;
  processed_data.filtered_gravity_aligned_point_cloud = filtered_gravity_aligned_point_cloud;
  processed_data.highlight_point_cloud = *laser_perception_->getHighlightCloud();
  
  return processed_data;
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
    const std::vector<sensor::LandmarkPointData> & reflector_data,
    const std::vector<SensorRawData>& sensor_raw_data_list)
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

  // 2. 将合并后的激光数据插入到submap中
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(range_data_in_local);

  active_submaps_.InsertCloudData(cloud_in_local, highlight_cloud_in_local);
  active_submaps_.InsertLineData(line_data_in_local);
  active_submaps_.InsertReflectorData(reflector_data);

  // 3. 分别插入每个传感器的原始数据到栅格地图
  for (const auto& sensor_raw_data : sensor_raw_data_list) {
    std::cout << "Inserting raw data for sensor: " << sensor_raw_data.sensor_id << std::endl;
    
    // 为每个传感器的原始数据单独插入到栅格地图
    active_submaps_.InsertRangeData(sensor_raw_data.range_data_in_local);
    
    // 转换传感器点云到local坐标系并插入
    sensor::PointCloud sensor_cloud_in_local =
        sensor::TransformPointCloud(
            sensor_raw_data.filtered_gravity_aligned_point_cloud,
            pose_estimate.cast<float>());
    
    sensor::PointCloud sensor_highlight_cloud_in_local =
        sensor::TransformPointCloud(
            sensor_raw_data.highlight_point_cloud,
            pose_estimate.cast<float>());
    
    active_submaps_.InsertCloudData(sensor_cloud_in_local, sensor_highlight_cloud_in_local);
  }

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
 * @param odometry_data OdometryData格式的数据
 */
void LocalTrajectoryBuilder2D::AddOdometryData(
    const sensor::OdometryData& odometry_data)
{
  curr_odom_data_ = odometry_data;
  // curr_odom_data_.time = odometry_data.time;
  // double yaw = transform::GetYaw(odometry_data.pose.rotation()) * 1.05;
  // auto rotation = transform::AngleAxisVectorToRotationQuaternion(Eigen::Vector3d(0, 0, yaw));
  // curr_odom_data_.pose = transform::Rigid3d(odometry_data.pose.translation(), rotation);
  
  // 将里程计数据添加到缓冲区
  AddOdometryToBuffer(curr_odom_data_);
  
  // 将里程计数据添加到所有已创建的数据预处理器中
  for (auto& preprocessor_pair : data_preprocessors_) {
    preprocessor_pair.second->AddOdometryData(curr_odom_data_);
  }
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

/**
 * @brief 获取指定sensor_id的数据预处理器
 * 
 * @param sensor_id 传感器ID
 * @return preprocess::DataPreprocessor* 数据预处理器指针，如果不存在则返回nullptr
 */
preprocess::DataPreprocessor* LocalTrajectoryBuilder2D::GetDataPreprocessor(const std::string& sensor_id) {
  auto it = data_preprocessors_.find(sensor_id);
  if (it == data_preprocessors_.end()) {
    LOG(ERROR) << "No data preprocessor found for sensor: " << sensor_id 
               << ". Expected sensors are: ";
    for (const auto& expected_id : expected_range_sensor_ids_) {
      LOG(ERROR) << "  - " << expected_id;
    }
    return nullptr;
  }
  return it->second.get();
}

/**
 * @brief 将里程计数据添加到缓冲区
 * 
 * @param odometry_data 里程计数据
 */
void LocalTrajectoryBuilder2D::AddOdometryToBuffer(const sensor::OdometryData& odometry_data) {
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  
  // 添加新数据到缓冲区
  odom_buffer_.push_back(odometry_data);
  
  // 维护缓冲区大小限制
  while (odom_buffer_.size() > kMaxOdomBufferSize) {
    odom_buffer_.pop_front();
  }
  
  // 维护缓冲区时间限制
  const common::Time current_time = odometry_data.time;
  while (!odom_buffer_.empty() && 
         common::ToSeconds(current_time - odom_buffer_.front().time) > kMaxOdomBufferDuration) {
    odom_buffer_.pop_front();
  }
}

/**
 * @brief 根据时间获取里程计数据（线性插值）
 * 
 * @param time 查询时间
 * @return absl::optional<sensor::OdometryData> 插值后的里程计数据
 */
absl::optional<sensor::OdometryData> LocalTrajectoryBuilder2D::GetOdometryAtTime(const common::Time& time) const {
  std::lock_guard<std::mutex> lock(odom_buffer_mutex_);
  
  if (odom_buffer_.empty()) {
    return absl::nullopt;
  }
  
  // 如果查询时间早于最早的数据，返回最早的数据
  if (time <= odom_buffer_.front().time) {
    return odom_buffer_.front();
  }
  
  // 如果查询时间晚于最新的数据，返回最新的数据
  if (time >= odom_buffer_.back().time) {
    return odom_buffer_.back();
  }
  
  // 寻找时间区间进行插值
  for (size_t i = 0; i < odom_buffer_.size() - 1; ++i) {
    const auto& odom1 = odom_buffer_[i];
    const auto& odom2 = odom_buffer_[i + 1];
    
    if (time >= odom1.time && time <= odom2.time) {
      // 计算插值比例
      const double duration_total = common::ToSeconds(odom2.time - odom1.time);
      if (duration_total < 1e-9) {
        return odom1; // 时间差太小，直接返回第一个
      }
      
      const double duration_to_query = common::ToSeconds(time - odom1.time);
      const double ratio = duration_to_query / duration_total;
      
      // 位置插值
      const Eigen::Vector3d pos1 = odom1.pose.translation();
      const Eigen::Vector3d pos2 = odom2.pose.translation();
      const Eigen::Vector3d interpolated_pos = pos1 + ratio * (pos2 - pos1);
      
      // 旋转插值（球面线性插值）
      const Eigen::Quaterniond rot1 = odom1.pose.rotation();
      const Eigen::Quaterniond rot2 = odom2.pose.rotation();
      const Eigen::Quaterniond interpolated_rot = rot1.slerp(ratio, rot2);
      
      sensor::OdometryData interpolated_odom;
      interpolated_odom.time = time;
      interpolated_odom.pose = transform::Rigid3d(interpolated_pos, interpolated_rot);
      
      return interpolated_odom;
    }
  }
  
  return absl::nullopt;
}

/**
 * @brief 计算相对于上次位姿的delta pose
 * 
 * @param laser_time 激光雷达时间戳
 * @return transform::Rigid3d delta pose
 */
transform::Rigid3d LocalTrajectoryBuilder2D::GetOdomDeltaPose(const common::Time& laser_time) {
  auto current_odom = GetOdometryAtTime(laser_time);
  if (!current_odom.has_value()) {
    LOG(WARNING) << "No odometry data available for laser time: " << laser_time;
    return transform::Rigid3d::Identity();
  }
  
  // 如果这是第一次调用，保存当前位姿作为基准
  if (!last_odom_for_delta_.has_value()) {
    last_odom_for_delta_ = current_odom.value();
    return transform::Rigid3d::Identity();
  }
  
  // 计算delta pose
  const transform::Rigid3d delta_pose = 
      last_odom_for_delta_->pose.inverse() * current_odom->pose;
  
  return delta_pose;
}

/**
 * @brief 更新用于计算delta pose的基准里程计数据
 * 
 * @param laser_time 激光雷达时间戳
 */
void LocalTrajectoryBuilder2D::UpdateLastOdomForDelta(const common::Time& laser_time) {
  auto current_odom = GetOdometryAtTime(laser_time);
  if (current_odom.has_value()) {
    last_odom_for_delta_ = current_odom.value();
  }
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
