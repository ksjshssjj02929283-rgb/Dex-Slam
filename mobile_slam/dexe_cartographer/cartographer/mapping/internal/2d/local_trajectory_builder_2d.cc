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
#include <thread>
#include <future>
#include <vector>

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

  LOG(INFO) << "Creating single data preprocessor for all sensors";
  data_preprocessor_ = absl::make_unique<preprocess::DataPreprocessor>(
      options_.data_preprocessor_options());

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
 * @param highlight_point_cloud 高亮点云
 * @param line_data 线数据
 * @param tracked_lane_data 跟踪的车道线数据
 * @param cov 协方差矩阵
 * @return std::unique_ptr<transform::Rigid2d> 位姿估计结果
 */
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
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
  
  // 构建地图数据结构
  scan_matching::CeresScanMatcher2D::MapData map_data;
  map_data.grid = matching_submap->grid();
  map_data.line_map = matching_submap->line_map_2d_list();
  map_data.cloud_ikdtree = matching_submap->cloud_ikdtree();
  map_data.highlight_cloud_ikdtree = matching_submap->highlight_cloud_ikdtree();
  map_data.tracked_line_data = &tracked_lane_data;
  
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  
  // 使用新的优化接口进行建图匹配
  ceres_scan_matcher_.Match(
      initial_ceres_pose,
      filtered_gravity_aligned_point_cloud,
      highlight_point_cloud,
      map_data,
      pose_observation.get(),
      &cov,  // 现在协方差会被正确计算
      &summary);

  // LOG(INFO) << "scan match: " << *pose_observation;
  
  // 可选：记录匹配质量
  if (summary.IsSolutionUsable()) {
    double pos_uncertainty = std::sqrt(cov(0,0) + cov(1,1));
    double ang_uncertainty = std::sqrt(cov(2,2));
    VLOG(2) << "Scan match uncertainty - pos: " << pos_uncertainty 
            << " m, ang: " << ang_uncertainty << " rad";
  }

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
  // 1. 将激光雷达数据添加到同步器
  if (!laser_synchronizer_.AddLaserData(sensor_id, unsynchronized_data)) {
    LOG(ERROR) << "Failed to add laser data for sensor: " << sensor_id;
    return nullptr;
  }

  // 2. 尝试获取同步后的激光数据
  auto synchronized_data = laser_synchronizer_.GetSynchronizedData();
  if (!synchronized_data.is_valid) {
    // LOG(INFO) << "Laser synchronizer filling buffer, waiting for more data.";
    return nullptr;
  }

  // 3. 处理同步后的激光数据
  return ProcessSynchronizedLaserData(synchronized_data);
}

/**
 * @brief 处理同步后的激光数据（简化版本，使用DataPreprocessor）
 * 
 * @param synchronized_data 同步后的激光数据
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 处理结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::ProcessSynchronizedLaserData(
    const preprocess::LaserSynchronizer::SynchronizedLaserData& synchronized_data) {
  
  const common::Time& sync_time = synchronized_data.sync_time;
  
  // 使用数据预处理器来处理所有传感器数据
  if (!data_preprocessor_) {
    LOG(ERROR) << "Data preprocessor not initialized";
    return nullptr;
  }
  
  // 从里程计缓冲区获取同步时间对应的里程计数据
  auto odom_at_sync_time = data_preprocessor_->GetOdometryAtTime(sync_time);
  if (!odom_at_sync_time.has_value()) {
    LOG(WARNING) << "No odometry data available for sync time: " << sync_time;
    return nullptr;
  }
  
  // 关键帧策略检查
  if (keyframe_motion_filter_.IsSimilar(sync_time, odom_at_sync_time->pose)) {
    return nullptr;
  }
  
  // 使用DataPreprocessor处理多传感器数据
  MergedData merged_data = data_preprocessor_->ProcessMultipleSensorData(
      synchronized_data.laser_data_map, sync_time);
  
  if (merged_data.IsEmpty()) {
    LOG(WARNING) << "Merged data from DataPreprocessor is empty";
    return nullptr;
  }
  
  // 估计重力对齐（如果使用IMU）
  transform::Rigid3d gravity_alignment = transform::Rigid3d::Identity();
  if (options_.use_imu_data() && extrapolator_ && is_exptrapolator_initialized_) {
    const Eigen::Quaterniond gravity_orientation = 
        extrapolator_->EstimateGravityOrientation(sync_time);
    // 在2D SLAM中，我们只需要补偿roll和pitch，不改变yaw
    const Eigen::Vector3d rpy = transform::RotationQuaternionToAngleAxisVector(gravity_orientation);
    const double roll = rpy.x();
    const double pitch = rpy.y();
    // yaw保持为0，只使用roll和pitch构建重力对齐
    const Eigen::Quaterniond gravity_alignment_2d = 
        transform::RollPitchYaw(roll, pitch, 0.);
    gravity_alignment = transform::Rigid3d::Rotation(gravity_alignment_2d);
  }
  
  // 更新传感器时间信息
  const common::Time current_sensor_time = sync_time;
  absl::optional<common::Duration> sensor_duration;
  if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
  }
  last_sensor_time_ = current_sensor_time;
  // 使用融合后的数据进行位姿估计和地图更新
  return ProcessDataAndEstimatePose(
      "merged_sensors",  // 使用一个标识来表示这是合并的传感器数据
      current_sensor_time,
      merged_data,
      gravity_alignment, 
      sensor_duration,
      data_preprocessor_.get());
}


/**
 * @brief 处理融合数据并进行位姿估计和地图更新
 * 
 * @param sensor_id 传感器ID标识符
 * @param time 时间戳
 * @param merged_data 融合后的传感器数据
 * @param gravity_alignment 重力对齐变换
 * @param sensor_duration 传感器持续时间
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult> 
 */
std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::ProcessDataAndEstimatePose(
    const std::string& sensor_id,
    const common::Time& time,
    const MergedData& merged_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration,
    preprocess::DataPreprocessor* data_preprocessor) {
  
  if (merged_data.merged_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty merged range data.";
    return nullptr;
  }

  // 根据激光雷达时间戳获取delta pose（关键帧之间的位姿增量）
  const transform::Rigid3d odom_delta_pose = GetOdomDeltaPose(time);
  const transform::Rigid2d pose_prediction = transform::Project2D(
    local_pose_ * odom_delta_pose);

  const sensor::PointCloud& filtered_gravity_aligned_point_cloud = merged_data.merged_filtered_cloud;
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // ScanMatch计算当前帧位姿：对齐重力坐标系在local坐标系中的位姿
  Eigen::Matrix3d loc_cov;
  std::map<int, std::vector<sensor::LineData>> tracked_lane_data;
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction,
                filtered_gravity_aligned_point_cloud,
                merged_data.merged_highlight_cloud,
                merged_data.merged_line_data,
                tracked_lane_data,
                loc_cov);
  
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }
  
  // pose_estimate_2d 是在 gravity-aligned frame 中的位姿
  // 需要转换回 tracking frame (local frame)
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment.inverse();
  local_pose_ = pose_estimate;
  
  // 更新位姿插值器
  if (extrapolator_) {
    extrapolator_->AddPose(time, pose_estimate);
  }
  
  // 位姿估计成功后，更新用于计算delta pose的基准里程计数据
  UpdateLastOdomForDelta(time);

  std::unique_ptr<InsertionResult>
    insertion_result = InsertIntoSubmap(
      time,
      merged_data,
      tracked_lane_data,
      pose_estimate,
      gravity_alignment.rotation());

  // 计算local slam的频率
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
  
  // 为返回值准备转换后的数据
  sensor::RangeData range_data_in_local =
      TransformRangeData(merged_data.merged_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));

  // 将线数据转换到local坐标系
  std::map<int, std::vector<sensor::LineData>> lines_in_local;
  for (const auto& data_vec: tracked_lane_data) {
    for (const auto& line: data_vec.second) {
      lines_in_local[data_vec.first].push_back(sensor::TransformLineData(*pose_estimate_2d, line));
    }
  }

  // 返回匹配结果
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
 * @param merged_data 融合后的传感器数据
 * @param pose_estimate 估计的位姿
 * @param gravity_alignment 重力对齐的旋转
 * @param tracked_lane_data 跟踪到的线特征
 * @return std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult> 插入的结果
 */
std::unique_ptr<LocalTrajectoryBuilder2D::InsertionResult>
LocalTrajectoryBuilder2D::InsertIntoSubmap(
    const common::Time time,
    const MergedData& merged_data,
    const std::map<int, std::vector<sensor::LineData>>& tracked_lane_data,
    const transform::Rigid3d& pose_estimate,
    const Eigen::Quaterniond& gravity_alignment)
{
  // 1. 判断是否是关键帧
  if (map_insertion_motion_filter_.IsSimilar(time, pose_estimate)) {
    return nullptr;
  }

  // 2. 将原始数据转换到local坐标系
  const transform::Rigid2d pose_estimate_2d = transform::Project2D(pose_estimate);

  // 转换点云到local坐标系
  sensor::PointCloud cloud_in_local =
            sensor::TransformPointCloud(
                        merged_data.merged_less_flat_cloud,
                        pose_estimate.cast<float>());

  sensor::PointCloud highlight_cloud_in_local =
            sensor::TransformPointCloud(
                        merged_data.merged_highlight_cloud,
                        pose_estimate.cast<float>());

  // 将线数据转换到local坐标系
  std::map<int, std::vector<sensor::LineData>> tracked_lines_in_local;
  for (const auto& data_vec: tracked_lane_data) {
    for (const auto& line: data_vec.second) {
      tracked_lines_in_local[data_vec.first].push_back(sensor::TransformLineData(pose_estimate_2d, line));
    }
  }

  // 3. 收集所有需要插入的range data到vector中
  std::vector<sensor::RangeData> all_range_data_to_insert;
  
  // 添加每个传感器的原始去畸变range data
  for (const auto& original_range_data : merged_data.original_undistorted_range_datas) {
    // 将原始去畸变数据转换到local坐标系
    sensor::RangeData sensor_range_data_in_local = sensor::TransformRangeData(
        original_range_data, pose_estimate.cast<float>());
    all_range_data_to_insert.push_back(sensor_range_data_in_local);
  }
  
  // 一次性批量插入所有range data
  std::vector<std::shared_ptr<const Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(all_range_data_to_insert);
  // 插入合并后的云数据、线数据和反射器数据
  active_submaps_.InsertCloudData(cloud_in_local, highlight_cloud_in_local);
  active_submaps_.InsertLineData(tracked_lines_in_local);
  active_submaps_.InsertReflectorData(merged_data.merged_reflector_data);

  // 3. 准备TrajectoryNode::Data所需的数据
  
  // 直接使用merged_data中的数据
  sensor::PointCloud merged_filtered_cloud = merged_data.merged_filtered_cloud;
  sensor::PointCloud merged_highlight_cloud = merged_data.merged_highlight_cloud;
  
  // 直接使用merged_data中的原始去畸变range data
  std::vector<sensor::RangeData> all_sensors_range_datas = merged_data.original_undistorted_range_datas;

  // 4. 返回插入结果
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const TrajectoryNode::Data>(TrajectoryNode::Data{
          time,
          gravity_alignment,
          merged_filtered_cloud,  // 合并后的滤波点云
          merged_highlight_cloud, // 合并后的highlight点云
          all_sensors_range_datas, // 各个传感器的原始去畸变range data
          tracked_lane_data,
          merged_data.merged_reflector_data,
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
  if (extrapolator_) {
    extrapolator_->AddImuData(imu_data);
  }
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
  
  // 将里程计数据添加到数据预处理器
  if (data_preprocessor_) {
    data_preprocessor_->AddOdometryData(curr_odom_data_);
  }
  
  // 将里程计数据添加到位姿插值器
  if (extrapolator_) {
    extrapolator_->AddOdometryData(curr_odom_data_);
  }
}

void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过，直接返回
  if (extrapolator_ == nullptr) {
    return;
  }
  
  // 如果已经初始化过，不要重复初始化
  if (is_exptrapolator_initialized_) {
    return;
  }

  // 设定初始位姿为单位阵
  extrapolator_->AddPose(time, local_pose_);
  is_exptrapolator_initialized_ = true;
  LOG(INFO) << "Extrapolator initialized at time: " << time;
}

/**
 * @brief 获取数据预处理器
 * 
 * @return preprocess::DataPreprocessor* 数据预处理器指针，如果不存在则返回nullptr
 */
preprocess::DataPreprocessor* LocalTrajectoryBuilder2D::GetDataPreprocessor() const {
  return data_preprocessor_.get();
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
 * @brief 获取指定时间的里程计数据
 * 
 * @param time 时间戳
 * @return absl::optional<sensor::OdometryData> 里程计数据，如果不存在则返回nullopt
 */
absl::optional<sensor::OdometryData> LocalTrajectoryBuilder2D::GetOdometryAtTime(const common::Time& time) const {
  assert(data_preprocessor_ != nullptr);
  // 使用数据预处理器来获取里程计数据
  if (data_preprocessor_) {
    return data_preprocessor_->GetOdometryAtTime(time);
  }
  
  return absl::nullopt;
}

/**
 * @brief 更新用于计算delta pose的基准里程计数据
 * 
 * @param time 时间戳
 */
void LocalTrajectoryBuilder2D::UpdateLastOdomForDelta(const common::Time& time) {
  auto current_odom = GetOdometryAtTime(time);
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
