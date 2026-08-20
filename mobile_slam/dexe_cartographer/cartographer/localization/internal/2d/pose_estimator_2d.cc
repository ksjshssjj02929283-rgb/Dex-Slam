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

#include "cartographer/localization/internal/2d/pose_estimator_2d.h"

#include <limits>
#include <memory>
#include <cmath>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/common/tic_toc.h"

namespace cartographer {
namespace localization {

static auto* kLocalSlamLatencyMetric = metrics::Gauge::Null();
static auto* kLocalSlamRealTimeRatio = metrics::Gauge::Null();
static auto* kLocalSlamCpuRealTimeRatio = metrics::Gauge::Null();
static auto* kRealTimeCorrelativeScanMatcherScoreMetric =
    metrics::Histogram::Null();
static auto* kCeresScanMatcherCostMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualDistanceMetric = metrics::Histogram::Null();
static auto* kScanMatcherResidualAngleMetric = metrics::Histogram::Null();

PoseEstimator2D::PoseEstimator2D(
    const proto::PoseEstimatorOptions2D& options,
    const std::vector<std::string>& expected_range_sensor_ids)
    : options_(options),
      active_submaps_(options.submaps_options()),
      keyframe_motion_filter_(options_.keyframe_motion_filter_options()),
      map_insertion_motion_filter_(options_.map_insertion_motion_filter_options()),
      real_time_correlative_scan_matcher_(
          options_.real_time_correlative_scan_matcher_options()),
      ceres_scan_matcher_(options_.ceres_scan_matcher_options()),
      range_data_collator_(expected_range_sensor_ids),
      laser_synchronizer_(expected_range_sensor_ids),
      expected_range_sensor_ids_(expected_range_sensor_ids)
{
  LOG(INFO) << "Creating single data preprocessor for all sensors";
  data_preprocessor_ = absl::make_unique<preprocess::DataPreprocessor>(
      options_.data_preprocessor_options());

  laser_perception_ =
      std::make_unique<laser_perception::LaserPerception>(
          options_.laser_perception_options());

  line_matcher_ =
      std::make_unique<mapping::scan_matching::LineMatcher2D>(
            options_.line_matcher_options());

  local_map_updater_ =
      std::make_unique<LocalMapUpdater2D>(options_.local_map_updater_options(),
                                          options_.submaps_options());

  // 初始化一致性检查器
  consistency_checker_ = std::make_unique<ConsistencyChecker>(options_);
  
  // 设置 scan matchers 到 consistency checker
  consistency_checker_->SetRealTimeCorrelativeScanMatcher(&real_time_correlative_scan_matcher_);
  consistency_checker_->SetCeresScanMatcher(&ceres_scan_matcher_);

  loc_state_ = LocalizationState::NOT_INITIALIZED;

  // std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  // time_file_.open(home_dir + "/.dexe/data/time.txt", std::ios::out);
}

PoseEstimator2D::~PoseEstimator2D() {
  // 1. 停止 pose_initializer（如果有后台线程）
  if (pose_initializer_) {
    pose_initializer_->stop();
    pose_initializer_.reset();
  }
  
  // 2. 清空 active_submaps，确保不再有新的插入操作
  active_submaps_.clear();
  
  // 3. 释放地图数据（shared_ptr 会处理引用计数）
  global_static_submap_.reset();
  
  // 4. 释放其他资源
  local_map_updater_.reset();
  data_preprocessor_.reset();
  laser_perception_.reset();
  line_matcher_.reset();
}

void PoseEstimator2D::InitGlobalMapsFromSubmap(std::shared_ptr<mapping::Submap2D> global_submap)
{
  if (!global_submap) {
    LOG(ERROR) << "Cannot initialize global maps from null submap";
    return;
  }
  global_static_submap_ = global_submap;

  // 初始化 local_map_updater_ 的全局地图引用
  CHECK(global_static_submap_->grid());
  CHECK(global_static_submap_->cloud());
  CHECK(global_static_submap_->highlight_cloud());
  CHECK(global_static_submap_->line_map_2d_list());

  // 直接将 global_static_submap_ 传递给 local_map_updater_
  local_map_updater_->SetGlobalSubmap(global_static_submap_);

  LOG(INFO) << "All global maps initialized successfully from global submap";

  // 地图初始化完成，状态设置为INITIALIZED
  loc_state_ = LocalizationState::INITIALIZED;

  // std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  // std::string folder = home_dir + "/.dexe/data/submaps/";
  // global_grid_map_->DrawAsImage(folder, "pose_estimator.png");
}

bool PoseEstimator2D::SetInitialPose(const transform::Rigid3d &initial_pose)
{
  if (!initial_pose.IsValid()) {
    LOG(ERROR) << "Invalid initial pose: " << initial_pose;
    return false;
  }
  
  // 初始化pose_initializer_
  if (!pose_initializer_) {
    pose_initializer_ = std::make_unique<PoseInitializer>(
                                  options_.pose_initializer_options());
  }

  local_map_updater_->UpdateRobotPose(transform::Project2D(initial_pose));
  if (!local_map_updater_->IsLocalMapUpdated()) {
    LOG(WARNING) << "There is no need to update local map for initial pose setting!";
  }

  auto local_submap = local_map_updater_->local_submap();
  if (!local_submap) {
    LOG(ERROR) << "Local submap is not available!";
    return false;
  }

  pose_initializer_->InitGlobalSubmap(local_submap);

  pose_initializer_->SetInitialPose(transform::Project2D(initial_pose));
  pose_initializer_->start();
  loc_state_ = LocalizationState::RELOCALIZING;
  
  // 重置全局重定位尝试标志，因为现在有了新的初始位姿
  has_tried_global_relocalization_ = false;

  return true;
}

void PoseEstimator2D::TriggerGlobalRelocalization()
{
  if (!global_static_submap_ || !global_static_submap_->grid()) {
    LOG(ERROR) << "Global static submap is not initialized, cannot trigger global relocalization.";
    return;
  }
  
  // 初始化pose_initializer_（如果还没有）
  if (!pose_initializer_) {
    pose_initializer_ = std::make_unique<PoseInitializer>(
                                  options_.pose_initializer_options());
  }

  // 使用 global_static_submap_ 初始化 pose_initializer_
  pose_initializer_->InitGlobalSubmap(global_static_submap_);

  // 触发全局重定位
  pose_initializer_->TriggerGlobalRelocalization();
  pose_initializer_->start();
  loc_state_ = LocalizationState::RELOCALIZING;

  LOG(INFO) << "Triggered global relocalization without initial pose";
}

sensor::RangeData
PoseEstimator2D::TransformToGravityAlignedFrameAndFilter(
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
PoseEstimator2D::FilterRangeData(
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
std::unique_ptr<transform::Rigid2d> PoseEstimator2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const std::vector<sensor::LineData> & line_data,
    std::map<int, std::vector<sensor::LineData>> & tracked_lane_data)
{
  // 1. 如果active_submaps_中没有submap，则直接返回pose_prediction
  //    一般情况下，第一帧激光数据进来时，active_submaps_中没有submap
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 2. 读取active_submaps_中的第一个submap，作为匹配的地图
  std::shared_ptr<const mapping::Submap2D> matching_submap =
      active_submaps_.submaps().front();
  
  // 3. 安全性检查：确保 submap 的关键成员已初始化
  if (!matching_submap || !matching_submap->grid() || 
      !matching_submap->cloud_ikdtree() ||
      !matching_submap->line_map_2d_list()) {
    LOG(WARNING) << "Matching submap has null components, using prediction";
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.

  // LOG(INFO) << "Initial prediction: " << pose_prediction;

  // 3. 进行correlative scan matching以获得更好的初始位姿估计
  transform::Rigid2d optimized_pose_prediction = pose_prediction;
  bool correlative_success = false;
  
  if (options_.use_online_correlative_scan_matching()) {
    common::TicToc tic_correlative;
    transform::Rigid2d correlative_pose = pose_prediction;
    const double score = real_time_correlative_scan_matcher_.Match(
        pose_prediction, filtered_gravity_aligned_point_cloud,
        *matching_submap->grid(), &correlative_pose);
    
    kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
    
    // 根据匹配分数决定是否使用correlative结果
    if (score > 0.4 && correlative_pose.IsValid()) {
      // 使用correlative的旋转，但保持预测的位置（更保守的策略）
      optimized_pose_prediction = transform::Rigid2d(
          pose_prediction.translation(), 
          correlative_pose.rotation());
      correlative_success = true;
      // LOG(INFO) << "Correlative matching successful, score: " << score 
      //           << ", optimized pose: " << optimized_pose_prediction;
    } else {
      LOG(WARNING) << "Correlative matching failed or low score: " << score 
                   << ", using original prediction";
    }
    double correlative_time = tic_correlative.toc();
    // LOG(INFO) << "Correlative scan matcher cost: " << correlative_time << " ms";
  }
  
  // 最终用于Ceres优化的初始位姿
  transform::Rigid2d initial_ceres_pose = optimized_pose_prediction;

  // LOG(INFO) << "csm: " << initial_ceres_pose;
  std::map<int, std::vector<sensor::LineData>> global_tracked_lane_data;
  if (active_submaps_.submaps().size() != 0) {
    line_matcher_->TrackLineFromMap(initial_ceres_pose,
                                    line_data,
                                    *active_submaps_.line_map_2d_list(),
                                    tracked_lane_data);
    // TODO: 这里使用的线特征是否需要会使用local dynamic map中的线特征
    line_matcher_->TrackLineFromMap(initial_ceres_pose,
                                    line_data,
                                    // *global_static_submap_->line_map_2d_list(),
                                    *local_map_updater_->GetGlobalSubmap()->line_map_2d_list(),
                                    global_tracked_lane_data);
                                    
  }

  // 4. 使用ceres进行位姿估计，得到pose_observation
  //    使用点到地图的误差作为cost function
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;

  auto local_submap = local_map_updater_->local_submap();
  if (!local_submap) {
    LOG(ERROR) << "Local submap is not available!";
    return nullptr;
  }

  common::TicToc tic_toc;

  Eigen::Matrix3d cov;
  ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
                          filtered_gravity_aligned_point_cloud,
                          highlight_point_cloud,
                          tracked_lane_data,
                          global_tracked_lane_data,
                          *matching_submap->grid(), *local_submap->grid(),
                          *matching_submap->line_map_2d_list(),
                          *local_submap->line_map_2d_list(),
                          *matching_submap->cloud_ikdtree(),
                          *local_submap->cloud_ikdtree(),
                          *matching_submap->highlight_cloud_ikdtree(),
                          pose_observation.get(), cov,
                          &summary);
  double match_time = tic_toc.toc();
  // LOG(INFO) << "ceres scan matcher cost time: " << match_time << " ms";
  // time_file_ << match_time << std::endl;
  if (!summary.IsSolutionUsable() || summary.final_cost == -1 ||
      std::isinf(summary.final_cost) || std::isnan(summary.final_cost))
  {
    LOG(WARNING) << "Normal matching failed, use correlative scan matcher help localization.";
    transform::Rigid2d check_pose;
    real_time_correlative_scan_matcher_.Match(
      initial_ceres_pose, filtered_gravity_aligned_point_cloud,
      *local_submap->grid(), &initial_ceres_pose);

    ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
                              filtered_gravity_aligned_point_cloud,
                              highlight_point_cloud,
                              *local_submap->grid(),
                              *local_submap->cloud_ikdtree(),
                              *local_submap->highlight_cloud_ikdtree(),
                              pose_observation.get(),
                              &summary);

    active_submaps_.clear();
    if (!summary.IsSolutionUsable() || summary.final_cost == -1 ||
        std::isinf(summary.final_cost) || std::isnan(summary.final_cost))
    {
      LOG(ERROR) << "Localization failed !!!!!" <<std::endl;
      loc_state_ = LocalizationState::LOCALIZATION_FAILED;
    }
  }

  // std::cout << "cov: \n" << cov.matrix() << std::endl;
  // LOG(INFO) << "scan match: " << *pose_observation;

  // 【新增】检查pose_observation是否包含NaN或Inf
  if (pose_observation) {
    const auto& trans = pose_observation->translation();
    const double angle = pose_observation->rotation().angle();
    
    if (!std::isfinite(trans.x()) || !std::isfinite(trans.y()) || !std::isfinite(angle)) {
      LOG(ERROR) << "ScanMatch produced invalid pose! "
                 << "Translation: [" << trans.x() << ", " << trans.y() << "], "
                 << "Rotation angle: " << angle;
      // 返回nullptr表示匹配失败
      return nullptr;
    }
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
 * @brief 获取当前定位置信度
 * 
 * @return double 置信度值 [0, 1]
 */
double PoseEstimator2D::GetLocalizationConfidence() const {
  if (consistency_checker_) {
    // 校准到 [0,1]：原始一致性 score × 1.25 截断到 1.0——重合率 0.8 即视为满分
    // （0.8 重合率实际已是优秀定位）。下游 NavSystemStatus / panel / command_board
    // 判据统一使用此校准后的置信度，保持定义与使用一致。
    const double scaled = consistency_checker_->GetConfidence() * 1.25;
    return scaled > 1.0 ? 1.0 : scaled;
  }
  return 0.0;
}

/**
 * @brief 向PoseEstimator2D中加入激光雷达数据，使用激光同步器进行同步
 * 
 * @param sensor_id 
 * @param unsynchronized_data 
 * @return std::unique_ptr<PoseEstimator2D::MatchingResult> 激光匹配的结果
 */
std::unique_ptr<PoseEstimator2D::MatchingResult>
PoseEstimator2D::AddRangeData(
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
 * @return std::unique_ptr<PoseEstimator2D::MatchingResult> 处理结果
 */
std::unique_ptr<PoseEstimator2D::MatchingResult>
PoseEstimator2D::ProcessSynchronizedLaserData(
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
  
  // 如果还没有初始化且没有进行重定位，根据配置决定是否自动触发全局重定位
  if (loc_state_ == LocalizationState::INITIALIZED && !pose_initializer_ && 
      options_.auto_trigger_global_relocalization())
  {
    LOG(INFO) << "No initial pose provided, triggering global relocalization automatically...";
    TriggerGlobalRelocalization();
  }
  
  // 只有在定位模式下才会用关键帧策略
  if (loc_state_ == LocalizationState::LOCALIZED) {
    if (keyframe_motion_filter_.IsSimilar(sync_time, odom_at_sync_time->pose)) {
      return nullptr;
    }
  }
  
  // 使用DataPreprocessor处理多传感器数据
  preprocess::MergedData merged_data = data_preprocessor_->ProcessMultipleSensorData(
      synchronized_data.laser_data_map, sync_time);
  
  if (merged_data.IsEmpty()) {
    LOG(WARNING) << "Merged data from DataPreprocessor is empty";
    return nullptr;
  }
  
  // 更新传感器时间信息
  const common::Time current_sensor_time = sync_time;
  absl::optional<common::Duration> sensor_duration;
  if (last_sensor_time_.has_value()) {
    sensor_duration = current_sensor_time - last_sensor_time_.value();
  }
  last_sensor_time_ = current_sensor_time;
  
  // 重定位逻辑
  if (loc_state_ == LocalizationState::RELOCALIZING) {
    CHECK(pose_initializer_ != nullptr);
    pose_initializer_->UpdateSensorData(merged_data.merged_filtered_cloud,
                                        merged_data.merged_line_data);

    PoseInitializerStatus initializer_status =
                              pose_initializer_->GetInitializerStatus();
    if (initializer_status == PoseInitializerStatus::Success) {
      auto initial_pose = pose_initializer_->GetEstimatedPose();
      if (initial_pose) {
        LOG(WARNING) << "Relocalization succeeded! Initial pose: " << *initial_pose;
        local_pose_ = transform::Embed3D(*initial_pose);
        // 更新用于计算delta pose的基准里程计数据
        UpdateLastOdomForDelta(sync_time);
        loc_state_ = LocalizationState::LOCALIZED;
        pose_initializer_->stop();
        active_submaps_.clear();
        local_map_updater_->UpdateRobotPose(transform::Project2D(local_pose_));
        
        // 重置全局重定位尝试标志
        has_tried_global_relocalization_ = false;
      }
    } else if (initializer_status == PoseInitializerStatus::Failed) {
      LOG(WARNING) << "Pose initializer failed.";
      
      // 检查当前是否是局部重定位失败，如果是，尝试全局重定位
      if (!has_tried_global_relocalization_) {
        LOG(INFO) << "Local relocalization failed, trying global relocalization...";
        pose_initializer_->stop();
        
        pose_initializer_->InitGlobalSubmap(global_static_submap_);

        pose_initializer_->TriggerGlobalRelocalization();
        pose_initializer_->start();
        
        has_tried_global_relocalization_ = true;
        // 保持RELOCALIZING状态，等待全局重定位结果
      } else {
        LOG(ERROR) << "Both local and global relocalization failed.";
        loc_state_ = LocalizationState::LOCALIZATION_FAILED;
        pose_initializer_->stop();
        has_tried_global_relocalization_ = false;  // 重置标志，以便下次尝试
      }
    }
  }

  if (loc_state_ != LocalizationState::LOCALIZED) {
    return nullptr;
  }
  
  // 使用融合后的数据进行位姿估计和地图更新
  return ProcessDataAndEstimatePose(
      "merged_sensors",  // 使用一个标识来表示这是合并的传感器数据
      current_sensor_time,
      merged_data,
      transform::Rigid3d::Identity(), 
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
 * @return std::unique_ptr<PoseEstimator2D::MatchingResult> 
 */
std::unique_ptr<PoseEstimator2D::MatchingResult>
PoseEstimator2D::ProcessDataAndEstimatePose(
    const std::string& sensor_id,
    const common::Time& time,
    const preprocess::MergedData& merged_data,
    const transform::Rigid3d& gravity_alignment,
    const absl::optional<common::Duration>& sensor_duration,
    preprocess::DataPreprocessor* data_preprocessor) {
  
  if (merged_data.merged_range_data.returns.empty()) {
    LOG(WARNING) << "Dropped empty merged range data.";
    return nullptr;
  }

  // 计算位姿预测值，确保有效性
  transform::Rigid2d pose_prediction;
  bool pose_prediction_valid = false;
  
  // 从里程计获取delta pose预测
  const transform::Rigid3d odom_delta_pose = GetOdomDeltaPose(time);
  if (local_pose_.IsValid() && odom_delta_pose.IsValid()) {
    const auto pose_prediction_odom = transform::Project2D(local_pose_ * odom_delta_pose);
    if (pose_prediction_odom.IsValid()) {
      pose_prediction = pose_prediction_odom;
      pose_prediction_valid = true;
      // LOG(INFO) << "Using odometry prediction: " << pose_prediction;
    }
  }
  
  // 如果没有有效的预测位姿，使用上次的位姿作为初始值
  if (!pose_prediction_valid) {
    if (local_pose_.IsValid()) {
      pose_prediction = transform::Project2D(local_pose_);
      LOG(WARNING) << "Using last known pose as prediction: " << pose_prediction;
    } else {
      LOG(ERROR) << "No valid pose prediction available, using identity";
      pose_prediction = transform::Rigid2d::Identity();
    }
  }

  const sensor::PointCloud& filtered_gravity_aligned_point_cloud = merged_data.merged_filtered_cloud;
  if (filtered_gravity_aligned_point_cloud.empty()) {
    return nullptr;
  }

  // ScanMatch计算当前帧位姿：对齐重力坐标系在local坐标系中的位姿
  std::map<int, std::vector<sensor::LineData>> tracked_lane_data;
  std::unique_ptr<transform::Rigid2d> pose_estimate_2d =
      ScanMatch(time, pose_prediction,
                filtered_gravity_aligned_point_cloud,
                merged_data.merged_highlight_cloud,
                merged_data.merged_line_data,
                tracked_lane_data);
  
  if (pose_estimate_2d == nullptr) {
    LOG(WARNING) << "Scan matching failed.";
    return nullptr;
  }

  // ========== 使用 ConsistencyChecker 进行多维度一致性检查 ==========
  // 获取局部地图用于一致性检查
  auto local_submap = local_map_updater_->local_submap();
  if (!local_submap) {
    LOG(ERROR) << "Local submap not available for consistency check";
    return nullptr;
  }
  
  // 步骤1: 一致性检查
  auto check_result = consistency_checker_->CheckPoseConsistency(
      time,
      pose_prediction,
      *pose_estimate_2d,
      filtered_gravity_aligned_point_cloud,
      *local_submap);
  
  // 打印一致性检查结果
  // LOG(INFO) << "Consistency Check Result:\n" << check_result.ToString();
  
  // 步骤2: 如果一致性检查失败，尝试验证和恢复
  if (!check_result.is_consistent) {
    LOG(WARNING) << "Pose consistency check failed, attempting validation and recovery...";
    
    // 步骤2a: 尝试验证位姿
    bool validation_passed = consistency_checker_->ValidatePoseEstimate(
        *pose_estimate_2d,
        filtered_gravity_aligned_point_cloud,
        *local_submap);
    
    if (!validation_passed) {
      LOG(WARNING) << "Pose validation also failed";
      
      // 步骤2b: 以估计位姿为中心同步重建 local_map_updater_ 内部的 local_submap_
      // 原因：当前 local_submap 可能以里程计位姿为中心，与 scan match 估计位置有偏差
      // 重建后 local_submap_ 以 pose_estimate_2d 为中心，搜索范围更加合理
      local_map_updater_->RebuildLocalSubmapAt(*pose_estimate_2d);
      auto recovery_local_submap = local_map_updater_->local_submap();
      if (!recovery_local_submap) {
        LOG(WARNING) << "RebuildLocalSubmapAt returned nullptr, falling back to current local submap";
        recovery_local_submap = local_submap;
      }
      
      // 步骤2c: 在以估计位姿为中心重建的 local map 上尝试恢复
      auto recovery_pose = consistency_checker_->RecoverFromLocalMap(
          time,
          *pose_estimate_2d,  // 使用当前估计位姿作为初始搜索位姿
          filtered_gravity_aligned_point_cloud,
          merged_data.merged_highlight_cloud,
          recovery_local_submap);
      
      // 步骤2d: 如果恢复失败，返回错误
      if (!recovery_pose) {
        LOG(ERROR) << "All recovery attempts failed! Localization lost.";
        loc_state_ = LocalizationState::LOCALIZATION_FAILED;
        active_submaps_.clear();
        return nullptr;
      }
      
      // 恢复成功（已经过一致性检查），更新位姿
      pose_estimate_2d = std::move(recovery_pose);
      
      LOG(INFO) << "Successfully recovered from localization failure!"
                << "\n  Recovered pose: " << *pose_estimate_2d
                << "\n  Cleared active submaps, restarting mapping";
      
      // 清空active_submaps重新开始
      active_submaps_.clear();
    } else {
      // 验证成功，继续使用当前估计位姿
      LOG(INFO) << "Pose validation successful, using estimated pose";
    }
  }

  // ========== P0 漂移门控 + 重定位：拦截自洽漂移，先恢复正确位姿，再决定是否更新地图 ==========
  // 现有 CheckPoseConsistency 三路信号都参照 odom 预测 + live local map（被同一漂移污染），
  // 对缓慢自洽漂移失明。这里用从不更新的 global_static_submap_ 做独立锚点检测漂移；一旦确认：
  //   ① 先在 static map 上重定位并做一致性校验，恢复【正确】位姿；
  //   ② 重定位成功 → 用校正后的正确位姿，并恢复地图更新（在正确位姿插入）；
  //   ③ 重定位失败/未通过校验 → 冻结地图，绝不在错误/不确定位姿更新。
  // 必须放在 pose_estimate / local_pose_ 计算之前，让校正后的位姿流入 TF 输出与后续插入。
  bool safe_to_update_map = true;
  {
    auto static_submap = local_map_updater_->GetStaticSubmap();
    auto live_local = local_map_updater_->local_submap();
    if (static_submap && static_submap->grid() && live_local) {
      DriftCheckResult drift_result = consistency_checker_->CheckMapDrift(
          *pose_estimate_2d,
          filtered_gravity_aligned_point_cloud,
          *static_submap,
          *live_local);
      if (drift_result.drift_suspected) {
        LOG(WARNING) << "Localization drift confirmed — relocalizing against static map "
                        "before any map update. " << drift_result.ToString();

        // 以 static map 再匹配位姿为初值，从【漂移免疫的 static 底图】重建干净 local map
        // （含 grid + cloud → ikdtree）。绝不能用 RebuildLocalSubmapAt（裁自已漂的
        // global_dynamic_submap_）——在漂图上 ceres 会把校正位姿拉回漂移方向（DEC-002 教训）。
        // 干净 static 上精修 + 校验 refined 自身重合度（见 RecoverFromLocalMap）。
        local_map_updater_->RebuildLocalSubmapFromStaticAt(drift_result.rematched_pose);
        auto reloc_submap = local_map_updater_->local_submap();
        std::unique_ptr<transform::Rigid2d> corrected;
        if (reloc_submap) {
          corrected = consistency_checker_->RecoverFromLocalMap(
              time,
              drift_result.rematched_pose,
              filtered_gravity_aligned_point_cloud,
              merged_data.merged_highlight_cloud,
              reloc_submap);
        }

        if (corrected) {
          // 重定位成功且通过一致性校验 → 用校正后的正确位姿，清空 active_submaps 重启匹配，
          // 位姿已确认正确 → 允许在正确位姿更新地图。
          pose_estimate_2d = std::move(corrected);
          active_submaps_.clear();
          safe_to_update_map = true;
          LOG(WARNING) << "Static-map relocalization succeeded — resuming map update at corrected pose "
                       << *pose_estimate_2d;
        } else {
          // 重定位失败/未通过校验 → 冻结地图，绝不在错误/不确定位姿更新（定位仍输出当前估计）。
          safe_to_update_map = false;
          LOG(ERROR) << "Static-map relocalization failed — freezing map update this frame.";
        }
      }
    }
  }

  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  local_pose_ = pose_estimate;

  // 位姿估计成功后，更新用于计算delta pose的基准里程计数据
  UpdateLastOdomForDelta(time);

  // ========== 更新机器人位姿到局部地图 ==========
  local_map_updater_->UpdateRobotPose(*pose_estimate_2d);

  sensor::RangeData range_data_in_local =
      sensor::TransformRangeData(merged_data.merged_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // Preserve the scan-matched highlight subset for the ROS visualization
  // path. This is independent of whether this frame updates a local submap.
  range_data_in_local.highlight_returns = sensor::TransformPointCloud(
      merged_data.merged_highlight_cloud, pose_estimate.cast<float>());

  // 将线数据转换到local坐标系
  std::map<int, std::vector<sensor::LineData>> lines_in_local;
  for (const auto& data_vec: tracked_lane_data) {
    for (const auto& line: data_vec.second) {
      lines_in_local[data_vec.first].push_back(sensor::TransformLineData(*pose_estimate_2d, line));
    }
  }

  // 使用merged_data中的数据进行插入（漂移嫌疑时冻结地图，不在错误位姿烤入数据）
  std::unique_ptr<InsertionResult> insertion_result;
  if (safe_to_update_map) {
    insertion_result = InsertIntoSubmap(
        time,
        merged_data,
        tracked_lane_data,
        pose_estimate,gravity_alignment.rotation());
  }

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
  
  // 返回匹配结果
  return absl::make_unique<MatchingResult>(
      MatchingResult{time, local_pose_,
                     std::move(range_data_in_local),
                     std::move(lines_in_local),
                     std::move(insertion_result)});
}



sensor::FeatureData PoseEstimator2D::GetFeatureData(
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
 * @param tracked_lane_data 跟踪的车道线数据
 * @param pose_estimate 估计的位姿
 * @param gravity_alignment 重力对齐的旋转
 * @return std::unique_ptr<PoseEstimator2D::InsertionResult> 插入的结果
 */
std::unique_ptr<PoseEstimator2D::InsertionResult>
PoseEstimator2D::InsertIntoSubmap(
    const common::Time time,
    const preprocess::MergedData& merged_data,
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

  
  // 一次性插入所有数据（range data、cloud data、line data和reflector data）
  std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps =
      active_submaps_.InsertData(all_range_data_to_insert,
                                  cloud_in_local,
                                  highlight_cloud_in_local,
                                  tracked_lines_in_local,
                                  merged_data.merged_reflector_data);

  // 4. 将数据插入到global_dynamic_submap_，实现全局地图的增量更新
  if (local_map_updater_) {
    local_map_updater_->UpdateDynamicMap(all_range_data_to_insert,
                                          cloud_in_local,
                                          highlight_cloud_in_local,
                                          tracked_lines_in_local);
  }

  // 5. 返回插入结果
  return absl::make_unique<InsertionResult>(InsertionResult{
      std::make_shared<const mapping::TrajectoryNode::Data>(mapping::TrajectoryNode::Data{
          time,
          gravity_alignment,
          merged_data.merged_filtered_cloud,  // 合并后的滤波点云
          merged_data.merged_highlight_cloud, // 合并后的highlight点云
          merged_data.original_undistorted_range_datas, // 各个传感器的原始去畸变range data
          tracked_lane_data,
          merged_data.merged_reflector_data,
          {},  // 'high_resolution_point_cloud' is only used in 3D.
          {},  // 'low_resolution_point_cloud' is only used in 3D.
          {},  // 'rotational_scan_matcher_histogram' is only used in 3D.
          pose_estimate}),
      std::move(insertion_submaps)});
}

/**
 * @brief 向PoseEstimator2D中加入IMU数据，同时初始化位姿插值器
 * 
 * @param imu_data ImuData格式的数据
 */
void PoseEstimator2D::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(options_.use_imu_data()) << "An unexpected IMU packet was added.";
}

/**
 * @brief 向PoseEstimator2D中加入odom数据
 * @param odometry_data OdometryData格式的数据
 */
void PoseEstimator2D::AddOdometryData(
    const sensor::OdometryData& odometry_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // 将里程计数据添加到数据预处理器
  if (data_preprocessor_) {
    data_preprocessor_->AddOdometryData(odometry_data);
  }
  
  // 将里程计数据添加到激光同步器，用于时间一致性检查
  laser_synchronizer_.AddOdometryData(odometry_data);
}

/**
 * @brief 计算相对于上次位姿的delta pose
 * 
 * @param laser_time 激光雷达时间戳
 * @return transform::Rigid3d delta pose
 */
transform::Rigid3d PoseEstimator2D::GetOdomDeltaPose(const common::Time& laser_time) {
  auto current_odom = GetOdometryAtTime(laser_time);
  if (!current_odom.has_value()) {
    LOG(WARNING) << "No odometry data available for delta pose calculation at time: " << laser_time;
    return transform::Rigid3d::Identity();
  }
  
  // 如果这是第一次调用，保存当前位姿作为基准
  if (!last_odom_for_delta_.has_value()) {
    last_odom_for_delta_ = *current_odom;
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
absl::optional<sensor::OdometryData> PoseEstimator2D::GetOdometryAtTime(const common::Time& time) const {
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
void PoseEstimator2D::UpdateLastOdomForDelta(const common::Time& time) {
  auto current_odom = GetOdometryAtTime(time);
  if (current_odom.has_value()) {
    last_odom_for_delta_ = *current_odom;
  }
}

void PoseEstimator2D::ResetMapping()
{
}

void PoseEstimator2D::RegisterMetrics(
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

}  // namespace localization
}  // namespace cartographer
