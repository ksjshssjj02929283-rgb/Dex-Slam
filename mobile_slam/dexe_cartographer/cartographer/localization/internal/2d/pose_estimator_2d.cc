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

  amcl_core_ =
      std::make_unique<amcl::AMCLCore>(options_.amcl_core_options());

  local_map_updater_ =
      std::make_unique<LocalMapUpdater2D>(options_.local_map_updater_options());

  loc_state_ = LocalizationState::NOT_INITIALIZED;

  // time_file_.open("/home/ubt/data/time.txt", std::ios::out);
}

PoseEstimator2D::~PoseEstimator2D() {}

void PoseEstimator2D::InitGlobalGridMap(std::shared_ptr<GridMap2D> grid_map)
{
  global_grid_map_ = grid_map;
  local_map_updater_->SetGlobalGridMap(grid_map);

  loc_state_ = LocalizationState::INITIALIZED;
}

void PoseEstimator2D::InitGlobalCloudMap(std::shared_ptr<CloudMap2D> cloud_map)
{
  if (global_grid_map_ == nullptr) {
    LOG(ERROR) << "Global grid map is not iUpdateLocalCloudMapnitialized!";
    return;
  }

  global_cloud_map_ = cloud_map;
  // insert成功，就获取submap的cloud，并将其加入的gicp的target中
    // 这个过程的平均时间是0.25~0.5ms
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : global_cloud_map_->cloud()->points()) {
    pcl::PointXYZ p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = 0.0;
    global_map_cloud->push_back(p);
  }

  local_map_updater_->SetGlobalCloudMap(cloud_map);
  // box_filter_->SetOrigin(pose_estimate.translation().cast<float>());
  // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_submap(new pcl::PointCloud<pcl::PointXYZ>());
  // box_filter_->Filter(submap_cloud, filtered_sub
}

void PoseEstimator2D::InitGlobalHighlightCloudMap(std::shared_ptr<CloudMap2D> highlight_cloud_map)
{
  global_highlight_cloud_map_ = highlight_cloud_map;
  // insert成功，就获取submap的cloud，并将其加入的gicp的target中
    // 这个过程的平均时间是0.25~0.5ms
  pcl::PointCloud<pcl::PointXYZ>::Ptr global_highlight_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : global_cloud_map_->cloud()->points()) {
    pcl::PointXYZ p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = 0.0;
    global_highlight_map_cloud->push_back(p);
  }

  global_highlight_cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();

  float resolution = global_grid_map_->limits().resolution();
  global_highlight_cloud_ikdtree_->set_downsample_param(resolution);
  global_highlight_cloud_ikdtree_->Build(global_highlight_map_cloud->points);
}

void PoseEstimator2D::InitGlobalLineMap(const std::shared_ptr<mapping::LineMap2DList>& line_map)
{
  global_line_map_ = line_map;
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
    pose_initializer_->InitGlobalLineMap(global_line_map_);
  }

  local_map_updater_->UpdateRobotPose(transform::Project2D(initial_pose));
  if (!local_map_updater_->IsLocalMapUpdated()) {
    LOG(ERROR) << "Local map is not updated! This should not happen!";
    return false;
  }

  pose_initializer_->InitGlobalGridMap(local_map_updater_->local_grid_map());
  pose_initializer_->InitGlobalCloudMap(local_map_updater_->local_cloud_map());

  pose_initializer_->SetInitialPose(transform::Project2D(initial_pose));
  pose_initializer_->start();
  loc_state_ = LocalizationState::RELOCALIZING;

  return true;
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
    std::map<int, std::vector<sensor::LineData>> & tracked_lane_data,
    Eigen::Matrix3d & cov)
{
  // 1. 如果active_submaps_中没有submap，则直接返回pose_prediction
  //    一般情况下，第一帧激光数据进来时，active_submaps_中没有submap
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  // 2. 读取active_submaps_中的第一个submap，作为匹配的地图
  std::shared_ptr<const mapping::Submap2D> matching_submap =
      active_submaps_.submaps().front();
  // The online correlative scan matcher will refine the initial estimate for
  // the Ceres scan matcher.

  // LOG(INFO) << "prediction: "<< pose_prediction;

  // 3. 如果使用correlative方法，则先用correlative方法进行位姿估计
  //    相关性扫描匹配器，速度快，但是精度低
  transform::Rigid2d correlative_pose = pose_prediction;
  transform::Rigid2d initial_ceres_pose = pose_prediction;

  common::TicToc tic_correlative;
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_.Match(
      pose_prediction, filtered_gravity_aligned_point_cloud,
      *matching_submap->grid(), &correlative_pose);
      kRealTimeCorrelativeScanMatcherScoreMetric->Observe(score);
    // LOG(INFO) << "correlative score: " << score;
    if (score > 0.4) {
      transform::Rigid2d temp(pose_prediction.translation(), correlative_pose.rotation());
      initial_ceres_pose = temp;
    } else {
      initial_ceres_pose = pose_prediction;
      LOG(WARNING) << "Correlative scan matching score is too small, abond correlative pose";
    }
  }
  double correlative_time = tic_correlative.toc();
  // LOG(INFO) << "correlative scan matcher cost time: " << correlative_time << " ms";
  // std::cout << "initial_ceres_pose: " << initial_ceres_pose << std::endl;

  // LOG(INFO) << "csm: " << initial_ceres_pose;
  std::map<int, std::vector<sensor::LineData>> global_tracked_lane_data;
  if (active_submaps_.submaps().size() != 0) {
    line_matcher_->TrackLineFromMap(initial_ceres_pose,
                                    line_data,
                                    *active_submaps_.line_map_2d_list(),
                                    tracked_lane_data);
    line_matcher_->TrackLineFromMap(initial_ceres_pose,
                                    line_data,
                                    *global_line_map_,
                                    global_tracked_lane_data);
  }

  // 4. 使用ceres进行位姿估计，得到pose_observation
  //    使用点到地图的误差作为cost function
  auto pose_observation = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;

  auto local_grid_map = local_map_updater_->local_grid_map();
  auto local_cloud_ikdtree = local_map_updater_->local_cloud_map_ikdtree();
  common::TicToc tic_toc;

  ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
                          filtered_gravity_aligned_point_cloud,
                          highlight_point_cloud,
                          tracked_lane_data,
                          global_tracked_lane_data,
                          *matching_submap->grid(), *local_grid_map,
                          *matching_submap->line_map_2d_list(), *global_line_map_,
                          *matching_submap->cloud_ikdtree(),
                          *local_cloud_ikdtree,
                          *global_highlight_cloud_ikdtree_,
                          pose_observation.get(), cov,
                          &summary);
  double match_time = tic_toc.toc();
  // LOG(INFO) << "ceres scan matcher cost time: " << match_time << " ms";
  // time_file_ << match_time << std::endl;

  // ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
  //                           filtered_gravity_aligned_point_cloud,
  //                           *laser_perception_->getHighlightCloud(),
  //                           *global_grid_map_,
  //                           *local_cloud_ikdtree.get(),
  //                           *global_highlight_cloud_ikdtree_,
  //                           pose_observation.get(),
  //                           &summary);
  cov = amcl_core_->GetCovariance();
  if (!summary.IsSolutionUsable() || summary.final_cost == -1 ||
      std::isinf(summary.final_cost) || std::isnan(summary.final_cost))
  {
    LOG(WARNING) << "Normal matching failed, use correlative scan matcher help localization.";
    transform::Rigid2d check_pose;
    real_time_correlative_scan_matcher_.Match(
      initial_ceres_pose, filtered_gravity_aligned_point_cloud,
      *local_grid_map, &initial_ceres_pose);

    ceres_scan_matcher_.Match(initial_ceres_pose.translation(), initial_ceres_pose,
                              filtered_gravity_aligned_point_cloud,
                              highlight_point_cloud,
                              *local_grid_map,
                              *local_cloud_ikdtree,
                              *global_highlight_cloud_ikdtree_,
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
    pose_initializer_->UpdateSensorData(merged_data.merged_range_data.returns,
                                        merged_data.merged_line_data);

    PoseInitializerStatus initializer_status =
                              pose_initializer_->GetInitializerStatus();
    if (initializer_status == PoseInitializerStatus::Success) {
      auto initial_pose = pose_initializer_->GetEstimatedPose();
      if (initial_pose) {
        LOG(WARNING) << "Initialized pose: " << *initial_pose;
        local_pose_ = transform::Embed3D(*initial_pose);
        // 更新用于计算delta pose的基准里程计数据
        UpdateLastOdomForDelta(sync_time);
        loc_state_ = LocalizationState::LOCALIZED;
        pose_initializer_->stop();
        active_submaps_.clear();
        local_map_updater_->UpdateRobotPose(transform::Project2D(local_pose_));
      }
    } else if (initializer_status == PoseInitializerStatus::Failed) {
      LOG(WARNING) << "Pose initializer failed.";
      loc_state_ = LocalizationState::LOCALIZATION_FAILED;
      pose_initializer_->stop();
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

  // 根据激光雷达时间戳获取delta pose（关键帧之间的位姿增量）
  const transform::Rigid3d odom_delta_pose = GetOdomDeltaPose(time);
  const auto pose_prediction_odom = transform::Project2D(local_pose_ * odom_delta_pose);
  
  // 更新AMCL
  if (!amcl_core_->IsInitialized()) {
    amcl_core_->SetInitialPose(transform::Project2D(local_pose_));
  } else {
    // 获取当前里程计数据用于AMCL更新
    auto current_odom = GetOdometryAtTime(time);
    if (current_odom.has_value()) {
      amcl_core_->UpdateWithSensorData(time,
                                       merged_data.merged_filtered_cloud,
                                       *current_odom);
    }
  }

  const auto amcl_pose = amcl_core_->GetPose2D();

  transform::Rigid2d pose_prediction;
  if (amcl_pose) {
    double delta_dist = (pose_prediction_odom.translation() - amcl_pose->translation()).norm();
    if (delta_dist > 0.1) {
      LOG(INFO) << "predict pose by odom";
      pose_prediction = pose_prediction_odom;
    } else {
      pose_prediction = *amcl_pose;
    }
  } else {
    pose_prediction = pose_prediction_odom;
  }

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
  
  const transform::Rigid3d pose_estimate =
      transform::Embed3D(*pose_estimate_2d) * gravity_alignment;
  local_pose_ = pose_estimate;
  
  // 位姿估计成功后，更新用于计算delta pose的基准里程计数据
  UpdateLastOdomForDelta(time);

  local_map_updater_->UpdateRobotPose(*pose_estimate_2d);

  if (amcl_pose != nullptr) {
    double delta_dist = (amcl_pose->translation() - pose_estimate_2d->translation()).norm();
    // 检查距离计算结果是否有效
    if (!std::isfinite(delta_dist)) {
      LOG(WARNING) << "Delta distance calculation resulted in invalid value, resetting AMCL";
      amcl_core_->SetInitialPose(*pose_estimate_2d);
    } else if (delta_dist > 0.1) {
      LOG(WARNING) << "delta_dist: " << delta_dist << ", resetting AMCL pose";
      amcl_core_->SetInitialPose(*pose_estimate_2d);
    }
  }

  sensor::RangeData range_data_in_local =
      sensor::TransformRangeData(merged_data.merged_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));

  // 将线数据转换到local坐标系
  std::map<int, std::vector<sensor::LineData>> lines_in_local;
  for (const auto& data_vec: tracked_lane_data) {
    for (const auto& line: data_vec.second) {
      lines_in_local[data_vec.first].push_back(sensor::TransformLineData(*pose_estimate_2d, line));
    }
  }

  // 使用merged_data中的数据进行插入
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time,
      merged_data,
      tracked_lane_data,
      pose_estimate,gravity_alignment.rotation());

  if (insertion_result) {
    amcl_core_->SetGrid(active_submaps_.submaps().front()->grid());
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
                     std::move(amcl_core_->GetParticlePoses()),
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
  
  // 一次性批量插入所有range data
  std::vector<std::shared_ptr<const mapping::Submap2D>> insertion_submaps =
      active_submaps_.InsertRangeData(all_range_data_to_insert);
  // 插入合并后的云数据、线数据和反射器数据
  active_submaps_.InsertCloudData(cloud_in_local, highlight_cloud_in_local);
  active_submaps_.InsertLineData(tracked_lines_in_local);
  active_submaps_.InsertReflectorData(merged_data.merged_reflector_data);

  // 4. 返回插入结果
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
