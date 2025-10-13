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

#include "cartographer/localization/internal/2d/pose_initializer_2d.h"

#include <limits>
#include <memory>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "absl/memory/memory.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/common/tic_toc.h"
#include "cartographer/transform/transform.h"
#include "cartographer/common/tic_toc.h"

namespace cartographer {
namespace localization {
PoseInitializer::PoseInitializer(const proto::PoseInitializerOptions2D& options)
    : options_(options)
{
  stop();
}

PoseInitializer::~PoseInitializer() {
  stop();
}

void PoseInitializer::Initialize()
{
  line_matcher_ =
    std::make_unique<mapping::scan_matching::LineMatcher2D>(
          options_.line_matcher_options());

  real_time_correlative_scan_matcher_ =
      std::make_unique<mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>(
        options_.real_time_correlative_scan_matcher_options());

  ceres_scan_matcher_ =
      std::make_unique<mapping::scan_matching::CeresScanMatcher2D>(
        options_.ceres_scan_matcher_options());
}

void PoseInitializer::ResetStatus()
{
  // 注意：此函数现在期望在锁内被调用
  need_apply_initial_pose_ = false;

  initial_pose_.reset();
  global_pose_estimate_.reset();

  relocalization_thread_.reset();
  curr_cloud_.reset();
  curr_line_data_.reset();
  fast_correlative_scan_matcher_.reset();
  real_time_correlative_scan_matcher_.reset();
  ceres_scan_matcher_.reset();
  line_matcher_.reset();
  initializer_status_ = PoseInitializerStatus::Idle;
}

void PoseInitializer::InitGlobalGridMap(const std::shared_ptr<GridMap2D>& grid_map)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  global_grid_map_ = grid_map;
  // 地图更新时重置快速匹配器，下次使用时重新创建
  fast_correlative_scan_matcher_.reset();
}

void PoseInitializer::InitGlobalCloudMap(const std::shared_ptr<CloudMap2D>& cloud_map)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  global_cloud_map_ = cloud_map;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (const auto& point : global_cloud_map_->cloud()->points()) {
    pcl::PointXYZ p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = 0.0;
    global_map_cloud->push_back(p);
  }

  global_cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();

  float resolution = global_grid_map_->limits().resolution();
  global_cloud_ikdtree_->set_downsample_param(resolution);
  global_cloud_ikdtree_->Build(global_map_cloud->points);

}

void PoseInitializer::InitGlobalLineMap(const std::shared_ptr<mapping::LineMap2DList>& line_map)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  global_line_map_ = line_map;
}

bool PoseInitializer::SetInitialPose(const transform::Rigid2d &initial_pose)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  // 将3为的位姿变成二维的
  initial_pose_ = std::make_shared<transform::Rigid2d>(initial_pose);
  global_pose_estimate_.reset();
  need_apply_initial_pose_ = true;
  return true;
}

void PoseInitializer::UpdateSensorData(const sensor::PointCloud& scan_cloud,
                    const std::vector<sensor::LineData> & line_data)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  curr_cloud_ = std::make_shared<sensor::PointCloud>(scan_cloud);
  curr_line_data_ = std::make_shared<std::vector<sensor::LineData>>(line_data);
}

void PoseInitializer::start()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  if (working_) {
    LOG(WARNING) << "PoseInitializer is already working";
    return;
  }
  Initialize();
  working_ = true;
  CreateRelocalzationThread();
}

void PoseInitializer::stop()
{
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    working_ = false;
    data_cv_.notify_all();  // 唤醒等待线程以便退出
  }
  
  // 在锁外等待线程结束，避免死锁
  if (relocalization_thread_ && relocalization_thread_->joinable()) {
      relocalization_thread_->join();
  }
  
  std::unique_lock<std::mutex> lock(data_mutex_);
  ResetStatus();
}

void PoseInitializer::CreateRelocalzationThread()
{
  if (!relocalization_thread_ || !relocalization_thread_->joinable()) {
    relocalization_thread_ = std::make_unique<std::thread>(&PoseInitializer::Run, this);    
  }
}

void PoseInitializer::Run()
{
  LOG(INFO) << "PoseInitializer relocalization thread started";
  
  while (working_) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    
    // 等待超时（500ms）或者停止信号
    data_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
      return !working_;
    });
    
    if (!working_) {
      break;
    }
    
    // 检查是否有数据并且需要重新定位
    if (curr_cloud_ != nullptr && !curr_cloud_->empty() && need_apply_initial_pose_) {
      RelocalzationOnce();  // 在锁内执行定位，保证数据一致性
    }
  }
  
  LOG(INFO) << "PoseInitializer relocalization thread stopped";
}

void PoseInitializer::RelocalzationOnce()
{
  if (global_grid_map_ == nullptr) {
    LOG(ERROR) << "Global grid map is not initialized, please set global grid map first.";
    initializer_status_ = PoseInitializerStatus::NotInitialized;
    return;
  }

  if (curr_cloud_ == nullptr || curr_cloud_->empty()) {
    return;
  }

  if (!need_apply_initial_pose_ || initial_pose_ == nullptr) {
    LOG(ERROR) << "Initial pose is not set, please set initial pose first.";
    return;
  }

  if (fast_correlative_scan_matcher_ == nullptr) {
    fast_correlative_scan_matcher_ =
        std::make_unique<mapping::scan_matching::FastCorrelativeScanMatcher2D>(
          *global_grid_map_, options_.fast_correlative_scan_matcher_options());
  }

  initializer_status_ = PoseInitializerStatus::Estimating;

  float score;
  if (options_.use_global_matching()) {
    fast_correlative_scan_matcher_->MatchFullSubmap(
      *curr_cloud_, options_.global_localization_min_score(), &score, initial_pose_.get());
    LOG(INFO) << "global localization score: " << score;

    if (score < options_. global_localization_min_score()) {
      LOG(WARNING) << "global localization score is too low, use initial pose instead.";
    }
  }

  transform::Rigid2d fast_correlative_pose;

  bool fast_match_success =
                fast_correlative_scan_matcher_->Match(
                                  *initial_pose_,
                                  *curr_cloud_,
                                  options_.global_localization_min_score(),
                                  &score,
                                  &fast_correlative_pose);
  if (!fast_match_success) {
    LOG(ERROR) << "fast match failed, apply initial pose failed.";
    LOG(ERROR) << "maybe the global localization min score is too small, current score is: " << score;
    initializer_status_ = PoseInitializerStatus::Failed;
    return;
  }

  LOG(INFO) << "fast scan matcher score: " << score;
  LOG(INFO) << "fast scan matcher pose: " << fast_correlative_pose;

  const double fast_dist_th = options_.fast_correlative_scan_matcher_options().linear_search_window();
  const double angular_th = options_.fast_correlative_scan_matcher_options().angular_search_window();
  // 确保 fast_correlative_scan_matcher 不会得到一个离谱的值
  if (!IsPoseValid(*initial_pose_, fast_correlative_pose, fast_dist_th, angular_th)) {
    LOG(ERROR) << "initial pose is not valid, apply initial pose failed.";
    initializer_status_ = PoseInitializerStatus::Failed;
    return;
  }
  initial_pose_ = std::make_shared<transform::Rigid2d>(fast_correlative_pose);

  // 后续可以增加地图切片的方式，使用切片的grid map替换掉全局grid map
  transform::Rigid2d online_correlative_pose;
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_->Match(
      *initial_pose_, *curr_cloud_,
      *global_grid_map_, &online_correlative_pose);

    if (score < options_.global_localization_min_score()) {
      LOG(WARNING) << "online correlative scan matcher score is too low.";
      LOG(INFO) << "online correlative scan matcher score: " << score;
      initializer_status_ = PoseInitializerStatus::Failed;
      return;
    }

    if (!IsPoseValid(*initial_pose_, online_correlative_pose)) {
      LOG(ERROR) << "initial pose is not valid, apply initial pose failed.";
      initializer_status_ = PoseInitializerStatus::Failed;
      return;
    }

    initial_pose_ = std::make_shared<transform::Rigid2d>(online_correlative_pose);
    need_apply_initial_pose_ = false;  // 只进行一次定位
  }

  // 得到粗定位的位姿，进行精定位
  global_pose_estimate_ = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
    // 使用线特征进行匹配
  if (curr_line_data_->empty()) {
    LOG(WARNING) << "no line data in scan matcher.";
  }
  std::map<int, std::vector<sensor::LineData>> tracked_lane_data;
  if (curr_line_data_->size() != 0 && global_line_map_->size() != 0) {
    line_matcher_->TrackLineFromMap(*initial_pose_,
                                    *curr_line_data_,
                                    *global_line_map_,
                                    tracked_lane_data);
  }

  ceres_scan_matcher_->Match(initial_pose_->translation(),
                            *initial_pose_,
                            *curr_cloud_,
                            tracked_lane_data,
                            *global_grid_map_,
                            *global_line_map_,
                            *global_cloud_ikdtree_,
                            global_pose_estimate_.get(),
                            &summary);

  if (!IsRelocalizationSuccess()) {
    global_pose_estimate_.reset();
    initializer_status_ = PoseInitializerStatus::Failed;
  } else {
    initializer_status_ = PoseInitializerStatus::Success;
  }

  // LOG(INFO) << "global pose estimate: "
  //           << global_pose_estimate.translation().x() << " "
  //           << global_pose_estimate.translation().y() << " "
  //           << global_pose_estimate.rotation().angle();
  // LOG(INFO) << "global pose obersevation: "
  //           << pose_observation->translation().x() << " "
  //           << pose_observation->translation().y() << " "
  //           << pose_observation->rotation().angle();
}


bool PoseInitializer::IsRelocalizationSuccess()
{
  if (fast_correlative_scan_matcher_ == nullptr) {
    LOG(ERROR) << "fast correlative scan matcher is not initialized.";
    return false;
  }

  if (global_pose_estimate_ == nullptr) {
    LOG(WARNING) << "Has no estimated global pose.";
    return false;
  }

  std::shared_ptr<transform::Rigid2d> best_pose = absl::make_unique<transform::Rigid2d>();

  float score = 0.0;
  bool fast_match_success =
            fast_correlative_scan_matcher_->Match(
                  *global_pose_estimate_,
                  *curr_cloud_,
                  options_.global_localization_min_score(),
                  &score,
                  best_pose.get());

  if (fast_match_success &&
      score < options_.global_localization_min_score())
  {
    LOG(INFO) << "Initial pose is not good. curren score is too small in validation: " << score;
    return false;
  }

  transform::Rigid2d online_correlative_pose;
  if (options_.use_online_correlative_scan_matching()) {
    score = real_time_correlative_scan_matcher_->Match(
      *global_pose_estimate_, *curr_cloud_,
      *global_grid_map_, best_pose.get());

    if (score < options_.global_localization_min_score()) {
      LOG(WARNING) << "online correlative scan matcher score is too low.";
      LOG(INFO) << "online correlative scan matcher score: " << score;
      return false;
    }
  }

  double delta_dist = (global_pose_estimate_->translation() - best_pose->translation()).norm();
  double delta_yaw = std::abs(global_pose_estimate_->rotation().angle() - best_pose->rotation().angle());
  delta_yaw = normalized_angle(delta_yaw);

  LOG(INFO) << "Initial pose validation: \n delta dist: " << delta_dist << " delta yaw: " << delta_yaw << std::endl;

  if (delta_dist > 0.05 || delta_yaw > 0.1) {
    LOG(ERROR) << "Initial pose is not good.";
    return false;
  }
  return true;

  // 与全局地图匹配的点数大于50%

  // 如果有线特征，则与全局线特征匹配距离小于xx，
}


std::shared_ptr<transform::Rigid2d> PoseInitializer::GetEstimatedPose()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  if (!working_) {
    LOG(ERROR) << "Pose initializer is not working, cannot get initial pose.";
    return nullptr;
  }

  // 返回全局估计位姿的副本
  if (global_pose_estimate_) {
    return std::make_shared<transform::Rigid2d>(*global_pose_estimate_);
  }
  return nullptr;
}

bool PoseInitializer::IsPoseValid(const transform::Rigid2d& pose,
                                  const transform::Rigid2d& target,
                                  const double dist_th,
                                  const double yaw_th)
{
  double dist_diff = (pose.translation() - target.translation()).norm();
  double yaw_diff = std::abs(pose.rotation().angle() - target.rotation().angle());
  yaw_diff = normalized_angle(yaw_diff);
  LOG(INFO) << "dist diff: " << dist_diff << " yaw diff: " << yaw_diff;
  return dist_diff < dist_th && yaw_diff < yaw_th;
}

}  // namespace localization
}  // namespace cartographer
