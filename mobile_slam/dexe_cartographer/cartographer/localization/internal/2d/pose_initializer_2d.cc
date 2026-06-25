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
  need_global_relocalization_ = false;

  initial_pose_.reset();
  global_pose_estimate_.reset();

  relocalization_thread_.reset();
  curr_cloud_.reset();
  curr_line_data_.reset();
  fast_correlative_scan_matcher_.reset();
  batch_global_matcher_.reset();
#ifdef BUILD_GPU_MATCHER
  gpu_global_matcher_.reset();
#endif
  real_time_correlative_scan_matcher_.reset();
  ceres_scan_matcher_.reset();
  line_matcher_.reset();
  initializer_status_ = PoseInitializerStatus::Idle;
}

void PoseInitializer::InitGlobalSubmap(const std::shared_ptr<mapping::Submap2D>& global_submap)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  global_submap_ = global_submap;
  // 地图更新时重置快速匹配器，下次使用时重新创建
  fast_correlative_scan_matcher_.reset();
  batch_global_matcher_.reset();
#ifdef BUILD_GPU_MATCHER
  gpu_global_matcher_.reset();
#endif
}

bool PoseInitializer::MatchFullSubmapDispatch(
    const mapping::Grid2D& grid, const sensor::PointCloud& cloud,
    const float min_score, float* score, transform::Rigid2d* pose) {
  const auto& fcsm_opts = options_.fast_correlative_scan_matcher_options();
  constexpr int kAngularBlockShift = 2;
  const int threads = options_.global_matcher_num_threads() > 0
                          ? options_.global_matcher_num_threads()
                          : 4;
  auto type = options_.global_matcher_type();
#ifndef BUILD_GPU_MATCHER
  if (type == proto::GLOBAL_MATCHER_GPU_ANGULAR) {
    LOG_FIRST_N(WARNING, 1)
        << "global_matcher_type=gpu but BUILD_GPU_MATCHER is not defined, fallback to CPU matcher.";
    type = proto::GLOBAL_MATCHER_CPU_ANGULAR;
  }
#endif

  switch (type) {
#ifdef BUILD_GPU_MATCHER
    case proto::GLOBAL_MATCHER_GPU_ANGULAR: {
      if (gpu_global_matcher_ == nullptr) {
        // 从 submap 取已载入/已缓存的地图缓存，避免首次现场构建。
        auto stack = global_submap_
                         ? global_submap_->GetOrBuildPrecomputationGridStack(
                               fcsm_opts)
                         : nullptr;
        gpu_global_matcher_ = std::make_unique<
            mapping::scan_matching::GpuBatchFastCorrelativeScanMatcher2D>(
            grid, fcsm_opts, stack);
        gpu_global_matcher_->set_use_angular_branching(true);
        gpu_global_matcher_->set_angular_block_shift(kAngularBlockShift);
        LOG(INFO) << "global matcher ready (GPU, shift=" << kAngularBlockShift
                  << ")";
      }
      return gpu_global_matcher_->MatchFullSubmap(cloud, min_score, score,
                                                  pose);
    }
#endif
    case proto::GLOBAL_MATCHER_CPU_ANGULAR: {
      if (batch_global_matcher_ == nullptr) {
        auto stack = global_submap_
                         ? global_submap_->GetOrBuildPrecomputationGridStack(
                               fcsm_opts)
                         : nullptr;
        batch_global_matcher_ = std::make_unique<
            mapping::scan_matching::BatchFastCorrelativeScanMatcher2D>(
            grid, fcsm_opts, stack);
        batch_global_matcher_->set_use_angular_branching(true);
        batch_global_matcher_->set_angular_block_shift(kAngularBlockShift);
        batch_global_matcher_->set_num_threads(threads);
        LOG(INFO) << "global matcher ready (CPU, shift=" << kAngularBlockShift
                  << ", threads=" << threads << ")";
      }
      return batch_global_matcher_->MatchFullSubmap(cloud, min_score, score,
                                                    pose);
    }
    case proto::GLOBAL_MATCHER_ORIGINAL:
    default: {
      if (fast_correlative_scan_matcher_ == nullptr) {
        fast_correlative_scan_matcher_ = std::make_unique<
            mapping::scan_matching::FastCorrelativeScanMatcher2D>(grid,
                                                                  fcsm_opts);
      }
      return fast_correlative_scan_matcher_->MatchFullSubmap(cloud, min_score,
                                                             score, pose);
    }
  }
}

bool PoseInitializer::SetInitialPose(const transform::Rigid2d &initial_pose)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  // 将3为的位姿变成二维的
  initial_pose_ = std::make_shared<transform::Rigid2d>(initial_pose);
  global_pose_estimate_.reset();
  need_apply_initial_pose_ = true;
  need_global_relocalization_ = false;  // 有初始位姿时，不需要全局重定位
  return true;
}

void PoseInitializer::TriggerGlobalRelocalization()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  initial_pose_.reset();
  global_pose_estimate_.reset();
  need_apply_initial_pose_ = false;
  need_global_relocalization_ = true;  // 触发全局重定位
  LOG(INFO) << "Triggered global relocalization";
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
    if (curr_cloud_ != nullptr && !curr_cloud_->empty()) {
      if (need_apply_initial_pose_) {
        RelocalzationOnce();  // 局部重定位（有初始位姿）
      } else if (need_global_relocalization_) {
        GlobalRelocalzationOnce();  // records info + logs total inside
      }
    }
  }
  
  LOG(INFO) << "PoseInitializer relocalization thread stopped";
}

void PoseInitializer::RelocalzationOnce()
{
  if (global_submap_ == nullptr) {
    LOG(ERROR) << "Global submap is not initialized, please set global submap first.";
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

  // 从 global_submap_ 获取 grid
  auto global_grid = global_submap_->grid();
  if (!global_grid) {
    LOG(ERROR) << "Global grid is not available in submap.";
    initializer_status_ = PoseInitializerStatus::NotInitialized;
    return;
  }

  if (fast_correlative_scan_matcher_ == nullptr) {
    fast_correlative_scan_matcher_ =
        std::make_unique<mapping::scan_matching::FastCorrelativeScanMatcher2D>(
          *global_grid, options_.fast_correlative_scan_matcher_options());
  }

  initializer_status_ = PoseInitializerStatus::LocalEstimating;

  float score;
  if (options_.use_global_matching()) {
    MatchFullSubmapDispatch(
      *global_grid, *curr_cloud_, options_.global_localization_min_score(),
      &score, initial_pose_.get());
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
      *global_grid, &online_correlative_pose);

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
  auto global_line_map = global_submap_->line_map_2d_list();
  if (curr_line_data_->size() != 0 && global_line_map && global_line_map->size() != 0) {
    line_matcher_->TrackLineFromMap(*initial_pose_,
                                    *curr_line_data_,
                                    *global_line_map,
                                    tracked_lane_data);
  }

  auto global_cloud_ikdtree = global_submap_->cloud_ikdtree();
  if (!global_cloud_ikdtree) {
    LOG(ERROR) << "Global cloud ikdtree is not available in submap.";
    initializer_status_ = PoseInitializerStatus::Failed;
    return;
  }

  ceres_scan_matcher_->Match(initial_pose_->translation(),
                            *initial_pose_,
                            *curr_cloud_,
                            tracked_lane_data,
                            *global_grid,
                            *global_line_map,
                            *global_cloud_ikdtree,
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

void PoseInitializer::GlobalRelocalzationOnce()
{
  using Clock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const Clock::time_point& s) {
    return std::chrono::duration<double, std::milli>(Clock::now() - s).count();
  };
  const auto t_total_start = Clock::now();

  // 每次执行都把本次信息记进成员，调用方用 GetLastGlobalRelocInfo() 读取。
  last_global_reloc_info_ = GlobalRelocInfo{};
  GlobalRelocInfo& info = last_global_reloc_info_;
  info.call_index = ++global_reloc_call_count_;  // M1 profiling：1 = 冷启动首帧

  if (global_submap_ == nullptr) {
    LOG(ERROR) << "Global submap is not initialized, please set global submap first.";
    initializer_status_ = PoseInitializerStatus::NotInitialized;
    return;
  }

  if (curr_cloud_ == nullptr || curr_cloud_->empty()) {
    return;
  }

  if (!need_global_relocalization_) {
    LOG(WARNING) << "Global relocalization is not triggered.";
    return;
  }

  // 从 global_submap_ 获取 grid
  auto global_grid = global_submap_->grid();
  if (!global_grid) {
    LOG(ERROR) << "Global grid is not available in submap.";
    initializer_status_ = PoseInitializerStatus::NotInitialized;
    return;
  }

  // M1 profiling：原版 FCSM 构造会建完整金字塔（不读 pbstream 固化），仅 validate 使用，
  // 首次定位时落在 t_bbs 之前的未计时区——单独计时确认是否为 1126ms 缺口主因。
  const auto t_fcsm_build = Clock::now();
  if (fast_correlative_scan_matcher_ == nullptr) {
    fast_correlative_scan_matcher_ =
        std::make_unique<mapping::scan_matching::FastCorrelativeScanMatcher2D>(
          *global_grid, options_.fast_correlative_scan_matcher_options());
  }
  info.t_fcsm_build_ms = elapsed_ms(t_fcsm_build);

  initializer_status_ = PoseInitializerStatus::GlobalEstimating;

  // 全局重定位：使用MatchFullSubmap进行全图匹配
  LOG(INFO) << "Starting global relocalization...";
  
  // 创建一个临时位姿用于存储全局匹配结果
  auto global_match_pose = std::make_shared<transform::Rigid2d>();
  float global_score = 0.0;

  // M1 profiling：t_total_start -> 此处的整个准备区（检查 + grid 获取 + 匹配器构造），
  // 5 段计时不覆盖它，缺口若在这里，t_setup_ms 会接近 ~1126ms。
  info.t_setup_ms = elapsed_ms(t_total_start);

  const auto t_bbs = Clock::now();
  bool global_match_success = MatchFullSubmapDispatch(
    *global_grid,
    *curr_cloud_,
    options_.global_localization_min_score(),
    &global_score,
    global_match_pose.get());
  info.t_bbs_ms = elapsed_ms(t_bbs);
  info.bbs_score = global_score;

  LOG(INFO) << "Global localization score: " << global_score;

  if (!global_match_success || global_score < options_.global_localization_min_score()) {
    LOG(ERROR) << "Global relocalization failed, score too low: " << global_score;
    initializer_status_ = PoseInitializerStatus::Failed;
    info.t_total_ms = elapsed_ms(t_total_start);
    return;
  }

  LOG(INFO) << "Global match pose: " << *global_match_pose;

  // 使用全局匹配的结果作为初始位姿进行精细匹配
  initial_pose_ = global_match_pose;

  // 进行在线相关扫描匹配以改进位姿
  const auto t_rtcsm = Clock::now();
  transform::Rigid2d online_correlative_pose;
  if (options_.use_online_correlative_scan_matching()) {
    const double score = real_time_correlative_scan_matcher_->Match(
      *initial_pose_, *curr_cloud_,
      *global_grid, &online_correlative_pose);

    if (score < options_.global_localization_min_score()) {
      LOG(WARNING) << "Online correlative scan matcher score is too low: " << score;
      // 不直接失败，继续使用全局匹配结果
    } else {
      LOG(INFO) << "Online correlative scan matcher improved pose, score: " << score;
      initial_pose_ = std::make_shared<transform::Rigid2d>(online_correlative_pose);
    }
  }
  info.t_rtcsm_ms = elapsed_ms(t_rtcsm);

  // 进行Ceres精细优化
  global_pose_estimate_ = std::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary summary;
  
  // 使用线特征进行匹配
  if (curr_line_data_->empty()) {
    LOG(WARNING) << "No line data in global scan matcher.";
  }
  
  const auto t_line = Clock::now();
  std::map<int, std::vector<sensor::LineData>> tracked_lane_data;
  auto global_line_map = global_submap_->line_map_2d_list();
  if (curr_line_data_->size() != 0 && global_line_map && global_line_map->size() != 0) {
    line_matcher_->TrackLineFromMap(*initial_pose_,
                                    *curr_line_data_,
                                    *global_line_map,
                                    tracked_lane_data);
  }
  info.t_line_ms = elapsed_ms(t_line);

  const auto t_ceres = Clock::now();
  auto global_cloud_ikdtree = global_submap_->cloud_ikdtree();
  if (!global_cloud_ikdtree) {
    LOG(ERROR) << "Global cloud ikdtree is not available in submap.";
    initializer_status_ = PoseInitializerStatus::Failed;
    info.t_total_ms = elapsed_ms(t_total_start);
    return;
  }

  ceres_scan_matcher_->Match(initial_pose_->translation(),
                            *initial_pose_,
                            *curr_cloud_,
                            tracked_lane_data,
                            *global_grid,
                            *global_line_map,
                            *global_cloud_ikdtree,
                            global_pose_estimate_.get(),
                            &summary);
  info.t_ceres_ms = elapsed_ms(t_ceres);

  const auto t_validate = Clock::now();
  const bool reloc_ok = IsRelocalizationSuccess();
  info.t_validate_ms = elapsed_ms(t_validate);

  if (!reloc_ok) {
    global_pose_estimate_.reset();
    initializer_status_ = PoseInitializerStatus::Failed;
    LOG(ERROR) << "Global relocalization validation failed";
  } else {
    initializer_status_ = PoseInitializerStatus::Success;
    need_global_relocalization_ = false;  // 全局重定位成功后重置标志
    LOG(INFO) << "Global relocalization succeeded! Final pose: " << *global_pose_estimate_;
    info.pose_estimate = *global_pose_estimate_;
  }

  LOG(INFO) << "Global relocalization info: "
            << "call_index=" << info.call_index << ", "
            << "t_setup_ms=" << info.t_setup_ms << ", "
            << "t_fcsm_build_ms=" << info.t_fcsm_build_ms << ", "
            << "t_bbs_ms=" << info.t_bbs_ms << ", "
            << "t_rtcsm_ms=" << info.t_rtcsm_ms << ", "
            << "t_line_ms=" << info.t_line_ms << ", "
            << "t_ceres_ms=" << info.t_ceres_ms << ", "
            << "t_validate_ms=" << info.t_validate_ms << ", "
            << "t_validate_fcsm_ms=" << info.t_validate_fcsm_ms << ", "
            << "t_validate_rtcsm_ms=" << info.t_validate_rtcsm_ms;

  const double total_ms = elapsed_ms(t_total_start);
  info.t_total_ms = total_ms;
  info.matched = (initializer_status_ == PoseInitializerStatus::Success);
  LOG(INFO) << "Global relocalization took " << total_ms << " ms";
}

bool PoseInitializer::IsRelocalizationSuccess()
{
  // M1 profiling：拆开 validate 的 362ms——原版 FCSM 全局重匹配 vs RTCSM 各占多少。
  using Clock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const Clock::time_point& s) {
    return std::chrono::duration<double, std::milli>(Clock::now() - s).count();
  };

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
  const auto t_validate_fcsm = Clock::now();
  bool fast_match_success =
            fast_correlative_scan_matcher_->Match(
                  *global_pose_estimate_,
                  *curr_cloud_,
                  options_.global_localization_min_score(),
                  &score,
                  best_pose.get());
  last_global_reloc_info_.t_validate_fcsm_ms = elapsed_ms(t_validate_fcsm);

  if (fast_match_success &&
      score < options_.global_localization_min_score())
  {
    LOG(INFO) << "Initial pose is not good. curren score is too small in validation: " << score;
    return false;
  }

  transform::Rigid2d online_correlative_pose;
  if (options_.use_online_correlative_scan_matching()) {
    auto global_grid = global_submap_->grid();
    if (!global_grid) {
      LOG(ERROR) << "Global grid is not available in submap for validation.";
      return false;
    }
    
    const auto t_validate_rtcsm = Clock::now();
    score = real_time_correlative_scan_matcher_->Match(
      *global_pose_estimate_, *curr_cloud_,
      *global_grid, best_pose.get());
    last_global_reloc_info_.t_validate_rtcsm_ms = elapsed_ms(t_validate_rtcsm);

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
