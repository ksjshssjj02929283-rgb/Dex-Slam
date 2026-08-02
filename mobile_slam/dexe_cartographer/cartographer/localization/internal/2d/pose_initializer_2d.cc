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

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

#include "absl/memory/memory.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
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
  using Clock = std::chrono::steady_clock;
  const auto t_total_start = Clock::now();

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

  const double total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - t_total_start)
          .count();
  LOG(INFO) << "Pose relocalization took " << total_ms << " ms";

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

  // M2（DEC-002）：原版 FCSM 仅旧 validate 的全局重匹配在用，已被轻量打分（ScorePoseAgainstGrid）
  // 替代 → GlobalReloc 路径不再构造它，省去冷启动 ~508ms 金字塔构建（M1 实测 t_fcsm_build）。
  // t_fcsm_build_ms 保持 0（不再发生）。该成员在 original 匹配器路径 / RelocalzationOnce 仍按需懒构造，保留。
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

  // 存下 BBS 全局解供校验门②复用（DEC-003，避免重跑确定性全局搜索）。
  last_global_bbs_pose_ = *global_match_pose;

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
  // 校验（DEC-003 方案 A）：位姿已由 BBS→RTCSM→Ceres 算出。三道门「得分 + 位姿一致性都看」：
  //   门① 绝对质量：最终位姿打分 ≥ min_score
  //   门② 全局-精配一致性：最终位姿 vs 主链路 BBS 全局解的 delta（复用 last_global_bbs_pose_，
  //       零额外搜索——加速 BBS 确定性，重跑 MatchFullSubmap 会返回同一位姿，故不重跑）
  //   门③ 局部稳定性：以最终位姿为初值 RTCSM 0.5m 重搜，确认仍收敛同一处且分够高
  // 旧实现（原版未加速 FCSM 5m 重搜 + RTCSM）实测 1670ms/帧，本版预计 ~RTCSM 一次（~百 ms）。
  // ⚠️ 下面 delta 阈值为初值，需 M3 用真实成/败样本标定。
  using Clock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const Clock::time_point& s) {
    return std::chrono::duration<double, std::milli>(Clock::now() - s).count();
  };
  // 门②比的是粗 BBS 解 vs 精位姿 → 放宽到「发散级」；门③比精位姿 vs RTCSM 精修 → 沿用原校验紧阈值。
  constexpr double kBbsConsistencyDistThresh = 0.30;   // m，门②（BBS 粗解，放宽）
  constexpr double kBbsConsistencyYawThresh = 0.15;    // rad，门②
  constexpr double kLocalStabilityDistThresh = 0.05;   // m，门③（沿用原校验）
  constexpr double kLocalStabilityYawThresh = 0.1;     // rad，门③

  if (global_pose_estimate_ == nullptr) {
    LOG(WARNING) << "Has no estimated global pose.";
    return false;
  }
  if (curr_cloud_ == nullptr || curr_cloud_->empty()) {
    LOG(WARNING) << "No current cloud for relocalization validation.";
    return false;
  }
  const mapping::Grid2D* global_grid = global_submap_->grid();
  if (!global_grid) {
    LOG(ERROR) << "Global grid is not available in submap for validation.";
    return false;
  }

  const float min_score = options_.global_localization_min_score();

  // 门①：绝对质量
  const float final_score =
      ScorePoseAgainstGrid(*global_pose_estimate_, *curr_cloud_, *global_grid);
  if (final_score < min_score) {
    LOG(ERROR) << "Validation gate1 failed: final pose score " << final_score
               << " < " << min_score;
    return false;
  }

  // 门②：最终位姿 vs 主链路 BBS 全局解的一致性（复用，零额外搜索）
  const double bbs_dist =
      (last_global_bbs_pose_.translation() - global_pose_estimate_->translation()).norm();
  double bbs_yaw = std::abs(last_global_bbs_pose_.rotation().angle() -
                            global_pose_estimate_->rotation().angle());
  bbs_yaw = normalized_angle(bbs_yaw);
  if (bbs_dist > kBbsConsistencyDistThresh || bbs_yaw > kBbsConsistencyYawThresh) {
    LOG(ERROR) << "Validation gate2 failed: refined pose diverged from BBS global "
               << "solution, dist=" << bbs_dist << " yaw=" << bbs_yaw;
    return false;
  }

  // 门③：局部稳定性——以最终位姿为初值 RTCSM 0.5m 重搜
  if (options_.use_online_correlative_scan_matching()) {
    const auto t_validate_rtcsm = Clock::now();
    transform::Rigid2d rtcsm_pose;
    const double rtcsm_score = real_time_correlative_scan_matcher_->Match(
        *global_pose_estimate_, *curr_cloud_, *global_grid, &rtcsm_pose);
    last_global_reloc_info_.t_validate_rtcsm_ms = elapsed_ms(t_validate_rtcsm);

    if (rtcsm_score < min_score) {
      LOG(ERROR) << "Validation gate3 failed: RTCSM recheck score " << rtcsm_score
                 << " < " << min_score;
      return false;
    }
    const double rtcsm_dist =
        (rtcsm_pose.translation() - global_pose_estimate_->translation()).norm();
    double rtcsm_yaw = std::abs(rtcsm_pose.rotation().angle() -
                                global_pose_estimate_->rotation().angle());
    rtcsm_yaw = normalized_angle(rtcsm_yaw);
    if (rtcsm_dist > kLocalStabilityDistThresh ||
        rtcsm_yaw > kLocalStabilityYawThresh) {
      LOG(ERROR) << "Validation gate3 failed: RTCSM recheck pose unstable, dist="
                 << rtcsm_dist << " yaw=" << rtcsm_yaw;
      return false;
    }
  }

  LOG(INFO) << "Relocalization validation passed: final_score=" << final_score
            << " bbs_dist=" << bbs_dist << " bbs_yaw=" << bbs_yaw;
  return true;
}

// 轻量位姿打分（M2, DEC-002）：镜像 RTCSM 的 ComputeCandidateScore，在 offset=0（位姿本身）
// 求扫描点平均占据概率。不做任何搜索，复杂度 O(点数)，实测从 1670ms/帧降到数 ms。
float PoseInitializer::ScorePoseAgainstGrid(
    const transform::Rigid2d& pose,
    const sensor::PointCloud& point_cloud,
    const mapping::Grid2D& grid) const
{
  if (point_cloud.empty()) {
    return 0.f;
  }
  const mapping::MapLimits& limits = grid.limits();

  if (grid.GetGridType() == mapping::GridType::TSDF) {
    const auto& tsdf = static_cast<const mapping::TSDF2D&>(grid);
    float score = 0.f;
    float summed_weight = 0.f;
    for (const sensor::RangefinderPoint& point : point_cloud) {
      const Eigen::Vector2d world =
          pose * Eigen::Vector2d(point.position.x(), point.position.y());
      const Eigen::Array2i cell = limits.GetCellIndex(world.cast<float>());
      const std::pair<float, float> tsd_and_weight = tsdf.GetTSDAndWeight(cell);
      const float normalized_tsd_score =
          (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
          tsdf.GetMaxCorrespondenceCost();
      score += normalized_tsd_score * tsd_and_weight.second;
      summed_weight += tsd_and_weight.second;
    }
    return summed_weight == 0.f ? 0.f : score / summed_weight;
  }

  // 默认 ProbabilityGrid：平均占据概率（与 global_localization_min_score 同尺度）。
  const auto& probability_grid = static_cast<const mapping::ProbabilityGrid&>(grid);
  float score = 0.f;
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const Eigen::Vector2d world =
        pose * Eigen::Vector2d(point.position.x(), point.position.y());
    const Eigen::Array2i cell = limits.GetCellIndex(world.cast<float>());
    score += probability_grid.GetProbability(cell);  // 越界内部返回 kMinProbability
  }
  return score / static_cast<float>(point_cloud.size());
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
