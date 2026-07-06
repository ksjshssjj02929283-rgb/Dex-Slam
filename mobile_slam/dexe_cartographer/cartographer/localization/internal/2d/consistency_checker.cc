/*
 * Copyright 2025 The Cartographer Authors
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

#include "cartographer/localization/internal/2d/consistency_checker.h"

#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include "cartographer/mapping/2d/probability_grid.h"
#include "glog/logging.h"

namespace cartographer {
namespace localization {

std::string ConsistencyCheckResult::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  
  oss << "ConsistencyCheckResult:\n";
  oss << "  Status: " << (is_consistent ? "PASSED" : "FAILED") << "\n";
  oss << "  Translation Error: " << translation_error << " m\n";
  oss << "  Rotation Error: " << rotation_error << " rad (" 
      << (rotation_error * 180.0 / M_PI) << " deg)\n";
  oss << "  Confidence: " << confidence << "\n";
  
  if (velocity_valid) {
    oss << "  Linear Velocity: " << linear_velocity << " m/s\n";
    oss << "  Angular Velocity: " << angular_velocity << " rad/s (" 
        << (angular_velocity * 180.0 / M_PI) << " deg/s)\n";
  } else {
    oss << "  Velocity: Not available (insufficient history)\n";
  }
  
  if (!is_consistent && !failure_reason.empty()) {
    oss << "  Failure Reason: " << failure_reason;
  }
  
  return oss.str();
}

ConsistencyChecker::ConsistencyChecker(
    const proto::PoseEstimatorOptions2D& options)
    : options_(options),
      current_confidence_(0.0) {}

void ConsistencyChecker::SetRealTimeCorrelativeScanMatcher(
    const mapping::scan_matching::RealTimeCorrelativeScanMatcher2D* matcher) {
  real_time_correlative_scan_matcher_ = matcher;
  LOG(INFO) << "RealTimeCorrelativeScanMatcher set in ConsistencyChecker";
}

void ConsistencyChecker::SetCeresScanMatcher(
    const mapping::scan_matching::CeresScanMatcher2D* matcher) {
  ceres_scan_matcher_ = matcher;
  LOG(INFO) << "CeresScanMatcher set in ConsistencyChecker";
}

ConsistencyCheckResult ConsistencyChecker::CheckPoseConsistency(
    const common::Time& time,
    const transform::Rigid2d& predicted_pose,
    const transform::Rigid2d& estimated_pose,
    const sensor::PointCloud& point_cloud,
    const mapping::Submap2D& local_submap) {
  
  ConsistencyCheckResult result;
  result.is_consistent = true;
  result.failure_reason = "";

  // 从配置获取阈值
  const double translation_threshold = 
      options_.consistency_checker_options().translation_threshold();
  const double rotation_threshold = 
      options_.consistency_checker_options().rotation_threshold();
  const double max_linear_velocity = 
      options_.consistency_checker_options().max_linear_velocity();
  const double max_angular_velocity = 
      options_.consistency_checker_options().max_angular_velocity();
  const double min_score =
      options_.consistency_checker_options().min_score();

  // 1. 检查预测位姿与估计位姿之间的误差
  result.translation_error = 
      (estimated_pose.translation() - predicted_pose.translation()).norm();
  result.rotation_error = 
      std::abs(estimated_pose.rotation().angle() - predicted_pose.rotation().angle());

  // 检查误差是否超过阈值
  if (result.translation_error > translation_threshold) {
    result.is_consistent = false;
    result.failure_reason += "Translation error exceeds threshold (" + 
        std::to_string(result.translation_error) + " > " + 
        std::to_string(translation_threshold) + " m). ";
  }

  if (result.rotation_error > rotation_threshold) {
    result.is_consistent = false;
    result.failure_reason += "Rotation error exceeds threshold (" + 
        std::to_string(result.rotation_error) + " > " + 
        std::to_string(rotation_threshold) + " rad). ";
  }

  // 2. 计算点云与地图的重合度（置信度）
  result.confidence = CalculateMapOverlapConfidence(
      point_cloud, estimated_pose, local_submap);
  current_confidence_ = result.confidence;

  // 如果置信度过低，也认为不一致
  if (result.confidence < min_score) {
    result.is_consistent = false;
    result.failure_reason += "Confidence too low (" + 
        std::to_string(result.confidence) + " < " + 
        std::to_string(min_score) + "). ";
  }

  // 3. 计算速度（基于最近的历史位姿和当前位姿）
  // 如果速度合理，当前位姿会被加入历史记录
  result.velocity_valid = CalculateVelocity(
      time, estimated_pose, &result.linear_velocity, &result.angular_velocity);

  if (result.velocity_valid) {
    // 检查速度是否异常（已经在CalculateVelocity中检查过）
    // 这里再次检查是为了设置一致性结果
    if (result.linear_velocity > max_linear_velocity) {
      result.is_consistent = false;
      result.failure_reason += "Linear velocity too high (" + 
          std::to_string(result.linear_velocity) + " > " + 
          std::to_string(max_linear_velocity) + " m/s). ";
    }

    if (result.angular_velocity > max_angular_velocity) {
      result.is_consistent = false;
      result.failure_reason += "Angular velocity too high (" + 
          std::to_string(result.angular_velocity) + " > " + 
          std::to_string(max_angular_velocity) + " rad/s). ";
    }
  }

  // 输出检查结果
  if (!result.is_consistent) {
    LOG(WARNING) << "Pose consistency check FAILED: " << result.failure_reason
                 << "\nPredicted: " << predicted_pose
                 << "\nEstimated: " << estimated_pose
                 << "\nTranslation error: " << result.translation_error << " m"
                 << "\nRotation error: " << result.rotation_error << " rad"
                 << "\nConfidence: " << result.confidence;
    if (result.velocity_valid) {
      LOG(WARNING) << "Linear velocity: " << result.linear_velocity << " m/s"
                   << "\nAngular velocity: " << result.angular_velocity << " rad/s";
    }
  }

  return result;
}

double ConsistencyChecker::CalculateMapOverlapConfidence(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose,
    const mapping::Submap2D& local_submap) {
  
  const auto* grid = dynamic_cast<const mapping::ProbabilityGrid*>(
      local_submap.grid());
  
  if (!grid || point_cloud.empty()) {
    return 0.0;
  }

  // 将点云转换到地图坐标系，计算置信度
  // 参考 RealTimeCorrelativeScanMatcher2D::ComputeCandidateScore 的方式
  // 使用所有点的概率之和的平均值作为置信度
  const transform::Rigid2f pose_float = pose.cast<float>();
  float candidate_score = 0.f;
  int points_in_map = 0;

  for (const sensor::RangefinderPoint& point : point_cloud) {
    const Eigen::Vector2f point_2d = pose_float * point.position.head<2>();
    
    // 检查点是否在地图范围内
    if (grid->limits().Contains(grid->limits().GetCellIndex(point_2d))) {
      points_in_map++;
      
      // 获取该点在地图中的占用概率并累加
      const float probability = grid->GetProbability(
          grid->limits().GetCellIndex(point_2d));
      candidate_score += probability;
    }
  }

  if (points_in_map == 0) {
    return 0.0;
  }

  // 置信度 = 概率之和 / 地图内点数（平均概率）
  double confidence = static_cast<double>(candidate_score) / points_in_map;
  
  CHECK_GE(confidence, 0.0);
  CHECK_LE(confidence, 1.0);
  
  return confidence;
}

bool ConsistencyChecker::CalculateVelocity(
    const common::Time& current_time,
    const transform::Rigid2d& current_pose,
    double* linear_velocity, 
    double* angular_velocity) {
  
  *linear_velocity = 0.0;
  *angular_velocity = 0.0;

  // 需要至少2帧历史位姿 + 当前位姿（共3帧）才能计算速度
  if (pose_history_.size() < (kMinPosesForVelocity - 1)) {
    // 历史帧数不足，直接加入当前位姿到历史
    AddPoseRecord(current_time, current_pose);
    return false;
  }

  // 使用最近的2帧历史 + 当前位姿计算速度
  // 获取最老的一帧（用于计算速度）
  auto it_oldest = pose_history_.rbegin();
  std::advance(it_oldest, std::min(kMinPosesForVelocity - 2, 
                                    pose_history_.size() - 1));
  
  const PoseRecord& oldest = *it_oldest;

  // 计算时间差
  const double time_diff = common::ToSeconds(current_time - oldest.time);
  
  if (time_diff < 1e-6) {  // 避免除以0
    AddPoseRecord(current_time, current_pose);
    return false;
  }

  // 计算线速度（位移距离 / 时间）
  const double distance = (current_pose.translation() - oldest.pose.translation()).norm();
  *linear_velocity = distance / time_diff;

  // 计算角速度（角度变化 / 时间）
  double angle_diff = current_pose.rotation().angle() - oldest.pose.rotation().angle();
  
  // 归一化角度差到 [-π, π]
  while (angle_diff > M_PI) {
    angle_diff -= 2 * M_PI;
  }
  while (angle_diff < -M_PI) {
    angle_diff += 2 * M_PI;
  }
  
  *angular_velocity = std::abs(angle_diff) / time_diff;

  // 从配置获取速度阈值
  const double max_linear_velocity = 
      options_.consistency_checker_options().max_linear_velocity();
  const double max_angular_velocity = 
      options_.consistency_checker_options().max_angular_velocity();

  // 检查速度是否在合理范围内
  const bool velocity_valid = 
      (*linear_velocity <= max_linear_velocity) && 
      (*angular_velocity <= max_angular_velocity);

  // 只有速度检查通过，才将当前位姿加入历史
  if (velocity_valid) {
    AddPoseRecord(current_time, current_pose);
  } else {
    LOG(WARNING) << "Velocity check failed, not adding pose to history. "
                 << "Linear velocity: " << *linear_velocity << " m/s (max: " 
                 << max_linear_velocity << "), "
                 << "Angular velocity: " << *angular_velocity << " rad/s (max: "
                 << max_angular_velocity << ")";
  }

  return true;
}

void ConsistencyChecker::AddPoseRecord(
    const common::Time& time, const transform::Rigid2d& pose) {
  
  pose_history_.push_back(PoseRecord{time, pose});

  // 保持历史记录在限定大小内
  while (pose_history_.size() > kPoseHistorySize) {
    pose_history_.pop_front();
  }
}

bool ConsistencyChecker::ValidatePoseEstimate(
    const transform::Rigid2d& estimated_pose,
    const sensor::PointCloud& point_cloud,
    const mapping::Submap2D& local_submap) {
  
  // 检查 matcher 是否已设置
  CHECK(real_time_correlative_scan_matcher_ != nullptr)
      << "RealTimeCorrelativeScanMatcher must be set before calling ValidatePoseEstimate";
  
  // 从配置文件读取最小分数阈值
  const double min_score = 
      options_.consistency_checker_options().min_score();

  // 使用real time correlative scan matcher在局部地图上进行验证匹配
  transform::Rigid2d validation_pose = estimated_pose;
  const double validation_score = real_time_correlative_scan_matcher_->Match(
      estimated_pose, 
      point_cloud, 
      *local_submap.grid(),
      &validation_pose);

  // 判断评分是否达到阈值
  const bool is_valid = validation_score >= min_score;

  // 记录验证信息
  if (!is_valid) {
    LOG(WARNING) << "Pose validation failed!"
                 << "\n  Validation score: " << validation_score 
                 << " (threshold: " << min_score << ")"
                 << "\n  Estimated pose: " << estimated_pose
                 << "\n  Validation pose: " << validation_pose;
  }
  
  return is_valid;
}

std::unique_ptr<transform::Rigid2d> ConsistencyChecker::RecoverFromLocalMap(
    const common::Time& time,
    const transform::Rigid2d& validation_pose,
    const sensor::PointCloud& point_cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::shared_ptr<mapping::Submap2D>& local_submap) {
  
  // 检查 matchers 是否已设置
  CHECK(real_time_correlative_scan_matcher_ != nullptr)
      << "RealTimeCorrelativeScanMatcher must be set before calling RecoverFromLocalMap";
  CHECK(ceres_scan_matcher_ != nullptr)
      << "CeresScanMatcher must be set before calling RecoverFromLocalMap";
  
  LOG(WARNING) << "Attempting recovery from local map...";
  
  if (!local_submap) {
    LOG(ERROR) << "Local submap not available for recovery!";
    return nullptr;
  }
  
  // 1. 使用验证位姿作为初始位姿，在局部地图上用real time matcher搜索
  //    注意：因为pose check失败，local map不会被错误更新，所以使用local map是安全的
  transform::Rigid2d recovery_pose = validation_pose;
  const double recovery_score = real_time_correlative_scan_matcher_->Match(
      validation_pose,
      point_cloud,
      *local_submap->grid(),
      &recovery_pose);
  
  LOG(INFO) << "Recovery correlative matching score: " << recovery_score;
  
  // 2. 如果correlative匹配分数太低，直接返回失败（使用统一的最小分数阈值）
  const double min_score = options_.consistency_checker_options().min_score();
  if (recovery_score < min_score) {
    LOG(ERROR) << "Recovery correlative match failed (score: " << recovery_score 
               << ", threshold: " << min_score << ")!";
    return nullptr;
  }
  
  // 3. Correlative匹配成功，使用ceres进行精细优化
  LOG(INFO) << "Recovery correlative match successful, performing ceres refinement...";
  
  // 4. 使用局部地图进行ceres优化
  auto recovery_pose_refined = absl::make_unique<transform::Rigid2d>();
  ceres::Solver::Summary recovery_summary;
  
  ceres_scan_matcher_->Match(
      recovery_pose.translation(),
      recovery_pose,
      point_cloud,
      highlight_cloud,
      *local_submap->grid(),
      *local_submap->cloud_ikdtree(),
      *local_submap->highlight_cloud_ikdtree(),
      recovery_pose_refined.get(),
      &recovery_summary);
  
  // 5. 检查恢复的位姿是否可用
  if (!recovery_summary.IsSolutionUsable() || 
      recovery_summary.final_cost == -1 ||
      std::isinf(recovery_summary.final_cost) || 
      std::isnan(recovery_summary.final_cost))
  {
    LOG(ERROR) << "Recovery ceres optimization failed!";
    return nullptr;
  }
  
  // 6. 恢复成功，校验【恢复后位姿自身的合法性】（而非与恢复前位姿比 delta）
  // ⚠️ 绝不能用 CheckPoseConsistency(validation_pose, refined) 比平移/旋转 delta：
  //    validation_pose 是恢复前的【错误/漂移位姿】，重定位本就要大幅挪位姿，
  //    delta 大恰恰说明恢复在起作用；拿它当误差门会把成功的恢复否决（漂移越大越否决，逻辑反）。
  //    正确做法：校验 refined 位姿与（干净参考）local map 的重合度是否达标。
  LOG(INFO) << "Recovery optimization successful, validating refined pose legitimacy...";

  const double recovery_confidence =
      CalculateMapOverlapConfidence(point_cloud, *recovery_pose_refined, *local_submap);
  LOG(INFO) << "Recovery refined pose overlap confidence: " << recovery_confidence
            << " (min_score=" << min_score << ")";

  // 7. 合法性不足（refined 位姿与地图重合度过低）→ 恢复失败
  if (recovery_confidence < min_score) {
    LOG(ERROR) << "Recovery pose legitimacy check failed: overlap confidence "
               << recovery_confidence << " < " << min_score;
    return nullptr;
  }
  
  // 8. 一致性检查通过，恢复成功
  LOG(WARNING) << "Recovery successful and passed consistency check!"
               << "\n  Recovered pose: " << *recovery_pose_refined;
  
  return recovery_pose_refined;
}

void ConsistencyChecker::Reset() {
  pose_history_.clear();
  current_confidence_ = 0.0;
  consecutive_drift_frames_ = 0;
  LOG(INFO) << "ConsistencyChecker reset";
}

std::string DriftCheckResult::ToString() const {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(4);
  oss << "DriftCheck "
      << (drift_suspected ? "DRIFT" : (judged ? "OK" : "SKIP"))
      << " | static_overlap=" << static_overlap
      << " live_overlap=" << live_overlap
      << " rematch_score=" << static_rematch_score
      << " delta_t=" << pose_delta_trans << "m"
      << " delta_r=" << (pose_delta_rot * 180.0 / M_PI) << "deg"
      << " streak=" << consecutive_frames;
  if (!reason.empty()) {
    oss << " | " << reason;
  }
  return oss.str();
}

DriftCheckResult ConsistencyChecker::CheckMapDrift(
    const transform::Rigid2d& estimated_pose,
    const sensor::PointCloud& point_cloud,
    const mapping::Submap2D& static_submap,
    const mapping::Submap2D& live_local_submap) {
  DriftCheckResult result;
  const auto& opt = options_.consistency_checker_options();

  // 未启用 → 直接放行（drift_suspected=false）
  if (!opt.enable_drift_gate()) {
    consecutive_drift_frames_ = 0;
    return result;
  }

  // static map 必须有可用 grid（这是漂移免疫的底图）
  if (static_submap.grid() == nullptr || point_cloud.empty()) {
    return result;
  }

  // ① 重合度指纹：估计位姿在 static / live 两张图上的重合度。
  //    自洽漂移的特征 = live 高（地图跟着漂、自我确认）但 static 低（static 不漂、对不齐）。
  result.static_overlap =
      CalculateMapOverlapConfidence(point_cloud, estimated_pose, static_submap);
  result.live_overlap =
      CalculateMapOverlapConfidence(point_cloud, estimated_pose, live_local_submap);

  const double static_map_min_score = opt.static_map_min_score();
  const double drift_score_gap = opt.drift_score_gap();

  const bool fingerprint =
      (result.static_overlap < static_map_min_score) &&
      ((result.live_overlap - result.static_overlap) > drift_score_gap);

  if (!fingerprint) {
    // 估计位姿在 static map 上同样自洽 → 无漂移嫌疑，计数清零
    consecutive_drift_frames_ = 0;
    result.judged = true;
    return result;
  }

  // ② 独立确认（仅指纹触发时才跑再匹配，开销低）：在 static map 上独立再定位
  if (real_time_correlative_scan_matcher_ == nullptr) {
    // 无匹配器，无法独立确认；保守不冻结（仅靠指纹在新区域易误判）
    consecutive_drift_frames_ = 0;
    result.reason = "fingerprint hit but no matcher to confirm";
    return result;
  }

  transform::Rigid2d static_aligned = estimated_pose;
  result.static_rematch_score = real_time_correlative_scan_matcher_->Match(
      estimated_pose, point_cloud, *static_submap.grid(), &static_aligned);
  result.rematched_pose = static_aligned;  // 供漂移确认后的重定位复用，省一次匹配

  // static map 在此处覆盖不足（再匹配得分太低）→ 很可能是 static map 未覆盖的新区域/
  // 改变区域，无法判定漂移，跳过（避免在合法的新区域建图时误冻结）
  if (result.static_rematch_score < static_map_min_score) {
    consecutive_drift_frames_ = 0;
    result.reason = "static map cannot localize here (new/changed area), skip";
    return result;  // judged 保持 false
  }

  result.judged = true;

  // 独立再匹配位姿与估计位姿的 delta —— static map 认为"对的位姿"离估计有多远
  result.pose_delta_trans =
      (static_aligned.translation() - estimated_pose.translation()).norm();
  double dtheta =
      static_aligned.rotation().angle() - estimated_pose.rotation().angle();
  while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
  while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  result.pose_delta_rot = std::abs(dtheta);

  const bool delta_exceeds =
      (result.pose_delta_trans > opt.drift_trans_threshold()) ||
      (result.pose_delta_rot > opt.drift_rot_threshold());

  if (delta_exceeds) {
    consecutive_drift_frames_++;
  } else {
    consecutive_drift_frames_ = 0;
  }
  result.consecutive_frames = consecutive_drift_frames_;

  // ③ 去抖：连续 drift_persist_frames 帧确认才判漂移（抗单帧遮挡/对称环境噪声）
  if (consecutive_drift_frames_ >= opt.drift_persist_frames()) {
    result.drift_suspected = true;
    result.reason = "self-consistent drift confirmed by static-map re-localization";
    LOG(WARNING) << "[DriftGate] " << result.ToString();
  }

  return result;
}

}  // namespace localization
}  // namespace cartographer
