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

#include "cartographer/localization/confidence/confidence_estimator_options.h"

namespace cartographer {
namespace localization {
namespace confidence {

proto::ConfidenceEstimatorOptions CreateConfidenceEstimatorOptions(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ConfidenceEstimatorOptions options;
  
  // ========== L1层: 扫描匹配内部权重 ==========
  options.set_w_ceres(parameter_dictionary->GetDouble("w_ceres"));
  options.set_w_match_ratio(parameter_dictionary->GetDouble("w_match_ratio"));
  options.set_w_covariance(parameter_dictionary->GetDouble("w_covariance"));
  
  // ========== 初始化阶段的4层融合权重 ==========
  options.set_init_w_scan_match(parameter_dictionary->GetDouble("init_w_scan_match"));
  options.set_init_w_motion(parameter_dictionary->GetDouble("init_w_motion"));
  options.set_init_w_map(parameter_dictionary->GetDouble("init_w_map"));
  options.set_init_w_degeneracy(parameter_dictionary->GetDouble("init_w_degeneracy"));
  
  // ========== 正常运行阶段的4层融合权重 ==========
  options.set_normal_w_scan_match(parameter_dictionary->GetDouble("normal_w_scan_match"));
  options.set_normal_w_motion(parameter_dictionary->GetDouble("normal_w_motion"));
  options.set_normal_w_map(parameter_dictionary->GetDouble("normal_w_map"));
  options.set_normal_w_degeneracy(parameter_dictionary->GetDouble("normal_w_degeneracy"));
  
  // ========== 置信度等级阈值 ==========
  options.set_excellent_threshold(parameter_dictionary->GetDouble("excellent_threshold"));
  options.set_good_threshold(parameter_dictionary->GetDouble("good_threshold"));
  options.set_medium_threshold(parameter_dictionary->GetDouble("medium_threshold"));
  options.set_poor_threshold(parameter_dictionary->GetDouble("poor_threshold"));
  
  // ========== 阶段切换时间 ==========
  options.set_initialization_duration_sec(
      parameter_dictionary->GetDouble("initialization_duration_sec"));
  
  // ========== 各层使能开关 ==========
  // 注意：L1扫描匹配是基础层，始终启用，不需要配置
  options.set_enable_motion_check(parameter_dictionary->GetBool("enable_motion_check"));
  options.set_enable_map_check(parameter_dictionary->GetBool("enable_map_check"));
  options.set_enable_degeneracy_check(parameter_dictionary->GetBool("enable_degeneracy_check"));
  
  // ========== 关键失败判定阈值 ==========
  options.set_enable_critical_failure_check(
      parameter_dictionary->GetBool("enable_critical_failure_check"));
  options.set_critical_min_match_ratio(
      parameter_dictionary->GetDouble("critical_min_match_ratio"));
  options.set_critical_max_pose_jump_distance(
      parameter_dictionary->GetDouble("critical_max_pose_jump_distance"));
  options.set_critical_max_pose_jump_angle(
      parameter_dictionary->GetDouble("critical_max_pose_jump_angle"));
  options.set_critical_min_slippage_score(
      parameter_dictionary->GetDouble("critical_min_slippage_score"));
  options.set_critical_min_point_count(
      parameter_dictionary->GetInt("critical_min_point_count"));
  options.set_critical_max_ceres_residual(
      parameter_dictionary->GetDouble("critical_max_ceres_residual"));
  options.set_critical_max_hessian_condition(
      parameter_dictionary->GetDouble("critical_max_hessian_condition"));
  
  return options;
}

}  // namespace confidence
}  // namespace localization
}  // namespace cartographer
