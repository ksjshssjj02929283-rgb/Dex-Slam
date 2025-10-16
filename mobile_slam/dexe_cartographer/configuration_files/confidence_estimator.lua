-- Copyright 2025 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- 置信度评估器配置
CONFIDENCE_ESTIMATOR = {
  -- ========== L1层: 扫描匹配内部权重 ==========
  -- 这些权重用于融合L1层内的各项指标
  w_ceres = 0.4,           -- Ceres优化残差权重
  w_match_ratio = 0.3,     -- 匹配点比例权重
  w_covariance = 0.3,      -- 协方差置信度权重
  
  -- ========== 初始化阶段的4层融合权重（推荐更依赖扫描匹配）==========
  init_w_scan_match = 0.6,      -- L1: 扫描匹配层权重（初始化）
  init_w_motion = 0.3,          -- L2: 运动一致性层权重（初始化）
  init_w_map = 0.1,             -- L3: 地图一致性层权重（初始化）
  init_w_degeneracy = 0.0,      -- L4: 几何退化层权重（初始化，建议关闭）
  
  -- ========== 正常运行阶段的4层融合权重（推荐均衡）==========
  normal_w_scan_match = 0.5,    -- L1: 扫描匹配层权重（正常）
  normal_w_motion = 0.3,       -- L2: 运动一致性层权重（正常）
  normal_w_map = 0.1,          -- L3: 地图一致性层权重（正常）
  normal_w_degeneracy = 0.1,    -- L4: 几何退化层权重（正常）
  
  -- ========== 置信度等级阈值 ==========
  excellent_threshold = 0.9,  -- EXCELLENT级别阈值 >= 0.9
  good_threshold = 0.7,       -- GOOD级别阈值 >= 0.7
  medium_threshold = 0.5,     -- MEDIUM级别阈值 >= 0.5
  poor_threshold = 0.4,       -- POOR级别阈值 >= 0.4
  -- < 0.4 为 FAILED

  -- ========== 阶段切换时间 ==========
  -- 初始化阶段持续时间（秒），超过此时间自动切换到正常运行阶段
  initialization_duration_sec = 5.0,
  
  -- ========== 各层使能开关 ==========
  -- 注意: L1扫描匹配是基础评估层，始终启用，不需要配置开关
  enable_motion_check = true,    -- 启用L2: 运动一致性检查
  enable_map_check = true,       -- 启用L3: 地图一致性评估
  enable_degeneracy_check = true, -- 启用L4: 几何退化检测

  -- ========== 关键失败判定阈值 ==========
  -- 用于直接判定定位失败的严重条件，触发时立即返回FAILED级别
  enable_critical_failure_check = true,  -- 启用关键失败检测
  
  -- 扫描匹配相关
  critical_min_match_ratio = 0.15,           -- 最小匹配率（低于此值判定失败）
  critical_max_ceres_residual = 1.0,         -- 最大Ceres平均残差
  critical_min_point_count = 50,             -- 最小有效点云数量
  
  -- 运动一致性相关
  critical_max_pose_jump_distance = 2.0,     -- 最大位姿跳变距离（米）
  critical_max_pose_jump_angle = 0.524,      -- 最大位姿跳变角度（弧度，约30°）
  critical_min_slippage_score = 0.2,         -- 最小打滑置信度（低于此值判定打滑严重）
  
  -- 退化检测相关
  critical_max_hessian_condition = 1000.0,   -- 最大Hessian条件数（走廊环境下）
}

return CONFIDENCE_ESTIMATOR
