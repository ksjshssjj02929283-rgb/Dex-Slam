# 定位置信度评估模块

## 概述

本模块为Cartographer SLAM系统提供了完整的2D激光定位置信度评估功能，用于实时评估机器人定位的可靠性。采用**四层评估架构 + 关键失败检测**机制，确保定位质量。

## 核心特性

- **四层置信度评估**: 扫描匹配、运动一致性、地图连贯性、退化检测
- **关键失败检测**: 直接判定严重定位失败的快速检测机制
- **两阶段参数**: 初始化阶段和正常运行阶段的自适应权重
- **Lua配置化**: 所有参数通过lua文件配置，方便调优
- **Optional返回**: 使用std::optional避免无效数据污染结果

## 架构设计

### Layer 1: 扫描匹配置信度评估 (ScanMatchConfidence)

评估激光扫描与地图的匹配质量：

- **Ceres置信度**: 基于Ceres优化器的残差分析
- **匹配率**: 激光点与地图匹配的比例
- **协方差置信度**: 位姿协方差矩阵的可靠性
- **角度覆盖度**: 匹配点的角度分布熵

文件: `scan_match_confidence.h/cc`

### Layer 2: 运动一致性检查 (MotionConsistencyChecker)

通过里程计预测与观测位姿对比评估定位一致性：

- **预测-观测一致性**: 预测位姿与扫描匹配位姿的偏差
- **速度平滑度**: 线速度和角速度的时间平滑性
- **打滑检测**: 多指标检测车轮打滑或运动异常

文件: `motion_consistency_checker.h/cc`

### Layer 3: 地图连贯性评估 (MapCoherenceEvaluator)

评估定位点与地图环境的一致性：

- **子图匹配得分**: 当前扫描与匹配子图的相似度
- **地图熵**: 局部地图的信息丰富度（高熵环境定位更可靠）

文件: `map_coherence_evaluator.h/cc`

### Layer 4: 退化检测 (DegeneracyDetector)

检测可能导致定位不可靠的退化环境：

- **走廊检测**: 识别平行墙壁等几何退化场景
- **特征分布**: 评估激光特征的空间分布均匀性
- **Hessian分析**: 评估优化问题的可观性

文件: `degeneracy_detector.h/cc`

### 关键失败检测 (CheckCriticalFailure)

**新功能**: 在四层评估之后，额外进行快速严重失败判定：

#### 检测条件

1. **点云数量过少**: `point_count < critical_min_point_count` (默认50)
2. **极低匹配率**: `match_ratio < critical_min_match_ratio` (默认0.15)
3. **Ceres残差过大**: `mean_residual > critical_max_ceres_residual` (默认1.0)
4. **位姿跳变过大**: 
   - 距离跳变 > `critical_max_pose_jump_distance` (默认2.0m)
   - 角度跳变 > `critical_max_pose_jump_angle` (默认30°)
5. **严重打滑**: `motion_consistency < critical_min_slippage_score` (默认0.2)
6. **走廊环境退化**: 走廊 + `degeneracy_factor < 0.2`

#### 触发效果

- **立即返回**: 一旦检测到关键失败，立即停止评估
- **强制FAILED**: 置信度强制设为0.0，等级设为FAILED
- **详细日志**: 输出所有触发的失败条件

```cpp
// 使用示例
auto metrics = confidence_estimator_->Evaluate(...);

if (metrics.level == ConfidenceLevel::FAILED) {
  // 触发了关键失败条件或总置信度极低
  LOG(ERROR) << "Localization FAILED! Confidence: " << metrics.total_confidence;
  // 立即触发重定位
  TriggerRelocalization();
}
```

## 主类: ConfidenceEstimator

综合四层评估结果，提供统一的置信度接口：

```cpp
// 主评估接口
Metrics Evaluate(
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose_estimate,
    const transform::Rigid2d& pose_prediction,
    const Eigen::Matrix3d& covariance,
    double ceres_final_cost,
    int num_ceres_residuals,
    const std::vector<sensor::LineData>& line_data = {},
    const mapping::Grid2D* matching_grid = nullptr,
    double runtime_seconds = 0.0,
    bool compute_uncertainty = false);

// 关键失败检测接口
CriticalFailureInfo CheckCriticalFailure(
    const Metrics& metrics,
    const sensor::PointCloud& point_cloud,
    const transform::Rigid2d& pose_prediction,
    const transform::Rigid2d& pose_estimate,
    double ceres_final_cost,
    int num_ceres_residuals) const;
```

### 输出

返回 `Metrics` 结构，包含：

- **total_confidence**: 总置信度 [0, 1]
- **level**: 置信度等级
  - EXCELLENT (≥0.9): 优秀
  - GOOD (≥0.8): 良好  
  - MEDIUM (≥0.6): 中等
  - POOR (≥0.4): 较差
  - FAILED (<0.4 或触发关键失败): 失败
- **各层详细得分**: 使用 std::optional 存储，未启用层返回 nullopt

**注意**: 当前配置中阈值已调整为更严格的标准（0.9/0.8/0.6/0.4），相比默认值（0.8/0.6/0.4/0.2）更高。

## 配置参数 (通过Lua文件)

配置文件位置: `configuration_files/confidence_estimator.lua`

### 完整配置示例

```lua
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
  normal_w_scan_match = 0.3,    -- L1: 扫描匹配层权重（正常）
  normal_w_motion = 0.25,       -- L2: 运动一致性层权重（正常）
  normal_w_map = 0.35,          -- L3: 地图一致性层权重（正常）
  normal_w_degeneracy = 0.1,    -- L4: 几何退化层权重（正常）
  
  -- ========== 置信度等级阈值 ==========
  excellent_threshold = 0.9,  -- EXCELLENT级别阈值 >= 0.9
  good_threshold = 0.8,       -- GOOD级别阈值 >= 0.8
  medium_threshold = 0.6,     -- MEDIUM级别阈值 >= 0.6
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
}
```

**说明**:
- 当前配置**未包含**关键失败检测参数（这些参数已在proto中定义，使用默认值）
- 如需自定义关键失败阈值，可在lua中添加对应参数（见下节）

### 关键失败检测参数（可选配置）

如需自定义关键失败检测阈值，可在lua配置中添加以下参数：

```lua
-- ========== 关键失败判定阈值（可选）==========
-- 如不配置，将使用proto中的默认值
enable_critical_failure_check = true,  -- 启用关键失败检测（默认true）

-- 扫描匹配相关
critical_min_match_ratio = 0.15,       -- 最小匹配率（默认0.15，低于此值判定失败）
critical_max_ceres_residual = 1.0,     -- 最大Ceres平均残差（默认1.0）
critical_min_point_count = 50,         -- 最小有效点云数量（默认50）

-- 运动一致性相关
critical_max_pose_jump_distance = 2.0, -- 最大位姿跳变距离（米，默认2.0）
critical_max_pose_jump_angle = 0.524,  -- 最大位姿跳变角度（弧度，约30°，默认0.524）
critical_min_slippage_score = 0.2,     -- 最小打滑置信度（默认0.2，低于此值判定打滑严重）

-- 退化检测相关
critical_max_hessian_condition = 1000.0, -- 最大Hessian条件数（走廊环境，默认1000.0）
```

**Proto默认值**:
- 如果lua中未配置这些参数，系统将使用proto3的默认值
- Proto3默认值: `bool = false`, `double = 0.0`, `int32 = 0`
- **建议**: 在lua中显式配置所有参数，避免使用proto默认值导致的逻辑错误

## 使用流程

### 1. 在 pose_estimator_options_2d.proto 中配置

```protobuf
import "cartographer/localization/proto/confidence_estimator_options.proto";

message PoseEstimatorOptions2D {
  // ...其他配置
  cartographer.localization.proto.ConfidenceEstimatorOptions confidence_estimator_options = 25;
}
```

### 2. 在 pose_estimator_2d.lua 中引入配置

```lua
include "confidence_estimator.lua"

POSE_ESTIMATOR_2D = {
  -- ...其他配置
  confidence_estimator = CONFIDENCE_ESTIMATOR,
}
```

### 3. 在PoseEstimator2D中集成

```cpp
// 初始化
confidence_estimator_ = std::make_unique<confidence::ConfidenceEstimator>(
    options.confidence_estimator_options());

// 在ScanMatch中调用
auto confidence_metrics = confidence_estimator_->Evaluate(
    filtered_point_cloud,
    *pose_observation,
    pose_prediction,
    covariance,
    summary.final_cost,
    static_cast<int>(filtered_point_cloud.size()),
    line_data,
    matching_submap->grid(),
    runtime_seconds);

// 处理结果
if (confidence_metrics.level == confidence::ConfidenceLevel::FAILED) {
  LOG(ERROR) << "Localization FAILED! Triggering relocalization...";
  // 触发重定位逻辑
  TriggerGlobalRelocalization();
} else if (confidence_metrics.level <= confidence::ConfidenceLevel::POOR) {
  LOG(WARNING) << "Low confidence: " << confidence_metrics.total_confidence;
  // 降低速度或增加扫描频率
}
```

## 两阶段参数设计

系统根据 `runtime_seconds` 自动选择参数：

### 初始化阶段 (0 ~ initialization_duration_sec 秒)

**特点**: 运动模型尚未建立，地图覆盖不充分

**参数策略**:
```lua
init_w_scan_match = 0.6    -- 高度依赖扫描匹配（主导）
init_w_motion = 0.3        -- 辅助参考
init_w_map = 0.1           -- 最小权重
init_w_degeneracy = 0.0    -- 建议关闭（避免误判）
```

**适用场景**:
- 机器人刚启动/重定位后
- 地图切换后
- 长时间静止后重新移动
- 运动历史数据不足

### 正常运行阶段 (> initialization_duration_sec 秒)

**特点**: 运动模型稳定，地图覆盖充分

**参数策略**:
```lua
normal_w_scan_match = 0.3   -- 基础权重
normal_w_motion = 0.25      -- 运动连续性
normal_w_map = 0.35         -- 略高权重（强调全局一致性）
normal_w_degeneracy = 0.1   -- 启用特殊场景检测
```

**适用场景**:
- 稳定运行状态
- 地图已充分探索
- 运动轨迹平滑
- 多层评估数据完整

**阶段切换**:
- 自动切换: 基于 `runtime_seconds` 参数
- 当前配置: `initialization_duration_sec = 5.0` 秒
- 无缝过渡: 切换时无跳变，权重直接替换

## 关键失败检测的优势

1. **快速响应**: 在四层评估后立即检测严重问题，避免误判
2. **明确判定**: 直接返回FAILED级别，触发明确的失败处理逻辑
3. **灵活配置**: 
   - 可通过lua自定义所有阈值
   - 可通过 `enable_critical_failure_check = false` 完全禁用
4. **详细反馈**: 输出所有触发的失败条件，便于调试和分析
5. **防止误导**: 避免低置信度但未达到严重失败的模糊状态

### 检测时机

```
Evaluate() 执行流程:
1. 执行L1-L4各层评估
2. 加权融合得到 total_confidence
3. 执行 CheckCriticalFailure()  ← 关键失败检测点
4. 如果触发关键失败:
   - 强制 total_confidence = 0.0
   - 强制 level = FAILED
   - 立即返回，跳过后续处理
5. 否则，继续正常流程
```

### 实际效果

- **防止定位飘移**: 检测到位姿大幅跳变时立即标记失败
- **防止打滑误差累积**: 检测到严重打滑时立即标记失败
- **防止无效匹配**: 匹配率过低或点云过少时立即标记失败
- **防止退化环境误差**: 走廊等退化环境下结合Hessian分析判定失败

## 性能特点与优化

### 计算性能

- **四层评估**: 每层独立计算，支持按需启用/禁用
- **关键失败优先**: 在融合后立即检测，避免后续无效处理
- **Optional返回**: 未启用的层不进行计算，减少开销
- **缓存机制**: 运动一致性检查器维护历史数据窗口

### 内存开销

- **轻量级结构**: Metrics使用std::optional，未启用层零开销
- **固定窗口**: 速度历史窗口固定大小（默认10帧）
- **无动态分配**: 评估过程中无额外内存分配

### 日志控制

```cpp
// VLOG级别分级日志
VLOG(2) << "Confidence Evaluation (INIT phase): ...";  // 详细评估信息
LOG(WARNING) << "Low confidence: ...";                  // 低置信度警告
LOG(ERROR) << "❌ CRITICAL FAILURE DETECTED";           // 关键失败错误
```

**日志配置**:
```bash
# 启用详细日志
--vmodule=confidence_estimator=2

# 禁用详细日志（仅显示WARNING和ERROR）
--vmodule=confidence_estimator=0
```

### 实时性能指标

典型场景下的执行时间（参考值，取决于硬件）：
- L1扫描匹配评估: ~0.1-0.3 ms
- L2运动一致性检查: ~0.05-0.1 ms  
- L3地图连贯性评估: ~0.2-0.5 ms
- L4退化检测: ~0.3-0.8 ms
- 关键失败检测: ~0.02-0.05 ms
- **总计**: ~0.7-1.8 ms

**优化建议**:
- 初始化阶段禁用L4（`init_w_degeneracy = 0.0`）
- 简单环境可禁用L3（`enable_map_check = false`）
- 使用VLOG(2)避免生产环境日志开销

## 编译

模块已集成到cartographer的CMakeLists.txt中，通过全局编译自动包含：

```bash
colcon build --packages-select cartographer
```

## 依赖

- Eigen3
- Ceres Solver
- Cartographer Core (mapping, sensor, transform等模块)

## 文件结构

```
localization/confidence/
├── README.md                          # 本文档
├── confidence_estimator.h             # 主类头文件
├── confidence_estimator.cc            # 主类实现
├── confidence_estimator_options.h     # Options加载头文件
├── confidence_estimator_options.cc    # Options加载实现
├── scan_match_confidence.h            # Layer 1: 扫描匹配评估
├── scan_match_confidence.cc
├── motion_consistency_checker.h       # Layer 2: 运动一致性
├── motion_consistency_checker.cc
├── map_coherence_evaluator.h          # Layer 3: 地图连贯性
├── map_coherence_evaluator.cc
├── degeneracy_detector.h              # Layer 4: 退化检测
└── degeneracy_detector.cc

localization/proto/
└── confidence_estimator_options.proto # Proto配置定义

configuration_files/
└── confidence_estimator.lua # 估计器配置文件

```

## 调优建议

### 置信度等级阈值调整

**当前配置（严格模式）**:
```lua
excellent_threshold = 0.9   -- 非常严格
good_threshold = 0.8
medium_threshold = 0.6
poor_threshold = 0.4
```

**推荐配置（平衡模式）**:
```lua
excellent_threshold = 0.8   -- 标准严格度
good_threshold = 0.6
medium_threshold = 0.4
poor_threshold = 0.2
```

**宽松配置（容错模式）**:
```lua
excellent_threshold = 0.7   -- 更宽松
good_threshold = 0.5
medium_threshold = 0.3
poor_threshold = 0.15
```

### 初始化阶段参数调优

**快速初始化（信任扫描匹配）**:
```lua
init_w_scan_match = 0.8    -- 极度依赖扫描匹配
init_w_motion = 0.2        -- 最小运动权重
init_w_map = 0.0           -- 完全忽略地图
init_w_degeneracy = 0.0    -- 关闭退化检测
```

**稳健初始化（当前配置）**:
```lua
init_w_scan_match = 0.6    -- 高度依赖扫描匹配
init_w_motion = 0.3        -- 辅助参考
init_w_map = 0.1           -- 少量地图信息
init_w_degeneracy = 0.0    -- 关闭退化检测
```

### 正常运行参数调优

**地图优先（已知环境）**:
```lua
normal_w_scan_match = 0.25  -- 降低扫描权重
normal_w_motion = 0.2       -- 降低运动权重
normal_w_map = 0.45         -- ⭐ 提高地图权重
normal_w_degeneracy = 0.1   -- 保持退化检测
```

**运动优先（动态环境）**:
```lua
normal_w_scan_match = 0.3   -- 保持扫描权重
normal_w_motion = 0.4       -- ⭐ 提高运动权重
normal_w_map = 0.2          -- 降低地图权重
normal_w_degeneracy = 0.1   -- 保持退化检测
```

**均衡模式（当前配置）**:
```lua
normal_w_scan_match = 0.3   -- 基础权重
normal_w_motion = 0.25      -- 运动连续性
normal_w_map = 0.35         -- 略高（全局一致性）
normal_w_degeneracy = 0.1   -- 特殊场景检测
```

### 关键失败阈值调优

**保守策略（减少误报，适合复杂环境）**:
```lua
enable_critical_failure_check = true

-- 更宽松的阈值
critical_min_match_ratio = 0.10        -- 更低匹配率要求
critical_max_pose_jump_distance = 3.0  -- 允许更大跳变
critical_max_pose_jump_angle = 0.785   -- 允许45°跳变
critical_min_slippage_score = 0.15     -- 更低打滑检测阈值
critical_min_point_count = 30          -- 更少点云要求
critical_max_ceres_residual = 1.5      -- 更高残差容忍
critical_max_hessian_condition = 1500.0 -- 更高条件数容忍
```

**激进策略（快速响应，适合结构化环境）**:
```lua
enable_critical_failure_check = true

-- 更严格的阈值
critical_min_match_ratio = 0.25        -- 更高匹配率要求
critical_max_pose_jump_distance = 1.0  -- 严格限制跳变
critical_max_pose_jump_angle = 0.349   -- 只允许20°跳变
critical_min_slippage_score = 0.3      -- 更敏感的打滑检测
critical_min_point_count = 80          -- 更多点云要求
critical_max_ceres_residual = 0.5      -- 更低残差容忍
critical_max_hessian_condition = 500.0 -- 更低条件数容忍
```

**推荐策略（默认值，适合通用场景）**:
```lua
enable_critical_failure_check = true

critical_min_match_ratio = 0.15        -- 标准匹配率
critical_max_pose_jump_distance = 2.0  -- 2米跳变阈值
critical_max_pose_jump_angle = 0.524   -- 30°跳变阈值
critical_min_slippage_score = 0.2      -- 标准打滑检测
critical_min_point_count = 50          -- 50点最小要求
critical_max_ceres_residual = 1.0      -- 标准残差阈值
critical_max_hessian_condition = 1000.0 -- 标准条件数阈值
```

### 场景特定调优建议

**室内结构化环境（如仓库、办公室）**:
- 使用**激进策略**的关键失败检测
- 提高 `normal_w_map` 权重（地图特征丰富）
- 降低 `excellent_threshold`（环境稳定，容易达到高置信度）

**室外开放环境**:
- 使用**保守策略**的关键失败检测
- 提高 `normal_w_scan_match` 权重（特征稀疏）
- 提高各级阈值（环境变化大，难达到高置信度）

**走廊/长通道环境**:
- 启用 `enable_degeneracy_check = true`
- 提高 `normal_w_degeneracy` 权重至 0.15-0.2
- 降低 `critical_max_hessian_condition` 至 500-800
- 考虑使用线特征匹配辅助

**动态环境（人流密集）**:
- 提高 `critical_max_pose_jump_distance` 至 2.5-3.0
- 提高 `critical_min_slippage_score` 至 0.15
- 降低 `normal_w_map` 权重（动态障碍物影响地图一致性）

## 故障排查

### 问题1: 置信度一直很低

**可能原因**:
1. 阈值设置过高（当前excellent=0.9, good=0.8较严格）
2. 某一层评估始终返回低分
3. 点云质量差或匹配率低

**排查方法**:
```cpp
// 启用详细日志
--vmodule=confidence_estimator=2

// 查看各层得分
metrics.Print();  // 或 LOG(INFO) << metrics.ToString();
```

**解决方案**:
- 降低阈值至标准配置（0.8/0.6/0.4/0.2）
- 检查各层得分，禁用异常层或调整权重
- 检查激光雷达和里程计数据质量

### 问题2: 频繁触发关键失败

**可能原因**:
1. 关键失败阈值过于严格
2. 打滑严重或轮子异常
3. 地图质量差导致匹配率低

**排查方法**:
```cpp
// 查看触发的失败条件
if (metrics.level == ConfidenceLevel::FAILED) {
  // 日志中会输出: "Critical localization failure detected - X condition(s) triggered: ..."
}
```

**解决方案**:
- 使用保守策略阈值
- 检查机器人机械状态（轮子、电机）
- 临时禁用: `enable_critical_failure_check = false`

### 问题3: 初始化阶段过长

**可能原因**:
1. `initialization_duration_sec` 设置过长
2. 初始阶段参数不合理

**解决方案**:
```lua
-- 缩短初始化时间
initialization_duration_sec = 3.0  -- 从5秒改为3秒

-- 或使用更激进的初始化参数
init_w_scan_match = 0.8  -- 更信任扫描匹配
```

### 问题4: 编译错误

**常见错误**: `undefined reference to CreateConfidenceEstimatorOptions`

**解决方案**:
```bash
# 清理并重新编译
cd /path/to/workspace
rm -rf build/cartographer install/cartographer
colcon build --packages-select cartographer
```

**常见错误**: `cannot find -lconfidence`

**原因**: 使用了独立的CMakeLists.txt而不是全局编译

**解决方案**: 删除 `localization/confidence/CMakeLists.txt`，使用全局CMake

### 问题5: Lua配置加载失败

**错误信息**: `Key 'confidence_estimator' was used the wrong number of times`

**原因**: pose_estimator_2d.lua中引用了但未读取

**解决方案**:
```cpp
// 在 pose_estimator_options_2d.cc 中添加
*options.mutable_confidence_estimator_options() =
  confidence::CreateConfidenceEstimatorOptions(
    parameter_dictionary->GetDictionary("confidence_estimator").get());
```

## 版本历史

### v1.2.0 (2025-01-16)
- ✨ 新增关键失败检测机制 (CheckCriticalFailure)
- ✨ Proto配置移至独立目录 (localization/proto/)
- 🔧 调整默认阈值为更严格模式 (0.9/0.8/0.6/0.4)
- 📝 完善文档和调优建议

### v1.1.0 (2025-01-15)
- ♻️ 去除Config结构体，直接使用proto
- ♻️ 简化为固定两阶段参数（初始化+正常）
- ♻️ 移除动态权重调整和闭环后阶段
- 📝 更新配置化文档

### v1.0.0 (2025-01-10)
- 🎉 初始版本
- ✨ 四层置信度评估架构
- ✨ Lua配置化支持
- ✨ Optional返回机制

## 未来优化方向

1. **场景自适应**: 
   - 自动识别环境类型（室内/室外/走廊）
   - 动态调整权重和阈值
   - 根据地图特征密度优化参数

2. **机器学习增强**: 
   - 使用训练数据学习最优权重
   - 基于历史数据预测置信度趋势
   - 强化学习优化参数配置

3. **多传感器融合**: 
   - 集成IMU姿态信息增强运动一致性
   - 集成轮速计改进打滑检测
   - 集成GPS/UWB增强全局定位置信度

4. **时间序列分析**: 
   - 利用历史置信度序列滤波
   - 检测置信度变化趋势
   - 预测未来置信度降低风险

5. **可视化工具**: 
   - RViz插件实时显示置信度热力图
   - 各层得分时序曲线
   - 关键失败条件触发历史

6. **自适应阈值**: 
   - 根据运行时统计自动调整阈值
   - 场景切换时自动优化参数
   - 在线学习最优配置

## 参考文档

- **理论设计**: `Cartographer框架下2D激光SLAM定位置信度评估方案.md`
- **配置重构**: `CONFIDENCE_ESTIMATOR_REFACTOR.md`
- **简化总结**: `CONFIDENCE_ESTIMATOR_SIMPLIFICATION.md`
- **实现总结**: `CONFIDENCE_IMPLEMENTATION_SUMMARY.md`

## 联系方式

如有问题或建议，请联系项目维护者或提交Issue。

---

**最后更新**: 2025-01-16  
**当前版本**: v1.2.0  
**维护状态**: ✅ 积极维护中
