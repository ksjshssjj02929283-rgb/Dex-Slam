# lightning-lm 回环检测模块文档

> 最后更新：2026-03-07
> 代码路径：`src/core/loop_closing/loop_closing.cc / .h`

---

## 概述

回环检测（Loop Closing）负责在机器人重访已建图区域时检测闭环并修正累积漂移。技术路线对齐 **LIO-SAM**：

- **候选检测**：KD-Tree 半径搜索 + 时间差过滤 + `loopIndexContainer` 防重复
- **点云配准**：多分辨率 NDT（粗对齐）→ GICP（精对齐）
- **后端优化**：miao 增量位姿图优化（LM 算法，基于 g2o 设计）

---

## 整体数据流

```
新关键帧到来（HandleKF）
     │
     ├─ DetectLoopCandidates()   ← KD-Tree 半径搜索 + 时间差 + 冷却期
     │         │
     │    candidates_ 非空？
     │         │ Yes
     ├─ ComputeLoopCandidates()  ← 对每个候选执行 NDT + GICP 配准
     │         │
     │    通过双重验证的候选 → loop_index_queue_ / loop_pose_queue_ / loop_score_queue_
     │
     ├─ PoseOptimization()
     │    ├─ AddLoopFactor()     ← 消费队列，加入因子图
     │    └─ miao Optimize(20)   ← 增量 PGO
     │
     └─ kf_poses_cloud_ 追加当前帧位置（供下一帧 KD-Tree 搜索）
```

---

## 第一阶段：候选帧检测（DetectLoopCandidates）

### 门控条件（短路逻辑，任一满足即跳过）

| 条件 | 说明 |
|------|------|
| `kf_poses_cloud_` 为空 | 尚无历史帧 |
| `last_loop_kf_ == nullptr` | 首次进入，初始化冷却起点后返回（防启动即触发） |
| `loop_index_container_.count(cur_id) > 0` | 当前帧已参与过回环，防重复 |
| `cur_id - last_loop_kf_->GetID() <= loop_kf_gap_` | 冷却期内（默认 50 帧） |

### KD-Tree 半径搜索

```
kf_poses_cloud_：与 all_keyframes_ 同步但滞后一帧（不含当前帧）
                索引 i 对应 all_keyframes_[i]
搜索半径：max_range_（默认 30m）
结果：按距离由近到远排序的历史帧索引
```

### 时间差过滤 + 密集度控制

```cpp
for (int idx : indices) {
    // 时间差过滤：排除同一轨迹段近邻帧（LIO-SAM historyKeyframeSearchTimeDiff）
    if (|cur_time - kf.timestamp| < history_search_time_diff_) continue;

    // 密集度控制：候选帧间 ID 间隔不能过小
    if (check_first && |kf.id - check_first.id| <= min_id_interval_) continue;

    candidates_.push_back(c);
    check_first = kf;
}

// 截取最近的前 N 个候选（KD-Tree 已按距离排序）
if (candidates_.size() > max_candidates_per_check_)
    candidates_.resize(max_candidates_per_check_);
```

---

## 第二阶段：点云配准（ComputeForCandidate）

针对每个候选帧对 `(idx1=历史帧, idx2=当前帧)` 执行以下步骤。

### 步骤 1：构建子地图（LoopFindNearKeyframes）

**关键设计：子地图建立在 kf1 的局部坐标系，而非世界坐标系。**

```
世界坐标系可能有很大的绝对坐标值（如几百米），
导致 PCL VoxelGrid 的整数索引溢出（[pcl::VoxelGrid] Integer indices would overflow）。
转换到 kf1 局部系后，所有点坐标接近 0，避免溢出。
```

```cpp
T_center_inv = kf1->GetLIOPose().inverse();

for i in [-search_num, +search_num]:
    T_rel = T_center_inv * kf[i]->GetLIOPose()       // 相对变换，值接近零
    cloud_local = T_rel * kf[i]->GetCloud()
    near_keyframes += cloud_local

near_keyframes = VoxelGrid(near_keyframes, icp_filter_size_)  // 0.4m
```

**source 点云**：直接取 `kf2->GetCloud()`（body 坐标系），单独 VoxelGrid 降采样后送入配准，不做世界坐标系变换。

### 步骤 2：点数检查

```
cur_cloud.size() < min_cur_cloud_size_  (默认 300)  → 跳过
prev_cloud.size() < min_prev_cloud_size_ (默认 1000) → 跳过
```

### 步骤 3：多分辨率 NDT（粗对齐）

**坐标系一致性**：
- target = `prev_cloud`（kf1 局部系）
- source = `cur_cloud`（kf2 body 系）
- 初始猜测 = `kf1->GetLIOPose().inverse() * kf2->GetLIOPose()`（kf1 局部系中 kf2 的相对位姿）

**自适应分辨率序列**（根据 prev_cloud 的空间范围 extent 选择）：

| 场景 | extent | 分辨率序列 |
|------|--------|-----------|
| 小场景（室内）| < 100m | `{4.0, 2.0, 1.0, 0.5}` m |
| 大场景（室外）| ≥ 100m | `{10.0, 5.0, 2.0, 1.0}` m |

> 若 extent < 100m 仍使用 10m 分辨率，10m 的 NDT voxel 比整个场景还大，无法建立有效正态分布，第一轮完全无效。

**级联精化**（每轮以上轮结果为初始，不能重用原始 T_init）：

```cpp
Mat4f T_ndt = T_init_lio;   // 初始为 LIO 里程计相对位姿
for r in res:
    rough_map1 = VoxelGrid(prev_cloud, r * 0.1)
    rough_map2 = VoxelGrid(cur_cloud,  r * 0.1)
    ndt.align(output, T_ndt)       // 关键：用 T_ndt 而非 T_init
    T_ndt = ndt.getFinalTransformation()  // 更新，供下轮使用
    ndt_score = ndt.getTransformationProbability()
```

### 步骤 4：GICP 精匹配

以 NDT 最终结果 `T_ndt` 作为初始位姿，运行 GICP：

```cpp
pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
gicp.setMaxCorrespondenceDistance(icp_max_correspondence_dist_);  // 默认 1.5m
gicp.setMaximumIterations(100);
gicp.setTransformationEpsilon(1e-6);
gicp.setEuclideanFitnessEpsilon(1e-6);
gicp.setRANSACIterations(0);
gicp.setInputTarget(prev_cloud);
gicp.setInputSource(cur_cloud);
gicp.align(icp_output, T_ndt);
```

**GICP vs 标准 ICP**：GICP（Generalized ICP）内部为每个点建立局部曲面协方差，使用面到面对应代替点到点，对 LiDAR 点云的稀疏性和噪声更鲁棒，收敛域更大。

**双重验证**：

```cpp
// NDT 粗筛（第三步中 ndt_score > ndt_score_th_ 才继续到 GICP）
// GICP 精筛
bool icp_ok = gicp.hasConverged() && gicp.getFitnessScore() < icp_fitness_score_th_;
if (!icp_ok) {
    c.ndt_score_ = 0;  // 丢弃该候选
    return;
}
```

GICP `getFitnessScore()` 为均方点距（m²），越小越好，阈值 `icp_fitness_score_th_` 默认 0.3。

### 步骤 5：位姿提取

```cpp
Mat4f T_icp = gicp.getFinalTransformation();
// T_icp 是 kf1 局部系中 kf2 的修正相对位姿，直接作为 c.Tij_
c.Tij_ = SE3(Quatd(T_icp.block<3,3>(0,0)), T_icp.block<3,1>(0,3));
```

### 步骤 6：调试数据保存

UI（Pangolin/RViz）以 `SE3()` 渲染 submap/aligned，期望点云已在 **world（map）坐标系**：

```cpp
T_kf1_world = kf1->GetLIOPose().matrix()
prev_cloud_world  = T_kf1_world * prev_cloud   // kf1 局部系 → world
icp_output_world  = T_kf1_world * icp_output

loop_debug_data_.loop_submap  = prev_cloud_world    // world 系
loop_debug_data_.current_scan = cur_cloud           // body 系（UI 侧乘 current_kf_pose）
loop_debug_data_.aligned_scan = icp_output_world    // world 系
loop_debug_data_.aligned_pose = SE3()               // identity，点云已在 world 系
```

---

## 第三阶段：位姿图优化（PoseOptimization + AddLoopFactor）

### 队列解耦设计（对应 LIO-SAM addLoopFactor）

配准验证与 PGO 解耦：`ComputeLoopCandidates` 将成功的候选写入三个队列，`PoseOptimization` 调用 `AddLoopFactor` 统一消费：

```
loop_index_queue_  → (hist_id, cur_id) 对
loop_pose_queue_   → 对应的相对位姿 Tij
loop_score_queue_  → 对应的 NDT score（用于计算自适应噪声）
```

### 图节点与图边

每个关键帧对应一个 `VertexSE3`（SE(3) 位姿），每次新帧到来时增量添加。

**三类约束**：

| 边类型 | 连接 | 信息矩阵 |
|--------|------|---------|
| 运动边（EdgeSE3） | 当前帧 ↔ 前 1、前 2 帧 | 平移 σ=0.1m，旋转 σ=3° |
| 回环边（EdgeSE3） | 回环帧对 | NDT 得分自适应噪声（见下） |
| 高度先验（EdgeHeightPrior） | 单帧 Z=0 | σ=0.1m（可关闭） |

**自适应噪声模型**（NDT score 越高 → 噪声越小 → 约束越强）：

```cpp
double noise_scale = max(ndt_score_th_ / ndt_score, 0.1);
t_sigma = loop_trans_noise_ * noise_scale;
r_sigma = loop_rot_noise_ * noise_scale;
```

**Cauchy 鲁棒核**附加在所有回环边上，抵御误检回环：

```cpp
rk->SetDelta(rk_loop_th_);  // 默认 5.2/5 = 1.04
e->SetRobustKernel(rk);
```

### 外点剔除（Optimize 后）

```
优化 20 次迭代
→ 遍历所有回环边：
    Chi2 > Delta  → SetLevel(1)，排除出优化
    Chi2 <= Delta → 移除鲁棒核，视为内点保留
→ 将优化结果写回 all_keyframes_[id]->SetOptPose(pose)
→ 触发 loop_cb_()（g2p5 重绘 2D 栅格地图）
```

---

## 配置参数完整列表

```yaml
loop_closing:
  loop_kf_gap: 50                 # 回环检测冷却期（关键帧数），防止密集区域频繁触发
  min_id_interval: 20             # 候选帧之间最小 ID 间隔，避免过密集候选
  max_range: 30.0                 # KD-Tree 半径搜索距离阈值（米）
  ndt_score_th: 1.0               # NDT 通过阈值（TransformationProbability，越高越严）
  with_height: true               # 高度约束（多层场景需设为 false）
  history_search_time_diff: 30.0  # 候选时间差阈值（秒），排除同轨迹段近邻
  max_candidates_per_check: 3     # 每次触发最多处理的候选数（取距离最近的）
  history_keyframe_search_num: 25 # 子地图前后各取帧数（共 2N+1 帧）
  icp_filter_size: 0.4            # 子地图 VoxelGrid 叶大小（米）
  min_cur_cloud_size: 300         # 当前帧最小点数
  min_prev_cloud_size: 1000       # 历史子地图最小点数
  icp_fitness_score_th: 0.3       # GICP fitness 阈值（均方点距 m²，越小越严）
  icp_max_correspondence_dist: 1.5 # GICP 最大对应点距离（米）
```

---

## 坐标系约定（避免踩坑）

| 数据 | 坐标系 | 备注 |
|------|--------|------|
| `prev_cloud`（submap） | kf1 局部系 | 避免世界坐标大值导致 VoxelGrid 溢出 |
| `cur_cloud`（source） | kf2 body 系 | 直接取 `kf2->GetCloud()` |
| NDT/GICP 初始猜测 | kf1 局部系 | = `kf1_pose_inv * kf2_pose` |
| NDT/GICP 结果 T | kf1 局部系 | 直接作为 `c.Tij_`，无需额外变换 |
| `loop_debug_data_.loop_submap` | world 系 | 发布前乘以 `kf1->GetLIOPose()` |
| `loop_debug_data_.aligned_scan` | world 系 | 发布前乘以 `kf1->GetLIOPose()` |
| `loop_debug_data_.current_scan` | body 系 | UI 侧乘以 `current_kf_pose` 渲染 |

---

## 常见问题排查

### 回环检测不触发

1. `system.with_loop_closing` 是否为 `true`
2. `max_range` 是否过小（机器人重访路径偏移超过阈值）
3. `history_search_time_diff` 是否过大（短时间内绕圈的场景需减小）
4. `loop_kf_gap` 是否过大（调小冷却期）

### NDT score 正常但 GICP 失败（fitness 过高）

1. `icp_max_correspondence_dist` 过小：NDT 粗对齐误差 > 对应距离时，GICP 找不到足够对应点
2. 场景点云过于稀疏（如长走廊），GICP 局部极值概率高；考虑降低 `icp_filter_size` 保留更多点
3. `icp_fitness_score_th` 可适当放宽（如 0.5）

### VoxelGrid 整数索引溢出

```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset.
Integer indices would overflow.
```

原因：`LoopFindNearKeyframes` 使用世界坐标系（大值）。应确保子地图在 kf1 **局部坐标系**构建。

### NDT score 始终为 0 / 配准不收敛

1. 多分辨率 NDT 级联失效：确认每轮 `ndt.align(output, T_ndt)` 使用的是上轮结果，不是原始 `T_init`
2. source/target 坐标系不一致：source 是 world 系但 target 是 local 系，或初始猜测是绝对位姿而非相对位姿
3. 系统刚启动即触发回环（`last_loop_kf_ == nullptr` 时的初始化守卫缺失）

### 多层场景位姿图优化发散

```yaml
loop_closing:
  with_height: false
```

---

## 关键文件索引

| 文件 | 说明 |
|------|------|
| `src/core/loop_closing/loop_closing.h` | 类定义、Options、LoopDebugData |
| `src/core/loop_closing/loop_closing.cc` | 完整实现 |
| `src/common/loop_candidate.h` | `LoopCandidate` 数据结构 |
| `src/core/miao/core/types/edge_se3.h` | SE3 约束边（运动边/回环边） |
| `src/core/miao/core/types/edge_se3_height_prior.h` | 高度先验边 |
| `src/core/system/slam.cc` | 系统集成，回环触发 g2p5 重绘 |
| `src/ui/pangolin_window_impl.cc` | 回环调试数据可视化（loop_submap/aligned 需 world 系） |
| `src/wrapper/rviz_publisher.cc` | RViz 回环调试点云发布 |
| `config/default.yaml` | 默认配置 |
