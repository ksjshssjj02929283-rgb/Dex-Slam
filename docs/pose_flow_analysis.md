# lightning-lm 后端位姿流转问题分析

> 创建时间：2026-03-17  
> 问题来源：用户反馈"AddKF 传入的位姿是未优化的 LIO 位姿，会导致回环检测和后端优化出现很大误差"

---

## 1. 问题描述

用户认为：`lc_->AddKF(cur_kf_)` 传入的关键帧携带的是 LIO 前端（ESKF）输出的**未优化位姿**，在进入回环检测和位姿图优化之前，应该先通过已有的位姿图优化结果进行校正，否则会引入累积误差。

---

## 2. 与参考系统的对比

### 2.1 LIO-SAM 的位姿流转

LIO-SAM 是**双节点**架构：

```
IMUPreintegration 节点（前端）
   → transformTobeMapped[6]  （LiDAR 里程计输出，含 IMU 预积分）
   → 发布 /odometry/imu_incremental

mapOptimization 节点（后端）
   → 订阅 /odometry/imu_incremental
   → addOdomFactor():
       poseFrom = cloudKeyPoses6D->back()   ← 上一帧已优化的后端位姿
       poseTo   = transformTobeMapped        ← 当前帧 LIO 里程计位姿
       motion   = poseFrom.between(poseTo)   ← 相对运动（仅用运动增量）
       initialEstimate.insert(N, poseTo)     ← 初始值用 LIO 位姿
   → isam->update() 后：
       isamCurrentEstimate.at<Pose3>(N)      ← 后端优化后的绝对位姿
       cloudKeyPoses6D->push_back(…)        ← 更新关键帧数据库
   → correctPoses() 在回环后更新全部历史帧
```

**关键点**：
1. `addOdomFactor` 里，里程计边的测量值是 **相对运动**（`poseFrom.between(poseTo)`），不是绝对位姿
2. iSAM2 每帧都更新，最新帧立即得到后端优化的绝对位姿（`isamCurrentEstimate`）
3. `cloudKeyPoses6D` 中存储的始终是 **iSAM2 输出的已优化位姿**
4. 回环检测（`detectLoopClosureDistance`）读取的是 `copy_cloudKeyPoses3D`（已优化位姿的副本）
5. `loopFindNearKeyframes` 用 `cloudKeyPoses6D` 变换点云（已优化位姿）

**结论**：LIO-SAM 的后端在每帧都运行 iSAM2 更新，回环检测和子地图构建始终用的是**最新优化后的位姿**。

### 2.2 Cartographer 的位姿流转

Cartographer 是 **子图-约束** 架构（见 `pose_graph_2d.cc`）：

```
LocalSLAM（前端）
   → 输出 local_pose（局部子图坐标系内的位姿）
   → 触发 AppendNode(constant_data, trajectory_id, insertion_submaps, optimized_pose)
       optimized_pose = GetLocalToGlobalTransform(trajectory_id) * local_pose
                     ← 用当前已知的 T_global_local 把局部位姿转换成全局位姿

PoseGraph2D（后端，独立线程）
   → ComputeConstraintsForNode() 异步计算约束（子图内+回环）
   → HandleWorkQueue() 每隔一段时间触发全局优化
   → RunOptimization() 之后更新所有节点的全局位姿
```

**关键点**：
1. 节点添加时，`optimized_pose` 由 `GetLocalToGlobalTransform() * local_pose` 计算，**使用了当前最新的全局到局部变换**
2. `GetLocalToGlobalTransform()` 根据已优化的子图位姿插值得到，是**近似优化后的全局位姿**
3. 回环约束用的初始位姿（`ComputeConstraint` 里的初始猜测）也来自这个已优化的全局位姿

**结论**：Cartographer 也是在添加节点时就用已优化信息初始化全局位姿，回环检测基于该位姿。

---

## 3. lightning-lm 当前的位姿流转

```
LaserMapping::MakeKF()（前端，laser_mapping.cc:430）
   Keyframe::Ptr kf = std::make_shared<Keyframe>(kf_id_++, scan_undistort_, state_point_)
   ↑ 构造函数：pose_lio_ = state_point_.GetPose()（ESKF 输出，纯前端位姿）
   ↑ 构造函数：pose_opt_ = pose_lio_（初始时两者相同）

   if (last_kf_) {
       SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose()  // LIO 相对运动
       kf->SetOptPose(last_kf_->GetOptPose() * delta)  // 用上一帧 opt_pose 递推！
   }

↓
slam.cc: lc_->AddKF(cur_kf_)
↓
LoopClosing::HandleKF()（后端）
   DetectLoopCandidates()：用 cur_kf_->GetOptPose() 做 KD-Tree 搜索
   ComputeLoopCandidates()：LoopFindNearKeyframes 用 GetOptPose() 变换点云
   PoseOptimization()：用 GetLIOPose() 的相对运动作为里程计边（第 638 行）
```

---

## 4. 核心问题：opt_pose 的初始化方式

### 4.1 MakeKF 中的递推逻辑（laser_mapping.cc:432-435）

```cpp
SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();  // LIO 相对运动
kf->SetOptPose(last_kf_->GetOptPose() * delta);                   // 用上一帧 opt_pose 递推
```

这里做了一个**正确的事情**：
- 取的是两帧 LIO 位姿之间的**相对运动** `delta`（LIO 的相对运动精度高，不含累积误差）
- 用上一帧**已优化**的 `opt_pose` 来递推，所以当前帧的 `opt_pose` 初始值继承了最新 PGO 的结果

### 4.2 PoseOptimization 中的运动边（loop_closing.cc:638）

```cpp
SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
e->SetMeasurement(motion);
```

运动边的测量值是 LIO 的**相对运动增量**，这与 LIO-SAM 的 `poseFrom.between(poseTo)` 等价，是正确的。

### 4.3 顶点初始值（loop_closing.cc:621）

```cpp
v->SetEstimate(cur_kf_->GetOptPose());
```

顶点初始值用 `GetOptPose()`，而 `GetOptPose()` 由 `MakeKF` 递推得到（基于上一帧 PGO 结果 + LIO 相对运动），与 LIO-SAM 的 `initialEstimate.insert(N, poseTo)` 语义一致。

---

## 5. 是否真的存在"位姿未优化"的问题？

### 结论：**问题存在，但性质比用户描述的更细微**

主要问题不是"传入了 LIO 的绝对位姿"，而是以下两个设计差异：

---

### 问题 A：PGO 触发频率低——运动边用绝对 LIO 位姿差存在隐患

**LIO-SAM**：每一帧都调用 `isam->update()`，每帧的顶点初始值和运动边立即得到 iSAM2 优化，`cloudKeyPoses6D` 始终是最新优化位姿。

**lightning-lm**：PGO 只在 `candidates_` 非空（有成功回环）时才触发。大多数帧直接跳过：

```cpp
if (candidates_.empty()) {
    return;  // 不做优化！
}
optimizer_->Optimize(20);
```

这意味着：**在两次回环之间，`kf_poses_cloud_` 里存储的 KD-Tree 搜索点是 MakeKF 递推的 opt_pose，没有经过真正的 PGO 优化**。

- 若 LIO 漂移小（室内短期），影响可忽略
- 若 LIO 漂移大（长廊/大场景），KD-Tree 中历史帧的位置与真实位置偏差大 → **回环搜索半径内找不到应该找到的历史帧**（漏检）

---

### 问题 B：`LoopFindNearKeyframes` 用 opt_pose 构建子地图，但 opt_pose 可能不一致

`LoopFindNearKeyframes` 变换点云到 kf1 局部系：
```cpp
SE3 T_center = all_keyframes_[key]->GetOptPose();
SE3 T_rel = T_center_inv * kf->GetOptPose();
pcl::transformPointCloud(*cloud, *cloud_local, T_rel.matrix());
```

当 PGO 刚运行完，所有历史帧的 `opt_pose` 都被更新，但 **`kf_poses_cloud_`（KD-Tree 里的点）没有立即更新**（只在 PGO 触发后才重建）。

此前已修复这一问题（PGO 后重建 `kf_poses_cloud_`），但在无回环时期，两者仍然靠 MakeKF 递推保持一致。

---

### 问题 C：运动边测量值用 LIO 相对运动，而非 PGO 优化后的相对运动

```cpp
SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
```

LIO-SAM 也是这样（用里程计的相对运动），这是**正确的**——相对运动误差小，即使绝对误差大，相邻帧的相对位姿也是准确的。

---

## 6. 与 LIO-SAM/Cartographer 的根本架构差异

| 维度 | LIO-SAM | Cartographer | lightning-lm |
|------|---------|-------------|--------------|
| PGO 触发频率 | **每帧** iSAM2 增量更新 | 每批约束计算完 | **仅有回环时** |
| 后端位姿实时性 | `cloudKeyPoses6D` 始终是优化后位姿 | 节点全局位姿近似更新 | `opt_pose` 在无回环期只靠递推 |
| 运动边测量值 | LIO 相对运动 ✓ | 局部子图内相对运动 ✓ | LIO 相对运动 ✓ |
| 顶点初始值 | LIO 绝对位姿 | 优化后 T_global_local × local_pose | MakeKF 递推 opt_pose ✓ |
| 回环搜索位姿 | 优化后 `cloudKeyPoses6D` ✓ | 优化后全局位姿 ✓ | 递推 opt_pose（无回环期有漂移）|

---

## 7. 建议修改方案

### 方案一（保守）：定期强制重置 opt_pose

即使无回环，也定期（如每 50 帧）将 `kf_poses_cloud_` 中的位置刷新为最新的递推 opt_pose。
- 成本：低
- 效果：减少 KD-Tree 与 opt_pose 不一致的窗口
- 局限：不改变递推累积误差本身

### 方案二（推荐）：增量 PGO，每帧都更新

类比 LIO-SAM，即使无回环，每帧也执行一次 `optimizer_->Optimize()`（轻量级增量优化，仅更新新顶点和运动边）。

修改 `PoseOptimization()`：
```cpp
// 当前：仅有回环时才优化
if (candidates_.empty()) return;

// 改为：始终优化（增量模式下开销可控）
optimizer_->InitializeOptimization();
optimizer_->Optimize(5);  // 少次迭代，保证实时性

// 写回 opt_pose
for (auto& vert : kf_vert_) {
    all_keyframes_[vert->GetId()]->SetOptPose(vert->Estimate());
}
```

同时，每帧优化后更新 `kf_poses_cloud_`（或只更新最新帧的位置）：
```cpp
// 不重建全部，仅追加/更新最新帧
Vec3d t = cur_kf_->GetOptPose().translation();
kf_poses_cloud_->push_back({...});
```

- 成本：中等（每帧跑几次 GN 迭代）
- 效果：`opt_pose` 始终与运动边约束一致，KD-Tree 搜索更准确

### 方案三（激进）：将 MakeKF 的递推改为显式优化继承

在 `MakeKF` 中，当 lc 存在且上一帧有优化结果时，直接从 lc 的优化器读取上一帧的最新 opt_pose，再递推：

```cpp
// 在 MakeKF 之后，slam.cc 里：
if (lc_) {
    SE3 last_opt_pose = lc_->GetLatestOptPose();   // 新接口，返回上一帧 PGO 后的 opt_pose
    SE3 delta = last_kf_->GetLIOPose().inverse() * kf->GetLIOPose();
    kf->SetOptPose(last_opt_pose * delta);          // 继承最新 PGO 结果
}
```

---

## 8. 当前代码是否能正常工作？

**短路程（< 100m）室内场景**：能正常工作。递推误差小，KD-Tree 搜索准确，回环能正确触发。

**长路程（> 500m）大场景/长廊**：可能出现漏检。原因：
1. LIO 累积误差大，递推的 opt_pose 与真实位置偏差 > `max_range_`（默认 30m）
2. KD-Tree 找不到真正靠近的历史帧，回环无法触发
3. 即使触发了，`LoopFindNearKeyframes` 构建的子地图也可能因 opt_pose 不准确而错位

**已经发生过回环后**：后续帧的 opt_pose 基于 PGO 修正后的结果递推，准确性恢复。

---

## 9. 优先级排序

| 优先级 | 问题 | 修改复杂度 | 影响范围 |
|-------|------|-----------|---------|
| 高 | 无回环期 KD-Tree 搜索位置偏漂 | 低（方案一） | 长路程回环漏检 |
| 中 | PGO 不每帧更新，opt_pose 非全局最优 | 中（方案二） | 所有场景精度 |
| 低 | MakeKF 递推 vs lc 显式继承 | 高（方案三） | 架构改动 |

**建议先实施方案二**，在 `PoseOptimization()` 中无回环时也执行轻量优化（1-3 次迭代），并每帧更新 `kf_poses_cloud_` 的最新帧位置，这样以最低代价对齐 LIO-SAM 的增量更新逻辑。
