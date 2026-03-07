# lightning-lm 后端不足分析与优化方案

> 初版分析：2026-03-05 ｜ 最后更新：2026-03-07
> 代码路径：`src/core/`
> 涉及模块：loop_closing、localization/pose_graph（PGO）、maps（TiledMap）、system

---

## 一、回环检测模块（loop_closing）

### 1.1 已完成改进（2026-03-05 ～ 2026-03-07）

#### ✅ 改进 1：候选检测对齐 LIO-SAM（KD-Tree + 时间差过滤 + loopIndexContainer）

**原实现**：遍历全量历史帧，仅用 XY 平面欧氏距离过滤，无防重复机制，启动即可能触发回环。

**现实现**：
- KD-Tree `radiusSearch` 半径搜索历史帧位置点云（`kf_poses_cloud_`）
- 时间差过滤（`history_search_time_diff_=30s`）排除同一轨迹段近邻帧
- `loopIndexContainer_`（map）防止当前帧重复触发回环
- `loop_kf_gap_`（默认 50 帧）冷却期
- 首帧初始化守卫（`last_loop_kf_==nullptr` 时直接返回，防启动即触发）
- `max_candidates_per_check_`（默认 3）限制单次 NDT+GICP 计算量

#### ✅ 改进 2：子地图坐标系修正（局部系代替世界系，解决 VoxelGrid 溢出）

**原问题**：`LoopFindNearKeyframes` 将所有帧变换到世界坐标系，坐标值（可能数百米）导致 PCL VoxelGrid 整数索引溢出：
```
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset.
Integer indices would overflow.
```
VoxelGrid 返回原始未采样点云（7万+点），NDT 在超量数据上运行，得分为 0。

**现实现**：以 kf1 的 LIO 位姿为局部坐标系原点，用相对变换 `T_center_inv * kf->GetLIOPose()` 替代绝对世界变换，坐标值接近 0，彻底消除溢出。

#### ✅ 改进 3：修复多分辨率 NDT 级联 Bug（Critical）

**原 Bug**：

```cpp
Mat4f T_ndt = T_init;
for (auto& r : res) {
    ndt.align(*output, T_init);       // ← 每轮都从原始 T_init 出发
    T_ndt = ndt.getFinalTransformation();  // 结果写入 T_ndt 但从不回代
}
// 效果：每轮 NDT 独立运行，多分辨率级联完全失效
// 等价于：4 次独立的 1m NDT，且每次初始猜测相同
```

**修复**：

```cpp
Mat4f T_ndt = T_init_lio;
for (auto& r : res) {
    ndt.align(*output, T_ndt);        // ← 每轮用上轮结果
    T_ndt = ndt.getFinalTransformation();  // 更新，供下轮使用
}
// 效果：真正的由粗到精级联精化
```

#### ✅ 改进 4：NDT 分辨率自适应（根据子地图空间范围选择）

**原问题**：固定 `{10, 5, 2, 1}m` 分辨率序列，小场景（室内）时 10m 的 NDT voxel 比整个场景还大，无法建立有效正态分布。

**现实现**：计算 `prev_cloud` 的空间范围 extent，自适应选择：

```cpp
pcl::getMinMax3D(*prev_cloud, min_pt, max_pt);
float extent = max(dx, dy, dz);
vector<double> res = (extent < 100.0f)
    ? {4.0, 2.0, 1.0, 0.5}   // 小场景（室内/中等场景）
    : {10.0, 5.0, 2.0, 1.0}; // 大场景（室外大范围）
```

#### ✅ 改进 5：标准 ICP 替换为 GICP + 修复验证条件

**原问题（ICP）**：标准 `IterativeClosestPoint`（点到点）对 LiDAR 点云的稀疏性和噪声敏感，容易陷入局部极值。验证条件存在逻辑漏洞：

```cpp
// 原代码：!icp_ok 已包含两种失败，再加 && fitness > th 造成漏洞
if (!icp_ok && icp_fitness > options_.icp_fitness_score_th_) { ... }
// 若 !converged 且 fitness 碰巧 <= th，错误地通过验证
```

**现实现**：
- 改用 `GeneralizedIterativeClosestPoint`（GICP，点到面），建立局部曲面协方差，对稀疏点云更鲁棒
- 修复验证条件：`if (!icp_ok)` 即 `!converged || fitness >= th`

#### ✅ 改进 6：调试数据坐标系修正（loop_submap/aligned 变换到 world 系）

UI（Pangolin/RViz）渲染时以 `SE3()` 渲染 submap 和 aligned scan，期望已在 world 坐标系。原实现保存的是 kf1 局部系点云，导致位置错误。现在统一在 `ComputeForCandidate` 末尾乘以 `kf1->GetLIOPose()` 变换到 world 系。

#### ✅ 改进 7：LIO-SAM 风格 AddLoopFactor（队列解耦 + 自适应噪声）

配准验证与 PGO 解耦，成功候选写入三队列（index/pose/score），`PoseOptimization` 统一通过 `AddLoopFactor()` 消费。NDT score 越高 → 自适应噪声越小 → 回环约束越强。

---

### 1.2 当前仍存在的不足

#### (1) 候选帧无描述符预筛选，长走廊场景误候选多

KD-Tree 仅用空间距离筛选候选，在走廊、对称场景下会产生外观完全不同的误候选，GICP 验证阶段才发现，浪费大量计算。

**推荐改进**：引入 Scan Context 描述符（构建方位-距离直方图），在 KD-Tree 搜索后追加描述符相似度排序，只对 Top-K 相似帧做 NDT+GICP。

#### (2) Submap 每次全量构建，无缓存

`LoopFindNearKeyframes` 每次调用都遍历 ±N 帧点云并做坐标变换，当候选量多或 N 大时开销显著。

**推荐改进**：增加 `unordered_map<int, CloudPtr> submap_cache_`，以 kf1 id 为 key，位姿未被回环修改时复用缓存。

#### (3) 候选串行计算，多候选时耗时线性增长

```cpp
std::for_each(candidates_.begin(), candidates_.end(),
              [this](LoopCandidate& c) { ComputeForCandidate(c); });
```

**推荐改进**：改为 `std::async` 并行（`ComputeForCandidate` 只读 `all_keyframes_`，无写竞争）。

---

## 二、位姿图优化模块（PGO）

### 2.1 当前不足

#### (1) 边缘化未实现（最严重）

**代码位置：** `pgo_impl.cc:Marginalize()`

当前边缘化等价于直接钉住下一帧位姿，协方差 `prior_cov_` 从未赋值，先验约束强弱与历史信息实际不确定度完全脱节。正确实现应对滑出帧执行 Schur 补：

```
H_new = H_kk - H_kr * H_rr^{-1} * H_rk
b_new = b_k  - H_kr * H_rr^{-1} * b_r
```

参考：VINS-Mono `marginalization_factor.cpp`。

#### (2) DR 因子从未加入优化

`AddDRFactors()` 实现完整，但 `BuildProblem()` 从未调用。DR（轮速/IMU 积分）作为帧间相对约束被完全丢弃。

#### (3) LidarOdom 退化检测硬编码，永远不退化

`trans_confidence / rot_confidence` 硬编码全 1，退化检测条件永远不成立。正确实现应从 ESKF Hessian PCA 获取各轴置信度。

#### (4) `RemoveOutliers()` 未实现

函数体为空，外点剔除完全依赖 Huber 核隐式降权，没有两阶段显式剔除。

#### (5) 回环优化结果未反馈给 PGO

建图阶段回环修正了历史关键帧位姿，但定位系统 PGO 的先验仍来自局部 LIO 轨迹，两者缺乏信息通路。

---

### 2.2 优化方案

#### 方案 A：实现正确边缘化（P0）

在 `Marginalize()` 中通过 miao 优化器获取雅可比和残差，计算 Schur 补后写入 `frame_to_keep->prior_cov_`。

#### 方案 B：激活 DR 因子（P0）

```cpp
void PGOImpl::BuildProblem() {
    AddVertex();
    if (is_in_map_) AddLidarLocFactors();
    AddLidarOdomFactors();
    AddDRFactors();   // ← 激活
    AddPriorFactors();
}
```

#### 方案 C：两阶段外点剔除（P1）

```cpp
optimizer_->Optimize(5);    // 第一轮
RemoveOutliers();            // 标记 outlier → Level(1)
optimizer_->Optimize(5);    // 第二轮（不含 outlier）
```

#### 方案 D：回环结果反馈 PGO（P1）

```cpp
lc_->SetLoopClosedCB([this]() {
    auto kf = lc_->GetLastLoopKF();
    LocalizationResult lr;
    lr.pose_ = kf->GetOptPose();
    lr.confidence_ = 1.0;
    pgo_->ProcessLidarLoc(lr);
});
```

---

## 三、分块地图模块（TiledMap）

### 3.1 当前不足

| 问题 | 说明 |
|------|------|
| 回环后地图未重绘 | 历史帧位姿调整后，TiledMap 中旧位姿点云仍在，局部出现叠影 |
| 动态地图持续增长 | PERSISTENT 模式下动态点云只增不减，长时间运行后内存线性增长 |
| 并发访问不完整 | 静态块和动态块的加载之间无原子性保证（代码注释 `// TODO: thread safety`） |

### 3.2 优化方案

**回环触发地图重绘**：在 `LoopClosedCallback` 中，对位姿改变量超阈值（如 0.1m / 0.5°）的关键帧对应 chunk 标记 dirty，后台线程重绘。

**动态地图时间戳老化**：为动态点云增加时间戳，定期清理超过 TTL（如 30 分钟）的旧点。

---

## 四、优先级汇总

| 状态 | 问题 | 模块 | 优先级 |
|------|------|------|--------|
| ✅ 已修复 | KD-Tree 候选检测 + 时间差过滤 | loop_closing | P0 |
| ✅ 已修复 | VoxelGrid 溢出（世界坐标系 → 局部坐标系）| loop_closing | P0 |
| ✅ 已修复 | 多分辨率 NDT 级联 bug（T_init 未回代）| loop_closing | P0 |
| ✅ 已修复 | NDT 分辨率自适应（小场景/大场景）| loop_closing | P1 |
| ✅ 已修复 | ICP → GICP + 验证条件修复 | loop_closing | P1 |
| ✅ 已修复 | 调试数据坐标系（局部系 → world 系）| loop_closing | P1 |
| ✅ 已修复 | AddLoopFactor 队列解耦 + 自适应噪声 | loop_closing | P1 |
| 🔲 待实现 | 边缘化（Schur 补）| PGO | P0 |
| 🔲 待实现 | DR 因子激活 | PGO | P0 |
| 🔲 待实现 | LidarOdom 退化检测（PCA 置信度）| PGO | P0 |
| 🔲 待实现 | 两阶段外点剔除 | PGO | P1 |
| 🔲 待实现 | 回环结果反馈 PGO | loop_closing / loc_system | P1 |
| 🔲 待实现 | Scan Context 描述符预筛选 | loop_closing | P2 |
| 🔲 待实现 | Submap 缓存 | loop_closing | P2 |
| 🔲 待实现 | 并行候选计算 | loop_closing | P2 |
| 🔲 待实现 | 回环触发地图重绘 | TiledMap | P2 |
| 🔲 待实现 | 动态地图时间戳老化 | TiledMap | P3 |

---

## 五、参考文献

- Grisetti et al., "Improved Techniques for Grid Mapping", TRO 2007
- Kim & Kim, "Scan Context", IROS 2018
- Qin et al., "VINS-Mono", TRO 2018（边缘化实现参考）
- Shan et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping", IROS 2020
- Segal et al., "Generalized-ICP", RSS 2009


> 分析日期：2026-03-05
> 代码路径：`src/core/`
> 涉及模块：loop_closing、localization/pose_graph（PGO）、maps（TiledMap）、system

---

## 一、回环检测模块（loop_closing）

### 1.1 当前不足

#### (1) 候选帧筛选只靠欧氏距离，无描述符预筛选

**代码位置：** `loop_closing.cc:DetectLoopCandidates()` 第 115-137 行

```cpp
Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
double t2d = dt.head<2>().norm();  // 只用 x-y 距离
if (t2d < range_th) {
    candidates_.emplace_back(c);
}
```

所有在 30m 范围内的历史帧都被列为候选，进入后续高代价 NDT 匹配。在长走廊、大厅等重复性结构场景下会产生大量误候选，而真正存在回环的帧也可能因点云外观不同被 NDT 误拒。

#### (2) 多分辨率 NDT 匹配无法并行，且对每个候选串行执行

**代码位置：** `loop_closing.cc:ComputeLoopCandidates()` 第 154 行

```cpp
std::for_each(candidates_.begin(), candidates_.end(),
              [this](LoopCandidate& c) { ComputeForCandidate(c); });
```

使用串行 `std::for_each`，多个候选帧的 NDT 计算无法并发。同时每次 `ComputeForCandidate` 都要现场构建 submap（遍历 ±40 帧），没有缓存，重复 I/O 和点云变换开销极大。

#### (3) submap 构建无缓存，每次全量重建

**代码位置：** `loop_closing.cc:ComputeForCandidate()` 第 176-207 行

```cpp
auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
    for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
        // 每次重建，无任何缓存
        auto kf = all_keyframes_[id];
        pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());
        *submap += *cloud_trans;
    }
};
```

`submap_idx_range=40` 意味着每次最多拼接 20 帧点云，当候选帧数量多时代价极高。

#### (4) 全局位姿图优化仅优化新加入顶点，历史帧固定

**代码位置：** `loop_closing.cc:PoseOptimization()` 第 274-371 行

每次 `PoseOptimization()` 只通过 `optimizer_->AddVertex(v)` 添加当前帧，历史帧顶点由增量优化器（`incremental_mode_=true`）管理。但增量模式下老顶点是否被正确更新取决于 miao 优化器的实现，代码注释中也有 `// TODO 3D激光最好是跟前面多个帧都有关联`，说明连接策略仍不完善。

#### (5) 回环检测结果未回传给定位系统（PGO）

回环检测在 SLAM 建图阶段优化了关键帧位姿，但定位系统（`loc_system`）中的 PGO 滑窗并不订阅回环优化结果。建图时地图的全局一致性提升了，但定位时 PGO 用的先验仍然是局部 LIO 轨迹，两者之间缺乏信息通路。

---

### 1.2 优化方案

#### 方案 A：引入点云描述符进行候选帧预筛选（推荐）

在 `DetectLoopCandidates()` 前增加描述符匹配阶段：

- **Scan Context**（[论文](https://arxiv.org/abs/2109.13494)）：构建方位-距离直方图描述符，查找 Top-K 相似帧；时间复杂度 O(N log N)，只对通过描述符检验的帧才做 NDT 验证。
- 实施步骤：
  1. 关键帧入库时同步建立 Scan Context 描述符索引（KD-tree 或 FAISS）；
  2. `DetectLoopCandidates()` 先做描述符最近邻搜索，将候选集缩减到 5 帧以内；
  3. 再做距离和 ID 间隔过滤；
  4. 最后做 NDT 验证。

```
候选帧数量：N → 描述符筛选 → K（K<<N） → NDT验证
```

#### 方案 B：submap 缓存（推荐）

```cpp
// 在 LoopClosing 中增加 submap cache
std::unordered_map<int, CloudPtr> submap_cache_;  // key: center kf id

CloudPtr GetOrBuildSubmap(int center_id) {
    auto it = submap_cache_.find(center_id);
    if (it != submap_cache_.end()) return it->second;
    auto submap = BuildSubmap(center_id, true);
    submap_cache_[center_id] = submap;
    return submap;
}
```

仅当关键帧位姿被回环修改时才失效对应缓存条目。

#### 方案 C：并行 NDT 匹配

```cpp
// 将串行 for_each 改为 tbb::parallel_for_each 或 std::async
std::vector<std::future<void>> futures;
for (auto& c : candidates_) {
    futures.emplace_back(std::async(std::launch::async,
        [this, &c]() { ComputeForCandidate(c); }));
}
for (auto& f : futures) f.get();
```

注意：需要保证 `ComputeForCandidate` 中只读 `all_keyframes_`，无写竞争。

#### 方案 D：回环结果反馈给 PGO

在 `LoopClosedCallback` 触发时，将经过全局优化的关键帧位姿作为绝对约束注入 PGO：

```cpp
loop_closing_->SetLoopClosedCB([this]() {
    // 取最近一次回环关键帧的优化位姿，作为 LidarLoc 绝对约束送入 PGO
    auto kf = loop_closing_->GetLastLoopKF();
    LocalizationResult lr;
    lr.pose_ = kf->GetOptPose();
    lr.confidence_ = 1.0;  // 回环结果高置信度
    pgo_->ProcessLidarLoc(lr);
});
```

---

## 二、位姿图优化模块（PGO）

### 2.1 当前不足

#### (1) 边缘化未实现（最严重问题）

**代码位置：** `pgo_impl.cc:Marginalize()` 第 845-854 行

```cpp
void PGOImpl::Marginalize(const PGOFramePtr& frame_to_remove,
                          const PGOFramePtr& frame_to_keep) {
    /// 严格来说，我们需要按照边缘化理论实现这套东西；
    /// TODO: @wgh 实现边缘化算法
    /// 但受限于时间，我们先极简处理一下，跑通流程要紧。

    frame_to_keep->prior_pose_ = frame_to_keep->opti_pose_;
    // frame_to_keep->prior_cov_;          // ← 协方差完全未设置
    frame_to_keep->prior_normalized_weight_ = Vec6d::Ones();
    frame_to_keep->prior_set_ = true;
    frame_to_keep->prior_valid_ = true;
}
```

当前边缘化等价于把下一帧的先验直接钉在当前优化位姿上，协方差 `prior_cov_` 从未赋值（保持 Identity 初值），这意味着先验约束的强弱与 `lidar_loc_noise_` 固定挂钩，无法反映历史信息的实际不确定度。正确的做法应是对滑出帧执行 Schur 补，将其信息阵边缘化到保留帧。

#### (2) DR 因子从未加入优化问题

**代码位置：** `pgo_impl.cc:BuildProblem()` 第 267-278 行

```cpp
void PGOImpl::BuildProblem() {
    AddVertex();
    if (is_in_map_) {
        AddLidarLocFactors();
    }
    AddLidarOdomFactors();
    AddPriorFactors();
    // AddDRFactors();  ← 此行从未被调用！
}
```

`AddDRFactors()` 函数存在且实现完整，但 `BuildProblem()` 从未调用它。DR（轮速/IMU 积分）作为帧间相对约束被完全丢弃，LidarOdom 退化时无法用 DR 补充。

#### (3) LidarOdom 退化检测逻辑失效

**代码位置：** `pgo_impl.cc:UpdateLidarOdomStatusInFrame()` 第 215-227 行

```cpp
void PGOImpl::UpdateLidarOdomStatusInFrame(NavState& lio_result,
                                            std::shared_ptr<PGOFrame> frame) {
    frame->lidar_odom_normalized_weight_ = lio_result.confidence_ * Vec6d::Ones();
    Eigen::Array3d trans_confidence = {1.0, 1.0, 1.0};  // ← 硬编码为1，永远不退化
    Eigen::Array3d rot_confidence = {1.0, 1.0, 1.0};    // ← 同上

    if ((trans_confidence < kLidarOdomTransDegenThres).any()) {
        frame->lidar_odom_trans_degenerated = true;  // 永远不会执行到
    }
    ...
}
```

`trans_confidence` 和 `rot_confidence` 被硬编码为全 1，LiDAR 退化判断条件永远不成立。实际应从 ESKF 的 Hessian 矩阵 PCA 分解中获取各方向的退化程度。

#### (4) RemoveOutliers() 未实现，lidar_loc outlier 检测被注释

**代码位置：** `pgo_impl.cc:RemoveOutliers()` 第 496-520 行；`UpdateFinalResultByWindow()` 第 682-699 行

`RemoveOutliers()` 函数体只有一个空 lambda，没有遍历 `lidar_loc_edges_`。outlier 的剔除完全依赖 Huber 核函数隐式降权，没有显式的两阶段优化（先优化 → 识别 outlier → 再优化）。

#### (5) 滑窗自适应逻辑被注释，固定使用简单策略

**代码位置：** `pgo_impl.cc:SlideWindowAdaptively()` 第 787-843 行

自适应窗口逻辑（根据收敛情况决定是否移除队头帧）全部注释掉，当前只要帧数达到 `PGO_MAX_FRAMES=5` 就无条件移除队头。窗口仅 5 帧，对激光定位 5Hz 的场景，覆盖时间约 1 秒，无法处理长时间激光失效场景。

#### (6) 外推逻辑中 LO 外推被禁用

**代码位置：** `pgo.cc:ExtrapolateLocResult()` 第 384-417 行

LO 外推和 LO-DR 冲突处理的代码段全部注释，实际只用 DR 做外推。当 DR（轮速计）失效时（地面打滑、抬起等），外推精度会急剧下降。

#### (7) 定位状态机全部注释

`UpdateFinalResultByWindow()` 和 `UpdateFinalResultByLastFrame()` 中所有 `GlobalPoseStatus` 赋值代码均注释，下游无法感知定位质量（是在融合多源还是仅靠 LO 递推）。

---

### 2.2 优化方案

#### 方案 A：实现正确的边缘化（核心）

对滑出帧 `frame_to_remove` 执行 Schur 补：

```
设保留帧变量为 x_k，被移除帧变量为 x_r
信息阵 H 分块：
    H = [H_rr  H_rk ]
        [H_kr  H_kk ]
边缘化后信息阵：H_new = H_kk - H_kr * H_rr^{-1} * H_rk
边缘化后信息向量：b_new = b_k - H_kr * H_rr^{-1} * b_r
```

具体实现参考 VINS-Mono 的 `marginalization_factor.cpp`，可在 `Marginalize()` 中通过 miao 优化器获取当前雅可比和残差，计算 Schur 补后存入 `frame_to_keep->prior_cov_`。

#### 方案 B：激活 DR 因子

在 `BuildProblem()` 中激活 DR 因子，并加入 LidarOdom 失效时的优先 DR 策略：

```cpp
void PGOImpl::BuildProblem() {
    AddVertex();
    if (is_in_map_) AddLidarLocFactors();
    AddLidarOdomFactors();
    AddDRFactors();        // ← 激活
    AddPriorFactors();
}
```

同时在 `AddLidarOdomFactors()` 中，当帧标记为 `lidar_odom_trans_degenerated` 时降低 LO 权重，提升 DR 权重。

#### 方案 C：修复 LidarOdom 退化检测

从 `NavState.confidence_` 或 ESKF 的 Hessian PCA 得到各轴置信度：

```cpp
// 在 ESKF 输出时附加退化信息
nav_state.degenerate_direction_ = pca_min_eigvec;  // 退化方向
nav_state.degenerate_ratio_ = min_eigenval / max_eigenval;  // 退化比例

// PGO 侧读取
Eigen::Array3d trans_confidence = {
    1.0 - std::max(0.0, 1.0 - lio_result.confidence_),
    // ... 按轴分量赋值
};
```

#### 方案 D：两阶段 outlier 剔除

```cpp
void PGOImpl::RunOptimization() {
    BuildProblem();
    optimizer_->InitializeOptimization();
    optimizer_->Optimize(5);          // 第一轮

    RemoveOutliers();                  // 标记 outlier 为 Level(1)

    optimizer_->InitializeOptimization();
    optimizer_->Optimize(5);          // 第二轮（不含 outlier）

    // 收集结果 ...
}

void PGOImpl::RemoveOutliers() {
    for (auto& e : lidar_loc_edges_) {
        e->ComputeError();
        if (e->Chi2() > options_.lidar_loc_outlier_th) {
            e->SetLevel(1);
            auto frame = frames_by_id_.find(e->GetVertex(0)->GetId())->second;
            frame->lidar_loc_inlier_ = false;
            LOG(WARNING) << "lidar_loc outlier at frame " << frame->frame_id_
                         << ", chi2=" << e->Chi2();
        }
    }
}
```

#### 方案 E：激活 LO 外推

在 `ExtrapolateLocResult()` 中恢复 LO 外推逻辑，优先级：LO 外推 > DR 外推：

```cpp
// 先用 LO 外推
if (impl_->lidar_odom_valid_ && !lo_pose_queue.empty() &&
    lo_pose_queue.back().timestamp_ > output_result.timestamp_) {
    // 原有注释代码恢复并修正
    ...
}
// LO 外推不足时，再用 DR 补充
```

---

## 三、分块地图模块（TiledMap）

### 3.1 当前不足

#### (1) 线程安全不完整

**代码位置：** `tiled_map.h` 注释 `// TODO: thread safety`

`static_data_mutex_` 和 `dynamic_data_mutex_` 分开管理，但 `LoadOnPose()` 等接口在加载静态块和动态块之间没有原子性保证。并发场景下可能读到静态已加载而动态未加载的不一致状态。

#### (2) 回环后地图未重绘

回环优化调整了历史关键帧位姿后，已经插入 `TiledMap` 的点云仍然保持旧位姿，导致地图局部存在双重叠影。当前没有"回环触发地图重绘"的机制。

#### (3) 动态地图 chunk 最大点数与卸载策略不联动

`max_pts_in_dyn_chunk_=50000`，但在 PERSISTENT 模式下动态点云只增不减，内存占用随时间线性增长，长时间运行后可能 OOM。

---

### 3.2 优化方案

#### 方案 A：回环触发地图重绘

在 `LoopClosedCallback` 中，遍历位姿改变量超过阈值（如 0.1m 或 0.5°）的关键帧对应的地图 chunk，标记为 dirty 并在后台线程中重绘：

```cpp
void System::OnLoopClosed() {
    for (auto& kf : loop_closing_->GetModifiedKeyframes()) {
        SE3 delta = kf->GetLIOPose().inverse() * kf->GetOptPose();
        if (delta.translation().norm() > 0.1 || delta.so3().log().norm() > 0.01) {
            tiled_map_->MarkChunkDirty(kf->GetOptPose());
        }
    }
    tiled_map_->RebuildDirtyChunks();  // 后台线程执行
}
```

#### 方案 B：动态地图时间戳老化

为动态点云增加时间戳，定期清理超过 TTL（如 30 分钟）的旧点：

```cpp
struct DynPoint {
    Vec3f pt;
    double timestamp;
};
// 周期性调用
void TiledMap::PurgeStaleDynamicPoints(double ttl_seconds) {
    double now = GetCurrentTime();
    for (auto& [idx, chunk] : dynamic_chunks_) {
        chunk->RemovePointsOlderThan(now - ttl_seconds);
    }
}
```

---

## 四、系统层面问题汇总

### 4.1 当前不足

| 问题 | 位置 | 严重程度 |
|------|------|----------|
| 边缘化未实现，先验约束不正确 | `pgo_impl.cc:Marginalize()` | 高 |
| DR 因子未加入优化 | `pgo_impl.cc:BuildProblem()` | 高 |
| LidarOdom 退化检测硬编码，永远不退化 | `pgo_impl.cc:UpdateLidarOdomStatusInFrame()` | 高 |
| Outlier 剔除未实现 | `pgo_impl.cc:RemoveOutliers()` | 中 |
| 回环结果未回传 PGO | `loop_closing.h` / `loc_system` | 中 |
| 回环候选无描述符预筛选 | `loop_closing.cc:DetectLoopCandidates()` | 中 |
| LO 外推被禁用 | `pgo.cc:ExtrapolateLocResult()` | 中 |
| 定位状态机全注释，下游无质量感知 | `pgo_impl.cc:UpdateFinalResultByWindow()` | 中 |
| submap 构建无缓存 | `loop_closing.cc:ComputeForCandidate()` | 低 |
| 动态地图内存持续增长 | `tiled_map.h` | 低 |
| 回环后地图未重绘 | `tiled_map.cc` | 低 |

### 4.2 建议优先级排序

**P0（影响定位精度和稳定性，必须修复）：**
1. 修复 `Marginalize()`：实现正确 Schur 补
2. 在 `BuildProblem()` 中激活 `AddDRFactors()`
3. 修复 LidarOdom 退化检测（读取真实 PCA 置信度）

**P1（影响长时间运行质量）：**
4. 实现 `RemoveOutliers()`，并改为两阶段优化
5. 恢复 LO 外推逻辑
6. 将回环结果反馈给 PGO

**P2（提升性能和可观测性）：**
7. 引入 Scan Context 描述符筛选候选帧
8. submap 构建加缓存
9. 恢复定位状态机
10. 动态地图老化机制

---

## 五、参考文献

- Grisetti et al., "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters", TRO 2007
- Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition", IROS 2018
- Qin et al., "VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator", TRO 2018（边缘化实现参考）
- He et al., "M2DGR: A Multi-sensor and Multi-scenario SLAM Dataset for Ground Robots", RAL 2022
