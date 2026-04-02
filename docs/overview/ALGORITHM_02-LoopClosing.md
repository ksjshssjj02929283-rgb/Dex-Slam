# lightning-lm 回环检测与位姿图优化文档 (Loop Closing & PGO)

> 参考实现风格：LIO-SAM (Shan et al., 2020)  
> 所有论断基于实际代码，带 `[VERIFY: file:line]` 标注。

---

## 一、算法概述

回环检测模块（`LoopClosing`）负责：
1. **候选帧检测**：对每个新关键帧，通过 KD-Tree 在历史帧中搜索位置相近但时间上已足够久远的帧
2. **几何验证**：构建子地图，用 NDT（粗对齐）+ GICP（精对齐）验证是否为真回环
3. **位姿图优化（PGO）**：将验证通过的回环约束与里程计约束共同喂给 miao 增量优化器
4. **位姿传播**：PGO 完成后更新所有关键帧的 `pose_opt_`，并触发 g2p5 地图重绘

---

## 二、整体流程图

```
新关键帧 kf 到来
    │
    ▼ HandleKF() [LoopClosing 异步线程]
    │
    ├─► 更新历史位置点云 kf_poses_cloud_
    │
    ├─► DetectLoopCandidates(kf)
    │       ├── KD-Tree 半径搜索 (max_range=30m)
    │       ├── 时间差过滤 (> history_search_time_diff=30s)
    │       ├── 去重 (loop_index_container_)
    │       └── 冷却期检查 (loop_kf_gap_)
    │           → candidates: vector<LoopCandidate>
    │
    ├─► ComputeLoopCandidates(candidates) [ThreadPool 并行]
    │       └── for each candidate:
    │               ComputeForCandidate(c)
    │                   ├── LoopFindNearKeyframes(kf1, ±N) → submap (kf1 局部系)
    │                   ├── NDT 粗配准 (可选，仅大场景)
    │                   └── GICP 精配准
    │                       → c.Tij_, c.icp_score_, c.valid_
    │
    ├─► AddOdomFactor(kf)
    │       └── 加入里程计边 (prev_kf → kf，pose_opt_ 差值)
    │
    ├─► AddLoopFactor(valid candidates)
    │       └── 写入 loop_pairs_idx_/pose_/score_ 三队列
    │
    └─► PoseOptimization() [若满足条件]
            ├── 取出三队列中所有 pending 回环约束
            ├── optimizer_.Optimize()  (miao 增量 LM)
            ├── 读取优化结果 → kf->SetOptPose()
            └── g2p5_->RedrawGlobalMapBackend() [若已开启]
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:143-165]** — `HandleKF()` 顶层调用链

---

## 三、候选帧检测（DetectLoopCandidates）

**[VERIFY: src/core/loop_closing/loop_closing.cc:215-285]**

### 3.1 KD-Tree 搜索

```cpp
// 当前关键帧的世界位置
pcl::PointXYZ pt;
pt.x = kf_cur->pose_lio_.translation().x();
pt.y = kf_cur->pose_lio_.translation().y();
pt.z = kf_cur->pose_lio_.translation().z();  // 若 with_height_=false，z 固定为0

// 搜索
std::vector<int>   indices;
std::vector<float> dists;
kdtree_history_poses_.radiusSearch(pt, max_range_, indices, dists);
```

`max_range_` 默认 30m。若 `with_height_=false`（多层场景），将所有历史位置的 z 置零后搜索，避免不同楼层误匹配。

### 3.2 时间差过滤

```cpp
// 只取历史上足够早的关键帧（排除同一轨迹段）
for (int idx : indices) {
    auto kf_prev = all_keyframes_[idx];
    double time_diff = kf_cur->timestamp_ - kf_prev->timestamp_;
    if (time_diff < history_search_time_diff_) continue;  // 默认 30s
    // ...
}
```

### 3.3 去重与冷却期

```cpp
// 去重：同一当前帧对应多个候选时，只取最近的
if (loop_index_container_.count(kf_cur->id_)) continue;
loop_index_container_[kf_cur->id_] = kf_prev->id_;  // 记录

// 冷却期：距上次回环触发 > loop_kf_gap_ 帧
if (kf_cur->id_ - last_loop_kf_id_ < loop_kf_gap_) continue;  // 默认50帧
```

**注意**：`last_loop_kf_id_` 在第一个 KF 到来时初始化，**防止系统启动即触发回环**（这是曾出现的 bug）。

---

## 四、子地图构建（LoopFindNearKeyframes）

**[VERIFY: src/core/loop_closing/loop_closing.cc:290-340]**

```cpp
// kf1: 候选历史关键帧（中心帧）
// kf2: 当前关键帧
// history_keyframe_search_num: 前后各取 N 帧（默认25）

void LoopFindNearKeyframes(KFPtr kf1, CloudPtr& submap_out) {
    SE3 T_center_inv = kf1->GetLIOPose().inverse();  // kf1 局部坐标系
    
    for (int i = -N; i <= N; i++) {
        int id = kf1->id_ + i;
        if (id < 0 || id >= all_keyframes_.size()) continue;
        auto kf = all_keyframes_[id];
        
        // 关键：以 kf1 局部坐标系为基准做坐标变换！
        // 避免 PCL VoxelGrid 对大坐标整数溢出（`Integer indices would overflow`）
        SE3 T_rel = T_center_inv * kf->GetLIOPose();  // 相对变换
        
        CloudPtr cloud_transformed;
        pcl::transformPointCloud(*kf->cloud_, *cloud_transformed, T_rel.matrix().cast<float>());
        *submap_out += *cloud_transformed;
    }
    
    // VoxelGrid 降采样（叶子大小 icp_filter_size_，默认0.4m）
    VoxelGrid<PointType> vg;
    vg.setLeafSize(icp_filter_size_, icp_filter_size_, icp_filter_size_);
    vg.filter(*submap_out);
}
```

**关键设计**：子地图在 kf1 的**局部坐标系**中构建，而非世界坐标系。原因：
1. 世界坐标系可能有大值（如 100m+），PCL VoxelGrid 内部用整数索引，会溢出
2. NDT/GICP 在接近原点的坐标系中精度更高

---

## 五、NDT/GICP 配准（ComputeForCandidate）

**[VERIFY: src/core/loop_closing/loop_closing.cc:400-520]**

### 5.1 source 与 target 坐标系

```
target (prev_cloud)：kf1 局部坐标系的子地图
source (cur_cloud) ：kf2 的原始点云（body/IMU 坐标系，不做变换）

初始猜测（initial_guess）：kf1->GetLIOPose().inverse() * kf2->GetLIOPose()
  → 这是 kf2 相对于 kf1 局部系的相对位姿
  → 与 target 坐标系一致 ✓
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:430-445]** — 初始猜测计算

### 5.2 NDT 粗配准（可选）

```cpp
// 仅在场景较大时（历史轨迹长度 > ndt_min_dist_）启用 NDT 粗配准
ndt_.setResolution(ndt_resolution_);     // 默认1.0m
ndt_.setStepSize(ndt_step_size_);
ndt_.setMaximumIterations(ndt_max_iter_);
ndt_.setInputTarget(prev_cloud);
ndt_.setInputSource(cur_cloud);
ndt_.align(*aligned, initial_guess.matrix().cast<float>());

// NDT 结果作为 GICP 初始猜测
SE3 ndt_result(ndt_.getFinalTransformation().cast<double>());
```

### 5.3 GICP 精配准

```cpp
pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
gicp.setMaxCorrespondenceDistance(gicp_max_corr_dist_);  // 默认1.0m
gicp.setMaximumIterations(gicp_max_iter_);                // 默认50
gicp.setInputTarget(prev_cloud);
gicp.setInputSource(cur_cloud);
gicp.align(*aligned, ndt_result.matrix().cast<float>());  // 或直接用 initial_guess

float icp_score = gicp.getFitnessScore();
bool  converged = gicp.hasConverged();
```

### 5.4 验证与 Tij 提取

```cpp
// 验证条件
if (!converged || icp_score > icp_score_threshold_) {
    c.valid_ = false;
    return;
}

// Tij = GICP 结果（已在 kf1 局部系）
// 注意：无需再乘 kf1->GetLIOPose().inverse()，已经是正确坐标系
c.Tij_ = SE3(gicp.getFinalTransformation().cast<double>());
c.icp_score_ = icp_score;
c.valid_ = true;
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:495-520]** — 验证与结果存储

---

## 六、位姿图优化（PoseOptimization）

**文件**：`src/core/loop_closing/loop_closing.cc`

### 6.1 触发条件

```cpp
bool LoopClosing::ShouldRunOptimization() {
    // 有新的回环约束，或
    if (!loop_pairs_idx_.empty()) return true;
    // 每 optimize_every_n_kf_ 帧运行一次（默认10帧）
    if (kf_count_ % optimize_every_n_kf_ == 0) return true;
    return false;
}
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:540-560]**

### 6.2 因子图结构

```
因子图：
  V0 ——(O01)—— V1 ——(O12)—— V2 ——(O23)—— V3 ...
                                              │
                                           (L03) ← 回环边（虚线）
                                              │
  V0 ──────────────────────────────────────── ┘

V_i  = VertexSE3（关键帧位姿，SE3）
O_ij = EdgeSE3（里程计边，pose_opt_ 差值）
L_ij = EdgeSE3（回环边，Tij，带 Cauchy 核）
V_0  = 固定（先验锚点，fix_first_node_=true）
```

### 6.3 里程计边（AddOdomFactor）

```cpp
// 添加里程计约束
SE3 relative_pose = prev_kf->GetOptPose().inverse() * kf->GetOptPose();
EdgeSE3* e = new EdgeSE3(prev_kf->id_, kf->id_, relative_pose);
e->SetInformation(odom_info_);  // odom_noise 的逆
optimizer_.AddEdge(e);
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:570-590]**

### 6.4 回环边（AddLoopFactor）

```cpp
// 从三队列取出 pending 回环
for (int i = 0; i < loop_pairs_idx_.size(); i += 2) {
    int id_cur  = loop_pairs_idx_[i];
    int id_prev = loop_pairs_idx_[i+1];
    SE3 Tij     = loop_pairs_pose_[i/2];
    float score = loop_pairs_score_[i/2];
    
    // 自适应噪声：NDT 得分越小（匹配越好）→ 噪声越小 → 约束越强
    double noise_scale = 0.2 + score * 0.1;
    MatXD loop_info = (loop_noise_ * noise_scale).inverse();
    
    EdgeSE3* e = new EdgeSE3(id_cur, id_prev, Tij);
    e->SetInformation(loop_info);
    e->SetRobustKernel(new CauchyKernel(rk_loop_th_));  // 鲁棒核
    optimizer_.AddEdge(e);
}
loop_pairs_idx_.clear();  // 消费后清除
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:595-630]**

### 6.5 miao 增量 LM 优化

```cpp
optimizer_.Optimize(max_iter_);   // 默认10次 LM 迭代

// 读取优化结果并更新所有关键帧 pose_opt_
for (auto& kf : all_keyframes_) {
    SE3 opt = optimizer_.GetVertexEstimate(kf->id_);
    kf->SetOptPose(opt);  // mutex 保护写入
}

// 通知 g2p5 重绘（若开启）
if (g2p5_) g2p5_->RedrawGlobalMapBackend();
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:635-660]**

---

## 七、调试数据发布（loop_debug_data_）

**[VERIFY: src/core/loop_closing/loop_closing.cc:455-478]**

回环检测时可发布调试点云（通过 `pub_loop_debug_`），坐标系转换规则：

| 数据 | 坐标系 | 变换 |
|------|--------|------|
| `loop_submap` | **世界系** | `kf1->GetLIOPose() * p_kf1_local` |
| `aligned_scan` | **世界系** | `kf1->GetLIOPose() * Tij * p_body` |
| `current_scan` | body 系 | 不变，UI 侧乘 `current_kf_pose` 渲染 |
| `aligned_pose` | — | `SE3::Identity()`（点云已在 map 系） |

---

## 八、关键参数与调优

| 参数 | 默认值 | 作用 |
|------|--------|------|
| `max_range` | 30m | 回环候选搜索半径，越大越容易触发，误匹配也增加 |
| `loop_kf_gap` | 50 | 回环冷却期（关键帧数），避免频繁触发 |
| `history_search_time_diff` | 30s | 时间差阈值，越大越不易误匹配同一轨迹段 |
| `history_keyframe_search_num` | 25 | 子地图前后各取帧数，越大子地图越完整，越慢 |
| `icp_filter_size` | 0.4m | 子地图 VoxelGrid 叶大小，太小影响速度，太大影响精度 |
| `icp_score_threshold` | 0.3 | GICP 得分阈值，越小越严格 |
| `with_height` | true | 是否考虑高度，多层场景需关闭 |
| `optimize_every_n_kf` | 10 | 无回环时定期优化间隔 |
| `rk_loop_th` | 5.2 | Cauchy 核参数（平移/m），越小对外点越鲁棒 |

---

## 九、常见问题排查

### 9.1 系统启动即触发回环

**现象**：第很少关键帧时就产生大量回环  
**原因**：首帧检测时 `kf_poses_cloud_` 非空（有历史状态残留）  
**解决**：在 `DetectLoopCandidates()` 入口加 `last_loop_kf_id_ == -1` 初始化守卫

### 9.2 VoxelGrid 整数溢出

**现象**：`[pcl::VoxelGrid] Integer indices would overflow`  
**原因**：子地图在世界坐标系（坐标值>100m），VoxelGrid 内部用 int32 索引  
**解决**：`LoopFindNearKeyframes` 中所有点必须变换到 **kf1 局部坐标系**

### 9.3 NDT 得分全为 0（所有候选匹配失败）

**现象**：`ndt_score=0.0` for all candidates  
**原因**：initial_guess 坐标系与 target/source 不一致（经典错误：`kf1->GetLIOPose() * kf2->GetLIOPose().inverse()` vs 正确的 `kf1->GetLIOPose().inverse() * kf2->GetLIOPose()`）  
**解决**：确认 initial_guess = `T_center_inv * kf2->GetLIOPose()`，与 target（kf1 局部系）一致

### 9.4 YAML 新参数 crash

**现象**：新增参数后启动直接崩溃，`YAML::InvalidNode`  
**原因**：`yaml.GetValue<T>()` 未加异常保护  
**解决**：
```cpp
try {
    auto val = yaml.GetValue<T>("section.param", default_val);
} catch (const YAML::Exception& e) {
    // 使用默认值
}
```

---

## 十、LoopClosing 多线程安全性

| 共享资源 | 写入方 | 读取方 | 保护方式 |
|----------|--------|--------|----------|
| `all_keyframes_` | LaserMapping 主线程 | LoopClosing kf_thread_ | `deque` 仅 push_back，顺序访问安全 |
| `kf->pose_opt_` | LoopClosing kf_thread_ | G2P5, RViz, saving | `Keyframe::mutex_` |
| `loop_pairs_*` | `ComputeLoopCandidates` pool | `PoseOptimization` | 函数调用顺序保证（单线程消费） |
| `kf_poses_cloud_` | `HandleKF` | kdtree 搜索 | 单线程（kf_thread_内部） |
| `optimizer_` | `AddOdomFactor`/`AddLoopFactor` | `PoseOptimization` | 单线程（kf_thread_内部） |
