# Lightning-LM Submap 架构设计方案

**目标**：参考 Cartographer 的 submap 机制，在 lightning-lm 的 LaserMapping（前端）和 LoopClosing（后端）中引入 submap 管理，使回环优化后**仅更新 submap 外参（T_world_submap），不重建 iVox/VoxelMap 内部数据结构**。

---

## 1. 现状分析

### 1.1 当前架构

```
LiDAR/IMU
  │
  ▼
LaserMapping (LIO 前端)
  ├── ESKF (IMU 预测 + 激光观测更新)
  ├── ivox_manager_  ──┐ 一个全局 IVox/VoxelMap
  ├── voxel_map_     ──┘ 所有点都在 world 坐标系，无 submap 概念
  └── MakeKF() → kf 包含 cloud(body) + LIO pose(world)
  │
  ▼ Keyframe
LoopClosing (后端)
  ├── AddOdomFactor() → 因子图
  ├── DetectLoopCandidates() → NDT/GICP
  ├── PoseOptimization() → miao 增量 PGO
  └── 回调 → g2p5 重绘全局地图
```

**现有问题**：
1. 前端只有一个全局 iVox/VoxelMap，所有点在 world 坐标系中
2. 回环修正后 opt_pose 变了，但 iVox/VoxelMap 中的点仍在**旧 world 坐标系**
3. 全局地图导出（`GetGlobalMap`）需要用 opt_pose 重新拼接所有关键帧点云
4. 定位时加载的 TiledMap 与建图时的 iVox 是独立的（非 submap 管理）

### 1.2 Cartographer 的 submap 设计精髓

```
[VERIFY: cartographer/mapping/submaps.h:62-80]
class Submap {
  const Rigid3d local_pose_;    // 固定不变的内部坐标系锚点
  bool insertion_finished_;
};

[VERIFY: cartographer/mapping/internal/optimization/optimization_problem_2d.h:52-54]
struct SubmapSpec2D {
  Rigid2d global_pose;          // 唯一可被优化的变量
};
```

Cartographer 的核心：**地图内容存在 submap 局部坐标系中，global_pose 独立于地图内容存储**。优化只改 global_pose，Grid2D 的每个像素都不动。

---

## 2. 方案总体设计

### 2.1 设计原则

1. **位姿-内容解耦**：submap 内部 iVox/VoxelMap 使用**局部坐标系**，外部用 `T_world_submap` 变换到世界
2. **最小侵入**：保留现有 IVoxManager / VoxelMapManager 接口不变，上层加管理层
3. **渐进式切换**：支持 single-submap 降级模式（等效当前行为）
4. **前后端一体**：前端、后端、保存/加载共用同一套 Submap 索引

### 2.2 新增数据结构

```cpp
// ======================== src/common/submap3d.h ========================

namespace lightning {

using SubmapId = uint32_t;  // 全局唯一，从 0 单调递增

/// 3D 子图：内部携带一个独立的 IVox 或 VoxelMap
struct Submap3D {
    SubmapId id;

    // ---- 位姿（解耦设计核心）----
    SE3 local_pose;        // submap 坐标系原点在 world 中的初始位姿（创建时锁死）
    SE3 global_pose;       // 可优化的全局位姿（初始 = local_pose，后端可修正）

    // ---- 地图内容（submap 局部坐标系）----
    std::shared_ptr<IVoxManager>              ivox;       // 可选
    std::shared_ptr<voxel_map::VoxelMapManager> voxelmap; // 可选

    // ---- 元数据 ----
    std::vector<uint64_t> kf_ids;          // 插入到此 submap 的关键帧 ID
    int num_insertions = 0;                // 已插入帧数（用于判断 finish 条件）
    bool insertion_finished = false;       // 是否已完成（类比 Cartographer）
    double creation_time = 0.0;            // 创建时间戳

    // ---- 便捷方法 ----
    /// world → submap 局部坐标系的变换
    SE3 WorldToLocal() const { return global_pose.inverse(); }

    /// submap 局部 → world 的变换
    SE3 LocalToWorld() const { return global_pose; }
};

} // namespace lightning
```

### 2.3 Submap 管理器（核心新增类）

```cpp
// ======================== src/core/maps/submap_manager.h ========================

namespace lightning {

class SubmapManager {
public:
    struct Options {
        int max_kf_per_submap = 50;       // 每个 submap 最大关键帧数
        int num_active_submaps = 2;        // 活跃 submap 数量（类比 Cartographer 的 2 个滑动窗口）
        bool use_ivox = true;
        bool use_voxel_map = true;
        IVoxConfig ivox_config;
        voxel_map::VoxelMapConfig vmap_config;
    };

    SubmapManager(Options opts);

    /// 获取当前应该插入数据的活跃 submap 列表
    std::vector<Submap3D*> GetActiveSubmaps();

    /// 获取匹配用的 submap（当前 + 前一个）
    Submap3D* GetMatchingSubmap();

    /// 插入一帧点云到活跃 submap（由 LaserMapping 调用）
    void InsertScan(const CloudPtr& cloud_world, const SE3& pose_world,
                    uint64_t kf_id, double timestamp);

    /// 通知匹配完成，推进 submap 状态机
    void NotifyMatchDone();

    /// 按 ID 获取 submap
    Submap3D* GetSubmap(SubmapId id);

    /// 获取所有 submap（后端需要）
    const std::vector<Submap3D>& GetAllSubmaps() const;

    /// 更新 submap 的 global_pose（后端优化后调用）
    void UpdateGlobalPose(SubmapId id, const SE3& new_global_pose);

    /// 获取当前 submap ID
    SubmapId CurrentSubmapId() const;

    /// 查找覆盖给定位置的 submap 集合（用于匹配）
    std::vector<SubmapId> FindSubmapsNear(const Vec3d& position, double radius);

private:
    void CreateNewSubmap(const SE3& origin_pose, double timestamp);
    void FinishSubmap(SubmapId id);

    Options options_;
    std::vector<Submap3D> submaps_;
    SubmapId next_id_ = 0;
};

} // namespace lightning
```

---

## 3. 前端改造（LaserMapping）

### 3.1 当前前端流程 vs 新流程

```
=== 当前（无 submap）===

ProcessPointCloud2 → SyncPackages → IMU Process
  → ESKF Predict
  → ObsModel（在全局 ivox/voxelmap 中找匹配）
  → ESKF Update → 得到 state_point_（world 系 pose）
  → MapIncremental（world 系点插入全局 ivox/voxelmap）
  → MakeKF

=== 改造后（submap）===

ProcessPointCloud2 → SyncPackages → IMU Process
  → ESKF Predict
  → ObsModel（在 matching_submap 中找匹配，坐标变换到 submap local 系）  ← 改动 1
  → ESKF Update → 得到 state_point_（world 系 pose）
  → MapIncremental（将点变换到 submap local 系后插入活跃 submap）        ← 改动 2
  → MakeKF（额外记录该帧属于哪个 submap）                                ← 改动 3
  → 检查是否需要切换 submap                                              ← 改动 4
```

### 3.2 关键改动详解

#### 改动 1：ObsModel — 在 submap 局部坐标系中做匹配

```
当前代码 [VERIFY: laser_mapping.cc:616-793]：
  point_world = pose * point_body;
  ivox_manager_->FindCorrespondences(scan_down_world, ...);  // 全局 world 系

改造后：
  matching_submap = submap_manager_->GetMatchingSubmap();
  SE3 T_submap_body = matching_submap->WorldToLocal() * T_world_body;

  // 将点变换到 submap 局部坐标系（而非 world）
  for (point_body) → point_submap = T_submap_body * point_body;

  // 在 submap 的 ivox/voxelmap 中找匹配
  matching_submap->ivox->FindCorrespondences(points_submap, ...);
  
  // 残差和 Jacobian 仍然对 world 系的 state 求导
  // 但匹配面片（对应点/平面参数）是 submap local 系的，需变换回 world 系
  // plane_normal_world = matching_submap->global_pose.rotationMatrix() * plane_normal_submap;
```

> **注意**：ESKF 的状态量仍然在 world 坐标系，只是"找最近点"的操作在 submap local 系中进行。残差表达式中，法向量和匹配点需通过 `global_pose` 变换回 world 系参与 Jacobian 计算。

#### 改动 2：MapIncremental — 插入到活跃 submap 的局部坐标系

```cpp
// 当前代码 [VERIFY: laser_mapping.cc:581-600]：
void LaserMapping::MapIncremental() {
    ivox_manager_->UpdateMap(scan_down_world_->points, ...);  // world 系
}

// 改造后：
void LaserMapping::MapIncremental() {
    auto active_submaps = submap_manager_->GetActiveSubmaps();
    for (auto* submap : active_submaps) {
        // 将世界系点云变换到 submap 局部坐标系
        CloudPtr scan_in_submap(new PointCloudType);
        pcl::transformPointCloud(*scan_down_world_, *scan_in_submap,
                                  submap->WorldToLocal().matrix());
        
        if (submap->ivox) {
            submap->ivox->UpdateMap(scan_in_submap->points, nearest_points_,
                                     flg_EKF_inited_, /* local pose */);
        }
        if (submap->voxelmap) {
            // VoxelMap 需要 pointWithCov，同样变换到 local 系
            auto pv_local = TransformPvListToLocal(pv_list_, submap->WorldToLocal());
            submap->voxelmap->UpdateMap(pv_local);
        }
    }
}
```

#### 改动 3：MakeKF — 记录所属 submap

```cpp
// Keyframe 新增字段
class Keyframe {
    // ... 现有字段 ...
    SubmapId submap_id_ = 0;  // 该关键帧所属的 submap
};

// MakeKF 中：
void LaserMapping::MakeKF() {
    auto kf = std::make_shared<Keyframe>(...);
    kf->SetSubmapId(submap_manager_->CurrentSubmapId());
    submap_manager_->InsertScan(scan_down_world_, state_point_.GetPose(),
                                 kf->GetID(), state_point_.timestamp_);
    // ...
}
```

#### 改动 4：Submap 切换策略

```
触发条件（类比 Cartographer ActiveSubmaps2D::InsertData）：
  if (current_submap->num_insertions >= max_kf_per_submap) {
      FinishSubmap(current_submap->id);
      CreateNewSubmap(T_world_body, timestamp);
  }

滑动窗口管理（保持 2 个活跃 submap）：
  - 新 submap 创建后，旧 submap 仍接受最后几帧的插入（overlap 区间）
  - 直到旧 submap 达到 2 × max_kf_per_submap，标记为 finished
  - finished 后不再接受新数据，但仍可用于 scan matching（如果在近邻范围内）
```

---

## 4. 后端改造（LoopClosing）

### 4.1 因子图扩展

```
当前因子图（仅节点）：
  Node(kf0) ─motion─ Node(kf1) ─motion─ Node(kf2) ...
       │                                       │
       └──────────── loop ──────────────────────┘

改造后因子图（节点 + submap）：
  Submap(s0)           Submap(s1)           Submap(s2)
    ↕ intra              ↕ intra              ↕ intra
  Node(kf0) ─motion─ Node(kf3) ─motion─ Node(kf6) ...
       │                                       │
       └──────────── loop ──────────────────────┘

约束类型：
  1. INTRA_SUBMAP: T_submap_node（节点在 submap 内的相对位姿，与 Cartographer INTRA 一致）
  2. INTER_NODE:   T_node1_node2（相邻节点的运动约束，保留现有 motion edge）
  3. LOOP:         T_submap_node（回环约束，连接 submap 和非归属节点）
```

### 4.2 Submap 顶点加入因子图

```cpp
void LoopClosing::AddOdomFactor(Keyframe::Ptr kf) {
    // === 现有逻辑保留：添加节点顶点 + 运动边 + 高度约束 ===
    // [VERIFY: loop_closing.cc AddOdomFactor]
    
    // === 新增：submap 顶点 ===
    SubmapId sid = kf->GetSubmapId();
    if (optimizer_->GetVertex(SubmapVertexId(sid)) == nullptr) {
        auto v_submap = std::make_shared<miao::VertexSE3>();
        v_submap->SetId(SubmapVertexId(sid));
        v_submap->SetEstimate(submap_manager_->GetSubmap(sid)->global_pose);
        optimizer_->AddVertex(v_submap);
        submap_verts_.emplace_back(v_submap);
    }
    
    // === 新增：INTRA_SUBMAP 边 ===
    auto* submap = submap_manager_->GetSubmap(sid);
    SE3 T_submap_node = submap->WorldToLocal() * kf->GetOptPose();
    auto e_intra = std::make_shared<miao::EdgeSE3>();
    e_intra->SetVertex(0, optimizer_->GetVertex(SubmapVertexId(sid)));
    e_intra->SetVertex(1, optimizer_->GetVertex(kf->GetID()));
    e_intra->SetMeasurement(T_submap_node);
    e_intra->SetInformation(info_intra_submap_);  // 高权重，INTRA 约束
    optimizer_->AddEdge(e_intra);
}
```

> **Vertex ID 设计**：Node ID 使用 kf_id（0 起），Submap Vertex ID 使用 `SUBMAP_ID_OFFSET + sid`（e.g. `1000000 + sid`），避免 ID 冲突。

### 4.3 回环检测 — 基于 submap 的匹配

```
当前流程 [VERIFY: loop_closing.cc ComputeForCandidate]：
  1. LoopFindNearKeyframes(prev_cloud, kf1_idx, N) → 世界系 submap
  2. cur_cloud = kf2->GetCloud() (body 系)
  3. GICP(target=prev_cloud_world, source=cur_cloud_body, init=kf2_opt_pose)
  4. Tij = kf1_opt_pose.inv * SE3(T_kf2_corrected)

改造后流程（可选，渐进升级）：
  Phase 1（最小改动）：保留当前逻辑，只是回环结果额外更新 submap global_pose
  Phase 2（完整改动）：利用 submap 内部 iVox/VoxelMap 做匹配，替代临时拼点云
```

**Phase 1（推荐先实现）**：
- 回环检测逻辑完全不变
- `PoseOptimization()` 后，除了写回 node 的 opt_pose，**额外写回 submap 的 global_pose**

```cpp
void LoopClosing::PoseOptimization() {
    // ... 现有优化逻辑 ...
    
    // === 新增：写回 submap global_pose ===
    for (auto& sv : submap_verts_) {
        SubmapId sid = sv->GetId() - SUBMAP_ID_OFFSET;
        submap_manager_->UpdateGlobalPose(sid, sv->Estimate());
    }
    
    // 现有逻辑保留
    for (auto& vert : kf_vert_) {
        all_keyframes_[vert->GetId()]->SetOptPose(vert->Estimate());
    }
    
    RebuildHistoryPoses();
    // ...
}
```

**Phase 2（后续升级）**：
- `LoopFindNearKeyframes` 改为直接从历史 submap 的 iVox/VoxelMap 中取匹配
- 无需临时拼接点云，利用 submap 内部的空间索引结构做 scan-to-submap 匹配

### 4.4 优化后的位姿传播

```
优化后 submap global_pose 变了，对前端的影响：

1. 当前 active submap 的 global_pose 更新后：
   → ObsModel 中 T_submap_body 自动用新 global_pose 计算
   → 前端下一帧配准自然平滑过渡

2. 历史 submap 的 global_pose 更新后：
   → 内部 iVox/VoxelMap 数据不动（仍在旧 local 系中）
   → 导出地图时用 new_global_pose * local_points 即可得到正确的世界坐标

3. 定位系统（TiledMap）加载时：
   → 每个 chunk 已自带位姿信息，加载后用 submap 的 global_pose 变换
```

---

## 5. 全局地图导出

### 5.1 当前方式

```cpp
// [VERIFY: laser_mapping.cc GetGlobalMap]
// 遍历所有关键帧，用 lio_pose 或 opt_pose 变换 body 点云到 world 拼合
```

### 5.2 改造后

```cpp
CloudPtr GetGlobalMap() {
    CloudPtr global(new PointCloudType);
    for (auto& submap : submap_manager_->GetAllSubmaps()) {
        // 方式 A：从 submap 的 iVox 中直接导出 local 系点云
        CloudPtr local_cloud = submap.ivox->ExportPoints();  // 需新增接口
        CloudPtr world_cloud(new PointCloudType);
        pcl::transformPointCloud(*local_cloud, *world_cloud, submap.global_pose.matrix());
        *global += *world_cloud;
    }
    return VoxelGrid(global, 0.1f);
}
```

优势：
- 直接用 submap 内部的 iVox/VoxelMap 数据，无需重新遍历关键帧
- 回环优化后只需用新 global_pose 变换，不重建体素结构

---

## 6. 坐标系设计

```
world (W)           ← ESKF 的全局坐标系，与 IMU 第一帧对齐
  │
  ├── submap_0 (S0)  ← T_world_s0 = global_pose_0（初始 = 创建时的 robot pose）
  │     │
  │     └── 点/体素在 S0 局部坐标系中
  │
  ├── submap_1 (S1)  ← T_world_s1 = global_pose_1
  │     │
  │     └── 点/体素在 S1 局部坐标系中
  │
  ├── submap_2 (S2)  ← T_world_s2 = global_pose_2
       ...

坐标变换公式：
  point_world = T_world_submap * point_submap
  point_submap = T_world_submap.inv * point_world
  point_submap = T_world_submap.inv * T_world_body * point_body
```

---

## 7. 与现有模块的接口变化

### 7.1 LaserMapping

| 成员 | 当前 | 改造后 |
|------|------|--------|
| `ivox_manager_` | 全局唯一 | 移除，由 SubmapManager 管理 |
| `voxel_map_` | 全局唯一 | 移除，由 SubmapManager 管理 |
| `submap_manager_` | 无 | **新增** `shared_ptr<SubmapManager>` |
| `ObsModel()` | world 系查找 | submap local 系查找 + 法向量变换 |
| `MapIncremental()` | world 系插入 | submap local 系插入 |
| `MakeKF()` | 无 submap 信息 | 关联 submap_id |

### 7.2 LoopClosing

| 成员 | 当前 | 改造后 |
|------|------|--------|
| `submap_manager_` | 无 | **新增** `shared_ptr<SubmapManager>`（与 LaserMapping 共享） |
| `submap_verts_` | 无 | **新增** submap 顶点列表 |
| `info_intra_submap_` | 无 | **新增** INTRA_SUBMAP 信息阵 |
| `AddOdomFactor()` | 只加 node 顶点 | 额外加 submap 顶点 + INTRA 边 |
| `PoseOptimization()` | 只写回 node pose | 额外写回 submap global_pose |

### 7.3 SlamSystem

```cpp
// Init 中创建 SubmapManager，传递给 lio_ 和 lc_
auto submap_mgr = std::make_shared<SubmapManager>(submap_opts);
lio_->SetSubmapManager(submap_mgr);
lc_->SetSubmapManager(submap_mgr);
```

### 7.4 Keyframe

```cpp
class Keyframe {
    // ... 现有字段 ...
    SubmapId submap_id_ = 0;         // 新增：所属 submap
    SE3 pose_in_submap_;             // 新增：在 submap 局部坐标系中的位姿
};
```

---

## 8. Submap 生命周期状态机

```
    ┌──────────────┐
    │   Creating   │  ← CreateNewSubmap(): 分配 ID, 初始化 iVox/VoxelMap
    │              │     local_pose = global_pose = 当前 robot pose
    └──────┬───────┘
           │ InsertScan() × max_kf_per_submap
           ▼
    ┌──────────────┐
    │   Active     │  ← 接受新数据插入 + 作为 scan matching 目标
    │ (Matching)   │     GetMatchingSubmap() 返回此 submap
    └──────┬───────┘
           │ InsertScan() 达到 2 × max_kf_per_submap
           │ 或新 submap 创建
           ▼
    ┌──────────────┐
    │  Finishing   │  ← 不再接受新数据，但后端可用于回环匹配
    │              │     insertion_finished = true
    └──────┬───────┘
           │ PoseOptimization 写回 global_pose
           ▼
    ┌──────────────┐
    │  Finished    │  ← global_pose 可被优化更新
    │ (Immutable)  │     内部 iVox/VoxelMap 数据完全不变
    └──────────────┘
```

---

## 9. 配置参数设计

```yaml
# config/default.yaml 新增

submap:
  enable: true                    # 是否启用 submap 模式
  max_kf_per_submap: 50           # 每个 submap 的最大关键帧数
  num_active_submaps: 2           # 同时维护的活跃 submap 数
  overlap_kf: 10                  # 新旧 submap 重叠的关键帧数
  
  # submap 内部的 iVox 配置（可独立于全局配置）
  ivox:
    resolution: 0.5
    capacity: 500000
  
  # submap 内部的 VoxelMap 配置
  voxel_map:
    voxel_size: 1.0
    max_layer: 2

  # 因子图权重
  intra_submap_trans_noise: 0.05   # INTRA_SUBMAP 边平移噪声
  intra_submap_rot_noise: 1.0      # INTRA_SUBMAP 边旋转噪声（度）
```

---

## 10. 实施路线图

### Phase 0：基础设施（预计工作量：小）

- [ ] 新增 `Submap3D` 数据结构（`src/common/submap3d.h`）
- [ ] 新增 `SubmapManager` 类（`src/core/maps/submap_manager.h/cc`）
- [ ] `Keyframe` 新增 `submap_id_` 和 `pose_in_submap_` 字段

### Phase 1：前端集成（预计工作量：中）

- [ ] `LaserMapping` 新增 `submap_manager_` 成员
- [ ] 改造 `MapIncremental()`：world 系点云 → submap local 系插入
- [ ] 改造 `ObsModel()`：在 submap local 系做匹配，法向量变换回 world
- [ ] 改造 `MakeKF()`：记录 submap_id，通知 SubmapManager
- [ ] 添加 submap 切换逻辑（配合 `max_kf_per_submap`）

### Phase 2：后端集成（预计工作量：中）

- [ ] `LoopClosing` 新增 `submap_manager_` 成员
- [ ] 改造 `AddOdomFactor()`：添加 submap 顶点 + INTRA_SUBMAP 边
- [ ] 改造 `PoseOptimization()`：写回 submap global_pose
- [ ] `SlamSystem` 中创建共享 SubmapManager 并分发给 lio_ 和 lc_

### Phase 3：保存和加载（预计工作量：中）

- [ ] `SaveMap`：按 submap 分块保存，每个 chunk 记录 global_pose
- [ ] `TiledMap`：加载时利用 submap 的 global_pose，chunk 数据保持 local 系
- [ ] 定位系统：`LoadOnPose` 时按 submap 加载，用 global_pose 变换

### Phase 4：回环精化（预计工作量：小）

- [ ] 回环检测可利用 submap 内部 iVox/VoxelMap 做 scan-to-submap 匹配
- [ ] 替代当前的"临时拼点云 + GICP"流程，提高效率和精度

---

## 11. 性能与风险评估

### 11.1 性能影响

| 操作 | 当前 | 改造后 | 差异 |
|------|------|--------|------|
| ObsModel（单帧匹配） | 1× hash 查找 | 1× hash 查找 + 1× SE3 变换 | ~+5% |
| MapIncremental | 1× 插入 | 1-2× 插入（active submaps） | ~+0-100% |
| 回环优化后处理 | 全量重建全局地图 | **仅更新 global_pose** | **巨大改善** |
| 内存 | 1 个全局 map | N 个 submap | 类似（总点数不变） |

### 11.2 风险

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| submap 切换时匹配断裂 | 前端漂移 | 重叠区间（overlap_kf）保证连续性 |
| submap 边界附近匹配质量下降 | 精度 | 使用两个 active submap 联合匹配 |
| ESKF 状态仍在 world 系 | 实现复杂度 | 法向量变换开销很小，Jacobian 推导不变 |
| 线程安全 | 崩溃 | SubmapManager 内部加锁保护 global_pose 读写 |

### 11.3 降级兼容

```yaml
submap:
  enable: false  # 关闭后，SubmapManager 退化为单 submap，等效当前行为
```

---

## 12. 与 Cartographer Submap 的对比

| 维度 | Cartographer 2D | Lightning-LM 3D（本方案） |
|------|----------------|---------------------------|
| Submap 内容 | ProbabilityGrid（2D 栅格） | IVox3D / VoxelMap（3D 体素） |
| 坐标系 | local_pose（Rigid3d，固定） | local_pose（SE3，创建时固定） |
| 优化变量 | global_pose（Rigid2d，3 DOF） | global_pose（SE3，6 DOF） |
| 活跃窗口 | 2 个 | 2 个（可配置） |
| 完成条件 | num_range_data ≥ 阈值 | num_insertions ≥ max_kf_per_submap |
| 优化器 | Ceres SPA | miao 增量 PGO（Levenberg-Marquardt） |
| 约束类型 | INTRA_SUBMAP + INTER_SUBMAP | INTRA_SUBMAP + motion + loop |
| 回环后处理 | 只更新 global_pose | 只更新 global_pose（相同！） |
| ID 系统 | `(trajectory_id, submap_index)` | `SubmapId`（uint32，全局唯一） |

---

## 附录 A：核心文件修改清单

| 文件 | 修改类型 | 说明 |
|------|----------|------|
| `src/common/submap3d.h` | **新建** | Submap3D 数据结构 |
| `src/common/keyframe.h` | 修改 | 新增 submap_id_, pose_in_submap_ |
| `src/core/maps/submap_manager.h` | **新建** | SubmapManager 声明 |
| `src/core/maps/submap_manager.cc` | **新建** | SubmapManager 实现 |
| `src/core/lio/laser_mapping.h` | 修改 | 新增 submap_manager_ 成员 |
| `src/core/lio/laser_mapping.cc` | 修改 | ObsModel, MapIncremental, MakeKF |
| `src/core/loop_closing/loop_closing.h` | 修改 | 新增 submap_manager_, submap_verts_ |
| `src/core/loop_closing/loop_closing.cc` | 修改 | AddOdomFactor, PoseOptimization |
| `src/core/system/slam.h` | 修改 | 新增 submap_manager_ |
| `src/core/system/slam.cc` | 修改 | Init 中创建并分发 SubmapManager |
| `config/default.yaml` | 修改 | 新增 submap 配置段 |

## 附录 B：Submap Local 系中 ObsModel 的数学推导

ESKF 状态向量 $\mathbf{x} = [^W\mathbf{p}, ^W\mathbf{R}, ...]$ 仍在 world 系。

匹配在 submap local 系完成，设 submap 外参为 $T_{WS} = ({^W\mathbf{R}_S}, {^W\mathbf{t}_S})$：

$$
^S\mathbf{p} = T_{WS}^{-1} \cdot {^W\mathbf{p}}
$$

匹配得到 submap 系的平面参数 $(^S\mathbf{n}, ^S d)$，残差：

$$
r = {^S\mathbf{n}}^T \cdot (T_{WS}^{-1} \cdot T_{WB} \cdot {^B\mathbf{p}_i}) + {^S d}
$$

其中 $T_{WB}$ 是 ESKF 估计的 body pose。等价地，变换到 world 系：

$$
r = ({^W\mathbf{R}_S} \cdot {^S\mathbf{n}})^T \cdot (T_{WB} \cdot {^B\mathbf{p}_i} - {^W\mathbf{t}_S}) + {^S d}
= {^W\mathbf{n}}^T \cdot {^W\mathbf{p}_i} + {^W d}
$$

其中 ${^W\mathbf{n}} = {^W\mathbf{R}_S} \cdot {^S\mathbf{n}}$，${^W d} = {^S d} - {^W\mathbf{n}}^T \cdot {^W\mathbf{t}_S}$。

**结论**：对 ESKF 状态的 Jacobian 推导与当前完全一致，只需在 `ObsModel` 中将 submap local 系的平面参数 $(^S\mathbf{n}, ^S d)$ 变换为 world 系的 $(^W\mathbf{n}, ^W d)$ 即可，无需修改 ESKF 内部。
