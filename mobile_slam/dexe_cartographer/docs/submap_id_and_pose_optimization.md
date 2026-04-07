# Cartographer 2D Submap：ID 索引与后端位姿调整

**文档范围**：`dexe_cartographer/cartographer/mapping/` 下 2D SLAM 的子图模块  
**核心问题**：
1. Submap 如何获得唯一 ID （与 iVox3D / VoxelMap 的 submap 形式对比）
2. 后端优化如何对 submap 的位姿进行调整（而无需重建地图内容）

---

## 1. Submap ID 数据结构

### 1.1 `SubmapId` 定义

```cpp
// [VERIFY: cartographer/mapping/id.h:90-103]
struct SubmapId {
  int trajectory_id;   // 轨迹 ID（多会话/多机器人扩展时区分来源）
  int submap_index;    // 轨迹内从 0 开始的顺序索引
};
```

与之对应的节点 ID：
```cpp
// [VERIFY: id.h:57-72]
struct NodeId {
  int trajectory_id;
  int node_index;
};
```

**关键设计**：`SubmapId` 是一个二元组 `(trajectory_id, submap_index)`，而非单一整数。这与 iVox3D 哈希 key / VoxelMap 世界坐标 key 完全不同——它与地图内容的位置解耦，不依赖任何空间坐标。

---

### 1.2 `MapById<SubmapId, DataType>` 容器

```
MapById
└── std::map<int, MapByIndex>         // 外层：按 trajectory_id 索引
        └── MapByIndex
                └── std::map<int, DataType>  // 内层：按 submap_index 索引
                    can_append_: bool
```

`Append()` 方法自动分配 submap_index：

```cpp
// [VERIFY: id.h:274-282]
IdType Append(const int trajectory_id, const DataType& data) {
  auto& trajectory = trajectories_[trajectory_id];
  const int index =
      trajectory.data_.empty() ? 0 : trajectory.data_.rbegin()->first + 1;
  trajectory.data_.emplace(index, data);
  return IdType{trajectory_id, index};  // 返回新分配的 SubmapId
}
```

索引分配规则：`submap_index = 轨迹内当前最大 index + 1`（从 0 起单调递增，无复用）。

---

## 2. Submap 生命周期与 ID 分配时机

### 2.1 前端：`ActiveSubmaps2D`（submap 对象生命周期）

`ActiveSubmaps2D` 维护最多 **2 个**活跃 submap 的滑动窗口：

```cpp
// [VERIFY: submap_2d.cc:719-757]
std::vector<std::shared_ptr<Submap2D>> submaps_;  // 始终 ≤ 2 个
```

**创建触发条件**：
```cpp
// [VERIFY: submap_2d.cc:734-737]
if (submaps_.empty() ||
    submaps_.back()->num_range_data() >= options_.num_range_data()) {
  AddSubmap(first_range_data.origin.head<2>());
}
```

当 back 的 range data 数达到上限（`num_range_data` 参数），就在当前机器人所在位置创建新 submap。

**完成触发条件**：
```cpp
// [VERIFY: submap_2d.cc:751-753]
if (submaps_.front()->num_range_data() >= 2 * options_.num_range_data()) {
  submaps_.front()->Finish();  // 设置 insertion_finished_ = true
}
```

**滑出窗口**（`AddSubmap` 内部）：
```cpp
// [VERIFY: submap_2d.cc:857-862]
if (submaps_.size() >= 2) {
  CHECK(submaps_.front()->insertion_finished());
  submaps_.erase(submaps_.begin());  // 从活跃窗口中移除（但 PoseGraph 仍持有引用）
}
```

> 注意：`ActiveSubmaps2D` 只管理内容插入窗口，**不分配 ID**。ID 由 `PoseGraph2D` 分配。

---

### 2.2 后端：`PoseGraph2D` 的 ID 分配

每帧调用 `AddNode()` → `AppendNode()` 时：

```cpp
// [VERIFY: pose_graph_2d.cc:157-172]
if (data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
    std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
            ->data.submap != insertion_submaps.back()) {
  // 第一次看到新 submap（pointer 比较）
  const SubmapId submap_id =
      data_.submap_data.Append(trajectory_id, InternalSubmapData());
  data_.submap_data.at(submap_id).submap = insertion_submaps.back();
  LOG(INFO) << "Inserted submap " << submap_id << ".";
}
```

**ID 分配时机**：当 `insertion_submaps.back()` 是 PoseGraph 从未见过的 submap 指针时，调用 `data_.submap_data.Append()` 分配新的 `SubmapId`。

---

### 2.3 全局初始位姿的初始化

`InitializeGlobalSubmapPoses()` 负责为新 submap 设定在 global frame 中的初始位姿：

```cpp
// [VERIFY: pose_graph_2d.cc:90-109]
// 第一个 submap（trajectory 首次插入）
optimization_problem_->AddSubmap(
    trajectory_id,
    transform::Project2D(
        ComputeLocalToGlobalTransform(data_.global_submap_poses_2d, trajectory_id)
        * insertion_submaps[0]->local_pose()));
// → SubmapId{trajectory_id, 0}

// [VERIFY: pose_graph_2d.cc:117-129]
// 后续 submap：由前一个 submap 的 global pose + 两个 submap 之间的 delta pose 推算
optimization_problem_->AddSubmap(
    trajectory_id,
    first_submap_pose *
        constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
        constraints::ComputeSubmapPose(*insertion_submaps[1]));
// → SubmapId{trajectory_id, last_index + 1}
```

公式：`T_global_s1 = T_global_s0 * T_local_s0^{-1} * T_local_s1`

---

## 3. Submap 的位姿表示（两层分离）

```
Submap 的位姿 = local_pose + global_pose（两层解耦）

local_pose_  : const Rigid3d，由首帧 origin 确定，永不修改（地图内容的内部坐标系锚点）
global_pose  : Rigid2d，存储在 optimization_problem_ 中，可被 Ceres 优化调整
```

```cpp
// [VERIFY: submaps.h:62-80]
class Submap {
  const transform::Rigid3d local_pose_;  // 固定，submap 内部坐标系
  bool insertion_finished_;              // 是否完成插入（决定是否参与回环检测）
};

// [VERIFY: optimization_problem_2d.h:52-54]
struct SubmapSpec2D {
  transform::Rigid2d global_pose;  // 优化变量，由 Ceres 调整
};
```

---

## 4. 后端优化流程（位姿调整）

### 4.1 约束类型

```
约束 Constraint = { submap_id, node_id, Pose{T_submap_node, trans_w, rot_w}, tag }
    tag = INTRA_SUBMAP  : 节点插入 submap 时直接建立（位姿已知，高权重）
    tag = INTER_SUBMAP  : 回环检测产生（通过 ConstraintBuilder2D 扫描匹配，带 HuberLoss）
```

`INTRA_SUBMAP` 约束的建立：
```cpp
// [VERIFY: pose_graph_2d.cc:434-447]
const transform::Rigid2d constraint_transform =
    constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
    local_pose_2d;
// T_submap_node = T_local_submap^{-1} * T_local_node（相对位姿）
data_.constraints.push_back(
    Constraint{submap_id, node_id,
               {transform::Embed3D(constraint_transform), ...},
               Constraint::INTRA_SUBMAP});
```

---

### 4.2 优化变量的构造（`Solve()`）

```cpp
// [VERIFY: optimization_problem_2d.cc:257-278]
// 每个 submap → C_submaps[id] = [tx, ty, theta]（3 自由度）
MapById<SubmapId, std::array<double, 3>> C_submaps;
for (const auto& submap_id_data : submap_data_) {
  C_submaps.Insert(submap_id_data.id,
                   FromPose(submap_id_data.data.global_pose));
  problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);
  if (first_submap || frozen) {
    problem.SetParameterBlockConstant(...);  // 第一个 submap 固定（全局锚点）
  }
}
```

从 `Rigid2d` 转换为 Ceres 数组：
```cpp
// [VERIFY: optimization_problem_2d.cc:79-83]
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}
```

---

### 4.3 残差项（SPA 代价函数）

```cpp
// [VERIFY: optimization_problem_2d.cc:281-291]
for (const Constraint& constraint : constraints) {
  problem.AddResidualBlock(
      CreateAutoDiffSpaCostFunction(constraint.pose),
      constraint.tag == Constraint::INTER_SUBMAP
          ? new ceres::HuberLoss(options_.huber_scale())  // 回环用鲁棒核
          : nullptr,
      C_submaps.at(constraint.submap_id).data(),   // submap 位姿（优化变量）
      C_nodes.at(constraint.node_id).data());       // node 位姿（优化变量）
}
```

代价函数 SPA（Sparse Pose Adjustment）：

$$\text{residual} = \left\| T_{s,n}^{\text{meas}} - C_s^{-1} \cdot C_n \right\|_{\Sigma}^2$$

其中 $T_{s,n}^{\text{meas}}$ 是约束中测量到的 submap→node 相对位姿，$C_s, C_n$ 是当前优化变量。

---

### 4.4 求解与结果写回

```cpp
// [VERIFY: optimization_problem_2d.cc:404-414]
ceres::Solve(..., &problem, &summary);

// 写回 submap 全局位姿
for (const auto& C_submap_id_data : C_submaps) {
  submap_data_.at(C_submap_id_data.id).global_pose =
      ToPose(C_submap_id_data.data);  // [tx, ty, theta] → Rigid2d
}

// 写回 node 全局位姿
for (const auto& C_node_id_data : C_nodes) {
  node_data_.at(C_node_id_data.id).global_pose_2d =
      ToPose(C_node_id_data.data);
}
```

---

### 4.5 `RunOptimization()` 的完整流程

```cpp
// [VERIFY: pose_graph_2d.cc:996-1043]
void PoseGraph2D::RunOptimization() {
  // 1. 调用 Ceres 求解（不加锁，避免阻塞前端）
  optimization_problem_->Solve(data_.constraints, ...);

  absl::MutexLock locker(&mutex_);

  // 2. 加锁后写回所有已优化节点的 global_pose
  for (const auto& node : node_data.trajectory(trajectory_id)) {
    data_.trajectory_nodes.at(node.id).global_pose =
        transform::Embed3D(node.data.global_pose_2d) * Rotation(gravity);
  }

  // 3. 对尚未进入优化的新节点，用变换差量外推
  const auto local_to_new_global = ComputeLocalToGlobalTransform(submap_data, ...);
  const auto local_to_old_global = ComputeLocalToGlobalTransform(data_.global_submap_poses_2d, ...);
  const auto old_global_to_new_global = local_to_new_global * local_to_old_global.inverse();
  for (/* 未优化的新节点 */) {
    node.global_pose = old_global_to_new_global * node.global_pose;
  }

  // 4. 更新 data_.global_submap_poses_2d（用于下次优化的初始值和外推基准）
  data_.global_submap_poses_2d = submap_data;
}
```

---

## 5. 完整数据流（时序图）

```
前端（LocalTrajectoryBuilder2D）
  │
  │  InsertData(range_data, cloud, ...)
  ▼
ActiveSubmaps2D
  ├─ submaps_[0]: 旧 submap（正在 Finish）
  └─ submaps_[1]: 新 submap（当前插入目标）
  │
  │  返回 insertion_submaps = [submaps_[0], submaps_[1]]
  ▼
PoseGraph2D::AddNode()
  │
  ├─ AppendNode()
  │    ├─ data_.trajectory_nodes.Append() → NodeId{traj, N}
  │    └─ data_.submap_data.Append()    → SubmapId{traj, M}  ← ID 此处分配
  │
  ├─ AddWorkItem(ComputeConstraintsForNode)
  │
  ▼  （后台线程）
ComputeConstraintsForNode()
  ├─ InitializeGlobalSubmapPoses()  → 为新 submap 设定 global 初始位姿
  ├─ optimization_problem_->AddTrajectoryNode()
  ├─ 添加 INTRA_SUBMAP 约束
  ├─ ComputeConstraint() × N  → 回环检测（约束搜索）
  └─ 返回 kRunOptimization / kDoNotRunOptimization
  │
  ▼
HandleWorkQueue() → RunOptimization()
  ├─ Solve()  → Ceres 更新 C_submaps, C_nodes
  └─ 写回 global_submap_poses_2d, trajectory_nodes.global_pose
```

---

## 6. 与 iVox3D / VoxelMap Submap 的对比

| 维度 | Cartographer 2D Submap | iVox3D / VoxelMap |
|------|----------------------|------------------|
| **索引方式** | `(trajectory_id, submap_index)` 二元组，与空间位置无关 | 哈希 Key = 世界坐标整数化，与位置强绑定 |
| **位姿调整** | 仅更新 `global_pose`（`Rigid2d`），**地图内容不移动** | 无 SE3 变换接口，需全量重键（等价于重建） |
| **地图内容** | 栅格地图（ProbabilityGrid / TSDF），坐标系由 `local_pose` 固定 | 3D 点集（LRU voxel / OctoTree plane），坐标直接编码在 key 中 |
| **回环后处理** | Ceres 更新所有 submap 的 `global_pose`，地图栅格不变 | 需重建受影响区域的所有 voxel |
| **解耦程度** | **完全解耦**：地图内容与位姿分离存储 | **强耦合**：位置变化 → key 变化 → 内容失效 |

**Cartographer 能做到"零重建位姿调整"的根本原因**：
- `Grid2D`（栅格内容）存储在 **submap 局部坐标系**中
- `submap.global_pose` 是一个独立的 `Rigid2d` 指针
- 优化时只更新 `global_pose`，不触碰 `Grid2D` 的任何一个像素

---

## 7. 核心文件索引

| 文件 | 内容 |
|------|------|
| `mapping/id.h` | `SubmapId`, `NodeId`, `MapById<>` 容器定义 |
| `mapping/submaps.h` | `Submap` 基类，`local_pose_` 定义 |
| `mapping/2d/submap_2d.h/cc` | `Submap2D`, `ActiveSubmaps2D`，前端滑动窗口管理 |
| `mapping/internal/pose_graph_data.h` | `PoseGraphData`, `InternalSubmapData`, `SubmapState` |
| `mapping/internal/2d/pose_graph_2d.h/cc` | `PoseGraph2D`，ID 分配、约束管理、优化调度 |
| `mapping/internal/optimization/optimization_problem_2d.h/cc` | `OptimizationProblem2D`, `SubmapSpec2D`, `Solve()` |
