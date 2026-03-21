# GLCTrimmer2D — 功能逻辑文档

> **文件路径**：`cartographer/mapping/internal/2d/glc_trimmer_2d.{h,cc}`
>
> **数学基础**：Carlevaris-Bianco et al., *"Generic Factor-Based Node Marginalization and Edge Sparsification for Pose-Graph SLAM"*, ICRA 2014.

---

## 1. 概述

`GLCTrimmer2D` 是一个基于**点云重叠率检测 + GLC 信息补偿**的位姿图节点裁剪器，
继承自 `PoseGraphTrimmer`，在每次后端优化后自动被调用（`Trim()`）。

核心目标：
- **控制位姿图规模**：删除冗余或过时的节点，防止图无限增长。
- **保留关键约束信息**：在删除节点前，通过 Schur Complement + Chow-Liu Tree（CLT）生成等价的稀疏约束（GLC 补偿约束），避免信息丢失导致定位精度下降。

---

## 2. 整体架构

```
Trim()
 │
 ├─ Step 1: 找最新完成的 submap（newest_submap）
 │
 ├─ 【批量收集阶段】遍历 newest_submap 中的每个节点 cur_node
 │    │
 │    ├─ Step 3: 在 newest_submap 以外找最近的 neighbor_count 个节点
 │    │          （已标记删除的节点自动跳过，不参与近邻搜索）
 │    │
 │    └─ Step 4: 对每个候选节点计算重叠率，按配额标记（不立即删除）
 │         ├─ 类型 A（其他轨迹，overlap < 0.5）→ 标记为待删
 │         └─ 类型 B（当前轨迹，overlap ≥ threshold）→ 标记为待删
 │              同一节点只会被标记一次（marked_for_trim 去重）
 │
 ├─ 【批量 Trim 阶段】对 global_trim_list 中所有节点执行 TrimNodeWithGLC
 │    每个节点只 GLC marginalize 一次，避免重复计算
 │
 └─ Step 5: 清理空 submap（TrimEmptySubmaps）
```

---

## 3. 核心算法详解

### 3.1 点云重叠率计算（`CalculateOverlapWithSubmap`）

**目的**：衡量候选节点（`cand`）的扫描数据在目标 submap 中的"覆盖度"。

**调用方式**：
```
overlap = CalculateOverlapWithSubmap(nodes[cur_node], cand_submap, pose_graph)
```
注意：这里计算的是 **`cur_node`（最新节点）的点云** 在 **`cand` 所属 submap** 上的重叠率，
用于判断 cand 的区域是否已被 cur_node 覆盖。

**坐标系推导（关键）**：

| 符号 | 含义 |
|------|------|
| `filtered_gravity_aligned_point_cloud` | 点云，在重力对齐帧（gravity-aligned frame）中，即 roll=pitch=0 的机体坐标系 |
| `node.constant_data->local_pose` | 节点在 local SLAM 坐标系下的位姿，其中包含了 `gravity_alignment` 旋转：`local_pose = Embed3D(pose_2d) * gravity_alignment` |
| `node.constant_data->gravity_alignment` | 重力对齐旋转（四元数），表示机体相对于重力方向的 roll/pitch 修正 |
| `ProbabilityGrid`（栅格地图） | 在 **local SLAM 坐标系**中构建（`InsertIntoSubmap` 使用 `local_pose_estimate`），`GetCellIndex` 期望输入点在 local SLAM 坐标系中 |

正确的 2D 节点位姿（与 `pose_graph_2d.cc` 中 `local_pose_2d` 的计算方式一致）：

$$T_{\text{node\_local\_2d}} = \text{Project2D}\!\left(\text{local\_pose} \times R(\text{gravity\_alignment}^{-1})\right)$$

最终将点云变换到 local SLAM 坐标系查询栅格：

```cpp
const transform::Rigid2d node_local_2d = transform::Project2D(
    node.constant_data->local_pose *
    transform::Rigid3d::Rotation(
        node.constant_data->gravity_alignment.inverse()));
// T_submap_cloud_2d = node_local_2d（无需 submap local_pose 的逆）
const transform::Rigid2f T_submap_cloud_2f = node_local_2d.cast<float>();
```

> **为什么不需要 submap local_pose 的逆**：
> ProbabilityGrid 的坐标原点是以 local SLAM 坐标系为参考的全局网格（通过 `limits_` 中的 `max_` 偏移），
> 而不是以 submap 原点为中心的局部坐标，因此直接用 local SLAM 坐标系下的点坐标查询即可。

**重叠率计算**：
```
overlap = hit_count / total_count
```
- `hit_count`：点落在 `GetProbability(cell) > 0.5f` 的单元数
- `total_count`：点落在 grid 范围内（`Contains(cell_index)` 为 true）的总数

---

### 3.2 Trim 决策逻辑

`Trim()` 分为**批量收集**和**批量执行**两个阶段，避免对同一节点重复 GLC 边缘化。

#### 批量收集阶段

对 `newest_submap` 中每个 `cur_node`，在 `newest_submap` 以外找最近的 `neighbor_count` 个候选节点，然后按以下规则分类并**标记**（不立即删除）：

##### 类型 A（**优先**）：来自其他轨迹，重叠率 < 0.5
```
条件：cand.trajectory_id ≠ cur_node.trajectory_id  AND  overlap < 0.5
含义：其他轨迹曾经过此区域，但当前地图已更新，旧节点的信息已过时（被覆盖）
行为：标记加入 global_trim_list
```

##### 类型 B（**次优**）：来自当前轨迹，重叠率 ≥ threshold
```
条件：cand.trajectory_id == cur_node.trajectory_id  AND  overlap ≥ overlap_high_threshold
含义：当前轨迹在同一区域有重复节点，信息冗余
行为：按 overlap 降序排列，优先标记最冗余的节点，加入 global_trim_list
```

##### 关键优化：`marked_for_trim` 去重
- 已被标记删除的节点加入 `marked_for_trim` 集合
- 后续 `cur_node` 的近邻搜索会**自动跳过**已标记节点
- 同一节点只会出现在 `global_trim_list` 中一次，无论被多少个 `cur_node` 检测到

##### 删除数量限制
每个 `cur_node` 处理时，按 `max_trim_per_check` 配额标记候选节点（A 优先，B 其次）。

#### 批量执行阶段

收集完成后，对 `global_trim_list` 中所有节点依次执行 `TrimNodeWithGLC`：
- 每个节点只执行一次 GLC marginalization（Schur Complement + CLT）
- 避免同一节点被多个 `cur_node` 重复触发边缘化计算

---

### 3.3 GLC 信息补偿（`TrimNodeWithGLC`）

在删除节点 `N_old` 之前，计算并插入等价的稀疏约束，保留被消除节点携带的信息量。

**步骤**：

1. **确定 del 集合与 Markov Blanket（MB）**：
   - `del` = `{N_old}`
   - `MB` = 所有与 `N_old` 存在约束的幸存 submap

2. **构建局部信息矩阵** $\Lambda_m$（`BuildLocalInfoMatrix`）：
   对每个涉及 `del ∪ MB` 的约束，计算 Jacobian 并累加：
   $$\Lambda_m \mathrel{+}= J^T \Omega J, \quad \Omega = \text{diag}(w_t^2, w_t^2, w_r^2)$$

3. **Schur Complement**（`SchurComplement`）：
   消去 `del` 变量，得到 MB 子图之间的稠密信息矩阵：
   $$\Lambda_t = \Lambda_{\alpha\alpha} - \Lambda_{\alpha\beta} \Lambda_{\beta\beta}^{-1} \Lambda_{\beta\alpha}$$

4. **Chow-Liu Tree（CLT）稀疏化**（`ChowLiuTree`）：
   以互信息为边权：
   $$I(x_i, x_j) = \frac{1}{2} \ln \frac{\det(\Lambda_{ii}) \cdot \det(\Lambda_{jj})}{\det(\Lambda_{ij}^{\text{joint}})}$$
   使用 Kruskal 算法求最大权生成树，输出树边集合。

5. **生成 GLC 约束**（`MakeGLCConstraint`）：
   每条树边 $(i, j)$ 生成一个 `INTER_SUBMAP` 约束，
   - 测量值 $z = T_{\text{submap}}^{-1} \cdot T_{\text{node}}$（当前优化位姿）
   - 权重从 $\Lambda_t$ 对角块导出：$w_t = \sqrt{\lambda_t}$，$w_r = \sqrt{\lambda_r}$

6. **插入约束并删除节点**：
   ```
   pose_graph->AddConstraint(glc_constraint)  // 先插入 GLC 约束
   pose_graph->TrimNode(node_id)              // 再删除节点
   ```

---

### 3.4 空 submap 清理（`TrimEmptySubmaps`）

当某个 finished submap 的所有节点均被删除后，该 submap 也将被清理：

1. 对 submap 整体进行 GLC 补偿（`ComputeGLCConstraints`）：
   - `del` = 待删 submap + 其 exclusive 节点（仅属于该 submap 的节点）
   - `MB` = 与 del 有 INTER_SUBMAP 约束的幸存实体
2. 插入 GLC 约束后调用 `pose_graph->TrimSubmap(sid)`

---

## 4. 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `overlap_high_threshold` | `double` | `0.8` | 类型 B 冗余阈值：同轨迹候选节点的重叠率超过此值时视为冗余 |
| `enable_glc` | `bool` | `true` | GLC 信息补偿开关；设为 `false` 时直接删除节点（仅调试用） |
| `neighbor_count` | `int` | `3` | 最近邻搜索的候选节点数量 |
| `max_neighbor_dist` | `double` | `2.0` | 最大搜索距离（米）；候选节点距 cur_node 超过此距离则跳过 |
| `max_trim_per_check` | `int` | `1` | 每个 cur_node 处理时允许删除的最大节点数（≤0 表示不限制） |

### 参数配置路径

```
Lua 配置文件
  └─ glc_trimmer_2d.max_trim_per_submap
        │
        ▼ pose_graph.cc::PopulateGLCTrimmer2DOptions()
  Proto: GLCTrimmer2DOptions.max_trim_per_submap
        │
        ▼ pose_graph_2d.cc
  GLCTrimmer2D::Options.max_trim_per_check
```

相关 Lua 配置示例：
```lua
glc_trimmer_2d = {
  overlap_high_threshold = 0.6,   -- 重叠率阈值，60% 以上视为冗余
  enable_glc = true,              -- 启用 GLC 信息补偿
  neighbor_count = 3,             -- 搜索最近 3 个候选
  max_neighbor_dist = 2.0,        -- 最大搜索距离 2 米
  max_trim_per_submap = 1,        -- 每轮最多删除 1 个节点
}
```

---

## 5. 关键数据结构

### `MBEntity`（Markov Blanket 实体）
```cpp
struct MBEntity {
  enum Type { SUBMAP, TRAJ_NODE } type;
  SubmapId submap_id;       // type == SUBMAP 时有效
  NodeId   node_id;         // type == TRAJ_NODE 时有效
  transform::Rigid2d pose;  // 当前优化后的 2D 位姿
  int local_idx;            // 在局部信息矩阵中的索引（每实体 3 DOF）
};
```

### `GLCTrimmer2D::Options`（选项结构体）
```cpp
struct Options {
  double overlap_high_threshold = 0.8;
  bool   enable_glc = true;
  int    neighbor_count = 3;
  double max_neighbor_dist = 2.0;
  int    max_trim_per_check = 1;
};
```

---

## 6. 调用流程

```
PoseGraph2D::HandleWorkQueue()
  └─ RunOptimization()
       └─ foreach trimmer in trimmers_:
            trimmer->Trim(this)
              └─ GLCTrimmer2D::Trim(pose_graph)
                   ├─ FindNodeSubmap()           // 找候选节点所属 submap
                   ├─ CalculateOverlapWithSubmap() // 计算重叠率
                   ├─ TrimNodeWithGLC()          // GLC 补偿 + 删除节点
                   │    ├─ ComputeGLCConstraintsForNode()
                   │    │    ├─ BuildLocalInfoMatrix()
                   │    │    ├─ SchurComplement()
                   │    │    ├─ ChowLiuTree()
                   │    │    └─ MakeGLCConstraint()
                   │    └─ pose_graph->TrimNode()
                   └─ TrimEmptySubmaps()
                        ├─ ComputeGLCConstraints() // submap 级别 GLC
                        └─ pose_graph->TrimSubmap()
```

---

## 7. 设计注意事项

1. **`last_trimmed_submap_` 防重复处理**：每次 `Trim()` 只处理一个新的 submap（按字典序最大），
   避免对同一 submap 反复触发，同时保证所有 submap 按序处理。

2. **`other_nodes` 实时更新**：每删除一个节点后，立刻从 `other_nodes` 列表中移除，
   防止后续 `cur_node` 对已删节点重复计算。

3. **GLC 约束过滤**：插入前检查 proxy node 和目标 submap 是否仍在图中，
   避免插入"悬空"约束导致后续优化崩溃。

4. **类型 B 排序**：按 overlap 降序排列，保证优先删除信息最冗余（重叠最多）的节点。

5. **坐标系一致性**：点云（gravity-aligned frame）→ local SLAM frame 的变换必须用
   `Project2D(local_pose * gravity_alignment^{-1})`，与 `pose_graph_2d.cc` 中
   `local_pose_2d` 的计算方式完全一致，否则会导致重叠率计算错误。

---

## 8. 相关文件索引

| 文件 | 说明 |
|------|------|
| `cartographer/mapping/internal/2d/glc_trimmer_2d.h` | 类声明、`Options` 结构体 |
| `cartographer/mapping/internal/2d/glc_trimmer_2d.cc` | 完整实现 |
| `cartographer/mapping/proto/pose_graph_options.proto` | `GLCTrimmer2DOptions` protobuf 定义 |
| `cartographer/mapping/pose_graph.cc` | `PopulateGLCTrimmer2DOptions()`：从 Lua 读取参数到 proto |
| `cartographer/mapping/internal/2d/pose_graph_2d.cc` | 将 proto 参数赋值给 `GLCTrimmer2D::Options` |
| `cartographer/configuration_files/pose_graph.lua` | 主 Lua 配置文件 |
| `cartographer_ros/configuration_files/dexe_slam_2d.lua` | 在线建图 Lua 配置 |
| `cartographer_ros/configuration_files/dexe_slam_2d_offline.lua` | 离线建图 Lua 配置 |
| `cartographer_ros/configuration_files/dexe_slam_2d_remapping.lua` | 重定位 Lua 配置 |
