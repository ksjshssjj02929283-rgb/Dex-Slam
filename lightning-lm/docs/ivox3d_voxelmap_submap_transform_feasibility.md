# iVox3D 与 VoxelMap 的整图位姿变换与 Submap 化可行性分析

## 1. 问题与结论

问题：
1. iVox3D / VoxelMap 当前是否支持“整张地图元素做位姿变换（SE3）”？
2. 是否可以整理成 submap，并通过 submap 整体坐标变换减少重建？

结论（基于当前代码）：
1. 不支持现成的整图 SE3 变换接口。
2. 两者当前都以“世界坐标体素哈希”作为主存储，做全局变换本质上仍需重索引（重哈希）。
3. 可以做 submap 化，并通过“submap 外参变换 + 局部地图不动”减少重建开销；该思路在工程上可行，且在本仓库的 TiledMap 已有分块思想先例。

---

## 2. 代码现状证据

### 2.1 iVox3D：仅增量增删与近邻查询，无整图变换 API

IVox 对外接口主要是：`AddPoints`、`GetClosestPoint`、`DeleteGrid/DeleteGridsFarFrom/DeleteGridsInAABB`。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:68]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:92]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:110]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:124]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:149]

IVoxManager 也是初始化、增量更新、匹配查询三类能力，没有整图位姿变换接口。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox_manager.hpp:76]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox_manager.hpp:122]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox_manager.hpp:223]

### 2.2 iVox3D 的主键决定了“整图变换”会触发重哈希

iVox 的 key 由点坐标经 `Pos2Grid` 计算得到，并存入 `grids_map_`；点插入时就是按 key 进哈希。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:175]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:349]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/ivox3d/ivox3d.h:442]

因此若做整图 SE3，key 大概率变化，必须重建 key->node 映射（至少 O(体素数)，通常 O(点数)）。

### 2.3 VoxelMap：同样是世界系体素哈希，无整图变换接口

VoxelMapManager 的核心接口是 `InitializeMap/UpdateMap/FindCorrespondences`，无 transform map 接口。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/include/voxel_map/voxel_map_manager.hpp:62]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/include/voxel_map/voxel_map_manager.hpp:68]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/include/voxel_map/voxel_map_manager.hpp:80]

内部主容器为 `std::unordered_map<VOXEL_LOC, OctoTree*> voxel_map_`。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/include/voxel_map/voxel_map_manager.hpp:135]

`UpdateMap` 里按世界坐标 `p_v.point` 计算体素 key 并写入 `voxel_map_`。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/src/voxel_map_manager.cpp:86]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/src/voxel_map_manager.cpp:95]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/voxel_map/src/voxel_map_manager.cpp:106]

### 2.4 LIO 主流程当前是“直接更新同一世界系地图”

LaserMapping 中：首帧初始化 + 每帧 UpdateMap，未见“地图整体位姿更新”过程。

- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/lio/laser_mapping.cc:260]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/lio/laser_mapping.cc:585]
- [VERIFY: dexe_mobile_slam/lightning-lm/src/core/lio/laser_mapping.cc:595]

---

## 3. 为什么当前实现下整图变换不划算

### 3.1 iVox3D

- 体素 key 绑定世界坐标；变换后 key 变，必须重索引。
- 仅做平移且恰为分辨率整数倍时，理论上可 key 平移，但代码并无该专用路径。
- 一般 SE3（含旋转）会改变大量点/体素归属，基本等价于重建。

### 3.2 VoxelMap

- 同样 key 绑定世界坐标；变换后需要重哈希。
- `OctoTree` 内部还持有平面参数、层级结构与增量状态；简单平移 key 不足以覆盖旋转情况下的几何一致性。
- 若做严格变换，还要处理平面参数（center/normal/d）及其协方差语义一致性，工程复杂度较高。

---

## 4. Submap 化是否可行

可行，且是减少重建开销的正确方向。

思路：
1. 把“全局一张 map”改为“多个 submap + 每个 submap 一个固定局部坐标系”。
2. 每个 submap 内部的 iVox/VoxelMap 不随全局优化移动（局部数据结构不改 key）。
3. 全局优化只更新 `T_ws`（submap pose），而非重建 submap 内体素结构。
4. 查询匹配时，把当前点从 world 变到 submap 局部后做匹配；残差/Jacobian 再转回统一坐标。

复杂度变化：
- 现状：回环后重建成本接近 O(全图点/体素)
- submap 化：回环后主成本接近 O(submap 数) 的位姿更新 + 在线匹配时多一次坐标变换

---

## 5. 本仓库中的可借鉴基础

`TiledMap` 已有分块管理、装卸载、按 AABB 删除 iVox grid 的机制，可作为 submap 管理层参考。

- 分块结构与装卸载：
  - [VERIFY: dexe_mobile_slam/lightning-lm/src/core/maps/tiled_map.h:44]
  - [VERIFY: dexe_mobile_slam/lightning-lm/src/core/maps/tiled_map.cc:273]
- iVox 按 chunk 卸载（AABB 删除）：
  - [VERIFY: dexe_mobile_slam/lightning-lm/src/core/maps/tiled_map.cc:343]

注意：这套逻辑主要用于定位地图管理，不是 LIO 在线建图的回环后图优化路径。

---

## 6. 工程建议（按风险从低到高）

### 方案 A：先做“Submap 容器层”，不改 iVox/VoxelMap 内核

1. 新增 `SubmapManager`：
   - `struct Submap { SE3 T_ws; std::shared_ptr<IVoxManager> ivox; std::shared_ptr<VoxelMapManager> vmap; Bounds; }`
2. LIO 写图按策略切新 submap（时间/里程/关键帧数阈值）。
3. 匹配时先选候选 submap（空间近邻），把查询点变到 submap 局部再调用原有匹配接口。
4. 回环/PGO 后仅更新 `T_ws`，不重建 submap 内部结构。

优点：改动集中在管理层，复用当前稳定模块。  
代价：匹配阶段多一层调度与坐标变换。

### 方案 B：给 iVox 增加“整格平移”快速路径（仅平移特例）

- 仅支持平移且为 `k * resolution` 的离散位移。
- 通过 key 偏移迁移 grid，避免逐点重插。
- 不处理旋转。

优点：实现相对简单。  
限制：适用场景窄，难覆盖回环优化常见的旋转修正。

### 方案 C：给 VoxelMap 做完整刚体变换接口

- 需要系统定义 plane 参数和协方差在变换下的传播规则。
- 还要重算/维护八叉树层级几何一致性与 key 索引。

优点：理论完备。  
风险：开发与验证成本最高，不建议作为第一步。

---

## 7. 对你的问题的直接回答

1. iVox3D 和 VoxelMap 目前都不支持“整图元素一键位姿变换”。
2. 可以整理为 submap，并通过 submap 级位姿更新减少重建，这是可行且推荐路线。
3. 建议优先做“Submap 管理层 + 查询时坐标变换”的架构，不要先硬改两套底层体素结构。

---

## 8. 后续落地建议

1. 先在 LIO 流程里定义 submap 切分与生命周期（创建、冻结、激活、淘汰）。
2. 复用现有 `IVoxManager` / `VoxelMapManager` 作为 submap 局部地图引擎。
3. 在回环优化回调中只更新 submap pose，不触发全量重建。
4. 增加 A/B 基准：
   - 回环后耗时（重建 vs submap pose update）
   - 每帧匹配耗时（单图 vs 多 submap）
   - 轨迹精度变化（ATE/RPE）
