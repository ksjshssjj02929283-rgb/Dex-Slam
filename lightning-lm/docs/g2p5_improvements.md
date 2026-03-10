# g2p5 模块改进记录

> 改进日期：2026-03-10
> 涉及文件：`src/core/g2p5/g2p5.h/.cc`、`src/core/system/slam.cc`、`config/*.yaml`

---

## 一、后端重绘（RenderBack）重影问题修复

### 问题根因

`RenderBack()` 逐帧调用 `AddKfToMap({*cur_kf}, backend_map_)`，每帧内部均触发
`ResizeMap()` → `map->Resize()`。每次 `Resize()` 都要把已有数据搬移到新坐标系，
浮点取整误差在 N 帧中累积，导致同一障碍物在地图上出现多次（重影）。

`RedrawGlobalMapOffline()` 一次性调用 `AddKfToMap(all_keyframes, map)`，只对**空地图**
执行一次 `Resize()`，无数据搬移，结果正确。

### 修复方案

在逐帧循环**之前**，用全部关键帧预计算最终包围盒，对空地图执行唯一一次 `Resize()`：

```cpp
// 预先对空地图做一次 ResizeMap，确定最终包围盒
// 后续逐帧 AddKfToMap 时 ResizeMap 不会再触发 Resize()
ResizeMap(all_keyframes, backend_map_);

for (auto cur_kf = all_keyframes.begin(); cur_kf != all_keyframes.end(); ++cur_kf) {
    AddKfToMap({*cur_kf}, backend_map_);
    if (backend_redraw_flag_) { abort = true; break; }
    if (quit_flag_)           { abort = true; break; }
}
```

**效果**：逐帧中断逻辑（`backend_redraw_flag_`、`quit_flag_`）完全保留，同时消除重影。

---

## 二、同步步骤（sync step）修复

### 问题根因

原 `RenderBack()` 末尾的同步步骤用 `GetID()` 当向量下标访问 `all_keyframes_[i]`：

```cpp
// 旧代码（有缺陷）
for (int i = cur_idx + 1; i <= frontend_idx; ++i) {
    kfs.emplace_back(all_keyframes_[i]);  // GetID() ≠ 向量下标时越界或访问错误帧
}
```

当关键帧 ID 与向量下标不一致时，可能访问错误的帧，导致帧被重复绘制产生重影；
同时 `frontend_current_` 未做 nullptr 检查，存在崩溃风险。

### 修复方案

改为直接切片（按向量顺序，无 ID/下标歧义）：

```cpp
// 新代码：直接取快照结束后新增的帧
std::vector<Keyframe::Ptr> new_kfs;
{
    UL lock(kf_mutex_);
    if (all_keyframes_.size() > all_keyframes.size()) {
        new_kfs.assign(all_keyframes_.begin() + all_keyframes.size(),
                       all_keyframes_.end());
    }
}
if (!new_kfs.empty()) {
    AddKfToMap(new_kfs, backend_map_);
}
```

---

## 三、API 重设计：地图重建与保存分离

### 变更前

- `RedrawGlobalMap()`：异步触发后端重绘，回环后调用
- 无离线同步重建接口，`SaveMap()` 依赖 `GetNewestMap()` 存在并发写竞争

### 变更后

| 方法 | 职责 | 触发时机 |
|------|------|----------|
| `RedrawGlobalMapBackend()` | 设置 `backend_redraw_flag_=true`（异步） | 回环检测回调 |
| `RedrawGlobalMapOffline()` | 同步全量重建，返回 `G2P5MapPtr`，**不写 `newest_map_`** | 保存地图前 |
| `SaveGridMap(save_path)` | 调用 `RedrawGlobalMapOffline` + 图像转换 + 可选过滤 + 写文件 | `slam.cc::SaveMap()` |

**`RedrawGlobalMapOffline` 不写入 `newest_map_`** 的原因：避免与并发运行的
`RenderBack()`/`RenderFront()` 产生写入竞争，导致 3D 全局地图与 2D 栅格地图不一致。

---

## 四、SaveMap 保存流程重构

### slam.cc 调用链（变更后）

```
SaveMap(save_path)
  ├─ Step 1: lc_->FinalizeAndWait()
  │           ├─ kf_thread_.Quit()（刷新队列）
  │           ├─ ForceFinalLoopCheck()（对最后一帧强制回环检测）
  │           ├─ AddLoopFactor() + Optimize(20)（添加约束并优化）
  │           └─ 写回 SetOptPose()，触发 loop_cb_
  │
  ├─ Step 2: 创建目录 + 保存 3D 地图（global.pcd、tiled map）
  │
  └─ Step 3: g2p5_->SaveGridMap(save_path)
              ├─ RedrawGlobalMapOffline()     ← 纯重建，返回 G2P5MapPtr
              ├─ ToROS() + 像素转换           ← OccupancyGrid → cv::Mat
              ├─ [build_nav_map=true] ApplyNavMapFilter()  ← 噪声过滤
              ├─ imwrite(map.png)             ← PNG 格式（原为 pgm）
              └─ 写 map.yaml
```

**关键顺序约束**：`SaveGridMap` 必须在目录创建之后调用，否则文件写入失败。

---

## 五、FinalizeAndWait 重复顶点 Bug 修复

### 问题根因

`kf_thread_.Quit()` 刷新队列时，`HandleKF(frame_N)` → `PoseOptimization()` 已将顶点 N
加入优化器。若 `FinalizeAndWait()` 随后再调用 `PoseOptimization()`，顶点 N 被二次添加，
破坏优化器状态。

### 修复方案

`FinalizeAndWait()` 不再调用 `PoseOptimization()`，只执行：

```cpp
ForceFinalLoopCheck();       // 对最后一帧检测回环候选
ComputeLoopCandidates();     // NDT 配准验证
if (candidates_.empty()) return;
AddLoopFactor();             // 添加回环边（不重复添加顶点）
optimizer_->Optimize(20);   // 仅优化
// 写回 SetOptPose()，触发 loop_cb_
```

---

## 六、导航地图噪声过滤（ApplyNavMapFilter）

### 功能

对占据栅格图像（White=自由, Black=障碍, Gray=未知）执行连通域噪声过滤，
移除孤立的小块黑色区域（LiDAR 单帧反射噪点、杆子、植被等），生成更干净的导航地图。

### 算法流程

```
1. 生成三类 mask：白色（≥250）、黑色（≤160）、灰色（中间值）
2. 对黑色区域做连通域分析（8 连通）
3. 对每个连通域：
   ├─ 面积 ≥ min_noise_area → 保留（真实障碍物）
   ├─ 膨胀后邻居含灰色像素 → 保留（墙边缘，有未知空间相邻）
   └─ 否则 → 置白（噪点，孤立在自由空间中的小黑块）
4. [save_debug_map=true] 输出 8 张中间结果图到 debug_map/ 子目录
```

### 配置参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `build_nav_map` | 是否启用噪声过滤 | `false` |
| `navigation_map_filter_mode` | 过滤强度 | `"normal"` |
| `save_debug_map` | 是否保存中间图片 | `false` |

**filter_mode 对应的面积阈值**（小于此值的连通域才参与过滤）：

| 模式 | 最小保留面积（像素） | 适用场景 |
|------|---------------------|----------|
| `normal` | 10 | 一般室内/室外 |
| `high` | 50 | 噪点较多（植被、玻璃） |
| `super` | 100 | 严重噪点场景 |

### 大地图性能优化（O(K×5n) → O(3n)）

> 优化日期：2026-03-10

**问题根因**

原实现对每个小连通域 i 逐一执行全图操作：

```
compare(labels, i, component_mask)   ← 扫描全图找第 i 个 label   O(n)
dilate(component_mask)               ← 全图膨胀                   O(n)
bitwise_and ×2                       ← 全图逐像素                 O(n)
countNonZero                         ← 全图统计                   O(n)
```

总复杂度 `O(K × 5n)`，K = 小连通域数量，n = 图像像素数。大地图 K 可达数千，极慢。

**优化思路：反转查询方向**

```
旧：为每个组件 i，去问"我的邻域里有没有 gray？"（K 次全图操作）
新：把 gray 向外膨胀一次，扫全图"哪些组件落在 gray 膨胀区内？"（固定 3 次全图操作）
```

**优化后算法**

```
Step 1: dilate(gray_mask, gray_expanded, kernel)    ← 1 次全图膨胀
Step 2: 初始化 label_action[K]（小连通域标记为 remove）  ← O(K)
Step 3: 顺序扫描全图：label 落在 gray_expanded 内 → 改为 keep  ← O(n)
Step 4: 顺序扫描全图：按 label_action LUT 批量置白   ← O(n)
```

总复杂度 `O(3n)`，与 K 无关。K=1000 时约 **1600× 加速**。

**实现细节**

- 两次扫描均使用 `ptr<>()` 行指针访问，缓存友好
- `label_action` 为 `std::vector<uint8_t>`，LUT 查询 O(1)
- debug 模式下 edge_mask 重建也从 O(K×n) 改为 O(n)（`is_small_kept[]` + 单遍扫描）
- 非 debug 路径不分配 `removed_mask`/`edge_mask`，减少内存分配

---

## 七、输出格式变更

| 项目 | 变更前 | 变更后 |
|------|--------|--------|
| 地图图像格式 | `map.pgm` | `map.png` |
| `map.yaml` image 字段 | `"map.pgm"` | `"map.png"` |
| 元数据来源 | `offline_gridmap->ToROS()` in slam.cc | `SaveGridMap` 内部 `ros_map.info` |

---

## 八、新增 yaml 配置项

所有 5 个配置文件（`default.yaml`、`default_livox.yaml`、`default_nclt.yaml`、
`default_utbm.yaml`、`default_vbr.yaml`）的 `g2p5` 节新增：

```yaml
g2p5:
  # ... 原有参数 ...
  build_nav_map: false                  # 是否对保存的地图做噪声过滤生成导航地图
  navigation_map_filter_mode: "normal"  # 滤波强度: "normal"(10), "high"(50), "super"(100)
  save_debug_map: false                 # 是否保存滤波中间结果（mask、连通域等）
```

---

## 九、g2p5_analysis.md 优先级状态更新

| 状态 | 问题 | 优先级 |
|------|------|--------|
| ✅ 已修复 | 前端 `AddKfToMap` 被注释，地图始终为空 | P0 |
| ✅ 已修复 | `map.yaml` 分辨率硬编码 0.05 | P0 |
| ✅ 已修复 | `RenderBack` 重影（多次 `Resize` 数据搬移误差累积） | P0（新发现） |
| ✅ 已修复 | sync step 用 `GetID()` 当向量下标 | P0（新发现） |
| ✅ 已修复 | `FinalizeAndWait` 重复添加顶点破坏优化器 | P0（新发现） |
| ✅ 已实现 | 角度分辨率提升 1° → 0.1° | P1 |
| ✅ 已修复 | hit 计数逻辑不统计同高度命中 | P1 |
| ✅ 已修复 | 斜装 LiDAR 建图辐射线条噪声（高度计算从 body 系改为世界 Z 轴） | P1 |

---

## 十、关键文件变更索引

| 文件 | 变更内容 |
|------|----------|
| `src/core/g2p5/g2p5.h` | 新增 `RedrawGlobalMapBackend()`、`RedrawGlobalMapOffline()`、`SaveGridMap()` 声明；新增 Options 字段 `build_nav_map_`、`nav_map_filter_mode_`、`save_debug_map_`；新增 `floor_z_world_` 成员与 `EstimateFloorZWorld()` 方法 |
| `src/core/g2p5/g2p5.cc` | 修复 `RenderBack` 重影（预 ResizeMap）；修复 sync step；实现 `RedrawGlobalMapOffline`、`SaveGridMap`；新增 `ApplyNavMapFilter` 静态函数（含大地图性能优化 O(K×5n)→O(3n)）；重写 `Convert3DTo2DScan`（高度计算改为世界 Z 轴）；新增 `EstimateFloorZWorld` |
| `src/core/loop_closing/loop_closing.h/.cc` | 新增 `FinalizeAndWait()`、`ForceFinalLoopCheck()`；修复重复顶点 bug |
| `src/core/system/slam.cc` | `SaveMap` 重构为 3 步流程；`lc_` 回环回调改用 `RedrawGlobalMapBackend()` |
| `config/*.yaml`（5个） | 新增 `build_nav_map`、`navigation_map_filter_mode`、`save_debug_map` |

---

## 十一、斜装 LiDAR 支持（高度计算改为世界 Z 轴）

> 改进日期：2026-03-10

### 问题根因

原 `Convert3DTo2DScan` 在 **LiDAR body 坐标系**中计算点云高度：

```cpp
// 旧代码
floor_coeffs_ = Vec4d(0, 0, 1, d);          // 假设 body Z 轴垂直 —— 斜装时错误
double dis_floor = pn.dot(floor_coeffs_);   // body 系 Z 分量，不等于真实垂直高度
```

当 LiDAR 以 50° 等角度倾斜安装时，body Z 轴不垂直于地面，导致：

1. `dis_floor` 计算错误，大量本该命中地面/障碍物的点被错误分类
2. miss 射线高度插值基于错误 `dis_floor`，朝下光束产生极长 miss 射线穿越墙体
3. `DetectPlaneCoeffs` 用 `values[2] < 0.99` 检测 body Z 方向法向量，斜装时永远失败
4. 建图结果：大量辐射线条噪声，墙体不清晰

**外参 `extrinsic_R` 补偿的是 LIO 前端（ESKF 点云配准），不补偿 g2p5 的 2D 投影几何。**

### 修复方案

把所有高度计算从 LiDAR body 系改到**世界坐标系 Z 轴**：

```
世界系地面 Z: floor_z_world = orig.z() - lidar_height_
点离地高度:  dis_floor     = p_world.z() - floor_z_world
LiDAR离地高: lidar_h       = orig.z()    - floor_z_world
```

- `orig = Twb.translation()` —— LiDAR 在世界系的位置，`.z()` 是真实垂直高度
- `lidar_height_` —— 物理垂直安装高度（配置参数，与倾角无关）
- `p_world = Twb * pc` —— 已变换到世界系，`.z()` 就是真实垂直高度

无论 LiDAR 以任何角度倾斜，`dis_floor` 始终是正确的垂直离地高度。

### 代码变更

**`g2p5.h`** 新增：
```cpp
double floor_z_world_ = 0.0;   // 地面世界 Z 坐标（每帧更新）

/// 在世界坐标系中估计地面 Z 高度（支持任意安装姿态）
bool EstimateFloorZWorld(Keyframe::Ptr kf, const SE3& Twb, double& floor_z_world);
```

**`g2p5.cc` `Convert3DTo2DScan` 核心改动**：

| 项目 | 旧（body 系） | 新（世界 Z 轴） |
|------|--------------|----------------|
| 地面高度 | `floor_coeffs_ = Vec4d(0,0,1,d)` | `floor_z_world_ = orig.z() - lidar_height_` |
| 点离地高 | `pn.dot(floor_coeffs_)` | `p_world.z() - floor_z_world_` |
| LiDAR离地高 | `options_.lidar_height_`（固定常量） | `orig.z() - floor_z_world_`（世界系真实值） |
| 方向角分箱 | body 系 `atan2(pc.y, pc.x)` | 世界系 `atan2(dy, dx)`（`d=p_world-orig`） |
| 距离过滤 | XY 平面距离 `pc.head<2>().norm()` | 3D 距离 `pc.norm()` |

**`EstimateFloorZWorld`**（替代 `DetectPlaneCoeffs` 的世界系版本）：
- 收集世界 Z 在 `[expected_floor_z ± search_half]` 范围内的点云
- 取底部 15% 分位数（`std::nth_element`）作为地面 Z
- 比 min 鲁棒（排除离群值），比 mean 不受天花板影响
- 少于 50 个点时返回 false，回退到 `orig.z() - lidar_height_`

### 配置参数说明

斜装 LiDAR 使用时，`lidar_height` 指的是**物理垂直安装高度**（雷达到地面的垂直距离），与倾角无关：

```yaml
g2p5:
  lidar_height: 0.3    # 垂直高度（米），与倾角无关
  esti_floor: false    # false：直接用 lidar_height 推算；true：动态估计
```

### 保留项

`DetectPlaneCoeffs()` 保留但不再调用（body 系平面估计，可能未来有用）。

### 验证方法

```bash
ros2 run lightning run_slam_offline \
  --config ./config/default.yaml \
  --input_bag <bag_with_tilted_lidar>
```

预期效果：斜装建图不再出现辐射线条噪声，墙体清晰，与水平安装效果相近。
