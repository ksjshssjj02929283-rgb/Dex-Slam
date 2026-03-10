# g2p5 模块实现分析与地图质量优化方案

> 分析日期：2026-03-07
> 代码路径：`src/core/g2p5/`

---

## 一、模块概述

g2p5（3D→2D）是 lightning-lm 的 2.5D 栅格地图模块，将 LiDAR 关键帧点云转换为 2D 占据栅格地图，供导航系统使用（输出 `map.pgm` + `map.yaml`）。

**核心假设**：
- LiDAR 水平安装，地面为水平平面
- 只统计距地面 `[min_th_floor, max_th_floor]`（默认 0.5～1.2m）范围内的障碍物
- 使用关键帧的 **优化后位姿**（`GetOptPose()`），受益于回环修正

---

## 二、完整数据流

```
关键帧（Keyframe::Ptr）
    ↓ PushKeyframe()
    ├─ all_keyframes_ 追加
    └─ [在线] draw_frontend_map_thread_ 队列 → RenderFront(kf)
       [离线] 同步调用 RenderFront(kf)

RenderFront(kf)
    ├─ AddKfToMap({kf}, frontend_map_)   ← 增量更新
    └─ map_update_cb_()                  ← 触发 RViz/UI 发布

回环检测触发 → RedrawGlobalMap() → backend_redraw_flag_ = true
    ↓
RenderBack()（后台线程）
    ├─ 从零重建全部关键帧 → backend_map_
    ├─ 补充前端新增帧（前后端同步）
    └─ 替换 frontend_map_ / newest_map_ → map_update_cb_()

AddKfToMap()
    ├─ ResizeMap()    ← 按关键帧轨迹扩展地图范围
    └─ Convert3DTo2DScan()  ← 核心算法
```

---

## 三、核心算法：Convert3DTo2DScan

### 3.1 地面参数

- `esti_floor_ = false`（默认）：使用固定参数 `floor_coeffs_ = (0, 0, 1, -floor_height)`
- `esti_floor_ = true`：每帧用 RANSAC 拟合水平平面（条件：点数 ≥ 200，内点 ≥ 100，Z 系数 ≥ 0.99）

点到地面的相对高度：`dis_floor = pn.dot(floor_coeffs_) = z + floor_height`

### 3.2 黑色点（占据）统计

遍历全部点云，按 **1° 整数角度** 分桶：

```cpp
int angle = int(round(dangle) + 360) % 360;  // 精度：±0.5°
```

只有 `min_th < dis_floor < max_th` 的点才作为障碍物，直接写入栅格：

```cpp
map->SetHitPoint(p_world[0], p_world[1], true, dis_floor);
```

`SetGridHitPoint`（hit=true）的更新条件：

```cpp
if (height < cell.height_) {   // 只有更低的高度才更新
    cell.height_ = height;
    cell.hit_cnt_ += 1;
    cell.visit_cnt_ += 1;
}
```

### 3.3 射线端点选取

每个 1° 方向上收集所有 (距离, 高度) 对，逆序扫描选出"障碍物-地面"边界点：
- 若方向上测量少于 2 个 → 无效方向（`r = -1`）
- 否则从远到近，找到障碍物（dis_floor > min_th）和地面（dis_floor < min_th）的过渡边界

### 3.4 白色点（自由空间）射线追踪

`SetMissPoint()`：Bresenham 直线栅格化，从 LiDAR 原点到检测端点的所有格子标记为白色，同时线性插值高度：

```
沿射线每格的高度 = lidar_height - dh × 步数
```

更新逻辑：
- 白色刷黑色（hit_cnt > 3 的格子）：**需要射线高度 < 格子最低障碍高度** 才能增加 visit_cnt（否则认为从障碍物下方穿过，不合物理）
- 白色刷白色：直接增加 visit_cnt

### 3.5 占据度计算（ToROS / ToCV）

```cpp
float occ = (visit_cnt > 3) ? (float)hit_cnt / (float)visit_cnt : -1;
// occ > occupancy_ratio_(0.3) → 占据（黑）
// 0 ≤ occ ≤ 0.3              → 自由（白）
// occ < 0 (visit < 4)        → 未知（灰）
```

---

## 四、已发现的问题

### 4.1 P0：前端增量更新被注释，地图实际为空

**代码位置**：`g2p5.cc:RenderFront()`

```cpp
{
    UL lock{newest_map_mutex_};
    // AddKfToMap COMMENTED OUT:
    // lightning::Timer::Evaluate([&]() { AddKfToMap({kf}, frontend_map_); }, ...);
    newest_map_ = frontend_map_;  // frontend_map_ 从未更新，始终为空！
}
```

**影响**：在线/离线建图过程中，2D 地图始终为空，直到第一次回环触发 `RenderBack()` 后才有数据。无回环的场景下，地图永远为空。

**修复**：恢复 `AddKfToMap` 调用。

---

### 4.2 P0：`map.yaml` 分辨率硬编码

**代码位置**：`system/slam.cc:SaveMap()`

```cpp
emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);  // 硬编码 0.05
```

实际地图分辨率由 `g2p5.grid_map_resolution` 配置决定（默认 0.2m）。硬编码 0.05 会导致 Nav2 以错误比例加载地图（地图比实际缩小 4 倍）。

**修复**：将 `0.05` 替换为实际分辨率。

---

### 4.3 P1：角度分辨率仅 1°，精度损失严重

**代码位置**：`g2p5.cc:Convert3DTo2DScan()` 第 324 行

```cpp
// NOTE 转成整形的360度是有精度损失的
int angle = int(round(dangle) + 360) % 360;
```

典型 32 线 LiDAR 水平角分辨率约 0.1°～0.2°，1° 分桶导致：
- 5～10 个连续点被合并到同一桶，丢失障碍物精细边界信息
- 相邻 1° 方向之间存在信息空洞

同样地，`SetWhitePoints()` 也只用 360 个整数方向做射线追踪，导致自由空间标记呈锯齿状（阶梯效应）。

---

### 4.4 P1：首次命中高度主导，后续观测被忽略

**代码位置**：`g2p5_subgrid.h:SetGridHitPoint()`

```cpp
if (hit) {
    if (height < cell.height_) {  // 只有"更低高度"才更新 hit_cnt_
        cell.height_ = height;
        cell.hit_cnt_ += 1;
        cell.visit_cnt_ += 1;
    }
    // 若 height >= cell.height_，hit_cnt 和 visit_cnt 均不更新
}
```

**问题**：若首次命中高度已是该格子的最低高度（如 0.5m），之后从 0.8m 或 1.0m 命中同一格子时，`hit_cnt_` 不增加，该格子实际只有 1 次命中记录，无法区分"真实障碍物"和"单帧噪声点"。

---

### 4.5 P1：黑色格子无法被刷白（高度条件过严）

当射线经过一个已被标记为黑色（`hit_cnt_ > 3`）的格子时：

```cpp
if (cell.hit_cnt_ > 3) {
    if (height < cell.height_) {   // 只有从"障碍物上方"经过才能刷白
        cell.visit_cnt_ += 1;
    }
}
```

几何含义正确（从障碍下方经过不应刷白），但对于动态场景或建图过程中的误检，实际上已被误标为黑色的格子极难被修正，导致地图中遗留大量幽灵障碍物。

---

### 4.6 P2：无动态障碍物过滤

地图中所有障碍物一经写入即永久保留（包括人、车等动态物体）。后端重绘会清理部分，但仅在回环触发时才执行全量重建。

---

### 4.7 P2：地面估计默认关闭

`esti_floor_` 默认为 `false`，使用固定的 `floor_height`（默认 -1.2m）。当机器人在坡道、斜面、弹性地板上运动时，固定地面参数导致高度过滤失效，误将地面点识别为障碍物，或将低矮障碍物过滤掉。

---

## 五、优化方案

### 方案一：恢复前端增量更新（P0，2 行修复）

**文件**：`src/core/g2p5/g2p5.cc`，`RenderFront()` 函数

```cpp
void G2P5::RenderFront(Keyframe::Ptr kf) {
    {
        UL lock(frontend_mutex_);
        frontend_current_ = kf;
    }
    {
        UL lock{newest_map_mutex_};
        AddKfToMap({kf}, frontend_map_);  // ← 恢复此行
        newest_map_ = frontend_map_;
    }
    if (map_update_cb_) {
        map_update_cb_(newest_map_);
    }
}
```

**预期效果**：每次关键帧到来时地图实时更新，无需等待回环。

---

### 方案二：修复 map.yaml 分辨率（P0，1 行修复）

**文件**：`src/core/system/slam.cc`，`SaveMap()` 函数

```cpp
// 修复前：
emitter << YAML::Key << "resolution" << YAML::Value << float(0.05);

// 修复后：
emitter << YAML::Key << "resolution" << YAML::Value << float(g2p5_->GetNewestMap()->GetGridResolution());
```

`G2P5Map` 需补充 `GetGridResolution()` 接口（返回 `options_.resolution_`）。

---

### 方案三：提升角度分辨率（P1）

将 360 分桶提升为 3600 分桶（0.1° 精度），匹配 LiDAR 实际水平角分辨率：

**文件**：`src/core/g2p5/g2p5.cc`

```cpp
// Convert3DTo2DScan()
constexpr int NUM_RAYS = 3600;
std::vector<std::map<double, double>> rays(NUM_RAYS);
std::vector<Vec2d> angle_distance_height(NUM_RAYS, Vec2d::Zero());

// 角度桶映射（0.1° 精度）
int angle = int(round(dangle * 10) + 3600) % NUM_RAYS;
```

```cpp
// SetWhitePoints()
for (int i = 0; i < NUM_RAYS; ++i) {
    double angle = float(i) * (M_PI / 1800.0);  // 0.1° → 弧度
    // ...
}
```

**预期效果**：障碍物边界更精确，自由空间无锯齿。

---

### 方案四：修复 hit 计数逻辑（P1）

**文件**：`src/core/g2p5/g2p5_subgrid.h`

允许在有效高度范围内的所有命中都计入 `hit_cnt_`，不再依赖高度严格递减：

```cpp
if (hit) {
    // 有效高度范围内的命中均统计（不仅限于更低高度）
    if (height < cell.height_) {
        cell.height_ = height;  // 仅在更低时更新最低高度
    }
    cell.hit_cnt_ += 1;
    cell.visit_cnt_ += 1;
}
```

同时配合 **最小命中次数阈值**：将占据判定从 `occ > 0.3` 改为要求 `hit_cnt_ >= 2`，过滤单帧噪声：

```cpp
// G2P5Map::ToROS() / ToCV()
float occ = (visit_cnt > 3 && hit_cnt >= 2)
            ? (float)hit_cnt / (float)visit_cnt
            : (visit_cnt > 3 ? 0.0f : -1.0f);
```

**预期效果**：减少单帧激光反射噪声误标为黑色的情况；多次观测到的障碍物仍能稳定保留。

---

### 方案五：启用地面动态估计（P1）

在配置文件中开启：

```yaml
g2p5:
  esti_floor: true   # 默认 false，建议开启
```

代码层面增加估计失败时的平滑处理：

```cpp
// g2p5.cc:Convert3DTo2DScan()
if (options_.esti_floor_) {
    if (!DetectPlaneCoeffs(kf)) {
        // 失败时保持上一帧的地面参数（而非回退到默认值），防止跳变
        // floor_coeffs_ 已保留上一帧结果，无需重置
    }
}
```

---

### 方案六：地图后处理——膨胀（P2）

在保存 `map.pgm` 前，对占据格子进行形态学膨胀，生成导航安全余量：

```cpp
// slam.cc:SaveMap()
if (options_.with_gridmap_) {
    cv::Mat nav_image = ConvertToNavImage();

    // 障碍物膨胀（安全余量 = inflation_radius / resolution 个像素）
    int inflate_pix = static_cast<int>(options_.inflate_radius_ / g2p5_resolution);
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * inflate_pix + 1, 2 * inflate_pix + 1)
    );
    cv::dilate(nav_image, nav_image, kernel);   // 只膨胀黑色（占据）区域

    cv::imwrite(save_path + "/map.pgm", nav_image);
}
```

**预期效果**：导航路径自动远离障碍物边缘，不依赖 Nav2 的 costmap inflation，地图本身即包含安全余量。

---

## 六、优先级汇总

| 状态 | 问题 | 位置 | 优先级 |
|------|------|------|--------|
| 🔲 待修复 | 前端 AddKfToMap 被注释，地图始终为空 | `g2p5.cc:RenderFront()` | **P0** |
| 🔲 待修复 | map.yaml 分辨率硬编码 0.05 | `slam.cc:SaveMap()` | **P0** |
| 🔲 待实现 | 角度分辨率提升 1° → 0.1° | `g2p5.cc:Convert3DTo2DScan()` | P1 |
| 🔲 待修复 | hit 计数逻辑不统计同高度命中 | `g2p5_subgrid.h:SetGridHitPoint()` | P1 |
| 🔲 待实现 | 地面动态估计默认关闭 | `config/default.yaml` | P1 |
| 🔲 待实现 | 最小命中次数阈值（减少噪声） | `g2p5_map.cc:ToROS()/ToCV()` | P2 |
| 🔲 待实现 | 保存时障碍物膨胀（导航安全余量） | `slam.cc:SaveMap()` | P2 |
| 🔲 待实现 | 动态障碍物时间戳老化 | `g2p5.cc` | P3 |

---

## 七、关键文件索引

| 文件 | 职责 |
|------|------|
| `src/core/g2p5/g2p5.h / .cc` | 主逻辑：3D→2D 转换、前后端更新 |
| `src/core/g2p5/g2p5_map.h / .cc` | 栅格存储、ToROS、ToCV、SetMissPoint |
| `src/core/g2p5/g2p5_subgrid.h / .cc` | SubGrid 单元读写、SetGridHitPoint |
| `src/core/g2p5/g2p5_grid_data.h` | GridData 结构（hit_cnt, visit_cnt, height）|
| `src/core/system/slam.cc` | 集成：PushKeyframe、SaveMap、回环回调 |
| `config/default.yaml` | `g2p5.*` 配置项 |
