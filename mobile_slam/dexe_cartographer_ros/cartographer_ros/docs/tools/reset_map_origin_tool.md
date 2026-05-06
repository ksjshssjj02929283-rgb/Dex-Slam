# reset_map_origin_tool

离线地图原点重置工具。无需启动 ROS 节点，直接对地图文件进行坐标变换并保存。

## 功能说明

传入地图目录（包含 `map.pbstream`、`map.yaml`、`map.png`）以及新原点在旧地图坐标系中的位姿 (x, y, yaw)，工具将：

1. **单次读盘**把 `map.pbstream` 全部加载到内存（header + pose_graph + options + 所有 SerializedData）
2. 计算 `T_inv = T_new_origin⁻¹`，对内存数据原地变换：
   - `pose_graph` 的 submap / node / landmark 全局位姿
   - `global_static_submap` 的 **local_pose、cloud、highlight_cloud、line_map_2d_list**（grid 见第 3 步）
   - `trajectory_data.fixed_frame_origin_in_map`
3. **重新生成最终栅格地图**：把所有 trajectory node 的 `filtered_gravity_aligned_point_cloud` 在新坐标系下插入一张干净的 `ProbabilityGrid`（与 `MapBuilderBridge::GenerateFinalSubmap` 的 range_data 重插逻辑一致）。**不再用旧 grid 重栅格化或拼接 submap 切片**，避免重叠区重影。
4. 用变换后的 `global_static_submap`（含新 grid + 已变换的 cloud/line）渲染 `map.png`：包成单 `SubmapSlice` → `PaintSubmapSlices` → `GeneratePngMat` → `WriteNavigationMap`，与节点 `~/save_map` 完全一致。
5. 把变换后的内存数据**整体写回** `map.pbstream`（原子 rename），多次调用变换会**累积**

> **回退路径**：若 pbstream 没有 trajectory node（纯 pose graph），改用旧 grid 的重栅格化（first-wins）；若连 `global_static_submap` 都没有，回退到拼接 trajectory submap 切片（旧行为，可能有重影）。

### pbstream 中被变换的字段

| 字段 | 是否变换 | 说明 |
|------|----------|------|
| `pose_graph.trajectory[].submap[].pose` | ✅ | submap 全局位姿 |
| `pose_graph.trajectory[].node[].pose` | ✅ | trajectory node 全局位姿 |
| `pose_graph.landmark_poses[].global_pose` | ✅ | 路标全局位姿 |
| `SerializedData.global_static_submap.local_pose` | ✅ | 全局静态 submap 位姿 |
| `SerializedData.global_static_submap.cloud / highlight_cloud` | ✅ | cloud map 内点云（每个点用 3D 刚体变换） |
| `SerializedData.global_static_submap.line_map_2d_list` | ✅ | feature map 内线段（端点用 2D 刚体，法向量仅旋转） |
| `SerializedData.global_static_submap.grid` | ♻️ | **从 trajectory node 的 cloud 重新插入生成**（避免重影；fallback 时用 first-wins 重栅格化旧 grid） |
| `SerializedData.trajectory_data.fixed_frame_origin_in_map` | ✅ | fixed frame 在 map 中的原点（如 GPS） |
| `pose_graph.constraint[].relative_pose` | ❌ | 相对位姿，与全局原点无关 |
| `SerializedData.submap.submap_2d.local_pose` | ❌ | trajectory 内部局部坐标系，不随全局原点变 |
| `SerializedData.submap.submap_2d.{grid, cloud, line_map_2d_list}` | ❌ | trajectory submap 内的数据在 submap 自身局部坐标系，会通过被变换的 submap global pose 间接得到正确全局位置 |
| `SerializedData.node.node_data.local_pose` | ❌ | trajectory 局部坐标系 |
| `SerializedData.imu_data` / `odometry_data` / `landmark_data` / `fixed_frame_pose_data` | ❌ | 传感器原始数据/观测，不在 map frame 内 |

> **关于 `global_static_submap` 与普通 trajectory submap 的差异**：dexe-cartographer 的 `global_static_submap` 是把所有 trajectory 数据合并后存成单一 submap，其 cloud / line / grid 都直接以**全局坐标**保存（参考 `MapBuilderBridge::GenerateFinalSubmap` 的插入逻辑）。所以变换 `local_pose` 不会带动这些数据：
> - **cloud / line**：逐点 / 逐线段做坐标变换（cloud 用 3D 刚体；line 端点用 2D 刚体、法向量仅 2D 旋转）。
> - **grid**：从 trajectory node 重新插入生成。pbstream 中每个 node 都存了 `filtered_gravity_aligned_point_cloud`（压缩点云），把它在新坐标系下用 `ProbabilityGridRangeDataInserter2D` 重插，得到无重影、参数与原始 mapping 一致的栅格地图。
>
> 普通 trajectory submap 则是把数据存在 submap 自身的局部坐标系，跟随 `pose_graph.submap.pose` 的变化即可，无需逐点变换。
>
> **栅格重生成参数**：
> - `hit/miss_probability`：从 `all_trajectory_builder_options[0].submaps_options.range_data_inserter_options` 读取（与 mapping 阶段同一参数）；缺失时用默认 hit=0.65, miss=0.48。
> - `origin`：用 node global pose 的平移近似激光起点（假设激光固定在机器人坐标原点；2D SLAM 通常成立）。
> - `bbox`：node 位姿凸包再加 30m padding（默认覆盖典型激光最大射程）。
> - 不重插 `misses` 字段（pbstream 不存）。射线追踪走 origin → returns 的方式标记自由空间，与 mapping 阶段一致。

> **注意：`map.png`、`map.yaml`、`map.pbstream` 三个文件都会被覆盖，请提前备份。**

---

## 用法（推荐）

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_name <地图名称> \
  --new_x <x> \
  --new_y <y> \
  --new_yaw <yaw_rad>
```

默认根目录是 `~/.dexe/data/maps`，工具会自动从 `~/.dexe/data/maps/<map_name>/` 加载地图，无需写完整路径。

如需用其他根目录或完整路径，可参考下方的 `--maps_root` / `--map_dir`。

---

## 参数说明

### 地图位置（三种方式，按优先级）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--map_name` | string | `""` | **推荐**。仅传地图名称，最终目录为 `<maps_root>/<map_name>` |
| `--maps_root` | string | `""` | 默认地图根目录，留空则使用 `~/.dexe/data/maps` |
| `--map_dir` | string | `""` | 地图目录完整路径，或相对 `maps_root` 的名称（向后兼容） |
| `--pbstream_filename` | string | `""` | 直接指定 pbstream 文件路径（最细粒度控制） |
| `--map_filestem` | string | `""` | 输出文件 stem（可选，默认为 `<map_dir>/map`，即输出 `map.png` / `map.yaml`） |

> 三者至少要传 `--map_name`、`--map_dir`、`--pbstream_filename` 之一。

### 几何参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--new_x` | double | `0.0` | 新原点在旧地图坐标系中的 X 坐标（米） |
| `--new_y` | double | `0.0` | 新原点在旧地图坐标系中的 Y 坐标（米） |
| `--new_yaw` | double | `0.0` | 新原点在旧地图坐标系中的旋转角（**弧度**） |
| `--resolution` | double | `0.05` | 输出地图分辨率（米/格） |
| `--occ_thresh` | double | `0.65` | 占用阈值：概率高于此值判定为障碍物 |
| `--free_thresh` | double | `0.196` | 空闲阈值：概率低于此值判定为自由空间 |

### 渲染参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--navigation_map_filter_mode` | string | `normal` | 导航地图噪声过滤强度：`normal` / `high` / `super`，与节点 `save_map` 共用同一逻辑 |
| `--save_origin_map` | bool | `false` | 同时保存未经噪声过滤的原始灰度图为 `<map_filestem>_origin.png`，便于对照 |
| `--save_debug_map` | bool | `false` | 输出噪声过滤的中间调试图到 `<map_dir>/debug_map/`（mask、连通域、膨胀等） |

---

## 使用示例

### 示例 1：仅传地图名称（推荐）

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_name floor1 \
  --new_x 1.5 \
  --new_y -2.0 \
  --new_yaw 0.785
```

地图目录解析为 `~/.dexe/data/maps/floor1/`，`map.png` / `map.yaml` / `map.pbstream` 被覆盖，原点平移到 (1.5, -2.0)、旋转 45°。

### 示例 2：自定义根目录

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --maps_root /mnt/data/maps \
  --map_name floor1 \
  --new_x 3.0 --new_y 1.0
```

地图目录解析为 `/mnt/data/maps/floor1/`，仅平移不旋转。

### 示例 3：用完整路径（向后兼容）

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_dir /home/robot/maps/floor1 \
  --new_x 1.5 --new_y -2.0 --new_yaw 0.785
```

### 示例 4：自定义 pbstream 与输出 stem

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --pbstream_filename /home/robot/maps/floor1/backup.pbstream \
  --map_filestem /home/robot/maps/floor1/map_new \
  --new_x 0.5 --new_y 0.5 --new_yaw 1.5708
```

输出 `map_new.png` 和 `map_new.yaml`。注意：若 `--map_filestem` 与 `--pbstream_filename` 指向不同位置则 pbstream 不变；否则会按"功能说明 4"覆盖写回。

### 示例 5：强噪声过滤并保留原始图对照

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_name floor1 \
  --new_x 1.5 --new_y -2.0 --new_yaw 0.0 \
  --navigation_map_filter_mode high \
  --save_origin_map
```

输出 `map.png`（已过滤）和 `map_origin.png`（未过滤），便于对照过滤效果。

---

## 与在线服务的区别

| | 在线服务 `~/reset_map_origin` | 离线工具 `reset_map_origin_tool` |
|---|---|---|
| 触发方式 | ROS2 Service Call | CLI 命令行 |
| 依赖 | 需运行定位节点 | 无需 ROS 节点运行 |
| 新原点来源 | 当前机器人定位位置 | 命令行参数（旧地图坐标） |
| 噪声过滤参数 | 来自 lua 配置（`navigation_map_filter_mode` 等） | 来自命令行 flag，默认 `normal` |
| 渲染管线 | `PaintSubmapSlices` + `WriteNavigationMap` | **同上**（已对齐） |
| 适用场景 | 运行时重置 | 地图文件离线编辑 |

> 两条路径目前共用 `GeneratePngMat` + `WriteNavigationMap` + `WriteYaml`，相同输入产出**完全一致**的 `map.png` / `map.yaml`。

---

## 源码位置

`cartographer_ros/cartographer_ros/tools/reset_map_origin_main.cc`
