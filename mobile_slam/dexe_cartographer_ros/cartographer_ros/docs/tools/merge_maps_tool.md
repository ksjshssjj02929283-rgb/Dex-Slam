# merge_maps_tool

离线地图拼接工具。将两个独立建图的 pbstream 合并为一个多轨迹地图，自动完成坐标系对齐，无需任何运行中的 ROS 节点。

## 适用场景

- 两次建图有重叠区域，需要扩展为一张完整地图
- 机器人在不同时间段建图，需要将多张子地图拼接
- target map 和 source map 建图时使用了不同的起点（坐标系不同），工具自动完成对齐

> **前提条件**：两张地图必须有重叠区域（激光可以看到同一段墙壁）。

---

## 用法

```bash
ros2 run cartographer_ros merge_maps_tool \
  --map_target <target map 名称或路径> \
  --map_source <source map 名称或路径> \
  [--output_dir <输出目录>]
```

### 路径解析规则

`--map_target` 和 `--map_source` 均支持两种输入：

| 输入形式 | 解析规则 |
|---------|---------|
| 地图名称（如 `floor1`） | 优先在 `~/.dexe/data/maps/floor1/` 下查找 |
| 完整路径（如 `/data/maps/floor1`） | 先尝试默认路径，不存在则直接使用该路径 |

若两种路径均不存在 `map.pbstream`，工具报错退出并提示两个尝试路径。

### 输出规则

- 默认将合并结果覆盖写入 `target map` 目录（`map.pbstream`、`map.png`、`map.yaml`）
- 指定 `--output_dir` 可写入独立目录，不影响 target map 原文件
- **source map 文件始终保留，不会被删除**

---

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--map_target` | string | 必填 | **target map**（trajectory 0），坐标系以此为准 |
| `--map_source` | string | 必填 | **source map**（trajectory 1），将被定位到 target map 坐标系中 |
| `--output_dir` | string | `""` | 输出目录；留空则覆盖 target map 目录 |
| `--min_score` | double | `0.45` | scan match 最低分阈值（0–1），低于此分数的帧不计入重定位投票 |
| `--min_votes` | int | `3` | 至少需要多少帧成功 match 才确认重定位结果（抗噪） |
| `--resolution` | double | `0.05` | 输出地图分辨率（米/格） |
| `--occ_thresh` | double | `0.65` | 占用阈值：概率高于此值判定为障碍物 |
| `--free_thresh` | double | `0.196` | 空闲阈值：概率低于此值判定为自由空间 |

---

## 使用示例

### 示例 1：按名称合并（常用）

```bash
ros2 run cartographer_ros merge_maps_tool \
  --map_target floor1 \
  --map_source floor1_ext
```

工具在 `~/.dexe/data/maps/` 下查找两个地图，合并后覆盖写入 `floor1/`（target map）目录。

### 示例 2：指定完整路径 + 独立输出目录

```bash
ros2 run cartographer_ros merge_maps_tool \
  --map_target /home/robot/maps/floor1 \
  --map_source /home/robot/maps/floor1_ext \
  --output_dir /home/robot/maps/floor1_merged
```

### 示例 3：重叠区域较小时降低匹配阈值

```bash
ros2 run cartographer_ros merge_maps_tool \
  --map_target floor1 \
  --map_source floor1_ext \
  --min_score 0.35 \
  --min_votes 1
```

---

## 工作流程

```
target map.pbstream ──┐
                       ├─→ [1] 加载 & 解析
source map.pbstream ──┘

source map 节点点云 ──→ [2] FastCorrelativeScanMatcher（全图搜索）
                         │  对 target map 所有 finished submap 逐帧匹配
                         ↓
                   [3] 投票选 T_src_in_tgt
                         │  取 top-N 匹配帧加权平均
                         ↓
                   [4] 合并 pose_graph
                         │  target map = trajectory 0（不变）
                         │  source map = trajectory 1（all poses × T_src_in_tgt）
                         ↓
                   [5] 写 merged.pbstream（原子 rename）
                         ↓
                   [6] 渲染 map.png + map.yaml
```

### 各步骤说明

**Step 1 — 加载**

读取两个 pbstream，反序列化：
- `pose_graph`：保存 submap/node 的全局位姿（global frame）
- `all_records`：submap 纹理数据、node 点云、IMU/里程计数据

**Step 2 — 全图 scan match**

从 source map 的所有节点（`Node.node_data.filtered_gravity_aligned_point_cloud`）提取点云。  
对每帧点云，在 target map 所有已完成 submap 上调用 `FastCorrelativeScanMatcher2D::MatchFullSubmap`：
- 线性搜索窗口：300 m（覆盖整张地图）
- 角度搜索窗口：±180°（全方向）
- 分支定界深度：7 层

source map 不需要第一帧就能匹配成功，只要任意中间帧与 target map 有重叠即可。

**Step 3 — 投票确认 T_src_in_tgt**

每帧成功匹配后，用匹配结果反推 source map 原点在 target map 中的位姿：

$$T_{\text{src in tgt}} = T_{\text{tracking in tgt}} \cdot T_{\text{tracking in src}}^{-1}$$

收集多帧投票后，取得分最高的 `min_votes` 帧做加权平均（权重 = match score），输出最终 $T_{\text{src in tgt}}$。

**Step 4 — 合并 pose_graph**

- target map：trajectory 0，所有位姿不变
- source map：trajectory 1，所有 submap/node 的 global pose 施加 $T_{\text{src in tgt}}$：

  $$\text{pose\_new} = T_{\text{src in tgt}} \cdot \text{pose\_old}$$

同时将 source map 所有记录（submap 纹理、node、imu 等）的 `trajectory_id` 字段改为 1。

**Step 5 — 写回 pbstream**

先写入临时文件 `map.pbstream.tmp`，成功后原子 `rename` 到 `map.pbstream`，避免写失败导致原文件损坏。

**Step 6 — 渲染地图**

用合并后的 submap slices 调用 `PaintSubmapSlices`，生成 `map.png` 和 `map.yaml`。

---

## 常见问题

### 工具报错 "Not enough successful scan matches"

原因：target map 和 source map 重叠区域太小，或两图分辨率/传感器差异大。

解决：
```bash
--min_score 0.3    # 降低分数阈值
--min_votes 1      # 只需 1 帧匹配
```

### 合并后地图有明显错位

原因：重叠区域特征不明显（空旷走廊），单帧匹配不稳定。

解决：保留默认 `min_votes 3`，增加建图时的重叠长度后重新建图。

### 想保留合并前的 target map 备份

在运行前手动备份：
```bash
cp -r ~/.dexe/data/maps/floor1 ~/.dexe/data/maps/floor1_backup
```

或者使用 `--output_dir` 输出到新目录，不覆盖原文件。

---

## 与在线拼接的区别

| | merge_maps_tool（离线） | 在线多轨迹 |
|---|---|---|
| 依赖 | 无需 ROS 节点运行 | 需要 SLAM 或定位节点 |
| 触发方式 | CLI 命令 | Service Call |
| 精度 | 取决于重叠面积和特征丰富度 | 实时闭环，精度更高 |
| 适用 | 历史地图的离线拼接 | 实时多轨迹融合 |

---

## 源码位置

`cartographer_ros/cartographer_ros/tools/merge_maps_main.cc`
