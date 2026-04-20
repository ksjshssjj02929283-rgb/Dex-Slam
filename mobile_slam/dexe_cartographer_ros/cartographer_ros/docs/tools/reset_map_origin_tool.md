# reset_map_origin_tool

离线地图原点重置工具。无需启动 ROS 节点，直接对地图文件进行坐标变换并保存。

## 功能说明

传入地图目录（包含 `map.pbstream`、`map.yaml`、`map.png`）以及新原点在旧地图坐标系中的位姿 (x, y, yaw)，工具将：

1. 加载 `map.pbstream`，反序列化所有 submap 切片
2. 对每个 submap 的全局位姿应用逆变换：`pose_new = T_new_origin⁻¹ * pose_old`
3. 重新渲染并覆盖保存 `map.png` + `map.yaml`

> **注意：原文件会被覆盖，请提前备份。**

---

## 用法

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_dir /path/to/map \
  --new_x <x> \
  --new_y <y> \
  --new_yaw <yaw_rad>
```

---

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--map_dir` | string | `""` | 地图目录路径，包含 `map.pbstream`、`map.yaml`、`map.png`。与 `--pbstream_filename` 至少指定一个 |
| `--pbstream_filename` | string | `""` | 显式指定 pbstream 文件路径（可选，覆盖 `map_dir/map.pbstream`） |
| `--map_filestem` | string | `""` | 输出文件 stem（可选，默认为 `map_dir/map`，即输出 `map.png` / `map.yaml`） |
| `--new_x` | double | `0.0` | 新原点在旧地图坐标系中的 X 坐标（米） |
| `--new_y` | double | `0.0` | 新原点在旧地图坐标系中的 Y 坐标（米） |
| `--new_yaw` | double | `0.0` | 新原点在旧地图坐标系中的旋转角（**弧度**） |
| `--resolution` | double | `0.05` | 输出地图分辨率（米/格） |
| `--occ_thresh` | double | `0.65` | 占用阈值：概率高于此值判定为障碍物 |
| `--free_thresh` | double | `0.196` | 空闲阈值：概率低于此值判定为自由空间 |

---

## 使用示例

### 示例 1：将地图原点移到当前位置 (1.5, -2.0)，旋转 45°

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_dir /home/robot/maps/floor1 \
  --new_x 1.5 \
  --new_y -2.0 \
  --new_yaw 0.785
```

执行后 `/home/robot/maps/floor1/map.png` 和 `map.yaml` 将被重写，地图原点平移到以前坐标 (1.5, -2.0, 45°) 处。

### 示例 2：仅重置平移，不旋转

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --map_dir /home/robot/maps/floor1 \
  --new_x 3.0 \
  --new_y 1.0
```

### 示例 3：指定自定义 pbstream 文件和输出 stem

```bash
ros2 run cartographer_ros reset_map_origin_tool \
  --pbstream_filename /home/robot/maps/floor1/backup.pbstream \
  --map_filestem /home/robot/maps/floor1/map_new \
  --new_x 0.5 \
  --new_y 0.5 \
  --new_yaw 1.5708
```

输出文件为 `map_new.png` 和 `map_new.yaml`，原 pbstream 不变。

---

## 与在线服务的区别

| | 在线服务 `~/reset_map_origin` | 离线工具 `reset_map_origin_tool` |
|---|---|---|
| 触发方式 | ROS2 Service Call | CLI 命令行 |
| 依赖 | 需运行定位节点 | 无需 ROS 节点运行 |
| 新原点来源 | 当前机器人定位位置 | 命令行参数（旧地图坐标） |
| 适用场景 | 运行时重置 | 地图文件离线编辑 |

---

## 源码位置

`cartographer_ros/cartographer_ros/tools/reset_map_origin_main.cc`
