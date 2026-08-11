# map_odom_tf_broadcaster 使用说明

## 功能说明

独立节点，用于发布 `map→odom` TF 变换，实现 3D 定位与轮式里程计的坐标系桥接。

### TF 树结构

```
map (3D定位全局坐标系)
  └── odom (轮式里程计坐标系，无长期漂移)
        └── base_link (机器人本体)
              └── lidar (激光雷达)
```

### 计算原理

```
map→odom = (map→base_link) ⊗ (odom→base_link)⁻¹
```

通过订阅两个话题：
- `/lightning/pose/current` - 3D 定位位姿（map→base_link）
- `/feedback/odom` - 轮式里程计（odom→base_link）

计算并发布 `map→odom` TF。

---

## 使用方法

### 方式 1：通过 launch 文件启动（推荐）

```bash
# 启用 map->odom TF 桥接
ros2 launch lightning loc_online.launch.py \
    enable_map_odom_tf:=true \
    odom_topic:=/feedback/odom \
    map_name:=your_map_name
```

### 方式 2：手动启动节点

```bash
# 先启动定位
ros2 launch lightning loc_online.launch.py map_name:=your_map_name

# 在另一个终端启动 TF 桥接节点
ros2 run lightning map_odom_tf_broadcaster \
    --ros-args \
    -p odom_topic:=/feedback/odom \
    -p pose_topic:=/lightning/pose/current \
    -p odom_frame:=odom \
    -p base_link_frame:=base_link \
    -p map_frame:=map
```

---

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `odom_topic` | string | `/feedback/odom` | 轮式里程计话题 |
| `pose_topic` | string | `/lightning/pose/current` | 3D 定位位姿话题 |
| `odom_frame` | string | `odom` | 里程计坐标系名称 |
| `base_link_frame` | string | `base_link` | 机器人本体坐标系 |
| `map_frame` | string | `map` | 全局地图坐标系 |

---

## 验证方法

### 1. 检查 TF 树

```bash
ros2 run tf2_tools view_frames
# 查看生成的 frames.pdf，确认 TF 树结构正确
```

### 2. 实时查看 TF 变换

```bash
ros2 run tf2_ros tf2_echo map odom
```

预期输出：
```
- Translation: [x, y, z]
- Rotation: in Quaternion [x, y, z, w]
```

### 3. 检查话题

```bash
# 查看定位位姿话题
ros2 topic echo /lightning/pose/current --once

# 查看轮式里程计话题
ros2 topic echo /feedback/odom --once

# 查看 TF 发布频率
ros2 topic hz /tf
```

### 4. RViz 可视化

```bash
rviz2
```

添加以下显示：
- **TF** - 显示坐标系
- **Pose** - 订阅 `/lightning/pose/current`
- **Odometry** - 订阅 `/feedback/odom`

---

## 时间对齐说明

本节点采用**时间戳匹配策略**：

1. **轮式里程计**通常频率较高（100Hz）
2. **3D 定位位姿**频率较低（5-10Hz）
3. 发布 `map→odom` 时使用**轮式里程计的时间戳**
4. 确保 TF 树在任意时刻都满足：`map→odom ⊗ odom→base_link = map→base_link`

### 异常处理

- 如果未收到轮式里程计数据：日志警告，不发布 TF
- 如果未收到定位位姿数据：日志警告，不发布 TF
- 使用 `RCLCPP_WARN_THROTTLE` 避免日志洪水（1秒最多1条）

---

## 性能指标

- **发布频率**: 50Hz（20ms 定时器）
- **CPU 占用**: < 1%
- **内存占用**: < 10MB
- **延迟**: < 1ms（纯 TF 计算）

---

## 故障排查

### 问题 1: TF 树断裂

**现象**: `map` 和 `odom` 之间没有连接

**排查**:
```bash
# 检查节点是否运行
ros2 node list | grep map_odom

# 检查话题是否有数据
ros2 topic hz /feedback/odom
ros2 topic hz /lightning/pose/current
```

### 问题 2: TF 跳变

**现象**: `map→odom` 突然大幅变化

**可能原因**:
- 定位重定位导致位姿跳变
- 轮式里程计数据异常

**排查**:
```bash
# 查看节点日志
ros2 node info /map_odom_tf_broadcaster
```

### 问题 3: 时间戳不匹配

**现象**: TF 查询失败

**解决**: 确保系统时间同步，检查 NTP 服务

---

## 代码位置

- 头文件: `src/wrapper/map_odom_tf_broadcaster.h`
- 实现: `src/wrapper/map_odom_tf_broadcaster.cc`
- CMake: `src/app/CMakeLists.txt`
- Launch: `launch/loc_online.launch.py`