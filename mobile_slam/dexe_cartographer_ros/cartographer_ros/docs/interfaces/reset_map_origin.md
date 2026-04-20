# 重置地图原点功能说明

> 适用版本：add_dynamic_0331 分支  
> 最后更新：2026-04-20

---

## 1. 功能概述

**重置地图原点**：在定位运行中，将当前机器人的定位位姿设为新的坐标原点（0,0,θ=0），把地图中所有存量数据——轨迹节点、submap、路标点、global_static_submap——统一平移旋转到新坐标系下，然后将新坐标系的地图图像（map.png/map.yaml）和状态文件（map.pbstream）写回磁盘。

**典型使用场景**：
- 建图完成后，希望以某个特定房间/门口为地图原点，但原始建图起点不在此处
- 多楼层/多区域地图拼接时统一原点约定
- 现场运维时"让机器人当前站立位置就是坐标原点"

**注意**：操作完成后需重启定位（`stop_localization` → `start_localization`）以使新地图生效。

---

## 2. 接口说明

### 2.1 ROS2 服务

| 字段 | 值 |
|------|-----|
| 服务名 | `~/reset_map_origin`（相对于 `LocalizationManagerNode` 命名空间） |
| 完整默认路径 | `/localization_manager/reset_map_origin` |
| 服务类型 | `std_srvs/srv/Trigger` |
| 请求字段 | 无（空请求） |
| 响应字段 | `bool success`，`string message` |

调用方式：

```bash
ros2 service call /localization_manager/reset_map_origin std_srvs/srv/Trigger
```

> 前置条件：定位必须处于 **RUNNING** 状态，且 `--map_folder` / `--map_name` gflag 已配置。

响应示例：

```
success: true
message: 'Map origin reset successfully. Restart localization to use the new map.'
```

失败时返回 `success: false`，`message` 中携带具体失败原因。

---

### 2.2 C++ 调用链

```
LocalizationManagerNode::HandleResetMapOrigin()   [ loc_node_main.cc ]
  └─ LocalizationNode::ResetMapOriginAndSave()    [ localization_node.cc ]
       └─ LocalizationUpdaterBridge::ResetMapOrigin()  [ localization_updater_bridge.cc ]
            ├─ LocalizationUpdater::ApplyOriginTransform()  [ localization_updater.cc ]
            │    ├─ PoseGraph2D::ApplyTransformToAllPoses()  [ pose_graph_2d.cc ]
            │    └─ global_static_submap_->set_local_pose()  [ submaps.h ]
            ├─ LocalizationUpdaterBridge::SaveMap()  → map.png + map.yaml
            └─ LocalizationUpdater::SerializeStateToFile()  → map.pbstream
```

各层函数签名：

```cpp
// loc_node_main.cc — 服务 handler
void HandleResetMapOrigin(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

// localization_node.h / .cc
bool ResetMapOriginAndSave(
    const std::string& map_base_filename,   // 不含后缀，如 "/maps/mymap/map"
    const std::string& pbstream_filename);  // 如 "/maps/mymap/map.pbstream"

// localization_updater_bridge.h / .cc
bool ResetMapOrigin(
    const std::string& map_base_filename,
    const std::string& pbstream_filename,
    float occ_thresh,    // 占用概率阈值（来自 node_options_.mapping_occ_thresh）
    float free_thresh);  // 空闲概率阈值（来自 node_options_.mapping_free_thresh）

// localization_updater.h / .cc
void ApplyOriginTransform(const cartographer::transform::Rigid3d& T);

// mapping/pose_graph_interface.h（虚函数，PoseGraph2D 实现）
virtual void ApplyTransformToAllPoses(const transform::Rigid3d& transform);

// mapping/submaps.h（新增 setter）
void set_local_pose(const transform::Rigid3d& local_pose);
```

---

## 3. 算法流程

### 3.1 流程图

```
调用 ~/reset_map_origin
        │
        ▼
[Guard] 定位是否 RUNNING？
   否 → 返回 false，提示未运行
        │
        ▼
[Step 1] 读取当前机器人的全局位姿
   traj_data = GetLocalTrajectoryData()
   T_robot = local_to_map × local_pose    ← 3D 全局位姿
   T_robot_2d = Embed3D(Project2D(T_robot)) ← 投影到 2D，消除 z/roll/pitch
   T_inv = T_robot_2d.inverse()           ← 旧系到新系的变换（"平移+旋转"）
        │
        ▼
[Step 2] 变换 Pose Graph 中所有数据
   PoseGraph2D::ApplyTransformToAllPoses(T_inv)
   ├─ 所有轨迹节点：node.global_pose  ← T_inv × node.global_pose
   ├─ 所有 2D submap：submap.global_pose_2d  ← Project2D(T_inv × Embed3D(pose))
   └─ 所有路标点：landmark.global_pose  ← T_inv × landmark.global_pose
        │
        ▼
[Step 3] 变换 global_static_submap 的 local_pose
   global_static_submap_.local_pose ← T_inv × local_pose
   （ local_pose 表示该 submap 在全局坐标系中的位置，必须同步更新 ）
        │
        ▼
[Step 4] 保存新地图图像
   SaveMap(map_base_filename, occ_thresh, free_thresh)
   → map.png + map.yaml（原点已对齐到新坐标系）
        │
        ▼
[Step 5] 保存新 pbstream
   SerializeStateToFile(true, pbstream_filename)
   → map.pbstream（含变换后的所有位姿 + global_static_submap）
        │
        ▼
返回 success = (map_ok && pb_ok)
```

### 3.2 坐标变换推导

设：
- $T_{R}^{M}$  = 机器人在旧地图坐标系下的位姿（`T_robot_2d`）
- $T_{inv}$ = $(T_{R}^{M})^{-1}$（旧系到新系的变换）

对任意一个旧地图坐标系中的点 $p^{M}$，其在新坐标系中的坐标为：

$$p^{M'} = T_{inv} \cdot p^{M}$$

调用完成后，机器人在新坐标系中的位姿为：

$$T_{R}^{M'} = T_{inv} \cdot T_{R}^{M} = I \quad (\text{即原点})$$

### 3.3 为什么限定在 2D（Project2D / Embed3D）

定位框架为 2D SLAM。全局位姿变换可能携带微小的 z 轴偏移或 roll/pitch 抖动（源自 IMU 积分误差）。先 `Project2D` 再 `Embed3D` 可消除这些分量，确保：
- 新原点的旋转严格在 XY 平面内（yaw-only）
- 不引入 z 轴漂移进入 map.yaml 的 origin 字段

---

## 4. 输出文件

调用成功后，`{map_folder}/{map_name}/` 目录下以下文件被**原地覆盖**：

| 文件 | 内容变化 |
|------|---------|
| `map.png` | 重新渲染，图像内容不变，但 yaml 中 origin 字段变为 `[0, 0, 0]`（或接近） |
| `map.yaml` | `origin` 字段更新为新坐标系下的值 |
| `map.pbstream` | 所有轨迹节点、submap、路标、global_static_submap 位姿已变换 |

---

## 5. 修改的源文件清单

| 文件 | 改动说明 |
|------|---------|
| `cartographer/mapping/submaps.h` | 将 `local_pose_` 改为非 `const`，新增 `set_local_pose()` setter |
| `cartographer/mapping/pose_graph_interface.h` | 新增虚函数 `ApplyTransformToAllPoses()`，默认空实现 |
| `cartographer/mapping/internal/2d/pose_graph_2d.h` | 声明 `ApplyTransformToAllPoses()` override |
| `cartographer/mapping/internal/2d/pose_graph_2d.cc` | 实现：遍历变换 nodes / global_submap_poses_2d / landmark_nodes |
| `cartographer/localization/localization_updater.h` | 新增 `ApplyOriginTransform(T)` 公开接口声明 |
| `cartographer/localization/localization_updater.cc` | 实现：调用 pose_graph + 更新 global_static_submap_.local_pose |
| `cartographer_ros/localization/localization_updater_bridge.h` | 新增 `ResetMapOrigin()` 声明 |
| `cartographer_ros/localization/localization_updater_bridge.cc` | 实现完整 4 步流程 |
| `cartographer_ros/localization/localization_node.h` | 新增 `ResetMapOriginAndSave()` 声明 |
| `cartographer_ros/localization/localization_node.cc` | 实现：加锁后委托 bridge |
| `cartographer_ros/localization/loc_node_main.cc` | 注册服务 `~/reset_map_origin`，实现 `HandleResetMapOrigin` |

---

## 6. 使用限制与注意事项

1. **不影响运行中的定位**：变换仅修改内存中的位姿数据。定位估计依赖 `global_static_submap` 的匹配，该 submap 的栅格/点云内容**不变**，仅 `local_pose` 字段更新，因此当次定位仍可正常运行。

2. **需重启才生效**：保存后需执行 `stop_localization` + `start_localization` 重新加载新 pbstream，新坐标系才对外生效。

3. **覆盖写入**：新文件直接覆盖旧文件，无备份机制。如有需要请在调用前手动备份。

4. **并发安全**：`LocalizationNode::ResetMapOriginAndSave` 持有 `localization_updater_mutex_`；`PoseGraph2D::ApplyTransformToAllPoses` 持有其内部 `mutex_`。两者均线程安全，但同一时刻不应与自动保存（`MapPersistenceManager`）并发——`HandleResetMapOrigin` 未检查 `IsSaving()`，如确有需要可在 handler 中补充该判断。

5. **定位状态为 LOC_NOT_INIT 时慎用**：Step 1 依赖 `local_slam_data` 非空，若机器人刚启动尚未完成初始定位，调用会返回 false 并打印错误日志。
