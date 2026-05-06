# DEXE 2D SLAM 接口文档

> 基于 Cartographer ROS2，适用于 DEXE W1 PRO。  
> 节点可执行文件均在 `cartographer_ros` 包中。

**目录**
- [一、实时建图接口](#一实时建图接口)
- [二、定位接口](#二定位接口)
- [三、地图管理接口（map_manager）](#三地图管理接口map_manager)
- [四、系统联动说明](#四系统联动说明)
- [五、Launch 启动参数](#五launch-启动参数)
- [六、地图文件结构](#六地图文件结构)

---

## 一、实时建图接口

### 节点信息

| 项目 | 内容 |
|------|------|
| 节点名 | `mapping_node` |
| 可执行文件 | `cartographer_node` |
| 配置文件 | `dexe_slam_2d.lua` |
| 地图默认保存路径 | `~/.dexe/data/maps/<map_name>/` |

### 1.1 传感器输入话题

| 功能 | 话题名 | 类型 | 描述 |
|------|--------|------|------|
| 前激光雷达 | `laser_scan/front` | `sensor_msgs/msg/LaserScan` | 主建图激光源 |
| 后激光雷达 | `laser_scan/rear` | `sensor_msgs/msg/LaserScan` | 辅助激光源 |
| IMU | `imu/chassis` | `sensor_msgs/msg/Imu` | 底盘 IMU |
| 里程计 | `feedback/odom` | `nav_msgs/msg/Odometry` | 轮式里程计 |

### 1.2 输出话题

| 功能 | 话题名 | 类型 | 描述 |
|------|--------|------|------|
| 建图状态 | `/mapping_node/mapping_status` | `std_msgs/msg/Int16` | `1`=建图中，`-1`=空闲 |
| 实时占用栅格地图 | `/cartographer_occupancy_grid_node/map` | `nav_msgs/msg/OccupancyGrid` | 5Hz 刷新，分辨率 0.04 m |
| 子图列表 | `submap_list` | `cartographer_ros_msgs/msg/SubmapList` | 可视化用 |
| 机器人轨迹 | `tracked_path` | `nav_msgs/msg/Path` | 建图过程轨迹 |
| 轨迹节点 | `trajectory_node_list` | `visualization_msgs/msg/MarkerArray` | RViz 可视化 |
| 约束线 | `constraint_list` | `visualization_msgs/msg/MarkerArray` | RViz 可视化 |

### 1.3 服务接口

| 功能 | 服务名称 | 类型 | 描述 | 使用示例 |
|------|----------|------|------|----------|
| 设置地图名称 | `/mapping_node/set_map_name` | `nav_interfaces/srv/SetMapName` | 建图前必须先设置，决定保存路径 | `ros2 service call /mapping_node/set_map_name nav_interfaces/srv/SetMapName "{map_name: 'office_1'}"` |
| 开始建图 | `/mapping_node/start_mapping` | `std_srvs/srv/Trigger` | 启动 Cartographer 轨迹，开始实时建图 | `ros2 service call /mapping_node/start_mapping std_srvs/srv/Trigger` |
| 完成建图并保存 | `/mapping_node/finish_mapping` | `std_srvs/srv/Trigger` | 跑最终全局优化，保存 `map.pbstream` + `map.png` + `map.yaml` | `ros2 service call /mapping_node/finish_mapping std_srvs/srv/Trigger` |
| 取消建图（不保存） | `/mapping_node/cancel_mapping` | `std_srvs/srv/Trigger` | 直接丢弃，不产生地图文件 | `ros2 service call /mapping_node/cancel_mapping std_srvs/srv/Trigger` |
| 保存当前地图 | `/mapping_node/save_map` | `std_srvs/srv/Trigger` | 等同于 `finish_mapping`，完成优化并保存 | `ros2 service call /mapping_node/save_map std_srvs/srv/Trigger` |
| 查询建图状态 | `/mapping_node/get_mapping_status` | `nav_interfaces/srv/GetString` | 返回 `"mapping"` 或 `"idle"` | `ros2 service call /mapping_node/get_mapping_status nav_interfaces/srv/GetString` |

### 1.4 典型建图流程

```bash
# 1. 启动建图
ros2 launch cartographer_ros dexe_slam_2d.launch.py map_name:=office_1

# 2. 设置地图名称（首次建图时，launch 已通过 gflag 传入，此步可省略；动态修改时使用）
ros2 service call /mapping_node/set_map_name nav_interfaces/srv/SetMapName \
  "{map_name: 'office_1'}"

# 3. 开始建图
ros2 service call /mapping_node/start_mapping std_srvs/srv/Trigger

# 4. 遥控机器人完成建图...
# 实时地图通过 /cartographer_occupancy_grid_node/map 话题输出，可在 RViz 或 Web 端订阅

# 5. 完成建图并保存
ros2 service call /mapping_node/finish_mapping std_srvs/srv/Trigger
# 保存文件：~/.dexe/data/maps/office_1/map.pbstream
#           ~/.dexe/data/maps/office_1/map.png
#           ~/.dexe/data/maps/office_1/map.yaml
```

---

## 二、定位接口

### 节点信息

| 项目 | 内容 |
|------|------|
| 节点名 | `localization_node` |
| 可执行文件 | `localization_node` |
| 配置文件 | `dexe_localization_2d.lua` |
| 依赖节点 | `robot_localization`（UKF 滤波器） |

### 2.1 传感器输入话题

| 功能 | 话题名 | 类型 | 描述 |
|------|--------|------|------|
| 前激光雷达 | `laser_scan/front` | `sensor_msgs/msg/LaserScan` | 定位主传感器 |
| 后激光雷达 | `laser_scan/rear` | `sensor_msgs/msg/LaserScan` | 辅助定位 |
| 精炼 IMU | `imu/chassis/refined` | `sensor_msgs/msg/Imu` | 底盘精炼 IMU |
| 里程计 | `feedback/odom` | `nav_msgs/msg/Odometry` | 轮式里程计 |
| 初始位姿 | `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 手动点击重定位 |

### 2.2 输出话题

| 功能 | 话题名 | 类型 | 描述 |
|------|--------|------|------|
| 定位位姿 | `loc_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 带协方差的定位输出 |
| 定位状态 | `localization_state` | `nav_interfaces/msg/LocalizationState` | 状态码 + 置信度 score |
| TF 变换 | `map` → `base_link` | `tf2` | 50 Hz，导航核心依赖 |
| 实时地图（定位叠加） | `/cartographer_occupancy_grid_node/map` | `nav_msgs/msg/OccupancyGrid` | 带新增子图的地图 |
| 机器人轨迹 | `tracked_path` | `nav_msgs/msg/Path` | 定位过程轨迹 |

#### `localization_state` 状态码说明

| 常量 | 值 | 含义 |
|------|----|------|
| `LOC_FAILED` | `-2` | 定位失败 |
| `LOC_INIT_FAILED` | `-1` | 初始化失败 |
| `LOC_NOT_INIT` | `0` | 未初始化 |
| `LOC_INITIALIZING` | `1` | 初始化中 |
| `LOC_INITIALIZED` | `2` | 初始化成功 |
| `LOC_RELOCALIZING` | `3` | 重定位中 |
| `LOC_SUCCESS` | `4` | 定位成功 |

### 2.3 定位管理服务（LocalizationManagerNode）

| 功能 | 服务名称 | 类型 | 描述 | 使用示例 |
|------|----------|------|------|----------|
| 启动定位 | `/localization_node/start_localization` | `std_srvs/srv/Trigger` | 加载地图并启动定位（需先设好 map_folder/map_name 或 auto_load_map=true） | `ros2 service call /localization_node/start_localization std_srvs/srv/Trigger` |
| 停止定位 | `/localization_node/stop_localization` | `std_srvs/srv/Trigger` | 保存地图并停止 | `ros2 service call /localization_node/stop_localization std_srvs/srv/Trigger` |
| 重置定位 | `/localization_node/reset_localization` | `std_srvs/srv/Trigger` | 停止后重新启动定位（保持当前地图） | `ros2 service call /localization_node/reset_localization std_srvs/srv/Trigger` |
| 地图已变更（触发换图+重定位） | `/localization_node/set_map_changed` | `std_srvs/srv/Trigger` | 从 `map_manager` 获取新地图路径，自动停止旧定位、加载新地图、启动新定位 | `ros2 service call /localization_node/set_map_changed std_srvs/srv/Trigger` |
| 立即保存地图 | `/localization_node/save_map` | `std_srvs/srv/Trigger` | 保存 pbstream + map.png/yaml | `ros2 service call /localization_node/save_map std_srvs/srv/Trigger` |
| 重置地图原点 | `/localization_node/reset_map_origin` | `std_srvs/srv/Trigger` | 以当前机器人位置为新原点，更新地图坐标系，需重启定位生效 | `ros2 service call /localization_node/reset_map_origin std_srvs/srv/Trigger` |

### 2.4 重定位服务（LocalizationNode 核心层）

| 功能 | 服务名称 | 类型 | 描述 | 使用示例 |
|------|----------|------|------|----------|
| 手动指定位姿重定位 | `/localization_node/relocalization` | `nav_interfaces/srv/SetPose` | 发送已知位姿，强制重定位到该位置 | 见下方示例 |

```bash
# 手动重定位示例（将机器人定位到地图坐标 x=1.5, y=2.0, yaw≈0）
ros2 service call /localization_node/relocalization nav_interfaces/srv/SetPose \
  "{pose: {header: {frame_id: 'map'},
            pose: {pose: {position: {x: 1.5, y: 2.0, z: 0.0},
                          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
                   covariance: []}}}"
```

### 2.5 典型定位流程

#### 场景一：启动时自动加载地图（`auto_load_map=true`）

```bash
# launch 时通过参数传入地图，节点启动后自动加载并开始定位
ros2 launch cartographer_ros dexe_localization.launch.py \
  map_folder:=$HOME/.dexe/data/maps \
  map_name:=office_1
# 节点启动 500ms 后自动调用 start_filter + start_localization
```

#### 场景二：等待地图下发后定位（`auto_load_map=false`，默认）

```bash
# 启动定位节点（不指定地图，等待 map_manager 下发）
ros2 launch cartographer_ros dexe_localization.launch.py

# 地图管理模块完成地图配置后，调用此服务触发定位
# 节点自动从 map_manager 参数服务读取 map_folder + map_name，完成换图+定位
ros2 service call /localization_node/set_map_changed std_srvs/srv/Trigger
```

#### 场景三：运行中切换地图

```bash
# map_manager 更新地图参数后，触发换图
# 定位节点自动停止当前定位 → 读取新地图路径 → 重新启动定位
ros2 service call /localization_node/set_map_changed std_srvs/srv/Trigger
```

#### 场景四：手动重定位（定位漂移时）

```bash
# 方式一：RViz 中点击 "2D Pose Estimate"，发布到 /initialpose 话题（自动触发重定位）

# 方式二：调用服务指定精确位姿
ros2 service call /localization_node/relocalization nav_interfaces/srv/SetPose \
  "{pose: {header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}}"

# 方式三：重置定位（在当前地图上重新初始化）
ros2 service call /localization_node/reset_localization std_srvs/srv/Trigger
```

---

---

## 三、地图管理接口（map_manager）

### 节点信息

| 项目 | 内容 |
|------|------|
| 节点名 | `map_manager` |
| 包名 | `map_manager` |
| 配置文件 | `config/config.yaml` |
| 地图根目录 | `~/.dexe/data/maps`（可在配置文件修改） |

`map_manager` 是地图的统一管理入口，负责：地图列表扫描、地图加载与切换、向各子系统（定位、导航、任务管理）广播地图变更事件。

### 3.1 输出话题

发布均采用 **`transient_local`（latched）** QoS，新订阅者上线时会立即收到最新值。

| 功能 | 话题名 | 类型 | 描述 |
|------|--------|------|------|
| 当前激活地图 | `map` | `nav_msgs/msg/OccupancyGrid` | 加载/切换地图后发布完整栅格数据 |
| 地图元数据 | `map_info` | `nav_msgs/msg/MapMetaData` | 分辨率、尺寸、原点等 |
| 当前地图根目录 | `map_manager/map_folder` | `std_msgs/msg/String` | 地图文件所在根目录路径 |
| 当前地图名称 | `map_manager/map_name` | `std_msgs/msg/String` | 当前激活地图的名称 |

### 3.2 服务接口

| 功能 | 服务名称 | 类型 | 请求字段 | 响应字段 | 使用示例 |
|------|----------|------|----------|----------|----------|
| 获取地图列表 | `map_manager/get_map_list` | `nav_interfaces/srv/GetMapList` | 无 | `bool success` `string[] map_list` | `ros2 service call /map_manager/get_map_list nav_interfaces/srv/GetMapList` |
| 加载/切换地图 | `map_manager/load_map` | `nav_interfaces/srv/LoadMap` | `string map_name` | `bool success` | `ros2 service call /map_manager/load_map nav_interfaces/srv/LoadMap "{map_name: 'office_1'}"` |
| 删除地图 | `map_manager/remove_map` | `nav_interfaces/srv/DeleteMap` | `string map_name` | `bool success` | `ros2 service call /map_manager/remove_map nav_interfaces/srv/DeleteMap "{map_name: 'old_map'}"` |
| 查询当前地图信息 | `map_manager/get_current_map_info` | `nav_interfaces/srv/GetMapInfo` | 无 | `bool success` `string map_name` `string map_folder` `string error_msg` `MapMetaData map_info` | `ros2 service call /map_manager/get_current_map_info nav_interfaces/srv/GetMapInfo` |
| 获取当前地图根目录 | `map_manager/get_map_folder` | `nav_interfaces/srv/GetString` | 无 | `bool success` `string data` | `ros2 service call /map_manager/get_map_folder nav_interfaces/srv/GetString` |
| 获取当前地图名称 | `map_manager/get_map_name` | `nav_interfaces/srv/GetString` | 无 | `bool success` `string data` | `ros2 service call /map_manager/get_map_name nav_interfaces/srv/GetString` |
| 获取完整地图数据 | `map_manager/get_current_map_data` | `nav_msgs/srv/GetMap` | 无 | `OccupancyGrid map` | `ros2 service call /map_manager/get_current_map_data nav_msgs/srv/GetMap` |

### 3.3 ROS2 参数

| 参数名 | 类型 | 说明 |
|--------|------|------|
| `map_folder` | `string` | 当前地图根目录（启动时从 config.yaml 写入，`load_map` 后不变） |
| `map_name` | `string` | 当前激活地图名称，每次 `load_map` 成功后动态更新 |

> `localization_node` 的 `set_map_changed` 服务正是通过读取 `map_manager` 节点的这两个参数来获取新地图路径。

### 3.4 配置文件（`config/config.yaml`）

```yaml
# 地图文件根目录
map_folder: "~/.dexe/data/maps"

# 启动时自动加载的默认地图名称
default_map_name: "map_2_standard"

# true：启动时自动加载 default_map_name，并在每次 load_map 成功后
#       把新地图名写回此配置文件（下次启动记住上次使用的地图）
# false：每次启动固定加载 default_map_name，不自动保存
use_default_map: true

# 地图变更时调用的各子系统通知服务
services:
  localization: "/localization_node/set_map_changed"   # 定位换图
  move_base:    "/move_base/set_map_changed"            # 导航换图
  # nav_point_manager 和 mission_manager 当前已注释，后续按需开启
```

### 3.5 地图切换完整流程

调用 `map_manager/load_map` 后，节点内部按以下顺序执行：

```
load_map(map_name)
    ├─ MapManager::LoadMap()          # 从磁盘读取 map.png + map.yaml
    ├─ set_parameter(map_name)        # 更新 ROS2 参数
    ├─ 等待 500ms                     # 让话题订阅者处理完毕
    ├─ 发布话题（transient_local）
    │     ├─ map          → OccupancyGrid
    │     ├─ map_info     → MapMetaData
    │     ├─ map_manager/map_folder → String
    │     └─ map_manager/map_name   → String
    └─ ActivateCurrentMap()           # 调用服务通知各子系统
          ├─ /localization_node/set_map_changed  （必须成功）
          └─ /move_base/set_map_changed           （可选，失败只告警）
```

### 3.6 典型使用示例

```bash
# 查看当前所有可用地图
ros2 service call /map_manager/get_map_list nav_interfaces/srv/GetMapList

# 切换到指定地图（自动触发定位和导航的地图更新）
ros2 service call /map_manager/load_map nav_interfaces/srv/LoadMap \
  "{map_name: 'office_1'}"

# 查询当前激活地图信息（名称/目录/分辨率/尺寸）
ros2 service call /map_manager/get_current_map_info nav_interfaces/srv/GetMapInfo

# 删除旧地图
ros2 service call /map_manager/remove_map nav_interfaces/srv/DeleteMap \
  "{map_name: 'old_test_map'}"
```

---

## 四、系统联动说明

### 4.1 地图下发后自动触发定位（标准流程）

```
外部系统（App / Web）
    │
    ▼
map_manager/load_map(map_name)
    │
    ├─ [1] 发布 map 话题（transient_local）
    │       导航、可视化等订阅方收到新地图数据
    │
    └─ [2] 调用 /localization_node/set_map_changed
                │
                ├─ 从 map_manager 参数服务读取 map_folder + map_name
                ├─ 停止当前定位
                ├─ 加载新地图（map.pbstream）
                └─ 启动新定位 + 启动 UKF 滤波器
```

### 4.2 重定位触发方式对比

| 方式 | 触发方法 | 适用场景 |
|------|----------|----------|
| 自动重定位（换图后） | `map_manager/load_map` → 内部调用 `set_map_changed` | 地图切换时自动执行 |
| 手动重定位（服务）| `/localization_node/relocalization`（指定位姿） | 已知精确位置，强制重定位 |
| 手动重定位（话题）| 发布到 `initialpose` 话题 | RViz 点击 "2D Pose Estimate" |
| 软重置 | `/localization_node/reset_localization` | 定位漂移，保持当前地图重新初始化 |

---

## 五、Launch 启动参数

### 建图 Launch：`dexe_slam_2d.launch.py`

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `map_folder` | `~/.dexe/data/maps` | 地图保存根目录 |
| `map_name` | `dex_map_test` | 地图名称（子目录名） |
| `resolution` | `0.04` | 栅格分辨率（m/格） |
| `config_file` | `dexe_slam_2d.lua` | Lua 配置文件名 |
| `use_sim_time` | `false` | 是否使用仿真时钟 |

### 定位 Launch：`dexe_localization.launch.py`

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `map_folder` | `~/.dexe/data/maps` | 地图根目录 |
| `map_name` | `offline_map_2` | 地图名称 |
| `resolution` | `0.04` | 栅格分辨率（m/格） |
| `auto_save_interval_s` | `1800.0` | 自动保存间隔（秒，机器人静止时触发） |
| `auto_save_linear_velocity_threshold` | `0.01` | 触发自动保存的线速度阈值（m/s） |
| `auto_save_angular_velocity_threshold` | `0.01` | 触发自动保存的角速度阈值（rad/s） |

> **`auto_load_map` 参数**：通过 gflag 传入 localization_node，默认 `false`。  
> `false`：节点启动后不加载地图，等待 `set_map_changed` 服务触发。  
> `true`：启动后自动加载 `map_folder/map_name` 指定的地图。

---

## 六、地图文件结构

```
~/.dexe/data/maps/
└── <map_name>/
    ├── map.pbstream   # Cartographer 内部状态（子图、轨迹、约束）
    ├── map.png        # 栅格地图图像（黑白，0.04m/px）
    └── map.yaml       # 地图元信息（origin、resolution、图像路径）
```
