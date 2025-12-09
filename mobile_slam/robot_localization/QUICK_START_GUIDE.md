# Robot Localization 快速上手指南

## 目录
1. [工程概述](#工程概述)
2. [核心架构](#核心架构)
3. [数据流Pipeline](#数据流pipeline)
4. [关键组件详解](#关键组件详解)
5. [参数配置](#参数配置)
6. [使用示例](#使用示例)
7. [常见问题](#常见问题)

---

## 工程概述

`robot_localization` 是一个ROS2状态估计包，提供非线性传感器融合功能。它可以融合来自多个传感器（IMU、里程计、GPS等）的数据，输出高精度的机器人位姿估计。

### 核心特性
- ✅ **多传感器融合**：支持IMU、轮速里程计、视觉里程计、GPS等
- ✅ **两种滤波器**：EKF（扩展卡尔曼滤波）和UKF（无迹卡尔曼滤波）
- ✅ **GPS转换**：通过`navsat_transform`节点将GPS数据转换为本地坐标系
- ✅ **3D/2D模式**：支持3D全姿态估计或2D平面运动
- ✅ **TF变换广播**：自动发布odom→base_link或map→odom变换

---

## 核心架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    Robot Localization 架构                       │
└─────────────────────────────────────────────────────────────────┘

         传感器输入层
    ┌──────────────────┐
    │  sensor_msgs/Imu │
    │  nav_msgs/Odom   │
    │  geometry_msgs/* │
    │  NavSatFix       │
    └────────┬─────────┘
             │
             ▼
    ┌────────────────────┐
    │  ROS Filter 节点    │
    │  (EKF/UKF Node)    │
    │                    │
    │  ┌──────────────┐  │
    │  │ RosFilter<T> │  │  ← 模板类，T为EKF或UKF
    │  └──────┬───────┘  │
    │         │          │
    │  ┌──────▼───────┐  │
    │  │ FilterBase   │  │  ← 滤波器基类
    │  │  (EKF/UKF)   │  │
    │  └──────────────┘  │
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │   输出 & 服务       │
    │                    │
    │  • /odometry/      │
    │    filtered        │
    │  • /accel/         │
    │    filtered        │
    │  • TF: odom→       │
    │    base_link       │
    │  • Services:       │
    │    - SetPose       │
    │    - GetState      │
    │    - Toggle        │
    └────────────────────┘

         GPS处理分支
    ┌──────────────────┐
    │  NavSatFix (GPS) │
    └────────┬─────────┘
             │
             ▼
    ┌────────────────────┐
    │ NavSatTransform    │
    │  (GPS→Odom转换)    │
    │                    │
    │  • 经纬度→XYZ      │
    │  • UTM投影         │
    │  • 坐标系对齐      │
    └────────┬───────────┘
             │
             ▼
    ┌────────────────────┐
    │  /odometry/gps     │
    │  (本地坐标系里程计) │
    └────────────────────┘
```

---

## 数据流Pipeline

### 1. 单滤波器工作流（Local Odometry）

```
传感器数据输入
    │
    ├─ IMU (/imu/data) ──────────┐
    │   [加速度 + 角速度 + 姿态]    │
    │                            │
    ├─ 轮速里程计 (/odom) ─────────┤
    │   [位置 + 速度]              │
    │                            │
    └─ 视觉里程计 (/vo) ──────────┤
        [位置 + 姿态]             │
                                │
                                ▼
                    ┌───────────────────────┐
                    │  EKF/UKF Filter Node  │
                    │                       │
                    │  1. 预测（Predict）    │
                    │     - 状态外推         │
                    │     - 协方差更新       │
                    │                       │
                    │  2. 校正（Correct）    │
                    │     - 测量融合         │
                    │     - 卡尔曼增益计算   │
                    │     - 状态更新         │
                    └───────────┬───────────┘
                                │
                                ▼
                    ┌───────────────────────┐
                    │  输出状态估计          │
                    │                       │
                    │  • 位置 (x,y,z)       │
                    │  • 姿态 (r,p,y)       │
                    │  • 速度 (vx,vy,vz)    │
                    │  • 角速度 (wr,wp,wy)  │
                    │  • 加速度 (ax,ay,az)  │
                    └───────────┬───────────┘
                                │
                                ├──► /odometry/filtered
                                ├──► TF: odom → base_link
                                └──► /accel/filtered
```

### 2. 双EKF+GPS工作流（Global Localization）

这是最常用的配置，用于融合GPS进行全局定位：

```
┌─────────────────────────────────────────────────────────────┐
│  阶段1: 本地里程计 (Local EKF)                                │
└─────────────────────────────────────────────────────────────┘

IMU + 轮速里程计
    │
    ▼
┌─────────────┐
│   EKF #1    │  world_frame: odom
│  (Local)    │  输出: odom → base_link
└──────┬──────┘
       │
       ├──► /odometry/local
       └──► TF: odom → base_link

┌─────────────────────────────────────────────────────────────┐
│  阶段2: GPS坐标转换 (NavSatTransform)                         │
└─────────────────────────────────────────────────────────────┘

GPS (经纬度) + Local Odom + IMU
    │
    ▼
┌──────────────────┐
│ NavSatTransform  │
│                  │
│ 1. GPS首次定位    │
│    - 设置原点     │
│    - 初始化变换   │
│                  │
│ 2. 持续转换       │
│    - 经纬度→UTM   │
│    - UTM→本地XYZ  │
│    - 姿态对齐     │
└────────┬─────────┘
         │
         └──► /odometry/gps (本地坐标系)

┌─────────────────────────────────────────────────────────────┐
│  阶段3: 全局定位 (Global EKF)                                 │
└─────────────────────────────────────────────────────────────┘

/odometry/gps + IMU
    │
    ▼
┌─────────────┐
│   EKF #2    │  world_frame: map
│  (Global)   │  输出: map → odom
└──────┬──────┘
       │
       ├──► /odometry/global
       └──► TF: map → odom

┌─────────────────────────────────────────────────────────────┐
│  最终TF树                                                     │
└─────────────────────────────────────────────────────────────┘

map → odom → base_link → [sensor frames]
 ↑      ↑
 │      └─ EKF #1 (连续、平滑、局部精确)
 │
 └─ EKF #2 (全局一致、可能跳变)
```

---

## 关键组件详解

### 1. FilterBase（滤波器基类）

**位置**: `include/robot_localization/filter_base.hpp`

**核心职责**:
- 维护状态向量（15维）：`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`
- 维护协方差矩阵（15×15）
- 提供predict和correct抽象接口

**状态向量说明**:
```cpp
enum StateMembers {
  StateMemberX = 0,      // 位置X
  StateMemberY,          // 位置Y
  StateMemberZ,          // 位置Z
  StateMemberRoll,       // 滚转角
  StateMemberPitch,      // 俯仰角
  StateMemberYaw,        // 偏航角
  StateMemberVx,         // X方向速度
  StateMemberVy,         // Y方向速度
  StateMemberVz,         // Z方向速度
  StateMemberVroll,      // 滚转角速度
  StateMemberVpitch,     // 俯仰角速度
  StateMemberVyaw,       // 偏航角速度
  StateMemberAx,         // X方向加速度
  StateMemberAy,         // Y方向加速度
  StateMemberAz          // Z方向加速度
};
```

### 2. RosFilter（ROS接口层）

**位置**: `include/robot_localization/ros_filter.hpp`

**核心职责**:
- 订阅传感器话题（IMU、Odom、Pose、Twist等）
- 管理测量队列和历史记录
- 调用FilterBase的predict/correct
- 发布滤波后的状态和TF变换
- 提供服务接口（SetPose、GetState等）

**关键成员函数**:
```cpp
// 传感器回调
void imuCallback(sensor_msgs::msg::Imu::SharedPtr);
void odomCallback(nav_msgs::msg::Odometry::SharedPtr);
void poseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr);

// 核心处理循环
void run();  // 主循环，处理测量队列
void integrateMeasurements(rclcpp::Time current_time);

// 输出发布
void publishTransform();
void publishOdometry();
```

### 3. EKF vs UKF

| 特性 | EKF | UKF |
|------|-----|-----|
| **算法** | 扩展卡尔曼滤波 | 无迹卡尔曼滤波 |
| **线性化** | 一阶泰勒展开 | Sigma点采样 |
| **精度** | 中等（非线性系统有误差） | 高（更好处理非线性） |
| **计算量** | 较低 | 较高 |
| **使用场景** | 一般机器人导航 | 高精度需求、强非线性 |
| **实现文件** | `ekf.cpp` | `ukf.cpp` |

**推荐**: 大多数情况下EKF已足够，除非有强非线性运动（如高速飞行器、水下机器人）。

### 4. NavSatTransform（GPS转换）

**位置**: `include/robot_localization/navsat_transform.hpp`

**核心职责**:
1. **坐标转换**: GPS经纬度 → UTM坐标 → 本地笛卡尔坐标
2. **原点设置**: 第一个有效GPS点或手动设置的基准点
3. **姿态对齐**: 使用IMU或里程计姿态
4. **变换发布**: utm → odom_frame 的静态变换

**工作流程**:
```
步骤1: 等待首个GPS信号
  ├─ 订阅 /gps/fix
  ├─ 订阅 /odometry/local
  └─ 订阅 /imu/data

步骤2: 初始化基准点（Datum）
  ├─ 自动: 使用首个GPS点
  └─ 手动: 通过SetDatum服务

步骤3: 持续转换
  对每个GPS测量:
    1. 经纬度(lat,lon,alt) → UTM(x,y,z)
    2. 查询 base_link → gps_frame 变换
    3. 计算机器人中心在odom系的位置
    4. 发布 /odometry/gps
```

---

## 参数配置

### 核心参数分类

#### 1. 坐标系配置
```yaml
ekf_filter_node:
  ros__parameters:
    map_frame: map            # 全局坐标系
    odom_frame: odom          # 里程计坐标系
    base_link_frame: base_link  # 机器人本体坐标系
    world_frame: odom         # 滤波器输出的参考系
    
    # world_frame选择指南:
    # - 本地里程计: world_frame = odom
    # - 全局定位:   world_frame = map
```

#### 2. 传感器配置
每个传感器需要配置：
- **话题名**: `odom0`, `imu0`, `pose0` 等
- **更新向量**: 15维布尔数组，指定哪些状态分量被更新
- **其他选项**: 队列大小、差分模式、相对模式等

**示例 - IMU配置**:
```yaml
imu0: /imu/data
imu0_config: [false, false, false,   # 位置（不使用）
              true,  true,  true,    # 姿态（使用roll,pitch,yaw）
              false, false, false,   # 线速度（不使用）
              true,  true,  true,    # 角速度（使用）
              true,  true,  true]    # 线加速度（使用）
imu0_differential: false
imu0_relative: true
imu0_queue_size: 10
imu0_remove_gravitational_acceleration: true
```

**示例 - 里程计配置**:
```yaml
odom0: /odom
odom0_config: [true,  true,  false,  # 位置（使用x,y，不用z）
               false, false, true,   # 姿态（只用yaw）
               true,  true,  false,  # 速度（使用vx,vy）
               false, false, true,   # 角速度（只用vyaw）
               false, false, false]  # 加速度（不使用）
odom0_differential: false
odom0_relative: false
```

#### 3. 过程噪声协方差
```yaml
process_noise_covariance: [
  0.05, 0.0,  0.0,  ... # 15×15矩阵对角线
]

# 或使用动态过程噪声（推荐）
use_dynamic_process_noise_covariance: true
```

#### 4. 初始协方差
```yaml
initial_estimate_covariance: [
  1e-9, 0.0,  0.0,  ... # 15×15矩阵
]
```

---

## 使用示例

### 场景1: 室内机器人（IMU + 轮速里程计）

**配置文件**: `params/indoor_robot.yaml`
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true  # 2D平面运动
    
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    
    # 轮速里程计
    odom0: /wheel/odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    
    # IMU
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,   # 只用yaw
                  false, false, false,
                  false, false, true,   # 只用vyaw
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
```

**启动命令**:
```bash
ros2 launch robot_localization ekf.launch.py \
  params_file:=/path/to/indoor_robot.yaml
```

### 场景2: 户外机器人（双EKF + GPS）

**架构**:
```
Local EKF → odom → base_link (连续、平滑)
Global EKF → map → odom (GPS校正)
NavSatTransform → GPS → Local Odom
```

**配置文件1**: `params/local_ekf.yaml`
```yaml
ekf_local_node:
  ros__parameters:
    frequency: 50.0
    world_frame: odom
    
    odom0: /wheel/odom
    odom0_config: [true, true, false, ...]
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true, ...]
```

**配置文件2**: `params/global_ekf.yaml`
```yaml
ekf_global_node:
  ros__parameters:
    frequency: 10.0  # GPS频率较低
    world_frame: map
    
    odom0: /odometry/gps  # 来自NavSatTransform
    odom0_config: [true, true, true, ...]
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true, ...]
```

**配置文件3**: `params/navsat.yaml`
```yaml
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    magnetic_declination_radians: 0.0
    yaw_offset: 0.0
    
    broadcast_utm_transform: true
    publish_filtered_gps: true
```

**启动文件**: `launch/dual_ekf_gps.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Local EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            parameters=['params/local_ekf.yaml'],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        
        # NavSat Transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=['params/navsat.yaml'],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/local'),
                ('odometry/gps', 'odometry/gps')
            ]
        ),
        
        # Global EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            parameters=['params/global_ekf.yaml'],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
    ])
```

### 场景3: 飞行器（高精度，使用UKF）

```yaml
ukf_filter_node:
  ros__parameters:
    frequency: 100.0  # 高频更新
    two_d_mode: false  # 3D运动
    
    # 使用所有6DOF
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
```

---

## 常见问题

### Q1: TF树关系混乱怎么办？

**检查步骤**:
```bash
# 1. 查看TF树
ros2 run tf2_tools view_frames

# 2. 检查是否有多个节点发布相同变换
ros2 run tf2_ros tf2_echo odom base_link

# 3. 确保只有一个节点发布odom→base_link
# Local EKF: publish_tf=true, world_frame=odom
# Global EKF: publish_tf=true, world_frame=map
```

### Q2: 滤波输出发散怎么办？

**诊断**:
```bash
# 开启诊断模式
print_diagnostics: true

# 查看协方差
ros2 topic echo /odometry/filtered --field pose.covariance
```

**常见原因**:
- 传感器配置冲突（多个传感器更新同一状态）
- 过程噪声太大
- 初始协方差设置不当
- 传感器数据质量差

**解决方案**:
```yaml
# 1. 减小过程噪声
process_noise_covariance: [0.05, ...] → [0.01, ...]

# 2. 增大初始协方差
initial_estimate_covariance: [1e-9, ...] → [1.0, ...]

# 3. 调整传感器配置，避免冲突
```

### Q3: GPS漂移严重怎么办？

**检查**:
```bash
# 查看GPS质量
ros2 topic echo /gps/fix

# 检查协方差
position_covariance: [x, 0, 0, 0, y, 0, 0, 0, z]
# 如果x,y,z > 10，说明精度很差
```

**改进方法**:
```yaml
navsat_transform_node:
  ros__parameters:
    # 1. 增加等待时间，确保首个GPS质量好
    wait_for_datum: true
    
    # 2. 使用磁偏角校正
    magnetic_declination_radians: 0.2  # 根据地区设置
    
    # 3. 在Global EKF中降低GPS权重
    odom0_config: [true, true, false, ...]  # 不使用z
```

### Q4: 如何调试滤波器？

**调试模式**:
```yaml
debug: true
debug_out_file: /tmp/robot_localization_debug.txt
```

**输出内容**:
- 预测状态和协方差矩阵
- 测量更新详情
- 卡尔曼增益
- 异常检测结果

**可视化**:
```bash
# 安装PlotJuggler
sudo apt install ros-humble-plotjuggler-ros

# 启动可视化
ros2 run plotjuggler plotjuggler

# 订阅话题:
# - /odometry/filtered
# - /diagnostics
```

### Q5: 如何初始化滤波器位姿？

**方法1: 服务调用**
```bash
ros2 service call /set_pose robot_localization/srv/SetPose \
  "{pose: {pose: {position: {x: 1.0, y: 2.0, z: 0.0}, \
                   orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
           covariance: [0.1, 0, 0, 0, 0, 0, \
                        0, 0.1, 0, 0, 0, 0, \
                        0, 0, 0.1, 0, 0, 0, \
                        0, 0, 0, 0.1, 0, 0, \
                        0, 0, 0, 0, 0.1, 0, \
                        0, 0, 0, 0, 0, 0.1]}}"
```

**方法2: 初始位姿话题**
```yaml
# 在配置文件中订阅initial_pose
pose0: /initialpose
pose0_config: [true, true, true, 
               true, true, true,
               false, false, false,
               false, false, false,
               false, false, false]
```

### Q6: 2D模式和3D模式如何选择？

| 场景 | 模式 | 说明 |
|------|------|------|
| 地面机器人（平坦） | 2D | z, roll, pitch固定为0 |
| 地面机器人（崎岖） | 3D | 需要IMU提供roll/pitch |
| 无人机 | 3D | 必须6DOF估计 |
| 水面船 | 2D或3D | 看波浪大小 |

```yaml
two_d_mode: true  # 会自动将z, roll, pitch, vz, vroll, vpitch清零
```

---

## 服务接口说明

### 1. SetPose - 设置位姿
```bash
ros2 service call /set_pose robot_localization/srv/SetPose ...
```

### 2. SetDatum - 设置GPS基准点（NavSatTransform）
```bash
ros2 service call /datum robot_localization/srv/SetDatum \
  "{geo_pose: {position: {latitude: 40.7128, longitude: -74.0060, altitude: 10.0}}}"
```

### 3. ToggleFilterProcessing - 暂停/恢复滤波
```bash
ros2 service call /toggle robot_localization/srv/ToggleFilterProcessing \
  "{on: false}"  # 暂停
```

### 4. GetState - 获取当前状态
```bash
ros2 service call /get_state robot_localization/srv/GetState
```

### 5. FromLL / ToLL - 经纬度转换
```bash
ros2 service call /fromLL robot_localization/srv/FromLL \
  "{ll_point: {latitude: 40.7128, longitude: -74.0060, altitude: 10.0}}"
```

---

## 性能调优

### 1. 频率设置
```yaml
frequency: 50.0  # 权衡精度和CPU

# 建议值:
# - 室内低速: 30 Hz
# - 室外中速: 50 Hz
# - 高速/飞行: 100+ Hz
```

### 2. 队列大小
```yaml
odom0_queue_size: 10  # 增大可减少丢包，但占内存

# 高频传感器 → 增大队列
# 低频传感器 → 默认即可
```

### 3. 超时设置
```yaml
sensor_timeout: 0.1  # 秒

# 如果传感器频率低，需增大此值
# 否则滤波器会认为传感器失效
```

---

## 总结

### 快速启动检查清单

- [ ] **确定坐标系**: base_link, odom, map的定义
- [ ] **选择滤波器**: EKF（通用）或UKF（高精度）
- [ ] **配置传感器**: 确定哪些传感器、哪些数据维度
- [ ] **设置world_frame**: odom（本地）或map（全局）
- [ ] **调整频率**: 根据传感器和应用场景
- [ ] **测试TF树**: 确保无环路、无冲突
- [ ] **监控诊断**: 查看协方差是否收敛

### 学习路径

1. **入门**: 单EKF + 单传感器（如IMU或Odom）
2. **进阶**: 多传感器融合（IMU + Odom + VO）
3. **高级**: 双EKF + GPS全局定位
4. **专家**: 自定义传感器、动态重配置、故障检测

### 参考资源

- [官方文档](http://docs.ros.org/en/humble/p/robot_localization/)
- [ROS Wiki](http://wiki.ros.org/robot_localization)
- [配置教程](https://kapernikov.com/the-ros-robot_localization-package/)
- [源码仓库](https://github.com/cra-ros-pkg/robot_localization)

---

**祝你使用愉快！有问题随时查阅本文档 😊**
