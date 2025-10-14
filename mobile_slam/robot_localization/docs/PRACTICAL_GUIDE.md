# Robot Localization 实践教程

## 目录
- [从零开始配置](#从零开始配置)
- [常见配置场景](#常见配置场景)
- [调试技巧](#调试技巧)
- [性能优化](#性能优化)
- [故障排查](#故障排查)

---

## 从零开始配置

### 步骤1: 确定硬件传感器

首先列出你的机器人有哪些传感器：

```bash
# 查看所有话题
ros2 topic list

# 常见传感器话题:
# /imu/data              (sensor_msgs/Imu)
# /odom                  (nav_msgs/Odometry)
# /visual_odom           (nav_msgs/Odometry)
# /gps/fix               (sensor_msgs/NavSatFix)
```

### 步骤2: 创建基础配置文件

```bash
cd ~/your_ws/src/robot_localization/params
cp ekf.yaml my_robot_ekf.yaml
```

### 步骤3: 配置坐标系

编辑 `my_robot_ekf.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    # 1. 确定坐标系名称
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    
    # 2. 选择world_frame
    # 本地里程计 → odom
    # 全局定位 → map
    world_frame: odom
    
    # 3. 运动模式
    two_d_mode: true  # 地面机器人用true，飞行器用false
```

### 步骤4: 添加第一个传感器（IMU）

```yaml
    # IMU传感器
    imu0: /imu/data
    
    # IMU提供什么数据？
    # 通常：姿态(roll,pitch,yaw) + 角速度 + 线加速度
    imu0_config: [
        false, false, false,  # 位置 (x,y,z) - IMU不提供
        true,  true,  true,   # 姿态 (roll,pitch,yaw) - 使用
        false, false, false,  # 线速度 - IMU不提供
        true,  true,  true,   # 角速度 - 使用
        true,  true,  true    # 线加速度 - 使用
    ]
    
    # IMU设置
    imu0_differential: false         # 姿态是绝对值
    imu0_relative: true              # 相对于首个测量
    imu0_queue_size: 10
    imu0_remove_gravitational_acceleration: true  # 移除重力
```

### 步骤5: 添加第二个传感器（Odometry）

```yaml
    # 轮速里程计
    odom0: /odom
    
    # 轮速里程计提供什么？
    # 通常：位置(x,y) + yaw + 速度(vx,vy,vyaw)
    odom0_config: [
        true,  true,  false,  # 位置 (x,y,z) - 使用x,y
        false, false, true,   # 姿态 (roll,pitch,yaw) - 只用yaw
        true,  true,  false,  # 线速度 - 使用vx,vy
        false, false, true,   # 角速度 - 使用vyaw
        false, false, false   # 加速度 - 不使用
    ]
    
    # Odom设置
    odom0_differential: false  # 绝对位置
    odom0_relative: false      # 绝对坐标系
    odom0_queue_size: 10
```

### 步骤6: 调整频率和超时

```yaml
    frequency: 50.0           # 50Hz输出
    sensor_timeout: 0.1       # 100ms超时
    transform_time_offset: 0.0
    transform_timeout: 0.0
```

### 步骤7: 设置协方差（关键！）

#### 方法1: 使用默认值（推荐初学者）
```yaml
    # 过程噪声 - 描述模型不确定性
    process_noise_covariance: [
        0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025, 0.0, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015
    ]
    
    # 初始协方差 - 描述初始不确定性
    initial_estimate_covariance: [
        1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,  0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9, 0.0,
        0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  1e-9
    ]
```

#### 方法2: 根据传感器规格设置（进阶）

查看传感器手册，找到噪声参数：

```yaml
# 示例：高精度IMU
imu0_config: [...]

# 如果IMU手册说：
# 角速度噪声: 0.01 rad/s
# 加速度噪声: 0.05 m/s²
# 姿态精度: 0.5°

# 则在过程噪声中设置:
# vroll/vpitch/vyaw (索引9,10,11): 0.01² = 0.0001
# ax/ay/az (索引12,13,14): 0.05² = 0.0025
# roll/pitch/yaw (索引3,4,5): (0.5°)² = (0.0087 rad)² = 0.000076
```

### 步骤8: 创建Launch文件

创建 `launch/my_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('robot_localization'),
        'params',
        'my_robot_ekf.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
```

### 步骤9: 启动并验证

```bash
# 终端1: 启动机器人驱动（假设已有）
ros2 launch my_robot robot.launch.py

# 终端2: 启动robot_localization
ros2 launch robot_localization my_robot.launch.py

# 终端3: 查看输出
ros2 topic echo /odometry/filtered

# 终端4: 查看TF树
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

## 常见配置场景

### 场景A: 最简单配置（仅IMU）

**适用**: 测试、学习、IMU校准

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 100.0
    two_d_mode: false
    world_frame: odom
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    imu0_remove_gravitational_acceleration: true
```

### 场景B: 地面机器人（IMU + 轮速）

**适用**: 室内AGV、移动机器人

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true  # 2D模式
    world_frame: odom
    
    # 轮速里程计 - 提供位置和速度
    odom0: /odom
    odom0_config: [true,  true,  false,  # x, y
                   false, false, true,   # yaw
                   true,  true,  false,  # vx, vy
                   false, false, true,   # vyaw
                   false, false, false]
    odom0_differential: false
    
    # IMU - 提供姿态和角速度
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,   # 只用yaw (2D模式)
                  false, false, false,
                  false, false, true,   # vyaw
                  false, false, false]  # 不用加速度（有轮速更准）
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

### 场景C: 户外机器人（双EKF + GPS）

**架构**: Local EKF → odom→base_link, Global EKF → map→odom

#### 配置1: Local EKF (`local_ekf.yaml`)
```yaml
ekf_local_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: true
    world_frame: odom  # 输出odom→base_link
    
    odom0: /odom
    odom0_config: [true, true, false,
                   false, false, true,
                   true, true, false,
                   false, false, true,
                   false, false, false]
    
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  true, true, true]
    imu0_remove_gravitational_acceleration: true
```

#### 配置2: NavSat Transform (`navsat.yaml`)
```yaml
navsat_transform_node:
  ros__parameters:
    frequency: 10.0
    delay: 3.0  # 等待3秒确保系统稳定
    
    magnetic_declination_radians: 0.0  # 磁偏角，根据地区设置
    yaw_offset: 0.0
    zero_altitude: false
    
    broadcast_utm_transform: true
    publish_filtered_gps: true
    use_odometry_yaw: false  # 使用IMU的yaw
    wait_for_datum: false
```

#### 配置3: Global EKF (`global_ekf.yaml`)
```yaml
ekf_global_node:
  ros__parameters:
    frequency: 10.0  # GPS频率低，不需要太高
    two_d_mode: false  # GPS提供3D位置
    world_frame: map  # 输出map→odom
    
    # 来自NavSatTransform的GPS里程计
    odom0: /odometry/gps
    odom0_config: [true, true, true,    # x, y, z
                   false, false, false, # 不用姿态
                   false, false, false, # 不用速度
                   false, false, false,
                   false, false, false]
    
    # IMU提供姿态
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  false, false, false]
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
```

#### Launch文件 (`dual_ekf_gps.launch.py`)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization')
    
    return LaunchDescription([
        # Local EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'params', 'local_ekf.yaml')],
            remappings=[
                ('odometry/filtered', 'odometry/local'),
            ],
        ),
        
        # NavSat Transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'params', 'navsat.yaml')],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/local'),
            ],
        ),
        
        # Global EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'params', 'global_ekf.yaml')],
            remappings=[
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ])
```

### 场景D: 视觉SLAM + IMU融合

**适用**: 视觉里程计（ORB-SLAM、VINS等）+ IMU

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    two_d_mode: false  # 视觉SLAM通常是3D
    world_frame: odom
    
    # 视觉里程计
    odom0: /visual_slam/odom
    odom0_config: [true,  true,  true,   # 位置
                   true,  true,  true,   # 姿态
                   false, false, false,  # 速度（可能不准）
                   false, false, false,
                   false, false, false]
    odom0_differential: false
    odom0_relative: false
    
    # IMU
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,   # 姿态（辅助视觉）
                  false, false, false,
                  true,  true,  true,   # 角速度
                  true,  true,  true]   # 加速度
    imu0_differential: false
    imu0_remove_gravitational_acceleration: true
    
    # 平滑设置（视觉可能有跳变）
    smooth_lagged_data: true
    history_length: 5.0
```

---

## 调试技巧

### 技巧1: 开启诊断模式

```yaml
print_diagnostics: true

# 查看诊断信息
ros2 topic echo /diagnostics
```

**诊断输出示例**:
```
- message: "Filter is running normally"
  level: 0  # OK

- message: "Sensor timeout for odom0"
  level: 1  # WARN

- message: "State covariance is diverging"
  level: 2  # ERROR
```

### 技巧2: 使用Debug模式

```yaml
debug: true
debug_out_file: /tmp/robot_localization_debug.txt

# 启动后查看
tail -f /tmp/robot_localization_debug.txt
```

**Debug输出包含**:
- 每次predict的状态变化
- 每次correct的测量残差
- 卡尔曼增益矩阵
- 协方差矩阵

### 技巧3: 可视化协方差

```bash
# 安装PlotJuggler
sudo apt install ros-humble-plotjuggler-ros

# 启动
ros2 run plotjuggler plotjuggler

# 订阅 /odometry/filtered
# 绘制 pose.covariance[0], pose.covariance[7], ... (对角元素)
```

**健康的协方差**:
- 初始时较大（1.0左右）
- 逐渐收敛（0.01~0.1）
- 稳定后保持小幅波动

**异常的协方差**:
- 持续增长 → 滤波器发散
- 突然跳变 → 传感器冲突或异常测量
- 始终很大 → 传感器质量差或配置错误

### 技巧4: 检查TF树

```bash
# 生成TF树图
ros2 run tf2_tools view_frames

# 实时监控TF
ros2 run tf2_ros tf2_echo odom base_link
```

**检查项**:
- [ ] 没有循环依赖
- [ ] 每个frame只有一个父节点
- [ ] 时间戳是最新的（<100ms old）

### 技巧5: 逐个传感器测试

```bash
# 步骤1: 只启用IMU
# 配置文件中注释掉所有其他传感器
ros2 launch robot_localization ekf.launch.py

# 观察：姿态是否正常？

# 步骤2: 添加Odom
# 取消odom0的注释
ros2 launch robot_localization ekf.launch.py

# 观察：位置是否随机器人运动？

# 步骤3: 添加其他传感器...
```

---

## 性能优化

### 优化1: 频率调整

```yaml
# 原则：不要超过传感器频率的2倍

# 例子：
# IMU: 100 Hz
# Odom: 50 Hz
# GPS: 10 Hz

# 推荐频率：50 Hz（接近中位数）
frequency: 50.0
```

### 优化2: 队列大小

```yaml
# 高频传感器增大队列
imu0_queue_size: 20  # IMU 100Hz

# 低频传感器使用默认
odom0_queue_size: 5  # Odom 50Hz

# 超低频传感器
gps0_queue_size: 2   # GPS 10Hz
```

### 优化3: 动态过程噪声

```yaml
# 自动根据速度调整过程噪声
use_dynamic_process_noise_covariance: true

# 速度慢时降低噪声（更信任模型）
# 速度快时增大噪声（更信任测量）
```

### 优化4: 选择性更新

```yaml
# 对于低质量传感器，只使用部分维度

# 例如：轮速里程计在打滑时yaw不准
odom0_config: [true,  true,  false,
               false, false, false,  # 不用yaw
               true,  true,  false,
               false, false, false,  # 不用vyaw
               false, false, false]

# 让IMU独占yaw
imu0_config: [false, false, false,
              false, false, true,   # 只用yaw
              ...
```

### 优化5: 相对/差分模式

```yaml
# 相对模式 (relative=true):
# 将首个测量作为原点，后续测量相对于它

# 适用于: 传感器坐标系与world_frame不一致
imu0_relative: true

# 差分模式 (differential=true):
# 只使用测量的变化量，不用绝对值

# 适用于: 传感器有偏移但变化准确
odom0_differential: true
```

---

## 故障排查

### 问题1: 滤波输出不更新

**症状**: `/odometry/filtered` 话题没有消息

**检查**:
```bash
# 1. 节点是否运行？
ros2 node list | grep ekf

# 2. 是否收到传感器数据？
ros2 topic hz /imu/data
ros2 topic hz /odom

# 3. 查看日志
ros2 node info /ekf_filter_node
```

**常见原因**:
- 传感器话题名错误
- 坐标系名称错误
- 没有收到足够的传感器数据初始化

**解决方案**:
```yaml
# 降低初始化门槛
# (至少需要一个传感器提供位置，一个提供姿态)
```

### 问题2: TF: "Lookup would require extrapolation"

**症状**: 
```
Lookup would require extrapolation at time XXX into the future.
```

**原因**: 传感器时间戳比系统时间快

**解决方案**:
```yaml
# 方法1: 使用transform_time_offset
transform_time_offset: 0.1  # 延迟100ms

# 方法2: 检查传感器时间戳
# 确保传感器使用ROS时间而非硬件时间
```

### 问题3: 位置漂移严重

**症状**: 机器人静止时，滤波输出仍在移动

**诊断**:
```bash
# 查看协方差
ros2 topic echo /odometry/filtered --field pose.covariance

# 查看每个传感器
ros2 topic echo /imu/data
ros2 topic echo /odom
```

**常见原因**:
- IMU偏置未校准
- 轮速里程计有偏移
- 过程噪声设置过大

**解决方案**:
```yaml
# 1. 增大传感器权重（减小协方差）
# 在传感器驱动中设置更小的协方差

# 2. 减小过程噪声
process_noise_covariance: [0.01, ...  # 从0.05降到0.01

# 3. 校准IMU
# 使用IMU厂商的校准工具
```

### 问题4: 姿态突然翻转

**症状**: roll/pitch/yaw突然变为相反值

**原因**: 四元数奇异性或万向锁

**解决方案**:
```yaml
# 对于地面机器人，使用2D模式
two_d_mode: true

# 对于3D运动，确保IMU质量好
imu0_config: [false, false, false,
              true, true, true,  # 姿态必须可靠
              ...
```

### 问题5: GPS位置偏移

**症状**: GPS转换后的位置与实际位置偏移几十米

**检查**:
```bash
# 1. 查看基准点
ros2 service call /datum robot_localization/srv/SetDatum

# 2. 检查磁偏角
# 访问 https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
# 获取当地磁偏角
```

**解决方案**:
```yaml
navsat_transform_node:
  ros__parameters:
    # 设置正确的磁偏角（例如北京：-6.5°）
    magnetic_declination_radians: -0.113  # -6.5° in radians
    
    # 手动设置基准点
    datum: [39.9042, 116.4074, 50.0]  # lat, lon, alt
```

### 问题6: 高频抖动

**症状**: 滤波输出频繁小幅抖动

**原因**: 
- 传感器噪声大
- 滤波器对测量过于敏感

**解决方案**:
```yaml
# 方法1: 增大过程噪声（降低测量权重）
process_noise_covariance: [0.1, ...  # 从0.05增到0.1

# 方法2: 使用平滑模式
smooth_lagged_data: true
history_length: 3.0

# 方法3: 降低频率
frequency: 30.0  # 从50降到30
```

---

## 高级技巧

### 技巧1: 自定义测量噪声

```yaml
# 如果传感器消息中的协方差不可靠，手动覆盖：

# 创建自定义节点修改协方差
# 或使用 topic_tools 重新发布
```

```python
# covariance_override_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class CovarianceOverride(Node):
    def __init__(self):
        super().__init__('covariance_override')
        self.sub = self.create_subscription(
            Imu, '/imu/data_raw', self.callback, 10)
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
    
    def callback(self, msg):
        # 覆盖协方差
        msg.orientation_covariance = [0.01, 0, 0, 
                                       0, 0.01, 0, 
                                       0, 0, 0.01]
        msg.angular_velocity_covariance = [0.001] * 9
        msg.linear_acceleration_covariance = [0.01] * 9
        self.pub.publish(msg)
```

### 技巧2: 条件性禁用传感器

```yaml
# 使用服务动态切换传感器

# 例如：GPS信号差时禁用GPS
ros2 service call /toggle_gps std_srvs/srv/SetBool "{data: false}"
```

### 技巧3: 多机器人融合

```yaml
# 为每个机器人使用不同的命名空间
# 例如: /robot1/ekf_filter_node, /robot2/ekf_filter_node

# 在launch文件中：
Node(
    package='robot_localization',
    executable='ekf_node',
    namespace='robot1',
    ...
)
```

---

## 总结

### 配置检查清单

- [ ] 坐标系名称正确且一致
- [ ] world_frame选择合适（odom或map）
- [ ] 传感器话题名正确
- [ ] 更新向量符合传感器特性
- [ ] 协方差设置合理
- [ ] TF树无环路
- [ ] 传感器时间戳正确
- [ ] 初始化成功（查看日志）
- [ ] 输出话题有数据
- [ ] 协方差收敛

### 学习建议

1. **从简单开始**: 单传感器 → 双传感器 → 多传感器
2. **理解原理**: 不要盲目调参，理解卡尔曼滤波原理
3. **记录配置**: 每次调整记录原因和效果
4. **可视化验证**: 使用RViz和PlotJuggler观察
5. **阅读源码**: 遇到问题查看源码实现

### 推荐资源

- [官方教程](http://docs.ros.org/en/humble/p/robot_localization/)
- [Kalman滤波详解](https://www.kalmanfilter.net/)
- [传感器融合课程](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

---

祝你配置顺利！🚀
