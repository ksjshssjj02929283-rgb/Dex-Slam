# lightning-lm 定位系统算法文档 (Localization System)

> 所有论断基于实际代码，带 `[VERIFY: file:line]` 标注。

---

## 一、定位系统概述

lightning-lm 的定位系统（`Localization`）是在**已有地图**上进行实时定位，无需重建地图。  
整体思路：**LIO 里程计（短期精确）+ 全局地图匹配（长期无漂移）+ 高频 IMU 外推**。

### 1.1 与建图模式的对比

| 特性 | 建图模式（SlamSystem） | 定位模式（LocSystem） |
|------|------------------------|----------------------|
| 是否生成新关键帧 | 是 | 否（仅内部使用） |
| 地图更新 | 实时更新 | 加载已有地图，只读 |
| 回环检测 | 是（LoopClosing） | 否 |
| 全局匹配 | 否 | 是（LidarLoc + TiledMap） |
| TF 输出频率 | 与 LiDAR 同频（10Hz） | IMU 同频（100Hz） |

### 1.2 架构概览

```
LocSystem（ROS2 节点）
    │
    └──► Localization loc_
              │
              ├── LaserMapping lio_         # LIO 里程计前端（非SLAM模式）
              │     └── ESKF + iVox（仅建局部地图，不存关键帧到磁盘）
              │
              ├── LidarLoc lidar_loc_       # 全局地图匹配
              │     └── TiledMap（从磁盘加载分块地图）
              │         └── NDT / IVox kNN
              │
              └── PGO pgo_                  # 位姿图融合（含高频外推）
                    └── PoseExtrapolator pe_
                          └── IMU 驱动 → 100Hz TF 发布
```

**[VERIFY: src/core/localization/localization.h:55-110]** — 成员声明  
**[VERIFY: src/core/system/loc_system.cc:30-80]** — `LocSystem::Init()`

---

## 二、异步双通道架构

**[VERIFY: src/core/localization/localization.cpp:49-130]**

```
收到 LiDAR 点云
    │
    ├──► lidar_odom_proc_cloud_.AddMessage()  ──► [LIO 通道，10Hz]
    │         ProcessLidarOdom()
    │             ├── lio_->ProcessPointCloud2(cloud)
    │             │   └── IEKF 更新  → pose_lio_
    │             └── pgo_->AddLidarOdomLoc(pose_lio_)
    │                   └── PoseExtrapolator::AddDRLoc()
    │
    └──► 每 lidar_loc_skip_num_ 帧：
              lidar_loc_proc_cloud_.AddMessage()  ──► [全局定位通道，~2Hz]
                    ProcessLidarLoc()
                        ├── lidar_loc_->Locate(cloud, init_pose)
                        │   └── TiledMap NDT 匹配
                        └── pgo_->AddLidarLocResult(pose_global)
                              └── PoseExtrapolator::AddGlobalLoc()
                                    └── 触发 TF 输出
```

**[VERIFY: src/core/localization/localization.cpp:65-75]** — 两条 `AsyncMessageProcess` 初始化

---

## 三、LIO 里程计通道

**[VERIFY: src/core/localization/localization.cpp:80-115]**

定位模式下，LIO 前端（`lio_`）以**非SLAM模式**运行：
- 不创建持久化关键帧
- 只维护局部 iVox（局部去畸变窗口）
- 输出每帧的 pose_lio_（SE3，相对于初始位姿）

```cpp
// localization.cpp: ProcessLidarOdom()
lio_->ProcessPointCloud2(cloud);
SE3 curr_pose = lio_->GetCurrentPose();     // IEKF 当前最优位姿

// 喂给 PGO 的里程计通道
pgo_->AddLidarOdomLoc(TimedSE3{timestamp, curr_pose});
```

---

## 四、全局 LiDAR 定位通道（LidarLoc）

**文件**：`src/core/localization/lidar_loc/`

### 4.1 LidarLoc::Locate() 流程

```
输入：
  cloud     ← 当前帧点云
  init_pose ← 初始猜测（来自上一次定位结果 + LIO 增量）

流程：
  1. 加载 init_pose 附近的 TiledMap 块（动态邻域加载）
  2. 构建用于匹配的局部地图（nearby chunks → 合并点云）
  3. NDT 匹配：
     target = 局部地图点云（或 TiledMap IVox）
     source = 当前帧点云（降采样）
     initial_guess = init_pose
  4. 验证得分，返回 SE3 global_pose

输出：
  SE3 global_pose  ← 当前帧在世界坐标系中的位姿
```

**[VERIFY: src/core/localization/lidar_loc/lidar_loc.h]** — `Locate()` 接口

### 4.2 与 TiledMap 的交互

```cpp
// 加载附近地图块
tiled_map_->LoadNearbyChunks(init_pose.translation());

// 获取匹配用的局部地图（合并附近 N 块）
CloudPtr local_map = tiled_map_->GetLocalMap();

// 使用 IVox 定位地图（精确 kNN，可选）
if (use_ivox_locmap_)
    tiled_map_->UpdateIVoxLocmap(local_map);
```

**[VERIFY: src/core/maps/tiled_map.h:80-120]** — `GetLocalMap()`

---

## 五、位姿图融合（PGO）

**文件**：`src/core/localization/pose_graph/pgo.h/cc`

### 5.1 PGO 在定位模式中的角色

与建图模式（miao 全局 PGO）不同，定位模式的 PGO 更轻量：
- 只维护**短窗口**的位姿历史（滑动窗口）
- 融合 LIO 里程计（高频，相对精确）+ 全局定位（低频，绝对精确）
- 输出给 PoseExtrapolator 做高频外推

```
PGO 融合逻辑（简化版）：
  [LIO 里程计] ──► 相对增量约束（连续帧间）
  [全局定位]   ──► 绝对位置校正（每 lidar_loc_skip_num_ 帧一次）
                       │
               ──► PoseExtrapolator::AddGlobalLoc()
                       │
               ──► 触发全局输出（100Hz TF 广播）
```

**[VERIFY: src/core/localization/pose_graph/pgo.h:50-95]** — `AddLidarOdomLoc` / `AddLidarLocResult`

### 5.2 高频输出回调注册

```cpp
// localization.cpp: Init()
pgo_->SetHighFrequencyGlobalOutputHandleFunction(
    [this](const SE3& pose, double t) {
        // 发布 TF: map → base_link
        PublishTF(pose, t);
        // 发布 RViz (可选)
        PublishPoseRViz(pose, t);
    }
);
```

**[VERIFY: src/core/localization/localization.cpp:70-80]** — 回调注册

---

## 六、高频位姿外推（PoseExtrapolator）

**文件**：`src/core/localization/pose_graph/pose_extrapolator.h/cc`

### 6.1 工作原理

```
维护状态：
  last_odom_pose_    ← 最新 LIO 里程计位姿
  last_global_pose_  ← 最新全局定位位姿
  imu_queue_         ← 最近的 IMU 数据队列

高频外推（每个 IMU 间隔 ~10ms）：
  1. 用 IMU 积分从 last_odom_pose_ 外推到当前时刻
     delta_R = IMU 角速度积分（last_imu_time → t_now）
     delta_p = 速度 × dt（近似）
  
  2. 用 last_global_pose_ 修正漂移（低通滤波）
     pose_output = global_anchor + odom_incremental
  
  3. 触发高频输出回调 → TF 发布 @100Hz
```

**[VERIFY: src/core/localization/pose_graph/pose_extrapolator.cc]** — 完整实现

### 6.2 初始化流程

```cpp
// 需要外部提供初始位姿（set_init_pose）
void PoseExtrapolator::SetInitPose(const SE3& init_pose);
```

对应 LocSystem 中的 `/initialpose`（RViz 2D Estimate 工具）或 ROS2 服务接口。

**[VERIFY: src/core/system/loc_system.cc:90-120]** — `SetInitPose()` 调用链

---

## 七、初始化定位（SetInitPose）

**[VERIFY: src/core/system/loc_system.cc:90-130]**

```cpp
// 使用 RViz "2D Pose Estimate" 设置初始位姿
void LocSystem::SetInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    SE3 init_pose = FromRosPose(msg.pose.pose);  // 从 ROS msg 转 SE3
    loc_->SetExternalPose(init_pose);
    loc_started_ = true;  // 启动定位流程
}
```

也可通过服务接口（`/lightning/loc_cmd`）传入初始位姿。

---

## 八、TF 发布机制

**[VERIFY: src/core/system/loc_system.cc:55-65]**

```cpp
// 条件：options_.pub_tf_ = true（默认开启）
if (options_.pub_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

// PoseExtrapolator 高频回调中：
// 发布 map → base_link 的 TF
geometry_msgs::msg::TransformStamped tf_msg;
tf_msg.header.frame_id = "map";
tf_msg.child_frame_id  = "base_link";
// ...填充位姿...
tf_broadcaster_->sendTransform(tf_msg);
```

输出频率：与 IMU 同频，约 100Hz（IMU 每条消息触发一次外推+发布）。

---

## 九、性能特征

| 环节 | 频率 | 延迟 | 备注 |
|------|------|------|------|
| LiDAR 接收 | 10Hz | — | ROS2 subscription |
| LIO 里程计 | 10Hz | ~20ms | IEKF 主循环 |
| 全局定位 | ~2-5Hz | ~50-100ms | `lidar_loc_skip_num_` 控制 |
| TF /位姿输出 | 100Hz | ~1ms | IMU 驱动外推 |

---

## 十、定位模式关键参数

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `localization.lidar_loc_skip_num` | 5 | 全局定位跳帧数（越大越省CPU） |
| `localization.use_ivox_locmap` | false | 是否用 IVox 加速定位地图搜索 |
| `localization.ndt_resolution` | 1.0m | 定位用 NDT 分辨率 |
| `localization.icp_max_corr_dist` | 1.5m | 定位 ICP 最大对应点距离 |
| `system.pub_tf` | true | 是否发布 TF（导航需要） |
| `system.map_path` | — | 已建地图路径 |

---

## 附录：定位模式相关源码索引

| 功能 | 文件 | 行号 |
|------|------|------|
| LocSystem::Init() | loc_system.cc | 30–80 |
| pub_tf_ 控制 | loc_system.cc | 55–65 |
| SetInitPose() | loc_system.cc | 90–130 |
| Localization::Init() | localization.cpp | 49–130 |
| 双异步通道初始化 | localization.cpp | 65–75 |
| PGO 回调注册 | localization.cpp | 70–80 |
| ProcessLidarOdom() | localization.cpp | 80–115 |
| PoseExtrapolator 接口 | pose_extrapolator.h | All |
| LidarLoc::Locate() | lidar_loc.h/cc | All |
| TiledMap 邻域加载 | tiled_map.cc | 90–130 |
