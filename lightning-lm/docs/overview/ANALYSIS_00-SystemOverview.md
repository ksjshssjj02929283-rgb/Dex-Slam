# lightning-lm 系统架构概述 (System Overview)

> 所有论断基于实际代码，带 `[VERIFY: file:line]` 标注。

---

## 一、项目定位

lightning-lm 是一套运行在 **ROS2 Humble** 上的 3D LiDAR-Inertial SLAM 与定位系统，面向移动机器人场景（DEXE W1 PRO）。

核心功能：
1. **在线/离线建图**：基于 AA-FasterLIO 前端，输出关键帧点云地图
2. **全局回环检测**：LIO-SAM 风格 NDT/GICP + PGO，消除累积漂移
3. **实时定位**：分块地图 NDT 匹配 + ESKF 高频位姿外推，100Hz TF 输出
4. **2D 栅格地图**：g2p5 模块将 3D LiDAR 投影为 2D 占据栅格（直接输出给导航）

---

## 二、模块层次结构

```
lightning-lm/src/
├── app/                     # 可执行程序入口（6个）
│   ├── run_slam_online.cc   # 在线建图
│   ├── run_slam_offline.cc  # 离线建图
│   ├── run_loc_online.cc    # 在线定位
│   ├── run_loc_offline.cc   # 离线定位
│   ├── run_frontend_offline.cc  # 仅前端运行（调试）
│   └── run_loop_offline.cc  # 仅回环运行（调试）
│
├── core/                    # 核心算法
│   ├── lio/                 # LiDAR-Inertial Odometry 前端
│   │   ├── eskf.hpp/cc      # Error-State Kalman Filter（24维状态）
│   │   ├── imu_processing.h/cc   # IMU 初始化、预积分、去畸变
│   │   ├── laser_mapping.h/cc    # 主处理循环（IEKF + iVox/VoxelMap）
│   │   ├── pointcloud_preprocess.h/cc  # 点云预处理（类型适配）
│   │   ├── anderson_acceleration.h     # AA 加速收敛
│   │   └── pose6d.h         # 关键帧6D位姿（去畸变用）
│   │
│   ├── loop_closing/        # 回环检测后端
│   │   └── loop_closing.h/cc  # LIO-SAM 风格 KD-Tree + NDT/GICP + PGO
│   │
│   ├── localization/        # 实时定位系统
│   │   ├── localization.h/cpp  # 定位顶层接口（异步双通道）
│   │   ├── lidar_loc/          # LiDAR 点云匹配定位（分块地图 NDT）
│   │   └── pose_graph/
│   │       ├── pgo.h/cc        # 位姿图优化
│   │       ├── pose_extrapolator.h/cc  # 高频位姿外推（IMU）
│   │       └── smoother.h      # 输出平滑
│   │
│   ├── maps/                # 分块地图（TiledMap）
│   │   ├── tiled_map.h/cc   # 动静态分块点云地图
│   │   └── tiled_map_chunk.h/cc  # 单块地图管理
│   │
│   ├── g2p5/                # 3D→2D 栅格地图（射线追踪）
│   │   ├── g2p5.h/cc        # 顶层接口（前端增量 + 后端重绘）
│   │   ├── g2p5_map.h/cc    # 地图数据结构
│   │   └── g2p5_subgrid.h/cc  # 子栅格
│   │
│   ├── ivox3d/              # iVox 增量式体素地图（kNN）
│   ├── voxel_map/           # VoxelMap（平面协方差地图）
│   ├── miao/                # 轻量 PGO 优化库（基于 g2o，支持增量模式）
│   └── system/
│       ├── slam.h/cc        # SLAM 系统调用接口（SlamSystem）
│       └── loc_system.h/cc  # 定位系统调用接口（LocSystem）
│
├── common/                  # 公共类型
│   ├── keyframe.h           # 关键帧（id, cloud, LIO pose, opt pose, NavState）
│   ├── nav_state.h          # 状态向量（23维，含旋转/位置/速度/偏置/重力）
│   ├── imu.h                # IMU 数据类型
│   ├── options.h/cc         # 全局编译参数（FasterLIO 参数）
│   └── params.h/cc          # 运行时参数
│
├── io/                      # 文件 I/O
│   ├── yaml_io.h/cc         # YAML 配置读取
│   └── file_io.h/cc         # 地图文件读写
│
├── wrapper/                 # ROS2 封装层
│   ├── bag_io.h/cc          # ROS2 Bag 播放
│   ├── ros_utils.h          # ROS2 工具函数（时间转换等）
│   └── rviz_publisher.h/cc  # RViz 可视化发布
│
└── ui/                      # Pangolin 3D 可视化
```

**[VERIFY: src/core/system/slam.h:24-31]** — `SlamSystem` 成员：`lio_`, `lc_`, `ui_`, `g2p5_`, `rviz_pub_`  
**[VERIFY: src/core/system/slam.h:66-82]** — `SlamSystem` 私有成员声明

---

## 三、整体数据流（建图模式）

```
传感器输入
    │
    ├── IMU (@50-400Hz) ──────────────────────────────────────────────┐
    │                                                                  │
    └── LiDAR (@10Hz) ──► PointCloudPreprocess                       │
                               │ (去除近点/时间戳归一化)              │
                               ▼                                       │
                          SyncPackages()                               │
                          (IMU+LiDAR 时间对齐，填充 MeasureGroup)      │
                               │                                       │
                               ▼                                       │
                          ImuProcess::Process()                        │
                          (IMU 初始化 / IMU-only 预测 / 去畸变)       │◄─┘
                               │ scan_undistort_
                               ▼
                         ┌─────────────────────────────────────────┐
                         │         IEKF 主循环                       │
                         │  kf_.Update(ObsType::LIDAR)              │
                         │   ├─ ObsModel(): 点面残差 + 雅可比         │
                         │   │   (IVox kNN 或 VoxelMap 协方差匹配)   │
                         │   └─ ESKF::Predict() ← IMU 预测          │
                         └─────────────────────────────────────────┘
                               │ state_point_ (NavState)
                               ├──► MapIncremental()
                               │    (IVox::AddPoints / VoxelMap::UpdateMap)
                               │
                               └──► MakeKF() [当平移>0.5m 或角度>15°]
                                    │ Keyframe (id, cloud, LIO pose)
                                    │
                    ┌───────────────┼───────────────────────────────┐
                    │               │                               │
                    ▼               ▼                               ▼
              LoopClosing      G2P5::PushKF()              RVizPublisher
              ::AddKF()        (2D 栅格增量渲染)            (可视化发布)
                    │
                    ▼
              HandleKF() [异步线程]
                    │
      ┌─────────────┼──────────────────────┐
      │             │                      │
      ▼             ▼                      ▼
DetectLoopCandidates  ComputeLoopCandidates  AddOdomFactor
(KD-Tree 搜索)       (NDT/GICP 配准)        (加入因子图)
                                             │
                                        AddLoopFactor → PoseOptimization
                                                        (miao 增量 PGO)
                                                             │
                                                        SetOptPose(kf)
                                                             │
                                                        G2P5::RedrawGlobalMap()
```

**[VERIFY: src/core/lio/laser_mapping.cc:216-218]** — `Run()` 调用序列：`p_imu_->Process` → `kf_.Update` → `MapIncremental` → `MakeKF`  
**[VERIFY: src/core/loop_closing/loop_closing.cc:143-165]** — `HandleKF()` 调用序列：`DetectLoopCandidates` → `ComputeLoopCandidates` → `AddOdomFactor` → `AddLoopFactor` → `PoseOptimization`

---

## 四、定位模式数据流

```
传感器输入
    │
    ├── IMU ──► LidarOdomProcCloud [异步线程1]
    │               │
    │               ├── LIO 前端（LaserMapping, 非SLAM模式）
    │               │   └── 输出 LIO 关键帧 → PGO::AddLidarOdomLoc()
    │               │       └── PoseExtrapolator::AddDRLocAndExtrapolate()
    │               │           → TF 发布 @100Hz（IMU 频率）
    │               │
    │               └── [每 lidar_loc_skip_num_ 帧触发一次]
    │                   ↓
    │             LidarLocProcCloud [异步线程2]
    │                   │
    │                   ├── LidarLoc::Locate()
    │                   │   (TiledMap NDT/ICP 匹配)
    │                   │   └── 输出全局位姿 → PGO::AddLidarLocResult()
    │                   │
    │                   └── PoseExtrapolator 校正补偿
    │                       └── TF 输出（高频补偿）
    │
    └── 初始位姿（人工设置或服务接口）
        └── SetExternalPose() → PoseExtrapolator 初始化
```

**[VERIFY: src/core/localization/localization.cpp:49-130]** — `Init()` 创建 `lio_`, `lidar_loc_`, `pgo_`, 两条异步处理链  
**[VERIFY: src/core/localization/localization.cpp:70-80]** — `pgo_->SetHighFrequencyGlobalOutputHandleFunction` 注册 TF 回调

---

## 五、线程模型

| 线程 | 触发方式 | 职责 |
|------|----------|------|
| ROS2 spin（主线程） | ROS2 消息 | IMU/LiDAR 订阅，放入 buffer |
| LaserMapping::Run() | 消息同步后调用 | IEKF 主循环，构建关键帧 |
| LoopClosing::kf_thread_ | AsyncMessageProcess | 回环检测 + PGO（在线模式） |
| ThreadPool（N/2 核） | task::Schedule | MDT/GICP 并行配准 |
| Localization::lidar_odom_proc_cloud_ | AsyncMessageProcess | LIO 里程计处理 |
| Localization::lidar_loc_proc_cloud_ | AsyncMessageProcess | 全局 LiDAR 定位 |
| G2P5 后端重绘线程 | `RedrawGlobalMapBackend()` | 回环后重绘 2D 栅格 |
| PoseExtrapolator | IMU 频率调用 | 高频位姿外推 + TF 发布 |

**[VERIFY: src/core/loop_closing/loop_closing.cc:133-139]** — `kf_thread_` 启动：`SetProcFunc` + `Start()`  
**[VERIFY: src/core/loop_closing/loop_closing.cc:141-143]** — `ThreadPool` 创建，线程数 = `hardware_concurrency()/2`

---

## 六、关键配置参数映射

| YAML 路径 | 对应代码 | 含义 |
|-----------|----------|------|
| `common.lidar_topic` | `SlamSystem::cloud_topic_` | 标准点云话题 |
| `common.imu_topic` | `SlamSystem::imu_topic_` | IMU 话题 |
| `fasterlio.lidar_type` | `LaserMapping::lidar_type_` | 1=Livox,2=Velodyne,3=Ouster |
| `fasterlio.time_scale` | `PointCloudPreprocess::time_scale_` | 时间戳单位 |
| `fasterlio.point_filter_num` | `PointCloudPreprocess::point_filter_num_` | 点云采样间隔 |
| `fasterlio.use_ivox` | `LaserMapping::use_ivox_` | 是否启用 iVox 地图 |
| `fasterlio.use_voxel_map` | `LaserMapping::use_voxel_map_` | 是否启用 VoxelMap |
| `fasterlio.kf_dis_th` | `LaserMapping::Options::kf_dis_th_` | 关键帧距离阈值（默认0.5m） |
| `fasterlio.kf_angle_th` | `LaserMapping::Options::kf_angle_th_` | 关键帧角度阈值（默认15°） |
| `loop_closing.loop_kf_gap` | `LoopClosing::Options::loop_kf_gap_` | 回环冷却期（关键帧数，默认50） |
| `loop_closing.max_range` | `LoopClosing::Options::max_range_` | KD-Tree 搜索半径（默认30m） |
| `loop_closing.history_search_time_diff` | `LoopClosing::Options::history_search_time_diff_` | 时间差阈值（默认30s） |
| `system.with_loop_closing` | `SlamSystem::Options::with_loop_closing_` | 是否启用回环 |
| `system.with_g2p5` | `SlamSystem::Options::with_gridmap_` | 是否生成 2D 栅格 |
| `system.map_path` | `LoopClosing, TiledMap` | 地图存储路径 |

**[VERIFY: src/core/lio/laser_mapping.cc:52-148]** — `LoadParamsFromYAML` 完整参数加载  
**[VERIFY: src/core/loop_closing/loop_closing.cc:45-135]** — `LoopClosing::Init` 参数读取

---

## 七、本地图结构

```
data/<map_name>/
├── global.pcd          # 完整全局点云（仅用于可视化，pcl_viewer 可打开）
├── map.pgm             # 2D 栅格地图（ROS nav_msgs/OccupancyGrid 格式）
├── map.yaml            # 栅格地图元数据（分辨率、origin 等）
├── *.pcd               # 分块地图文件（TiledMapChunk，100m×100m 网格）
└── *.dyn               # 动态图层（可选，保存动态障碍物历史点云）
```

---

## 八、性能指标参考

| 指标 | 值 | 环境 |
|------|-----|------|
| 在线定位 CPU | 0.8 核 | 32 线 LiDAR，无 UI |
| 在线建图 CPU | 1.2 核 | 32 线 LiDAR，无 UI |
| TF 输出频率 | 100Hz（IMU 同频） | — |
| 关键帧创建触发 | 平移 > 0.5m 或旋转 > 15° | **[VERIFY: src/core/lio/laser_mapping.h:46-47]** |
| IEKF 最大迭代次数 | 4（可配置） | `fasterlio.max_iteration` |
| 回环检测 KD-Tree 半径 | 30m（默认） | `loop_closing.max_range` |

---

## 九、模块依赖关系图

```
SlamSystem
├──► LaserMapping (lio_)
│     ├──► ESKF (kf_)
│     ├──► ImuProcess (p_imu_)
│     ├──► IVoxManager (ivox_manager_)
│     ├──► VoxelMapManager (voxel_map_)
│     └──► PointCloudPreprocess (preprocess_)
│
├──► LoopClosing (lc_)
│     ├──► miao::Optimizer (optimizer_)
│     ├──► pcl::KdTreeFLANN (kdtree_history_poses_)
│     └──► common::ThreadPool (thread_pool_)
│
├──► G2P5 (g2p5_)
│     ├──► G2P5Map
│     └──► AsyncMessageProcess
│
└──► RVizPublisher (rviz_pub_)

LocSystem
└──► Localization (loc_)
      ├──► LaserMapping (lio_) [非SLAM模式]
      ├──► LidarLoc (lidar_loc_)
      │     └──► TiledMap
      ├──► PGO (pgo_)
      │     └──► PoseExtrapolator
      └──► AsyncMessageProcess (lidar_odom/lidar_loc)
```

**[VERIFY: src/core/system/slam.h:83-92]** — `SlamSystem` 私有成员：`lio_`, `lc_`, `ui_`, `g2p5_`, `rviz_pub_`  
**[VERIFY: src/core/localization/localization.h:80-100]** — `Localization` 私有成员声明
