# Localization 初始化流程详解

## 概述
`Localization::Init` 函数是定位系统的核心初始化函数，负责初始化所有必要的模块并配置系统参数。

---

## 初始化流程图

```
Init(yaml_path, global_map_path)
    │
    ├─► 1. 互斥锁保护 & 重复初始化检查
    │
    ├─► 2. 读取YAML配置文件
    │
    ├─► 3. 初始化激光里程计(LIO)
    │
    ├─► 4. 初始化激光定位模块(LidarLoc)
    │
    ├─► 5. 初始化UI界面(可选)
    │
    ├─► 6. 初始化位姿图优化(PGO)
    │
    ├─► 7. 配置异步处理模块
    │
    ├─► 8. 设置结果回调函数
    │
    └─► 9. 初始化点云预处理器
```

---

## 详细流程解析

### 1. 互斥锁保护与重复初始化检查
```cpp
UL lock(global_mutex_);
if (lidar_loc_ != nullptr) {
    Finish();  // 若已初始化，先清理
}
```

**作用：**
- 使用互斥锁防止多线程并发初始化导致的冲突
- 检查是否已初始化，如果是则先调用 `Finish()` 清理资源
- 保证系统可以重新初始化

---

### 2. 读取YAML配置文件
```cpp
YAML_IO yaml(yaml_path);
options_.with_ui_ = yaml.GetValue<bool>("system", "with_ui");
```

**参数：**
- `yaml_path`: YAML配置文件路径
- `with_ui_`: 是否启用3D可视化界面

**配置文件示例** (default_livox.yaml):
```yaml
system:
  with_ui: true           # 是否需要3D UI
  with_2dui: false        # 是否需要2D UI
  with_loop_closing: true # 是否需要回环检测
  with_g2p5: true         # 是否需要栅格地图
```

---

### 3. 初始化激光里程计模块 (LIO - Laser Inertial Odometry)

```cpp
LaserMapping::Options opt_lio;
opt_lio.is_in_slam_mode_ = false;  // 定位模式，非建图模式

lio_ = std::make_shared<LaserMapping>(opt_lio);
if (!lio_->Init(yaml_path)) {
    LOG(ERROR) << "failed to init lio";
    return false;
}
```

**作用：**
- LIO模块负责融合激光雷达和IMU数据，提供高频率的里程计估计
- `is_in_slam_mode_ = false`: 设置为定位模式而非建图模式
- 提供局部精确的位姿估计，作为定位的前端

**相关配置参数** (fasterlio):
```yaml
fasterlio:
  lidar_type: 1              # 1=Livox, 2=Velodyne, 3=Ouster
  scan_line: 32              # 扫描线数
  blind: 0.5                 # 盲区距离(米)
  time_scale: 1e-3           # 时间单位缩放
  point_filter_num: 10       # 点云采样参数
  max_iteration: 6           # ESKF最大迭代次数
  filter_size_scan: 0.5      # scan降采样尺寸
  filter_size_map: 0.5       # map降采样尺寸
  ivox_grid_resolution: 0.5  # iVox网格分辨率
  ivox_nearby_type: 18       # 邻近类型: 6/18/26
  acc_cov: 0.1              # 加速度协方差
  gyr_cov: 0.1              # 陀螺仪协方差
  extrinsic_T: [0, 0, 0.28] # IMU-LiDAR外参平移
  extrinsic_R: [1,0,0, 0,1,0, 0,0,1] # IMU-LiDAR外参旋转
```

---

### 4. 初始化激光定位模块 (LidarLoc)

```cpp
LidarLoc::Options lidar_loc_options;
lidar_loc_options.update_dynamic_cloud_ = yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
lidar_loc_options.map_option_.map_path_ = global_map_path;

lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);
lidar_loc_->Init(yaml_path);
```

**作用：**
- 激光定位模块负责在全局地图中匹配当前扫描，提供全局一致的位姿估计
- 使用NDT或ICP等点云配准算法

**核心参数：**
| 参数 | 含义 | 默认值 |
|------|------|--------|
| `update_dynamic_cloud_` | 是否更新动态点云 | true |
| `force_2d_` | 是否强制2D定位 | false |
| `enable_dynamic_polygon_` | 是否启用动态多边形 | false |
| `map_path_` | 全局地图路径 | 从参数传入 |

**相关配置参数** (lidar_loc):
```yaml
lidar_loc:
  force_2d: false                    # 是否强制2D定位
  update_dynamic_cloud: true         # 是否更新动态点云
  init_with_fp: true                 # 是否使用快速点进行初始化
  min_init_confidence: 1.8           # 最小初始化置信度
  loc_on_kf: true                    # 是否仅在关键帧上定位
  lidar_loc_odom_th: 0.5            # 里程计阈值
  update_kf_dis: 2.0                # 关键帧更新距离(米)
  update_lidar_loc_score: 2.5       # 激光定位更新分数
  grid_search_angle_range: 180.0    # 网格搜索角度范围(度)
  grid_search_angle_step: 60        # 网格搜索角度步长
  filter_z_min: -1.0                # Z轴过滤最小值
  filter_z_max: 30.0                # Z轴过滤最大值
  filter_intensity_min: 10.0        # 强度过滤最小值
  filter_intensity_max: 100.0       # 强度过滤最大值
```

---

### 5. 初始化UI界面 (可选)

```cpp
if (options_.with_ui_) {
    ui_ = std::make_shared<ui::PangolinWindow>();
    ui_->SetCurrentScanSize(10);  // 显示最近10帧扫描
    ui_->Init();
    
    lidar_loc_->SetUI(ui_);
}
```

**作用：**
- 使用Pangolin库创建3D可视化界面
- 实时显示定位结果、点云、轨迹等
- 便于调试和监控

**参数：**
- `SetCurrentScanSize(10)`: 在UI中显示最近10帧的点云扫描

---

### 6. 初始化位姿图优化模块 (PGO - Pose Graph Optimization)

```cpp
pgo_ = std::make_shared<PGO>();
pgo_->SetDebug(false);
```

**作用：**
- PGO模块融合多种位姿源（里程计、定位、RTK等）
- 通过图优化提供平滑、全局一致的位姿估计
- 处理不同传感器的噪声和误差

**相关配置参数** (pgo):
```yaml
pgo:
  # 里程计噪声参数
  dr_pos_noise: 1.0              # DR位置噪声
  dr_ang_noise: 0.5              # DR角度噪声
  dr_pos_noise_ratio: 1.0        # DR位置噪声比例
  
  # 激光里程计噪声
  lidar_odom_pos_noise: 0.3      # 激光里程计位置噪声
  lidar_odom_ang_noise: 1.0      # 激光里程计角度噪声
  
  # 激光定位噪声
  lidar_loc_pos_noise: 0.3       # 激光定位位置噪声
  lidar_loc_ang_noise: 1.0       # 激光定位角度噪声
  lidar_loc_outlier_th: 5.0      # 激光定位离群值阈值
  
  # 优化参数
  smooth_factor: 0.01            # 平滑因子
  pgo_frame_converge_pos_th: 0.05  # 收敛位置阈值
  pgo_frame_converge_ang_th: 1.0   # 收敛角度阈值
```

---

### 7. 配置异步处理模块

```cpp
/// 读取跳帧参数
options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
options_.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
options_.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");

/// 配置消息队列
lidar_odom_proc_cloud_.SetMaxSize(1);   // 队列大小为1
lidar_loc_proc_cloud_.SetMaxSize(1);

lidar_odom_proc_cloud_.SetName("激光里程计");
lidar_loc_proc_cloud_.SetName("激光定位");

/// 设置跳帧策略
lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_, options_.lidar_loc_skip_num_);
lidar_odom_proc_cloud_.SetSkipParam(options_.enable_lidar_odom_skip_, options_.lidar_odom_skip_num_);

/// 设置处理函数
lidar_odom_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarOdomProcCloud(cloud); });
lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

/// 在线模式启动异步处理
if (options_.online_mode_) {
    lidar_odom_proc_cloud_.Start();
    lidar_loc_proc_cloud_.Start();
}
```

**作用：**
- 使用异步消息队列处理点云数据，提高处理效率
- 支持跳帧策略，减少计算负载
- 区分在线模式和离线模式

**参数详解：**

| 参数 | 含义 | 典型值 |
|------|------|--------|
| `enable_lidar_loc_skip_` | 是否允许激光定位跳帧 | false |
| `lidar_loc_skip_num_` | 定位跳帧数（每N帧处理一次） | 4 |
| `enable_lidar_odom_skip_` | 是否允许激光里程计跳帧 | false |
| `lidar_odom_skip_num_` | 里程计跳帧数 | 2 |
| `loc_on_kf_` | 是否仅在关键帧上定位 | true |
| `online_mode_` | 在线/离线模式 | true(在线) |

**跳帧策略说明：**
- **激光里程计**：通常不跳帧或少跳帧，保证高频率位姿输出
- **激光定位**：可以适当跳帧，因为计算量大且不需要太高频率
- **关键帧定位**：只在关键帧处进行定位，进一步减少计算量

---

### 8. 设置结果回调函数

```cpp
pgo_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& res) {
    loc_result_ = res;

    // TF变换发布
    if (tf_callback_ && loc_result_.valid_) {
        tf_callback_(loc_result_.ToGeoMsg());
    }

    // UI更新
    if (ui_) {
        ui_->UpdateNavState(loc_result_.ToNavState());
        ui_->UpdateRecentPose(loc_result_.pose_);
    }
});
```

**作用：**
- 设置PGO模块的高频输出回调
- 当有新的定位结果时，自动更新UI和发布TF变换
- 提供实时的定位状态

**回调内容：**
1. 保存定位结果到 `loc_result_`
2. 如果有TF回调且结果有效，发布TF变换
3. 如果启用UI，更新导航状态和位姿显示

---

### 9. 初始化点云预处理器

```cpp
preprocess_.reset(new PointCloudPreprocess());
preprocess_->Blind() = yaml.GetValue<double>("fasterlio", "blind");
preprocess_->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
preprocess_->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
preprocess_->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");

LOG(INFO) << "lidar_type " << lidar_type;
if (lidar_type == 1) {
    preprocess_->SetLidarType(LidarType::AVIA);
    LOG(INFO) << "Using AVIA Lidar";
} else if (lidar_type == 2) {
    preprocess_->SetLidarType(LidarType::VELO32);
    LOG(INFO) << "Using Velodyne 32 Lidar";
} else if (lidar_type == 3) {
    preprocess_->SetLidarType(LidarType::OUST64);
    LOG(INFO) << "Using OUST 64 Lidar";
} else {
    LOG(WARNING) << "unknown lidar_type";
}
```

**作用：**
- 对原始点云进行预处理，统一格式
- 处理不同类型激光雷达的数据差异
- 去除无效点、降采样等

**参数详解：**

| 参数 | 含义 | 说明 |
|------|------|------|
| `Blind()` | 盲区距离 | 小于此距离的点被过滤 (0.5m) |
| `TimeScale()` | 时间缩放 | 不同雷达时间单位转换 (1e-3) |
| `NumScans()` | 扫描线数 | 雷达线数 (32) |
| `PointFilterNum()` | 点过滤数 | 每N个点采样一个 (10) |

**支持的雷达类型：**
1. **AVIA (Livox)** - type=1
   - 非重复扫描模式
   - 高FOV覆盖
   - 需要特殊的时间戳处理

2. **Velodyne 32** - type=2
   - 机械旋转式
   - 固定扫描线
   - 标准ROS格式

3. **Ouster 64** - type=3
   - 固态激光雷达
   - 高分辨率
   - 特殊的数据格式

---

## 模块关系图

```
                    ┌─────────────────┐
                    │  Localization   │
                    │   (主控模块)     │
                    └────────┬────────┘
                             │
                ┌────────────┼────────────┐
                │            │            │
         ┌──────▼──────┐ ┌──▼───────┐ ┌─▼────────┐
         │   LIO       │ │ LidarLoc │ │   PGO    │
         │(激光里程计)  │ │(激光定位) │ │(位姿图)  │
         └──────┬──────┘ └──┬───────┘ └─┬────────┘
                │            │            │
                │            │            │
                └────────────┼────────────┘
                             │
                    ┌────────▼────────┐
                    │  点云预处理器    │
                    └─────────────────┘
```

**数据流向：**
1. 原始点云 → 预处理器 → 处理后点云
2. 处理后点云 → LIO → 里程计位姿
3. 处理后点云 → LidarLoc → 定位位姿
4. 里程计位姿 + 定位位姿 + IMU → PGO → 融合后位姿
5. 融合后位姿 → 回调函数 → TF发布/UI更新

---

## 关键概念说明

### LIO (Laser Inertial Odometry)
- **作用**：融合激光雷达和IMU，提供高频率、局部精确的位姿估计
- **优点**：频率高(10-20Hz)，短期精度好
- **缺点**：长期累积误差

### LidarLoc (Lidar Localization)
- **作用**：在预先建好的全局地图中定位
- **优点**：无累积误差，全局一致
- **缺点**：计算量大，频率低(1-5Hz)

### PGO (Pose Graph Optimization)
- **作用**：融合多种位姿估计，通过图优化得到最优结果
- **优点**：综合多源信息，平滑且全局一致
- **输入**：LIO位姿、LidarLoc位姿、IMU数据、(可选)RTK数据

### 异步处理
- **目的**：提高实时性，避免阻塞主线程
- **实现**：使用消息队列 + 独立线程处理
- **策略**：支持跳帧、限制队列大小

---

## 典型使用场景

### 场景1：在线定位（机器人导航）
```cpp
Localization::Options opt;
opt.online_mode_ = true;      // 在线模式
opt.with_ui_ = false;         // 不需要UI
opt.enable_lidar_loc_skip_ = true;   // 允许跳帧
opt.lidar_loc_skip_num_ = 4;  // 定位每4帧处理一次

Localization loc(opt);
loc.Init("config.yaml", "/path/to/map");
```

### 场景2：离线数据处理（测试/调试）
```cpp
Localization::Options opt;
opt.online_mode_ = false;     // 离线模式
opt.with_ui_ = true;          // 启用UI
opt.enable_lidar_loc_skip_ = false;  // 不跳帧

Localization loc(opt);
loc.Init("config.yaml", "/path/to/map");
```

---

## 初始化检查清单

初始化成功需要满足：
- ✅ YAML配置文件存在且格式正确
- ✅ 全局地图路径有效且包含必要文件
- ✅ LIO模块初始化成功
- ✅ LidarLoc模块初始化成功
- ✅ PGO模块创建成功
- ✅ 点云预处理器配置正确
- ✅ 异步处理模块启动（在线模式）

---

## 常见问题排查

### 问题1：初始化失败
**检查：**
- YAML文件路径是否正确
- 配置参数是否完整
- 地图文件是否存在

### 问题2：定位不收敛
**检查：**
- 初始位姿是否接近真实位置
- `min_init_confidence` 是否设置过高
- 地图是否与环境匹配

### 问题3：性能问题
**调整：**
- 启用跳帧：`enable_lidar_loc_skip_ = true`
- 增加跳帧数：`lidar_loc_skip_num_ = 4`
- 关闭UI：`with_ui_ = false`
- 仅关键帧定位：`loc_on_kf_ = true`

---

## 总结

`Localization::Init` 函数按照以下顺序初始化整个定位系统：

1. **安全检查** - 互斥锁保护，避免重复初始化
2. **配置加载** - 从YAML读取所有参数
3. **核心模块** - LIO、LidarLoc、PGO依次初始化
4. **辅助功能** - UI、异步处理、预处理器
5. **回调设置** - 连接各模块输出到外部接口

整个系统采用模块化设计，各模块职责清晰，通过异步消息队列实现高效的数据流转。
