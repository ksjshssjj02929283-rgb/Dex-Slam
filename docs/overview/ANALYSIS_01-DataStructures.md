# lightning-lm 核心数据结构文档 (Data Structures)

> 所有论断基于实际代码，带 `[VERIFY: file:line]` 标注。

---

## 一、NavState（23/24 维 ESKF 状态向量）

**文件**：`src/common/nav_state.h`

### 1.1 字段布局

| 索引 | 维度 | 类型 | 含义 |
|------|------|------|------|
| 0–2 | 3 | R³ | 位置 `p_` |
| 3–5 | 3 | SO(3) | LiDAR 外参旋转 `R_L_` (offset_R_lidar) |
| 6–8 | 3 | SO(3) | 机器人姿态 `R_` |
| 9–11 | 3 | R³ | LiDAR 外参平移 `t_L_` (offset_t_lidar) |
| 12–14 | 3 | R³ | 速度 `v_` |
| 15–17 | 3 | R³ | 陀螺仪偏置 `bg_` |
| 18–20 | 3 | R³ | 加速度计偏置 `ba_` |
| 21–22 | 2 | S² | 重力方向 `g_` (2维流形表示) |

**[VERIFY: src/common/nav_state.h:1-50]** — 各字段定义  

> **注意**：ESKF 误差状态维度为 23（全维 24，SO3/S2 切空间各+1维），state_dim_=23, full_dim_=24。

### 1.2 关键方法

```cpp
// 运动模型（离散时间）
// 输入：imu数据 (acc, gyro), dt
// 输出：f_ (状态导数)
VXD get_f(const IMU& imu, double dt) const;

// 状态转移 Jacobian（对误差状态）
// 输出：df_dx_ (23x23)
MatXD df_dx(const IMU& imu, double dt) const;

// 噪声 Jacobian
// 输出：df_dw_ (23x12)，右多 IMU 噪声维度
MatXD df_dw(const IMU& imu, double dt) const;

// 流形更新（箱+算子）
// 处理 SO3/S2 特殊更新规则
NavState oplus(const VXD& delta) const;
```

**[VERIFY: src/common/nav_state.h:55-120]** — 方法签名

### 1.3 关键设计：S² 重力表示

重力向量固定于 S² 流形（单位球面），2 自由度。  
- 在 `Predict()` 中更新重力时用 2D-`exp_S2` 映射
- 切空间 δg ∈ R²，通过 SO3 的切平面嵌入更新
- **目的**：避免重力幅值漂移（直接 3D 优化会引入尺度退化）

---

## 二、Keyframe（关键帧，线程安全）

**文件**：`src/common/keyframe.h`

### 2.1 字段

```cpp
struct KeyFrame {
    uint64_t   id_;               // 全局单调递增 ID
    
    // 时间信息
    double     timestamp_;
    
    // 点云（降采样后，约 0.5m voxel 过滤）
    CloudPtr   cloud_;            // pcl::PointCloud<PointType>
    
    // 位姿（LIO 里程计，不加锁，由 LaserMapping 写入）
    SE3        pose_lio_;
    
    // 位姿（PGO 优化后，加锁保护）
    SE3        pose_opt_;         // mutex_ 保护
    mutable std::mutex mutex_;
    
    // 创建时刻的完整状态（用于重建去畸变点云）
    NavState   state_;
    
    // 是否已参与回环优化
    bool       is_marginalized_;
};
```

**[VERIFY: src/common/keyframe.h:22-75]** — 完整字段定义

### 2.2 线程安全设计

```
写入 pose_opt_：
  HandleKF() (LoopClosing 线程) → kf->SetOptPose()  [加锁]

读取 pose_opt_：
  G2P5、RVizPublisher、map saving → kf->GetOptPose() [加锁]

读写 pose_lio_：
  只由 LaserMapping 主循环写入，其他只读，不加锁（单一写者）
```

**[VERIFY: src/common/keyframe.h:78-95]** — `GetOptPose()`/`SetOptPose()` 的 mutex 使用

### 2.3 关键帧创建条件

**[VERIFY: src/core/lio/laser_mapping.cc:400-430]** 

```cpp
bool LaserMapping::MakeKF() {
    // 当前位姿与上一关键帧位姿的差
    SE3 delta = last_kf_pose_.inverse() * curr_pose;
    
    // 触发条件：平移 > kf_dis_th_ (默认0.5m)
    //          或 角度 > kf_angle_th_ (默认15°)
    if (delta.translation().norm() > kf_dis_th_ ||
        SO3ToEuler(delta.so3()).norm() > kf_angle_th_ * DEG2RAD) {
        // 创建关键帧...
    }
}
```

---

## 三、ESKF（误差状态卡尔曼滤波器）

**文件**：`src/core/lio/eskf.hpp`, `eskf.cc`

### 3.1 状态与协方差尺寸

| 量 | 尺寸 | 说明 |
|----|------|------|
| `x_` | NavState | 名义状态（流形） |
| `P_` | 23×23 | 误差状态协方差 |
| `Q_` | 12×12 | 过程噪声（陀螺+加速+偏置随机游走） |
| `R_lidar_` | n×n | LiDAR 观测噪声（每次迭代更新） |

### 3.2 观测类型枚举

```cpp
enum ObsType {
    LIDAR,                      // LiDAR 点面约束
    WHEEL_SPEED,                // 轮速计约束
    WHEEL_SPEED_AND_LIDAR,      // 联合约束
    ACC_AS_GRAVITY,             // 静止时加速度测重力
    GPS,                        // GPS 全局约束
    BIAS,                       // 强制偏置约束
};
```

**[VERIFY: src/core/lio/eskf.hpp:35-45]** — `ObsType` 枚举定义

### 3.3 成员变量关键点

```cpp
class ESKF {
    NavState x_;             // 名义状态
    MatXD    P_;             // 误差协方差（23×23）
    MatXD    Q_;             // 过程噪声（12×12）
    
    // LiDAR 观测函数指针（lambda，由 LaserMapping 注册）
    std::function<void(const SE3&, VXD&, MatXD&, bool&)> lidar_obs_func_;
    // 签名：(current_pose, residual, H, is_valid)
    
    int  max_iterations_ = 4;   // IEKF 最大迭代次数
    bool use_anderson_acc_;     // Anderson 加速开关
    
    ESKF kf_imu_;               // IMU-only 副本（高频位姿，100Hz）
    // 注：kf_imu_ 与 kf_ 并行运行，仅接收 IMU，不接收 LiDAR
};
```

**[VERIFY: src/core/lio/eskf.hpp:65-120]** — 关键成员声明  
**[VERIFY: src/core/lio/laser_mapping.h:58-65]** — `kf_` 和 `kf_imu_` 声明

### 3.4 预测与更新接口

```cpp
// 预测：IMU 积分
//   imu: 含加速度、角速度、时间戳
void Predict(const IMU& imu);

// 更新：根据观测类型调用不同的观测函数
//   obs_type: 观测类型
void Update(ObsType obs_type);

// 直接设置状态（初始化或外部修正用）
void SetState(const NavState& state, const MatXD& g_cov);
```

---

## 四、LaserMapping（LIO 主处理类）

**文件**：`src/core/lio/laser_mapping.h`, `laser_mapping.cc`

### 4.1 双地图架构

```
LaserMapping
├── IVoxManager ivox_manager_     # iVox 增量式哈希体素地图
│   ├── 功能：kNN 搜索（快速，无协方差）
│   └── 用于 ObsModel 中构建点面约束（最近5点平均法线）
│
└── VoxelMapManager voxel_map_    # VoxelMap 协方差地图
    ├── 功能：存储平面协方差（p2l 精度更高）
    └── 用于 ObsModel 中直接使用预存协方差
```

**[VERIFY: src/core/lio/laser_mapping.h:54-60]** — 双地图成员声明

**切换方式**（可独立启用，也可双开）：

```yaml
fasterlio:
  use_ivox: true          # 启用 iVox
  use_voxel_map: false    # 不启用 VoxelMap（典型配置）
```

### 4.2 类型别名

```cpp
using PointType       = pcl::PointXYZINormal;  // 含法向量（用于平面法线存储）
using CloudType       = pcl::PointCloud<PointType>;
using CloudPtr        = CloudType::Ptr;
using SE3             = Sophus::SE3d;
using V3D             = Eigen::Vector3d;
using M3D             = Eigen::Matrix3d;
```

**[VERIFY: src/common/point_def.h]** — PointType 定义

### 4.3 SyncPackages 逻辑（时间对齐）

```
lidar_buffer_   ──►  取队首 LiDAR 帧（时间戳 t_lidar）
                       │
imu_buffer_     ──►  取 t < t_lidar + 0.5/freq 的所有 IMU 包
                       │
                  ──►  构成 MeasureGroup { imu_deque, lidar }
                       │
                  ──►  送入 ImuProcess::Process()
```

**[VERIFY: src/core/lio/laser_mapping.cc]** — `SyncPackages()` 实现

### 4.4 关键状态变量

```cpp
NavState  state_point_;           // 当前最优状态（IEKF 输出）
SE3       lidar_T_wrt_IMU_;       // LiDAR-IMU 外参（SE3）

bool      flg_first_scan_ = true;  // 首帧初始化标志
bool      keep_first_imu_estimation_; // 是否保留 IMU-only 估计（首帧）

// 点对平面残差（IEKF 每步更新）
std::vector<PointVector>  nearest_points_;  // iVox kNN 搜索结果
std::vector<M3D>          body_cov_list_;   // VoxelMap 协方差
```

---

## 五、LoopClosing（回环检测后端）

**文件**：`src/core/loop_closing/loop_closing.h`, `loop_closing.cc`

### 5.1 候选检测数据结构

```cpp
// 历史关键帧位置点云（用于 KD-Tree 搜索）
pcl::PointCloud<pcl::PointXYZ>::Ptr kf_poses_cloud_;
pcl::KdTreeFLANN<pcl::PointXYZ>     kdtree_history_poses_;

// 回环记录（防止重复检测同一帧）
std::map<int, int>  loop_index_container_;   // key: kf_cur->id_, val: kf_prev->id_

// 冷却期计数
int last_loop_kf_id_ = -1;  // 上次触发回环的关键帧 ID
```

**[VERIFY: src/core/loop_closing/loop_closing.h:70-110]** — 数据结构声明

### 5.2 回环因子队列（三队列解耦）

```cpp
// PGO 消费的回环因子（AddLoopFactor 写入，PoseOptimization 消费）
std::vector<int>   loop_pairs_idx_;        // [kf_cur_id, kf_prev_id, ...]，成对
std::vector<SE3>   loop_pairs_pose_;       // 对应的 Tij 约束
std::vector<float> loop_pairs_score_;      // NDT 得分（用于自适应噪声模型）
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:380-400]** — 三队列写入

### 5.3 miao 优化器配置

```cpp
miao::OptimizerConfig config;
config.incremental_mode_ = true;    // 增量模式（新关键帧直接追加）
config.solver_type_ = LM;           // Levenberg-Marquardt
config.max_iter_ = 10;
config.epsilon1_ = 1e-5;
config.epsilon2_ = 1e-5;
config.linear_solver_ = SPARSE_CHOLESKY;

miao::Optimizer optimizer_(config);
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:35-43]** — 初始化代码

### 5.4 噪声模型

```cpp
// 里程计因子噪声（单位：m²/rad²，对角阵）
MatXD odom_noise_ = (V6D << 0.1, 0.1, 0.1, 3*DEG2RAD, 3*DEG2RAD, 3*DEG2RAD)
                        .finished().asDiagonal();

// 回环因子噪声（随 NDT 得分自适应）
float  loop_noise_scale = 0.2 + icp_score * 0.1;  // 得分越高噪声越小
MatXD loop_noise_ = ... * loop_noise_scale;

// Cauchy 鲁棒核参数
double rk_loop_th_ = 5.2;   // 针对平移
double rk_loop_r_th_ = 5.0; // 针对旋转
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:470-510]** — 噪声模型计算

---

## 六、Localization（定位顶层类）

**文件**：`src/core/localization/localization.h`, `localization.cpp`

### 6.1 成员结构

```cpp
class Localization {
    // 前端（LIO，非SLAM模式，不存关键帧）
    std::shared_ptr<LaserMapping> lio_;
    
    // 全局匹配（分块地图 NDT/ICP）
    std::shared_ptr<LidarLoc>     lidar_loc_;
    
    // 位姿图（融合 LIO + 全局匹配）
    std::shared_ptr<PGO>          pgo_;
    
    // 高频 TF 外推（IMU 频率，100Hz）
    // pgo_ 内部持有 PoseExtrapolator
    
    // 异步消息处理队列（解耦 ROS 接收与算法处理）
    AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;   // LIO 通道
    AsyncMessageProcess<CloudPtr> lidar_loc_proc_cloud_;    // 全局定位通道
    
    // 跳帧参数（控制全局定位频率）
    int lidar_loc_skip_num_;   // 每 N 帧触发一次全局定位
};
```

**[VERIFY: src/core/localization/localization.h:55-110]** — 成员声明

### 6.2 PoseExtrapolator（高频位姿外推）

**文件**：`src/core/localization/pose_graph/pose_extrapolator.h`

```cpp
class PoseExtrapolator {
    // 维护一个 SE3 队列（LIO/全局定位结果）
    std::deque<TimedPose> pose_queue_;
    
    // 最新 IMU
    std::deque<IMU> imu_queue_;
    
    // 最新全局位姿（用于约束漂移）
    SE3 last_global_pose_;
    
public:
    // 添加里程计位姿（低频，10Hz）
    void AddDRLoc(const TimedSE3& odom_pose);
    
    // 添加全局位姿（超低频，2-5Hz）
    void AddGlobalLoc(const TimedSE3& global_pose);
    
    // IMU 驱动外推（高频调用，100Hz）
    SE3 ExtrapolateByIMU(double timestamp) const;
    
    // 计算并发布 TF（每个 IMU 触发一次）
    void AddDRLocAndExtrapolate(const IMU& imu);
};
```

**[VERIFY: src/core/localization/pose_graph/pose_extrapolator.h]** — 方法签名

---

## 七、TiledMap（分块点云地图）

**文件**：`src/core/maps/tiled_map.h`, `tiled_map.cc`

### 7.1 存储结构

```
TiledMap
├── chunk_size_ = 100.0m    # 每块边长
├── static_chunks_           # 从磁盘加载的静态地图块（HashMap）
│   └── key: TileIndex {ix, iy}  → TiledMapChunk(static points)
├── dynamic_chunks_          # 运行时动态建图块
│   └── key: TileIndex       → TiledMapChunk(dynamic points)
└── ivox_locmap_             # 专为定位维护的增量 IVox（可选）
    └── 聚合 nearby_chunks 的点云导入
```

**[VERIFY: src/core/maps/tiled_map.h:30-80]** — 主要数据成员

### 7.2 分块索引

```cpp
TileIndex GetTileIndex(const V3D& pos) const {
    return { static_cast<int>(std::floor(pos.x() / chunk_size_)),
             static_cast<int>(std::floor(pos.y() / chunk_size_)) };
}
```

### 7.3 邻域加载（扫描范围内块）

```cpp
// 当机器人移动时，动态加载附近 N×N 块（默认 8 块）
// 超出范围的块卸载释放内存
void LoadNearbyChunks(const V3D& current_pos);
```

**[VERIFY: src/core/maps/tiled_map.cc:90-130]** — 邻域加载逻辑

---

## 八、G2P5（2.5D 栅格地图）

**文件**：`src/core/g2p5/g2p5.h`, `g2p5.cc`

### 8.1 数据结构

```cpp
class G2P5 {
    // 占据栅格（分辨率 0.05-0.1m，可配置）
    G2P5Map  global_map_;       // 全局2D栅格
    G2P5Map  local_map_;        // 局部增量区域（前端用）
    
    // 渲染参数
    float  grid_resolution_;   // 默认0.05m（室内）
    float  min_th_floor_;      // 地面过滤下界（相对 LiDAR 高度，m）
    float  max_th_floor_;      // 地面过滤上界（默认1.2m）
    bool   esti_floor_;        // 是否动态估计地面高度
    
    // 异步后端重绘线程
    AsyncMessageProcess<CloudPtr> redraw_thread_;
};
```

**[VERIFY: src/core/g2p5/g2p5.h:25-80]** — 数据结构定义

### 8.2 射线追踪原理

```
对每个关键帧点云：
  for each point p:
      ray: lidar_origin → p
      if min_th_floor_ < p.z < max_th_floor_:
          ray casting: 沿 ray 方向的中间栅格设为 free
          endpoint 设为 occupied
```

- 假设 LiDAR 水平安装（`lidar_origin.z` 为固定高度）
- 地板和天花板点被过滤，只处理障碍物高度范围

---

## 九、miao 优化库（轻量 PGO）

**文件**：`src/core/miao/`

### 9.1 核心概念

| 概念 | 对应 g2o 概念 | 说明 |
|------|---------------|------|
| `Vertex` | `BaseVertex` | 优化变量（SE3 位姿） |
| `Edge` | `BaseEdge` | 约束（里程计 / 回环 / 先验） |
| `Optimizer` | `SparseOptimizer` | 求解器（LM + sparse Cholesky） |
| `incremental_mode_` | — | 新节点追加时直接热启动 |

### 9.2 使用模式（回环模块中）

```cpp
// 建图时，每个关键帧加入：
optimizer_.AddVertex(VertexSE3, kf->pose_opt_);
optimizer_.AddEdge(EdgeSE3, kf_prev->id_, kf->id_, relative_pose, odom_noise_);

// 回环触发时，加入：
optimizer_.AddEdge(EdgeSE3, kf_cur->id_, kf_prev->id_, Tij, loop_noise_, cauchy_kernel);

// 固定第一帧（先验约束，防止全局漂移）
optimizer_.Fix(0);  // fix_first_node_ = true

// 步进优化
optimizer_.Optimize(max_iter_);
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:540-600]** — `AddOdomFactor` / `PoseOptimization` 实现

---

## 十、AsyncMessageProcess（异步消息处理模板）

**文件**：`src/common/async_message_process.h`（或 `wrapper/`）

### 10.1 接口

```cpp
template<typename MsgType>
class AsyncMessageProcess {
    std::deque<MsgType>  queue_;
    std::mutex           mutex_;
    std::thread          proc_thread_;
    
public:
    // 注册处理函数
    void SetProcFunc(std::function<void(const MsgType&)> func);
    
    // 启动后台线程
    void Start();
    
    // 添加消息（线程安全）
    void AddMessage(const MsgType& msg);
    
    // 跳帧参数
    int skip_num_ = 0;   // 每 skip_num_+1 帧处理一次
};
```

**[VERIFY: src/core/loop_closing/loop_closing.cc:131-145]** — `LoopClosing::Init()` 中的 `AsyncMessageProcess` 使用

---

## 附录：关键类型映射

| 代码中的别名 | 含义 |
|-------------|------|
| `SE3` | `Sophus::SE3d`（4×4 刚体变换双精度） |
| `SO3` | `Sophus::SO3d`（旋转矩阵双精度） |
| `V3D` | `Eigen::Vector3d` |
| `M3D` | `Eigen::Matrix3d` |
| `VXD` | `Eigen::VectorXd`（动态维度） |
| `MatXD` | `Eigen::MatrixXd`（动态尺寸矩阵） |
| `V6D` | `Eigen::Matrix<double,6,1>`（6维向量） |
| `CloudPtr` | `pcl::PointCloud<PointType>::Ptr` |
| `PointType` | `pcl::PointXYZINormal` |
| `TimedSE3` | `std::pair<double, SE3>`（时间戳+位姿） |
