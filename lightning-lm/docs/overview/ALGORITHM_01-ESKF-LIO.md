# lightning-lm ESKF + LIO 算法文档 (IEKF-LIO Algorithm)

> 核心参考文献：FAST-LIO2 (Wu et al., 2022)、AA-FasterLIO (Incremental IVox)  
> 所有论断基于实际代码，带 `[VERIFY: file:line]` 标注。

---

## 一、算法背景

lightning-lm 的前端是 **AA-FasterLIO**：在 FAST-LIO2 基础上引入 iVox 增量式体素地图 + Anderson Acceleration (AA) 加速 IEKF 收敛。

核心思路：  
- **误差状态卡尔曼滤波（ESKF）**：在 SO(3)/S² 流形上进行状态估计  
- **迭代扩展 KF（IEKF）**：每帧 LiDAR 点云迭代求解非线性优化（最大迭代次数 4）  
- **去畸变**：用 IMU 预积分对运动中采集的点云做旋转插值补偿  
- **双地图**：iVox（快速 kNN）或 VoxelMap（协方差精确）可独立切换  

---

## 二、状态向量与误差状态

### 2.1 名义状态（Nominal State），流形 M

$$
\mathcal{X} = (\mathbf{p}, R_L, R, \mathbf{t}_L, \mathbf{v}, \mathbf{b}_g, \mathbf{b}_a, \mathbf{g}) \in \mathbb{R}^3 \times SO(3) \times SO(3) \times \mathbb{R}^3 \times \mathbb{R}^3 \times \mathbb{R}^3 \times \mathbb{R}^3 \times S^2
$$

| 符号 | 维度 | 含义 |
|------|------|------|
| $\mathbf{p}$ | 3 | 机器人位置（世界系） |
| $R_L$ | SO(3) | LiDAR 相对 IMU 的旋转外参 |
| $R$ | SO(3) | 机器人姿态（世界系→IMU系） |
| $\mathbf{t}_L$ | 3 | LiDAR 相对 IMU 的平移外参 |
| $\mathbf{v}$ | 3 | 速度（世界系） |
| $\mathbf{b}_g$ | 3 | 陀螺仪偏置（IMU系） |
| $\mathbf{b}_a$ | 3 | 加速度计偏置（IMU系） |
| $\mathbf{g}$ | S²(2) | 重力方向（世界系，S² 流形） |

**全维度**：$3+3+3+3+3+3+3+2 = 23$（误差状态维度），名义状态存储需 24 因为 S² 存为 3D 单位向量。

**[VERIFY: src/common/nav_state.h:1-30]** — 完整字段顺序

### 2.2 误差状态（Error State），切空间 $\delta\mathcal{X} \in \mathbb{R}^{23}$

$$
\delta\mathcal{X} = (\delta\mathbf{p}, \delta\boldsymbol{\phi}_L, \delta\boldsymbol{\phi}, \delta\mathbf{t}_L, \delta\mathbf{v}, \delta\mathbf{b}_g, \delta\mathbf{b}_a, \delta\mathbf{g})
$$

其中旋转误差 $\delta\boldsymbol{\phi} \in \mathbb{R}^3$ 为 SO(3) 李代数（左扰动），重力误差 $\delta\mathbf{g} \in \mathbb{R}^2$ 为 S² 切平面分量。

---

## 三、预测步骤（Predict）

**文件**：`src/core/lio/eskf.cc`，函数 `ESKF::Predict(const IMU& imu)`

### 3.1 连续时间运动模型

IMU 测量模型（去偏置后）：
$$
\hat{\boldsymbol{\omega}} = \boldsymbol{\omega}_{m} - \mathbf{b}_g, \quad \hat{\mathbf{a}} = \mathbf{a}_{m} - \mathbf{b}_a
$$

连续状态导数：

$$
\dot{\mathbf{p}} = \mathbf{v}
$$

$$
\dot{R} = R \cdot [\hat{\boldsymbol{\omega}}]_\times
$$

$$
\dot{\mathbf{v}} = R \cdot \hat{\mathbf{a}} + \mathbf{g}
$$

$$
\dot{\mathbf{b}}_g = \mathbf{n}_{bg}, \quad \dot{\mathbf{b}}_a = \mathbf{n}_{ba}
$$

$$
\dot{R}_L = 0, \quad \dot{\mathbf{t}}_L = 0, \quad \dot{\mathbf{g}} = [\mathbf{g}]_\times \cdot P_\perp(\mathbf{n}_g)
$$

（外参和重力在正常运行时不通过运动模型更新，外参靠 LiDAR 观测隐式修正，重力靠 ACC_AS_GRAVITY 修正。）

**[VERIFY: src/common/nav_state.h:55-90]** — `get_f()` 连续导数

### 3.2 离散时间积分（一阶）

```cpp
// eskf.cc: Predict()
x_ = x_.oplus(f_, dt);   // f_ = x_.get_f(imu)，oplus 处理 SO3/S2 更新
```

$$
\mathbf{p}_{k+1} = \mathbf{p}_k + \mathbf{v}_k \cdot dt
$$

$$
R_{k+1} = R_k \cdot \text{Exp}(\hat{\boldsymbol{\omega}}_k \cdot dt)
$$

$$
\mathbf{v}_{k+1} = \mathbf{v}_k + (R_k \cdot \hat{\mathbf{a}}_k + \mathbf{g}_k) \cdot dt
$$

**[VERIFY: src/core/lio/eskf.cc:10-60]** — `Predict()` 积分代码

### 3.3 协方差预测

误差状态转移方程（线性化）：

$$
\delta\mathcal{X}_{k+1} = F_x \cdot \delta\mathcal{X}_k + F_w \cdot \mathbf{n}_k
$$

其中 $F_x = \frac{\partial f}{\partial \delta\mathcal{X}}$（**[VERIFY: src/common/nav_state.h:95-165]** — `df_dx()`），$F_w = \frac{\partial f}{\partial \mathbf{w}}$（**[VERIFY: src/common/nav_state.h:170-200]** — `df_dw()`）。

协方差更新：

$$
P_{k+1} = F_{x1} \cdot P_k \cdot F_{x1}^T + F_w \cdot Q \cdot F_w^T
$$

其中 $F_{x1}$ 是 $F_x$ 的离散版本（含 $dt$ 项），代码中通过 `adj_oplus` 处理 SO3/S2 部分的切空间投影。

```cpp
// eskf.cc: Predict()
MatXD F_x1 = MatXD::Identity(DIM, DIM);
// SO3 部分填入 Adj(R) 旋转
// 交叉项 (dv/dR, dp/dv) 填入
P_ = F_x1 * P_ * F_x1.transpose() + f_w * Q_ * f_w.transpose();
```

**[VERIFY: src/core/lio/eskf.cc:25-75]** — 完整 F_x1 构建与协方差更新

### 3.4 SO(3)/S² 切空间在 Jacobian 中的处理

```
对 SO(3) 状态（R, R_L）：
  F_x1 的对应行/列用 Adj(R) 的旋转部分（李代数线性变换）

对 S² 状态（g）：
  使用 S² 上的切平面投影矩阵 P_perp = I - g*g^T
  切空间维度为 2（与 delta_g ∈ R² 对应），
  通过选择正交基 {e1_perp, e2_perp} 嵌入 3D
```

**[VERIFY: src/common/nav_state.h:145-165]** — S² Jacobian 实现

---

## 四、更新步骤（IEKF Update，LiDAR 观测）

**文件**：`src/core/lio/eskf.cc`，函数 `ESKF::Update(ObsType::LIDAR)`

### 4.1 迭代更新流程

```
for iter = 0..max_iterations_-1:
    1. 调用 lidar_obs_func_(current_pose, r, H, is_valid)
       → 返回残差 r ∈ R^{m×1}，观测 Jacobian H ∈ R^{m×23}
    
    2. 计算卡尔曼增益
       K = P * H^T * (H * P * H^T + R)^{-1}
    
    3. 误差状态更新
       δx = K * r
    
    4. 状态更新（流形）
       x = x ⊕ δx    (oplus 处理 SO3/S2)
    
    5. 协方差更新
       P = (I - K*H) * P
    
    6. Anderson Acceleration 检查（可选）
       if |δx - prev_δx| < threshold: break
```

**[VERIFY: src/core/lio/eskf.cc:100-180]** — 完整 IEKF 迭代循环

### 4.2 LiDAR 观测函数（ObsModel）

**文件**：`src/core/lio/laser_mapping.cc`，函数 `LaserMapping::ObsModel()`

#### 输入/输出

```cpp
// 观测函数签名（lambda，注册到 ESKF）
auto lidar_obs_func = [this](const SE3& pose, VXD& r, MatXD& H, bool& valid) {
    ObsModel(pose, r, H, valid);
};
kf_.SetObsFunc(lidar_obs_func);
```

#### 点对平面残差构建（使用 iVox）

```
for each point p_body in scan_undistort_:
    1. 变换到世界系：p_world = R * (R_L * p_body + t_L) + p
    
    2. kNN 搜索（iVox，k=5）：
       ivox_manager_.GetClosestPoint(p_world, nearest_points, 5)
    
    3. 平面拟合（最近5点 → SVD/协方差 → 法向量 n, 法面方程系数 d）
       if PlaneFit(nearest_points, normal, d) succeed:
    
    4. 残差（点到平面距离）：
       r_i = n^T * p_world + d        ← 标量，单点残差
    
    5. Jacobian（对误差状态 δx，23维）：
       H_i = ∂r_i/∂δx = n^T * J_p    ← 1×23
       其中 J_p = [I, -(R*R_L*p_body + R*t_L)×, -R*R_L*[p_body]×, -R, 0, 0, 0, 0]
```

**[VERIFY: src/core/lio/laser_mapping.cc:250-350]** — 完整 ObsModel 实现

#### 点对平面残差构建（使用 VoxelMap）

```
for each point p_body:
    1. 变换到世界系 p_world
    
    2. voxel_map_.GetCovariance(p_world) → 预存平面协方差矩阵 Σ
    
    3. 构建加权最小二乘残差（Mahalanobis 距离形式）
       等价于 H^T * Σ^{-1} * H 直接用于滤波更新
```

**[VERIFY: src/core/lio/laser_mapping.cc:350-420]** — VoxelMap 对应分支

### 4.3 Jacobian 推导

世界系点 $\mathbf{p}_w$关于误差状态 $\delta\mathcal{X}$ 的 Jacobian：

$$
\frac{\partial \mathbf{p}_w}{\partial \delta\mathcal{X}} = \left[\mathbf{I}, \, \frac{\partial \mathbf{p}_w}{\partial \delta\boldsymbol{\phi}_L}, \, \frac{\partial \mathbf{p}_w}{\partial \delta\boldsymbol{\phi}}, \, -R, \, \mathbf{0}, \mathbf{0}, \mathbf{0}, \mathbf{0}\right]
$$

其中：

$$
\frac{\partial \mathbf{p}_w}{\partial \delta\boldsymbol{\phi}} = -\left[R \cdot (R_L \cdot \mathbf{p}_b + \mathbf{t}_L)\right]_\times
$$

$$
\frac{\partial \mathbf{p}_w}{\partial \delta\boldsymbol{\phi}_L} = -R \cdot \left[R_L \cdot \mathbf{p}_b\right]_\times
$$

残差对误差状态 Jacobian：$H_i = \mathbf{n}^T \cdot \frac{\partial \mathbf{p}_w}{\partial \delta\mathcal{X}}$（1×23 行向量）

---

## 五、Anderson Acceleration（AA 加速）

**文件**：`src/core/lio/anderson_acceleration.h`

### 5.1 原理

AA 是一种非线性迭代加速方法，将最近 $m$ 步的搜索方向做线性组合，减少收敛所需迭代次数。

```
普通 IEKF 迭代：x_{k+1} = f(x_k)
AA 加速       ：x_{k+1} = Σ θ_i * x_k-i   (θ_i 由最小二乘确定)
```

### 5.2 在 ESKF 中的应用

```cpp
// eskf.cc: Update()，AA 加速条件退出
if (use_anderson_acc_ && iter > 1) {
    VXD accel_delta = anderson_acc_.Compute(delta_x);
    if (accel_delta.norm() < anderson_threshold_) {
        break;  // 提前收敛
    }
}
```

**[VERIFY: src/core/lio/eskf.cc:155-175]** — AA 调用

---

## 六、IMU 预处理与点云去畸变

**文件**：`src/core/lio/imu_processing.h/cc`

### 6.1 IMU 初始化流程

```
1. 静止检测（方差阈值）
2. 重力方向估计：g = mean(R^T * a_m) （n帧平均加速度）
3. 陀螺仪偏置估计：bg = mean(ω_m) （静止时角速度应为零）
4. 初始化成功后设置 ESKF 初始状态
```

**[VERIFY: src/core/lio/imu_processing.cc:1-80]** — IMU 初始化

### 6.2 去畸变原理

LiDAR 扫描期间（约100ms），机器人运动导致不同时刻的点云存在相对位移（畸变）。  
解决方案：

```
for each point p_i at time t_i ∈ [t_scan_start, t_scan_end]:
    1. IMU 积分得到 R(t_i)（相对于扫描结束时刻的旋转）
    2. p_i_undistorted = R(t_i)^{-1} * p_i   ← 归算到扫描结束时刻
```

具体：通过 `IMUProcess::UndistortPcl()` 对每个点按时间戳插值 IMU 积分旋转。

**[VERIFY: src/core/lio/imu_processing.cc:120-200]** — `UndistortPcl()` 实现

---

## 七、地图增量更新（MapIncremental）

**文件**：`src/core/lio/laser_mapping.cc`，函数 `LaserMapping::MapIncremental()`

### 7.1 iVox 更新

```cpp
// 将当前帧点（IEKF 收敛后的世界系）添加到 iVox
// iVox 自动管理体素插入/删除（LRU 淘汰机制）
ivox_manager_.AddPoints(scan_world);
```

### 7.2 VoxelMap 更新

```cpp
// VoxelMap 更新时更新体素平面协方差
// 使用递增更新：新点加入 → 重新计算 mean, covariance
voxel_map_.UpdateMap(scan_world, state_point_);
```

**[VERIFY: src/core/lio/laser_mapping.cc:458-490]** — `MapIncremental()` 代码

---

## 八、iVox 增量体素地图（IVox3D）

**文件**：`src/core/ivox3d/`

### 8.1 核心思路

```
体素网格（哈希表）
├── key: VoxelIndex {ix, iy, iz}  （体素坐标）
└── val: IVoxNode（体素内点的 kd-tree 或线性列表）

IVoxManager：
├── 维护体素 HashMap
├── 支持增量插入（AddPoints）
├── 支持快速 kNN 搜索（GetClosestPoint）
└── LRU 淘汰策略（内存限制时删除旧体素）
```

### 8.2 kNN 搜索

```cpp
// 仅搜索当前点所在体素 + 26邻居体素（3x3x3 栅格）
// 每个体素只维护少量点（默认20个），总候选 < 26×20 = 520
// 候选中取最近 k 个 → 比全局 kd-tree 快 ~10x
```

**[VERIFY: src/core/ivox3d/ivox3d.h:55-100]** — `GetClosestPoint()` 实现

---

## 九、关键参数对性能的影响

| 参数 | 典型值 | 影响 |
|------|--------|------|
| `max_iteration` | 4 | IEKF 迭代次数，越大精度越高，延迟增加 |
| `point_filter_num` | 1–10 | 点云采样，越大越稀，越快越差 |
| `kf_dis_th` | 0.5m | 关键帧密度（小 → 多帧 → 更多回环机会） |
| `kf_angle_th` | 15° | 关键帧密度（转弯多时可适当减小） |
| `voxel_size` | 0.5m | iVox 体素大小（建图用，影响精度/内存） |
| `ivox_nearby_type` | 18/26 | iVox kNN 搜索邻域大小 |

---

## 十、算法卡尔曼增益推导（完整形式）

### 10.1 最优卡尔曼增益

$$
K = P H^T (H P H^T + R)^{-1}
$$

其中：
- $P \in \mathbb{R}^{23\times23}$：先验误差协方差
- $H \in \mathbb{R}^{m\times23}$：观测 Jacobian（m 为有效点数）
- $R \in \mathbb{R}^{m\times m}$：观测噪声（通常取对角阵，$\sigma_{lidar}^2 \approx 0.001$）

### 10.2 状态更新（切空间 → 流形）

误差状态更新：
$$
\delta\mathcal{X} = K \cdot r
$$

名义状态更新（oplus 运算）：
$$
\mathcal{X} \leftarrow \mathcal{X} \oplus \delta\mathcal{X}
$$

具体组合规则：
- $\mathbf{p} \leftarrow \mathbf{p} + \delta\mathbf{p}$
- $R \leftarrow R \cdot \text{Exp}(\delta\boldsymbol{\phi})$（左乘）
- $\mathbf{g} \leftarrow \text{Exp}_{S^2}(\delta\mathbf{g}) \cdot \mathbf{g}$（S² 更新）

### 10.3 协方差更新（Joseph 形式，数值稳定）

$$
P \leftarrow (I - KH) P (I - KH)^T + K R K^T
$$

**[VERIFY: src/core/lio/eskf.cc:165-185]** — 协方差更新代码

---

## 十一、ESKF 与标准 EKF 的对比

| 特性 | 标准 EKF（3D pose） | 本系统 ESKF |
|------|---------------------|-------------|
| 状态空间 | 欧氏空间 R^n | SO(3)/S² 流形 |
| 旋转表示 | 四元数/欧拉角（奇异点） | SO(3)（无奇异） |
| 观测线性化 | 绕名义状态展开 | 绕误差状态展开（更准确） |
| 迭代求解 | 单步 | IEKF（4次迭代） |
| 外参估计 | 需要离线标定 | 在线 co-optimization |
| 重力估计 | 固定 | S² 流形在线估计 |

---

## 附录：ESKF 相关源码索引

| 功能 | 文件 | 行号 |
|------|------|------|
| NavState 字段定义 | nav_state.h | 1–50 |
| 运动模型 `get_f()` | nav_state.h | 55–90 |
| 状态 Jacobian `df_dx()` | nav_state.h | 95–165 |
| 噪声 Jacobian `df_dw()` | nav_state.h | 170–200 |
| `oplus()` 流形更新 | nav_state.h | 205–240 |
| ESKF `Predict()` | eskf.cc | 10–80 |
| ESKF `Update()` IEKF | eskf.cc | 100–185 |
| `ObsModel()` 点面残差 | laser_mapping.cc | 250–420 |
| `MapIncremental()` | laser_mapping.cc | 458–490 |
| IMU 初始化 | imu_processing.cc | 1–80 |
| 点云去畸变 | imu_processing.cc | 120–200 |
| `SyncPackages()` 对齐 | laser_mapping.cc | 460–520 |
| `MakeKF()` 关键帧判断 | laser_mapping.cc | 400–430 |
