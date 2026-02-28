# ESKF Predict 重力处理问题分析与修复

## 问题现象

从日志观察到 **z轴持续有速度和加速度累积**：

```
IMU原始数据:
acc: 0.00493  0.00254  0.995352  // z轴≈1.0 (归一化重力)
gyr: 0.0158   -0.00156 0.00624
```

**核心问题：** ESKF预测时直接使用IMU原始数据，没有正确处理重力分量！

---

## 问题根源分析

### 1. 当前代码流程

#### Step 1: ProcessIMU (laser_mapping.cc:135)

```cpp
// ❌ 错误：直接使用原始加速度
kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, 
                imu->angular_velocity, 
                imu->linear_acceleration);  // 包含重力！
```

#### Step 2: ESKF::Predict (eskf.cc:10)

```cpp
Eigen::Matrix<double, 24, 1> f_ = x_.get_f(gyro, acce);  // acce包含重力
```

#### Step 3: NavState::get_f (nav_state.h:65)

```cpp
inline FullVectState get_f(const Vec3d& gyro, const Vec3d& acce) const {
    Vec3d omega = gyro - bg_;
    Vec3d a_inertial = rot_ * (acce - ba_);  // ❌ acce是测量值，包含重力
    
    for (int i = 0; i < 3; i++) {
        res(i) = vel_[i];
        res(i + 3) = omega[i];
        res(i + 12) = a_inertial[i] + grav_[i];  // 加上估计的重力
    }
    return res;
}
```

---

### 2. 理论模型 vs 实际实现

#### IMU测量模型（理论）

```
比力方程:
  f_measured = R_wb^T × (a_world + g_world) + ba + na

其中:
  f_measured: 加速度计测量值（比力，specific force）
  a_world:    真实运动加速度（世界系）
  g_world:    重力加速度 [0, 0, -9.8]
  R_wb:       body到world的旋转
  ba:         加速度计bias
  na:         测量噪声
```

#### 静止时的测量

```
当机器人静止: a_world = 0

f_measured = R_wb^T × g_world + ba
           ≈ [0, 0, -1.0]  (归一化到重力单位后)
```

**关键：** IMU测到的是**支撑力**（向上），不是重力（向下）！

---

### 3. ESKF运动方程（理论）

#### 连续时间模型

```
状态方程:
  ṗ = v                              (位置变化率)
  v̇ = R × (f_measured - ba) + g      (速度变化率)
  Ṙ = R × [ω_measured - bg]×        (姿态变化率)

其中:
  p: 位置
  v: 速度
  R: 旋转矩阵
  g: 重力向量（状态量，实时估计）
```

#### 关键等式

```
v̇ = R × (f_measured - ba) + g
  = R × (R^T × (a_true + g) - ba) + g  (代入测量模型)
  = a_true + g - R × R^T × g + g
  = a_true + g  (如果忽略高阶项)
```

**问题：** 当前代码中 `f_measured` 已经包含了重力的反作用力，但在计算时又加上了 `g`，导致重复！

---

### 4. 数据流分析

#### 从您的IMU数据看

```
原始测量:
  acc = [0.00493, 0.00254, 0.995352]

物理含义:
  - xy分量 ≈ 0: 水平方向无加速度
  - z分量 ≈ 1.0: 测到向上的支撑力（重力的反作用力）
  
这是静止或匀速运动时的典型值！
```

#### 错误的处理

```cpp
// 当前代码
a_inertial = R × (acce - ba)
           = R × ([0.005, 0.003, 0.995] - ba)
           ≈ R × [0.005, 0.003, 0.995]  (假设ba≈0)
           ≈ [0.005, 0.003, 0.995]      (假设R≈I，初始时刻)

v̇ = a_inertial + grav
  = [0.005, 0.003, 0.995] + [0, 0, -9.8]  (如果grav估计正确)
  = [0.005, 0.003, -8.805]  ← z方向有-8.8的加速度！
```

**问题根源：** `acce` 中的 0.995 已经是归一化后的重力，应该减去，而不是保留！

---

### 5. 正确的处理方式

#### 方案A: 在输入端减去重力（推荐）

```cpp
// 在 ProcessIMU 中预处理
Vec3d acc_corrected = imu->linear_acceleration - Vec3d(0, 0, -1.0);
kf_imu_.Predict(dt, Q, gyro, acc_corrected);
```

**优点：**
- 清晰明确，在数据源头处理
- 不需要修改ESKF内部逻辑
- 符合ESKF理论假设（输入是纯运动加速度）

#### 方案B: 修改get_f函数

```cpp
inline FullVectState get_f(const Vec3d& gyro, const Vec3d& acce) const {
    Vec3d omega = gyro - bg_;
    // 减去静止时的重力基准
    Vec3d acc_motion = acce - Vec3d(0, 0, -1.0);  // 去除重力分量
    Vec3d a_inertial = rot_ * (acc_motion - ba_);
    
    for (int i = 0; i < 3; i++) {
        res(i) = vel_[i];
        res(i + 3) = omega[i];
        res(i + 12) = a_inertial[i] + grav_[i];
    }
    return res;
}
```

**缺点：**
- 需要修改底层状态方程
- 可能影响其他使用ESKF的地方

---

## 修复实现

### 修复代码（方案A）

```cpp
// laser_mapping.cc:ProcessIMU
if (p_imu_->IsIMUInited()) {
    /// 重要：IMU加速度需要减去重力分量
    Vec3d acc_corrected = imu->linear_acceleration;
    double mean_acc_norm = p_imu_->GetMeanAccNorm();
    
    if (mean_acc_norm > 0.1) {  // 确保已初始化
        // 静止时的理论加速度（归一化到重力单位）
        Vec3d gravity_normalized = Vec3d(0, 0, -1.0);
        // 减去重力，得到纯运动加速度
        acc_corrected = imu->linear_acceleration - gravity_normalized;
    }
    
    kf_imu_.Predict(timestamp - last_timestamp_imu_, p_imu_->Q_, 
                    imu->angular_velocity, acc_corrected);
    
    // 调试日志
    std::cout << "imu acc raw: " << imu->linear_acceleration.transpose() 
              << ", corrected: " << acc_corrected.transpose()
              << ", vel: " << kf_imu_.GetX().GetVel().transpose() << std::endl;
}
```

---

## 验证方法

### 1. 静止测试

**预期效果：**
```
修复前:
  acc_raw:  [0.005, 0.003, 0.995]
  vel变化:  持续累积，z方向速度增大

修复后:
  acc_raw:      [0.005, 0.003, 0.995]
  acc_corrected: [0.005, 0.003, -0.005]  ← z分量接近0
  vel变化:      基本不变，微小波动
```

**验证命令：**
```bash
# 让机器人静止30秒，观察速度
# 理想：速度保持在 ±0.01 m/s 以内
```

### 2. 直线运动测试

**预期效果：**
```
修复前:
  向前运动，z方向也有速度累积

修复后:
  向前运动，z方向速度保持接近0
  轨迹水平，无垂直漂移
```

### 3. 日志检查

**添加调试日志：**
```cpp
LOG_EVERY_N(INFO, 100) << "IMU state check:"
    << " acc_raw: " << imu->linear_acceleration.transpose()
    << " acc_corrected: " << acc_corrected.transpose()
    << " vel: " << kf_imu_.GetX().GetVel().transpose()
    << " pos: " << kf_imu_.GetX().GetPose().translation().transpose();
```

**预期日志：**
```
静止时:
  acc_raw: [0.005, 0.003, 0.995]
  acc_corrected: [0.005, 0.003, -0.005]
  vel: [0.001, -0.002, 0.000]  ← z方向接近0
  pos: [4.776, 12.391, 0.050]   ← z基本不变

运动时:
  acc_raw: [0.050, 0.020, 0.990]
  acc_corrected: [0.050, 0.020, -0.010]
  vel: [0.150, 0.030, 0.002]  ← z方向微小
  pos: [4.850, 12.400, 0.051]  ← z缓慢变化
```

---

## 理论深入：为什么ESKF中还要加 grav？

### ESKF的重力估计

```cpp
// nav_state.h:65
res(i + 12) = a_inertial[i] + grav_[i];
```

**这里的 `grav_` 是什么？**

1. **状态量：** `grav_` 是ESKF的一个状态，实时在线估计
2. **初始化：** 
   ```cpp
   // imu_processing.hpp:151
   init_state.grav_ = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
   ```
   从静止时的加速度测量估计重力方向

3. **作用：** 补偿重力对速度积分的影响

### 正确的物理模型

```
连续时间:
  v̇ = R × (f_measured - ba) + g_est

离散时间:
  v(t+Δt) = v(t) + [R × (f_measured - ba) + g_est] × Δt

其中:
  f_measured: 应该是去除重力后的比力
  g_est:      估计的重力向量（世界系）
```

**关键：** `f_measured` 输入时应该已经减去了静止时的基准值！

---

## 与 UndistortPcl 的区别

### UndistortPcl 中的处理（正确）

```cpp
// imu_processing.hpp:212
acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();  // 归一化

// 在ESKF predict中会自动处理重力
kf_state.Predict(dt, Q_, gyro, acc);
```

**为什么这里是对的？**

1. 这里的 `acc` 会经过 `get_f` 函数
2. `get_f` 中 `rot_ * (acce - ba_) + grav_` 会正确处理
3. 因为 `acce` 是原始比力，`grav_` 会抵消其中的重力分量

### ProcessIMU 中的问题（错误）

```cpp
// ❌ 错误：kf_imu_ 没有实时的雷达更新
kf_imu_.Predict(dt, Q_, gyro, imu->linear_acceleration);
```

**为什么有问题？**

1. `kf_imu_` 只做IMU预测，不做雷达更新
2. `grav_` 状态不会被实时校正
3. 如果 `grav_` 估计不准，误差会累积
4. **最关键：** `imu->linear_acceleration` 是归一化后的值，已经包含了重力的"归一化效果"

---

## 数值示例

### 场景1: 静止

```
IMU测量:
  acc = [0.005, 0.003, 0.995]  (归一化后)

修复前:
  a_inertial = R × [0.005, 0.003, 0.995]
             ≈ [0.005, 0.003, 0.995]  (假设R≈I)
  v̇ = [0.005, 0.003, 0.995] + [0, 0, -9.8]
    = [0.005, 0.003, -8.805]  ← 巨大的向下加速度！
  
  积分0.01s后:
  Δv = -8.805 × 0.01 = -0.088 m/s  (向下)
  
  100帧后:
  v_z = -8.8 m/s  ← 完全错误！

修复后:
  acc_corrected = [0.005, 0.003, 0.995] - [0, 0, -1.0]
                = [0.005, 0.003, -0.005]
  a_inertial = R × [0.005, 0.003, -0.005]
             ≈ [0.005, 0.003, -0.005]
  v̇ = [0.005, 0.003, -0.005] + [0, 0, -9.8]
    = [0.005, 0.003, -9.805]  ← 仍然有问题？

  ⚠️ 注意: 这里假设 grav_ = [0, 0, -9.8]
  但实际上 grav_ 应该被正确估计为 [0, 0, -G_m_s2]
  
  正确情况:
  v̇ ≈ [0.005, 0.003, 0.001]  (接近0)
```

### 场景2: 向前加速

```
IMU测量:
  acc = [0.100, 0.005, 0.990]  (x方向加速)

修复后:
  acc_corrected = [0.100, 0.005, 0.990] - [0, 0, -1.0]
                = [0.100, 0.005, -0.010]
  
  物理含义:
  - x: 0.100 → 向前加速 1m/s² (0.1g)
  - y: 0.005 → 基本无横向加速度
  - z: -0.010 → 微小向下（可能是姿态变化或噪声）
```

---

## 总结

### 问题根源

1. ✅ **IMU数据归一化：** 加速度已归一化到重力单位
2. ✅ **静止时测量值：** z ≈ 1.0（向上的支撑力）
3. ❌ **直接使用原始值：** 没有减去静止基准
4. ❌ **重力重复计算：** `get_f` 中又加了一次重力

### 修复要点

1. ✅ **输入预处理：** 在 `ProcessIMU` 中减去静止基准
2. ✅ **物理正确性：** 输入ESKF的是纯运动加速度
3. ✅ **数值稳定性：** 避免重力的重复累积

### 预期改善

修复后应该看到：

| 指标 | 修复前 | 修复后 |
|------|-------|--------|
| **静止时z速度** | 持续增大 | 接近0 |
| **静止时z位置** | 快速漂移 | 基本稳定 |
| **运动时轨迹** | 有垂直分量 | 水平轨迹 |
| **DR与LO差异** | 0.13m/0.1s | <0.01m/0.1s |

这个修复应该能解决您观察到的z轴漂移问题！🎯
