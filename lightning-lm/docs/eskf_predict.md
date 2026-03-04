# ESKF::Predict 函数详解

> 源文件：`src/core/lio/eskf.hpp`

---

## 函数签名

```cpp
void ESKF::Predict(const double& dt,
                   const ESKF::ProcessNoiseType& Q,
                   const Vec3d& gyro,
                   const Vec3d& acce);
```

| 参数 | 含义 |
|------|------|
| `dt` | 时间步长（秒） |
| `Q` | 过程噪声协方差矩阵（12×12） |
| `gyro` | IMU 角速度测量值（rad/s） |
| `acce` | IMU 加速度测量值（m/s²） |

---

## 函数功能概述

这是**误差状态卡尔曼滤波（ESKF）的预测步骤**，完成两件事：

1. **标称状态前向传播**：将 IMU 测量值积分，更新位置、速度、姿态等状态量
2. **误差协方差前向传播**：追踪状态不确定度随时间的增长

难点在于状态中混有 **SO3**（旋转）和 **S2**（重力球面）两种非欧流形，无法直接套用线性卡尔曼公式，需要在各自的切空间做修正。

---

## 整体结构

```
Predict(dt, Q, gyro, acce)
│
├─ 1. 计算连续时间动力学 f 和 Jacobian（f_x_, f_w_）
│
├─ 2. 标称状态积分：x ← x ⊕ (f · dt)
│
├─ 3. 构造离散化转移矩阵 F_x1_（线性化 + 流形修正）
│   ├─ 普通向量状态：F ≈ I + f_x · dt
│   ├─ SO3 状态：    指数映射修正
│   └─ S2 状态：     切空间投影修正
│
└─ 4. 协方差传播：P ← F·P·Fᵀ + (dt·f_w)·Q·(dt·f_w)ᵀ
```

---

## 逐段解析

### 第一段：计算动力学和 Jacobian

```cpp
Eigen::Matrix<double, 24, 1>  f_   = x_.get_f(gyro, acce); // 状态导数 ẋ
Eigen::Matrix<double, 24, 23> f_x_ = x_.df_dx(acce);        // ∂f/∂x（过程 Jacobian）
Eigen::Matrix<double, 24, 12> f_w_ = x_.df_dw();            // ∂f/∂w（噪声 Jacobian）
```

`f_` 是连续时间动力学方程（IMU 积分模型）：

$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{R}} = \mathbf{R}\,[\boldsymbol{\omega} - \mathbf{b}_g]_\times$$

$$\dot{\mathbf{v}} = \mathbf{R}(\mathbf{a} - \mathbf{b}_a) + \mathbf{g}$$

**维度说明：**

| 维度 | 含义 |
|------|------|
| **24** | 状态切空间维度（SO3/S2 各展开为 3D 局部坐标） |
| **23** | 误差状态维度（SO3 用 3D 切向量，S2 用 2D 切向量，比 24 少 1 维） |
| **12** | 过程噪声维度（陀螺仪噪声 3 + 加速度噪声 3 + 陀螺偏置随机游走 3 + 加速度偏置随机游走 3） |

---

### 第二段：标称状态积分

```cpp
NavState x_before = x_;   // 保存积分前的状态（S2 修正需用到）
x_.oplus(f_, dt);          // x ← x ⊕ (f · dt)，在各自流形上积分
```

`oplus` 对不同类型的状态分类处理：

| 状态类型 | 积分方式 |
|----------|---------|
| 向量（pos、vel、bg、ba） | `x += f * dt`（普通欧氏积分） |
| SO3（旋转矩阵 R） | `R ← R · Exp(ω · dt)`（李群积分，保持正交性） |
| S2（重力方向 g） | `g ← Exp(ġ · dt) · g`（球面测地线积分，保持模值不变） |

---

### 第三段：构造离散转移矩阵

#### 3a. 向量状态（普通线性化）

```cpp
F_x1_ = CovType::Identity();   // 初始化为 I（23×23）

for (auto st : x_.vect_states_) {
    f_x_final(idx + j, i) = f_x_(dim + j, i);  // 重新排列到误差状态索引
    f_w_final(idx + j, i) = f_w_(dim + j, i);
}
// 最后统一加入：F_x1_ += f_x_final * dt
```

离散化近似：

$$\mathbf{F} \approx \mathbf{I} + \mathbf{f}_x \cdot dt$$

---

#### 3b. SO3 状态修正（旋转流形）

```cpp
for (auto st : x_.SO3_states_) {
    seg_SO3 = -f_(dim:dim+3) * dt;   // = -ω·dt

    // SO3 对角块：切空间转移矩阵
    F_x1_.block<3,3>(idx, idx) = math::exp(seg_SO3, 0.5).matrix();

    // 对其他列的修正：乘以左 Jacobian A(seg_SO3)
    res_temp_SO3 = math::A_matrix(seg_SO3);
    f_x_final.block<3,1>(idx, i) = res_temp_SO3 * f_x_.block<3,1>(dim, i);
}
```

**为什么需要修正？**

旋转矩阵活在 SO3 流形上，协方差在切空间（$\mathfrak{so}(3)$，即李代数）里追踪。当标称状态更新后，切空间的基底随之变化，需要用伴随映射修正：

$$\mathbf{F}^{SO3}_{ii} = \operatorname{Exp}\!\left(-\frac{\boldsymbol{\omega} \cdot dt}{2}\right)$$

$$\mathbf{f}_x^{\mathrm{corrected}} = \mathbf{A}(-\boldsymbol{\omega} dt) \cdot \mathbf{f}_x$$

其中 $\mathbf{A}(\boldsymbol{\phi})$ 是 SO3 的**左 Jacobian**：

$$\mathbf{A}(\boldsymbol{\phi}) = \mathbf{I} + \frac{1-\cos\|\boldsymbol{\phi}\|}{\|\boldsymbol{\phi}\|^2}[\boldsymbol{\phi}]_\times + \frac{\|\boldsymbol{\phi}\| - \sin\|\boldsymbol{\phi}\|}{\|\boldsymbol{\phi}\|^3}[\boldsymbol{\phi}]_\times^2$$

---

#### 3c. S2 状态修正（重力球面流形）

```cpp
for (auto st : x_.S2_states_) {
    seg_S2 = f_(dim:dim+3) * dt;   // 重力的变化量（3D 切向量）

    SO3 res = math::exp(seg_S2, 0.5f);

    // 积分后切空间基（2×3）和积分前切空间基（3×2）
    Eigen::Matrix<double, 2, 3> Nx = x_.grav_.S2_Nx_yy();
    Eigen::Matrix<double, 3, 2> Mx = x_before.grav_.S2_Mx(vec);

    // S2 对角块：2×2 的切空间转移
    F_x1_.block<2,2>(idx, idx) = Nx * res.matrix() * Mx;

    // 对其他列的修正
    x_before_hat = x_before.grav_.S2_hat();
    res_temp_S2 = -Nx * res.matrix() * x_before_hat * A(seg_S2).transpose();
    f_x_final.block<2,1>(idx, i) = res_temp_S2 * f_x_.block<3,1>(dim, i);
}
```

**为什么 S2 对角块是 2×2？**

重力的模值 $\|\mathbf{g}\| = 9.81\,\text{m/s}^2$ 由 S2 流形约束固定，只有方向（2 个自由度）参与 EKF 估计。

```
         S2 球面                   切空间（2D）
             g                      Tg S²
            /|\                    ↑  ↗
           / | \                  e₁  e₂（两个切方向）
```

- `S2_Nx_yy()`（2×3）：将 3D 切向量投影到新切空间的 2D 基底
- `S2_Mx()`（3×2）：将旧切空间的 2D 基底映射回 3D

---

### 第四段：协方差传播

```cpp
F_x1_ += f_x_final * dt;   // F ← I + f_x·dt（完成线性化）

P_ = F_x1_ * P_ * F_x1_.transpose()                            // 状态转移
   + (dt * f_w_final) * Q * (dt * f_w_final).transpose();      // 过程噪声
```

对应连续时间 Lyapunov 方程的**一阶离散化**：

$$P_{k+1} = \mathbf{F}\, P_k\, \mathbf{F}^\top + \mathbf{G}\, Q\, \mathbf{G}^\top$$

其中：

$$\mathbf{F} = \mathbf{I} + \mathbf{f}_x \cdot dt + \text{（SO3/S2 修正项）}$$

$$\mathbf{G} = dt \cdot \mathbf{f}_w$$

---

## 数据流总结

```
输入：dt, gyro(ω), acce(a), Q(过程噪声协方差)
          │
          ▼
f_ = [ṗ, ṙ, v̇, ḃg, ḃa, ġ]   ← 24维连续动力学
          │
     ┌────┴────┐
     │         │
     ▼         ▼
x ⊕ f·dt   构造离散 F（23×23）
（标称状态）  ├─ 向量状态：I + f_x·dt（普通线性化）
             ├─ SO3 旋转：exp 修正 + 左 Jacobian A(·)
             └─ S2 重力： Nx·exp(·)·Mx 切空间投影
                    │
                    ▼
         P = F·P·Fᵀ + G·Q·Gᵀ   （协方差传播）
```

---

## 关键矩阵维度速查

| 矩阵 | 维度 | 含义 |
|------|------|------|
| `f_` | 24×1 | 连续时间状态导数 |
| `f_x_` | 24×23 | 动力学对状态的 Jacobian |
| `f_w_` | 24×12 | 动力学对过程噪声的 Jacobian |
| `f_x_final` | 23×23 | 重排后的状态 Jacobian（误差状态维度） |
| `f_w_final` | 23×12 | 重排后的噪声 Jacobian |
| `F_x1_` | 23×23 | 离散转移矩阵（含流形修正） |
| `P_` | 23×23 | 误差状态协方差矩阵 |
| `Q` | 12×12 | 过程噪声协方差矩阵 |

---

## 与标准 EKF 的区别

| 对比项 | 标准 EKF | 本 ESKF |
|--------|---------|---------|
| 状态空间 | 欧氏空间 | 混合流形（向量 + SO3 + S2） |
| 协方差传播 | $F = I + f_x dt$ | $F = I + f_x dt + \text{流形修正}$ |
| 旋转表示 | 欧拉角 / 四元数（奇异点） | SO3 李代数（全局无奇异） |
| 重力表示 | 固定常数或 3D 向量 | S2 流形（2 自由度，在线估计方向） |
| 更新步 | 标准卡尔曼增益 | 误差状态更新后通过 `oplus` 注入标称状态 |
