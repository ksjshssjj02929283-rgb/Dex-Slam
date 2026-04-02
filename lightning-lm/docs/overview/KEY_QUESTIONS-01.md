# lightning-lm 工程关键问题与设计决策 (Key Questions & Design Decisions)

---

## Q1：为什么使用双地图（iVox + VoxelMap）？

**背景**：`LaserMapping` 同时维护两个地图：`IVoxManager` 和 `VoxelMapManager`，可独立开关。

**设计考量**：

| 特性 | iVox | VoxelMap |
|------|------|----------|
| 精度 | 中（kNN 平均法线） | 高（协方差加权） |
| 速度 | 快（哈希 kNN，O(1)邻域） | 稍慢（协方差矩阵构建） |
| 内存 | 低（只存点） | 高（存点+协方差矩阵） |
| 适合场景 | 快速在线建图 | 精细定位/高精地图 |

**实践建议**：
- 室内机器人：`use_ivox=true, use_voxel_map=false`（快，省内存）
- 室外/大场景精度要求高：考虑 `use_voxel_map=true`
- 两者都关闭会导致无观测，系统崩溃

**[VERIFY: src/core/lio/laser_mapping.h:54-60]** — 双地图声明

---

## Q2：为什么维护 kf_imu_（IMU-only 副本 ESKF）？

**背景**：`LaserMapping` 中有两个 ESKF：`kf_`（完整 LIO）和 `kf_imu_`（仅 IMU）。

**原因**：  
LiDAR @10Hz，IMU @100Hz。在两帧 LiDAR 之间，需要用 IMU 高频预测姿态用于：
1. 下一帧点云**去畸变**（需要高频 IMU 积分姿态）
2. 定位模式下的 **TF 高频输出**（PoseExtrapolator 基于 IMU）

`kf_imu_` 是 `kf_` 经过 LiDAR Update 后的状态克隆，然后单独用 IMU Predict 推进。

```
kf_ (完整 IEKF)：
  IMU Predict → IMU Predict → ... → LiDAR Update → 重置 kf_imu_
  
kf_imu_ (IMU only)：
  ← 从 kf_ 最新状态克隆
  → IMU Predict @100Hz（插帧）
  → 提供高频当前姿态
```

**[VERIFY: src/core/lio/laser_mapping.h:58-65]** — `kf_` 和 `kf_imu_` 声明

---

## Q3：pose_opt_ 为何初始化为 pose_lio_，而非等待 PGO？

**背景**：`Keyframe::pose_opt_` 在创建时等于 `pose_lio_`，由 PGO 后续更新。

**原因**：
1. **容错**：即使回环检测未触发，`pose_opt_` 仍是有效位姿
2. **初始化顺序**：`MakeKF()` 先于 `HandleKF()` 运行，如果 `pose_opt_` 为零会导致 PGO 首次优化失效
3. **g2p5 实时渲染**：需要立即用 `pose_opt_` 渲染，不能等待 PGO

**注意**：在 `HandleKF()` 中添加里程计因子时使用 `prev_kf->GetOptPose()` 而非 `pose_lio_`，这在大漂移初始化时可能引入误差——但实践中 LIO 漂移小，影响可接受。

---

## Q4：回环 Tij 的坐标系是什么？

**背景**：回环约束 `c.Tij_` 表达的是什么坐标系之间的变换？

**答案**：`Tij_` = kf2 在 kf1 **局部坐标系**中的位姿（即 kf1 局部系 → kf2 body 系的变换）。

```
Tij = T_{kf1_local → kf2_body}
    = kf1->GetLIOPose().inverse() * (GICP结果)
```

因此在 `AddLoopFactor()` 中因子边表达的是：
- 节点 id_cur (kf2) 相对于节点 id_prev (kf1) 的期望相对位姿 = `Tij`

**[VERIFY: src/core/loop_closing/loop_closing.cc:495-515]** — Tij 提取逻辑

---

## Q5：如何调试"定位漂移"或"回环失败"？

### 5.1 定位漂移

1. **检查 TiledMap 是否正确加载**：启动时看日志，确认 `LoadNearbyChunks()` 成功
2. **增大 `lidar_loc_skip_num_`**：减少全局定位频率，让 LIO 稳定后再介入
3. **检查初始位姿精度**：`/initialpose` 需在2m以内才能使 NDT 收敛
4. **降低 `ndt_resolution`**：提高全局定位精度，但会变慢

### 5.2 回环失败（`icp_score` 全部超阈值）

1. **检查 `icp_filter_size`**：太大导致子地图过稀，匹配失败，建议 0.2-0.4m
2. **检查坐标系**：`ComputeForCandidate()` 的初始猜测必须在 kf1 局部系
3. **放宽 `icp_score_threshold`**：从 0.3 放宽到 0.5，看是否有候选通过
4. **增大 `history_keyframe_search_num`**：子地图更完整，匹配更易成功

### 5.3 建图轨迹跳变

1. 说明 PGO 优化后大幅修正，可能是回环约束质量差
2. 增大 `rk_loop_th_`（Cauchy 核），使外点更被忽略
3. 提高 GICP 验证分数门槛（降低 `icp_score_threshold`）

---

## Q6：g2p5 生成的 2D 地图精度如何保证？

### 高度参数（最关键）

```yaml
g2p5:
  min_th_floor: 0.3      # 地面以上 0.3m 开始算障碍（过小 = 把地面误判为障碍）
  max_th_floor: 1.2      # 地面以上 1.2m 以下的点（过大 = 把悬空物体误判为障碍）
  esti_floor: false      # 是否动态估计地面高度（动态场景建议开启）
```

### 典型问题解法

| 问题 | 原因 | 解法 |
|------|------|------|
| 地面出现栅格 | `min_th_floor` 太小 | 增大 min_th_floor（如0.5m） |
| 天花板出现栅格 | `max_th_floor` 太大 | 减小 max_th_floor（如1.0m） |
| 桌子/椅子不出现 | 高度范围不覆盖 | 调整两个阈值覆盖障碍物高度 |
| 地图大量噪点 | 动态障碍物（人/移动物体） | 开启 `esti_floor` 或平滑滤波 |

---

## Q7：LiDAR 时间戳单位如何确认？

**背景**：`fasterlio.time_scale` 控制点云时间戳转换。

| LiDAR 品牌 | 时间戳单位 | time_scale |
|------------|------------|-----------|
| Livox | 纳秒 (ns) | 1e-9 |
| Velodyne | 毫秒 (ms) | 1e-3 |
| Ouster | 纳秒 (ns) | 1e-9 |
| Robosense | 秒 (s) | 1.0 |

**验证方法**：
```bash
# 查看 bag 中点云的时间戳范围
ros2 bag info <bag_file>
# 或播放 bag 后运行：
ros2 topic echo /your/lidar/topic | head -20
# 检查 points[0].time 是否在合理范围（0.0 ~ 0.1s 内）
```

**[VERIFY: src/core/lio/pointcloud_preprocess.cc]** — `time_scale_` 乘法使用

---

## Q8：多线程安全的最佳实践（本项目经验）

1. **Keyframe 双 pose 设计**：`pose_lio_`（单写单读）和 `pose_opt_`（mutex 保护多读一写）
2. **AsyncMessageProcess 队列**：将 ROS 回调与算法计算解耦，避免阻塞 spin
3. **ThreadPool 只做无状态计算**：`ComputeForCandidate()` 不修改共享状态，只返回结果
4. **三队列解耦**：回环结果写队列，PGO 统一消费，避免边添加边优化的竞争

---

## Q9：如何扩展支持新的 LiDAR 型号？

**文件**：`src/core/lio/pointcloud_preprocess.h/cc`

### 扩展步骤

1. 在 `LidarType` 枚举中增加新类型（如 `HESAI = 4`）

2. 在 `PointCloudPreprocess::Process()` 中添加新分支：
   ```cpp
   case LidarType::HESAI:
       PCL_HESAI_handler(laser_msg);
       break;
   ```

3. 实现 `PCL_HESAI_handler()`：
   - 将厂商 PointCloud 类型转换为 `PointXYZINormal`
   - 设置 `point.curvature` = 相对时间戳（秒，通过 `time_scale_` 换算）
   - 过滤 NaN、近点（距离 < `blind_`）

4. 更新 `config/default.yaml` 中 `fasterlio.lidar_type`

---

## Q10：系统初始化失败排查清单

| 现象 | 可能原因 | 解法 |
|------|----------|------|
| IMU 初始化卡住 | 机器人一直在运动 | 启动后静止5-10秒 |
| IMU 初始化失败 | 噪声太大或频率太低 | 检查 `imu_topic` 是否正确，IMU 频率需 ≥ 50Hz |
| 点云不处理 | 话题名不匹配 | 检查 `lidar_topic` 与实际 bag/驱动一致 |
| 时间戳警告 | `time_scale` 错误 | 参见 Q7 |
| 建图不走 | 外参过大 | 外参默认为零，若硬件偏差超过 0.5m 需手动设置 |
| 定位不收敛 | 初始位姿误差太大 | 使用 RViz `/initialpose` 精确设置 |

---

## Q11：ESKF 参数调优建议

### 过程噪声 Q（影响预测值的置信度）

```yaml
fasterlio:
  gyr_cov: 0.1         # 陀螺仪噪声方差，越大表示 IMU 越不可信，LiDAR 权重越高
  acc_cov: 0.1         # 加速度计噪声方差
  b_gyr_cov: 0.0001    # 陀螺偏置随机游走
  b_acc_cov: 0.0001    # 加速计偏置随机游走
```

- **Q 太小**：过于信任 IMU，LiDAR 修正困难，可能漂移
- **Q 太大**：过于信任 LiDAR，噪声抖动严重，对 IMU 精度要求低的场景可适当调大

### 观测噪声 R（LiDAR 点面约束）

- 固定值，通常 `0.001 m²`，无需大幅调整
- 若点云噪声大（如雨雾天），可适当增大

---

## Q12：回环检测性能估算

| 场景 | 关键帧数 | KD-Tree 搜索时间 | GICP 时间/次 | 总回环检测时间 |
|------|----------|-----------------|-------------|----------------|
| 小室内（50x50m） | ~100 | <1ms | ~50ms | <100ms |
| 中型室内（200x200m） | ~500 | ~2ms | ~100ms | ~200ms |
| 大型室外（500x500m） | ~2000 | ~5ms | ~200ms | ~500ms |

**注**：ThreadPool 并行处理多个候选，总时间取决于候选数量。  
若回环检测阻塞主流程，可减小 `history_keyframe_search_num` 或增大 `max_range_/2`（减少候选数）。
