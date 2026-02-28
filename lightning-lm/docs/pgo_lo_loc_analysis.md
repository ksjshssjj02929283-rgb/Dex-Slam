# Localization 中 PGO、Lidar Odom 和 Lidar Loc 的处理分析

## 系统架构概览

```
                    ┌──────────────────────────────┐
                    │      Localization            │
                    │      (主控制器)              │
                    └──────────┬───────────────────┘
                               │
                ┌──────────────┼──────────────┐
                │              │              │
         ┌──────▼──────┐ ┌────▼─────┐ ┌─────▼──────┐
         │    LIO      │ │ LidarLoc │ │    PGO     │
         │(激光里程计)  │ │(激光定位) │ │(位姿图优化)│
         └──────┬──────┘ └────┬─────┘ └─────┬──────┘
                │              │              │
                │  LO State    │ Loc Result   │
                └──────────────┼──────────────┘
                               │
                        ┌──────▼──────┐
                        │  融合结果输出 │
                        └─────────────┘
```

---

## 一、数据流向与处理流程

### 1.1 整体数据流

```
传感器输入
    │
    ├─► 点云(Lidar/Livox) ──► ProcessLidarMsg/ProcessLivoxLidarMsg
    │                              │
    │                              ▼
    │                         点云预处理
    │                              │
    │                              ▼
    │                      LidarOdomProcCloud ─────┐
    │                              │                │
    └─► IMU数据 ──► ProcessIMUMsg │                │
                         │         │                │
                         ▼         ▼                ▼
                    ┌─────────┐ ┌─────────┐  ┌──────────┐
                    │   LIO   │ │LidarLoc │  │   PGO    │
                    │ProcessIMU│ │ProcessLO│  │ProcessDR │
                    └────┬────┘ │ProcessDR│  │ProcessLO │
                         │      │ProcessCloud│ProcessLoc│
                         │      └────┬────┘  └────┬─────┘
                         │           │            │
                         └───────────┼────────────┘
                                     │
                              ┌──────▼──────┐
                              │  融合输出    │
                              │  (高频TF)   │
                              └─────────────┘
```

### 1.2 三大核心模块职责

| 模块 | 输入 | 输出 | 频率 | 作用 |
|------|------|------|------|------|
| **LIO** | 点云+IMU | NavState(位姿+速度) | 10-20Hz | 提供高频、短期精确的里程计 |
| **LidarLoc** | 去畸变点云+LO状态+DR状态 | LocalizationResult | 1-5Hz | 提供全局一致的定位修正 |
| **PGO** | LO+DR+Loc结果 | 融合后的定位结果 | IMU频率(~200Hz) | 多源融合+高频外推 |

---

## 二、Lidar Odom (LIO) 的处理

### 2.1 处理流程

```cpp
void Localization::LidarOdomProcCloud(CloudPtr cloud) {
    // 1. 点云送入LIO处理
    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    // 2. 获取LIO状态
    auto lo_state = lio_->GetState();

    // 3. 分发给LidarLoc和PGO
    lidar_loc_->ProcessLO(lo_state);
    pgo_->ProcessLidarOdom(lo_state);

    // 4. 关键帧处理
    if (options_.loc_on_kf_) {
        auto kf = lio_->GetKeyframe();
        if (kf == lio_kf_) {
            return;  // 关键帧未更新
        }
        lio_kf_ = kf;
        auto scan = lio_->GetScanUndist();
        // 发送去畸变点云给LidarLoc
        LidarLocProcCloud(scan);
    } else {
        auto scan = lio_->GetScanUndist();
        LidarLocProcCloud(scan);
    }
}
```

### 2.2 LIO状态(NavState)内容

```cpp
struct NavState {
    double timestamp_;           // 时间戳
    SE3 pose_;                   // 位姿(世界系->车体系)
    Vec3d vel_;                  // 速度(世界系)
    bool pose_is_ok_;           // 位姿是否有效
    bool lidar_odom_reliable_;  // 里程计是否可靠
    double confidence_;          // 置信度
    // ... 其他IMU状态
};
```

### 2.3 LIO在系统中的角色

**提供给LidarLoc:**
- 用于位姿预测(guess_from_lo)
- 作为定位初始值
- 提供速度信息判断停车

**提供给PGO:**
- 作为帧间相对约束
- 用于高频位姿外推
- 提供可靠性标志

**关键特性:**
- **高频率**: 10-20Hz，跟随点云频率
- **局部精确**: 短期内精度高，长期累积误差
- **可靠性标志**: `lidar_odom_reliable_` 表示是否存在退化

---

## 三、Lidar Loc (激光定位) 的处理

### 3.1 处理流程

```cpp
void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    // 1. 点云送入激光定位
    lidar_loc_->ProcessCloud(scan_undist);

    // 2. 获取定位结果
    auto res = lidar_loc_->GetLocalizationResult();

    // 3. 发送给PGO处理
    pgo_->ProcessLidarLoc(res);

    // 4. UI更新
    if (ui_) {
        ui_->UpdateScan(scan_undist, res.pose_);
    }

    // 5. 状态回调
    if (loc_state_callback_) {
        auto loc_state = std::make_shared<std_msgs::msg::Int32>();
        loc_state->data = static_cast<int>(res.status_);
        loc_state_callback_(*loc_state);
    }
}
```

### 3.2 LocalizationResult 内容

```cpp
struct LocalizationResult {
    double timestamp_;                    // 时间戳
    SE3 pose_;                           // 全局位姿
    double confidence_;                   // 匹配置信度(NDT分数)
    LocalizationStatus status_;          // 状态: GOOD/FOLLOWING_DR/FAIL
    bool valid_;                         // 是否有效
    bool lidar_loc_valid_;               // 激光定位是否有效
    bool lidar_loc_smooth_flag_;         // 是否平滑
    bool lidar_loc_odom_error_normal_;   // LO-Loc一致性
    double lidar_loc_odom_delta_;        // LO-Loc差异值
    bool is_parking_;                    // 是否停车
};
```

### 3.3 Lidar Loc的关键信息

**位姿信息:**
- `pose_`: 全局一致的位姿，无累积误差
- `confidence_`: NDT匹配分数，反映定位质量

**状态标志:**
- `lidar_loc_smooth_flag_`: 与预测的平滑性
  - 用于PGO判断是否可信
  - 不平滑可能表示重定位或跳跃
  
- `lidar_loc_odom_error_normal_`: LO-Loc一致性检查
  - 比较LO和Loc的相对运动
  - 差异过大标记LO不可靠

**状态枚举:**
```cpp
enum class LocalizationStatus {
    INITIALIZING,    // 初始化中
    GOOD,           // 正常 (失败<100)
    FOLLOWING_DR,   // 跟随DR (失败100-300)
    FAIL            // 失败 (失败>300)
};
```

---

## 四、PGO (位姿图优化) 的处理

### 4.1 PGO的核心思想

```
输入源:
  ├─ DR (IMU航迹推算)    ──► 高频(~200Hz)，精度中等
  ├─ Lidar Odom          ──► 中频(10-20Hz)，短期精确
  └─ Lidar Loc           ──► 低频(1-5Hz)，全局一致

处理:
  ├─ 构建位姿图
  ├─ 添加约束(绝对+相对)
  ├─ 图优化求解
  └─ 高频外推输出

输出:
  └─ 融合定位结果 ──► 高频(IMU频率)，全局一致+平滑
```

### 4.2 三个主要处理函数

#### 4.2.1 ProcessDR - 处理航迹推算

```cpp
bool PGO::ProcessDR(const NavState& dr_result) {
    UL lock(impl_->data_mutex_);
    
    // 1. 时间检查 - 防止回退
    if (!impl_->dr_pose_queue_.empty()) {
        const double last_stamp = impl_->dr_pose_queue_.back().timestamp_;
        double delta_timestamp = dr_result.timestamp_ - last_stamp;
        if (dr_result.timestamp_ < last_stamp) {
            LOG(WARNING) << "DR时间回退";
            return false;
        }
    }

    // 2. 保存到DR队列
    impl_->dr_pose_queue_.emplace_back(dr_result);
    
    // 3. 队列大小限制
    while (impl_->dr_pose_queue_.size() >= pgo::PGO_MAX_SIZE_OF_RELATIVE_POSE_QUEUE) {
        impl_->dr_pose_queue_.pop_front();
    }
    
    // 4. 发布高频结果
    if (!impl_->dr_pose_queue_.empty() && !is_parking_) {
        PubResult();  // 使用DR外推到最新时刻
    } else if (is_parking_ && high_freq_output_func_) {
        parking_result_.timestamp_ = dr_result.timestamp_;
        high_freq_output_func_(parking_result_);
    }
    
    return true;
}
```

**DR的作用:**
- 提供**高频位姿外推**基础
- 作为**帧间相对约束**
- 在LO失效时作为备用方案

**关键机制:**
- 维护DR队列(最大10000帧)
- 每次DR更新触发高频输出
- 停车状态特殊处理

#### 4.2.2 ProcessLidarOdom - 处理激光里程计

```cpp
bool PGO::ProcessLidarOdom(const NavState& lio_result) {
    UL lock(impl_->data_mutex_);
    
    // 1. 时间检查
    if (!impl_->lidar_odom_pose_queue_.empty()) {
        const double last_stamp = impl_->lidar_odom_pose_queue_.back().timestamp_;
        if (lio_result.timestamp_ < last_stamp) {
            LOG(WARNING) << "LidarOdom时间回退";
        }
    }

    // 2. 保存到LO队列
    impl_->lidar_odom_pose_queue_.emplace_back(lio_result);
    
    // 3. 队列大小限制
    while (impl_->lidar_odom_pose_queue_.size() >= pgo::PGO_MAX_SIZE_OF_RELATIVE_POSE_QUEUE) {
        impl_->lidar_odom_pose_queue_.pop_front();
    }

    // 4. 检查LO可靠性
    if (!lio_result.lidar_odom_reliable_) {
        impl_->lidar_odom_conflict_with_dr_ = true;
        impl_->lidar_odom_conflict_with_dr_cnt_ = 200;
    } else {
        if (impl_->lidar_odom_conflict_with_dr_cnt_ > 0) {
            impl_->lidar_odom_conflict_with_dr_cnt_--;
        } else {
            impl_->lidar_odom_conflict_with_dr_ = false;
        }
    }

    // 5. 如果LO比DR更新，发布LO的外推结果
    if (!impl_->dr_pose_queue_.empty() && 
        lio_result.timestamp_ >= impl_->dr_pose_queue_.back().timestamp_ &&
        !is_parking_) {
        PubResult();
    }
    
    return true;
}
```

**LO的作用:**
- 提供**中频相对约束**(比DR更准确)
- 用于**位姿外推**(优先于DR)
- 检测**退化情况**

**可靠性机制:**
- `lidar_odom_reliable_` 标志来自LidarLoc的一致性检查
- 不可靠时标记 `lidar_odom_conflict_with_dr_`
- 需要连续200帧正常才能恢复

#### 4.2.3 ProcessLidarLoc - 处理激光定位(触发优化)

```cpp
bool PGO::ProcessLidarLoc(const LocalizationResult& loc_result) {
    UL lock(impl_->data_mutex_);
    
    // 1. 停车状态处理
    is_parking_ = loc_result.is_parking_;
    if (is_parking_ && high_freq_output_func_) {
        parking_result_ = loc_result;
        high_freq_output_func_(loc_result);
        return true;
    }

    // 2. 检查相对位姿队列
    if (RelativePoseQueueEmpty()) {
        LOG(WARNING) << "PGO received LidarLoc, but waiting for LO or DR";
        return false;
    }

    // 3. 检查有效性
    if (!loc_result.lidar_loc_valid_) {
        return false;
    }

    // 4. 时间检查 - 不允许回退
    static double last_lidar_loc_timestamp = -1;
    double lidar_loc_delta_t = loc_result.timestamp_ - last_lidar_loc_timestamp;
    if (last_lidar_loc_timestamp > 0) {
        if (lidar_loc_delta_t < 0) {
            LOG(ERROR) << "lidar loc时间回退";
            return false;
        }
    }
    last_lidar_loc_timestamp = loc_result.timestamp_;

    // 5. 创建PGO Frame
    auto new_frame = std::make_shared<PGOFrame>();
    new_frame->timestamp_ = loc_result.timestamp_;
    new_frame->opti_pose_ = loc_result.pose_;
    new_frame->last_opti_pose_ = loc_result.pose_;
    new_frame->lidar_loc_set_ = true;
    new_frame->lidar_loc_valid_ = loc_result.lidar_loc_valid_;
    new_frame->lidar_loc_pose_ = loc_result.pose_;
    new_frame->lidar_loc_delta_t_ = lidar_loc_delta_t;

    // 6. 设置置信度权重
    new_frame->confidence_ = loc_result.confidence_;
    if (loc_result.confidence_ > 0.6) {
        new_frame->lidar_loc_normalized_weight_ = Vec6d::Ones();
    } else {
        const double clap_weight = std::max(loc_result.confidence_, 0.5);
        new_frame->lidar_loc_normalized_weight_ = clap_weight * Vec6d::Ones();
    }

    // 7. 更新LO可靠性
    impl_->result_.lidar_loc_odom_error_normal_ = loc_result.lidar_loc_odom_error_normal_;
    impl_->result_.lidar_loc_smooth_flag_ = loc_result.lidar_loc_smooth_flag_;

    if (!loc_result.lidar_loc_odom_error_normal_) {
        impl_->lidar_odom_valid_ = false;
        impl_->lidar_odom_valid_cnt_ = 10;
    } else {
        if (impl_->lidar_odom_valid_cnt_ > 0) {
            impl_->lidar_odom_valid_cnt_--;
        } else {
            impl_->lidar_odom_valid_ = true;
        }
    }

    // 8. 触发PGO优化
    LOG(INFO) << "PGO received LidarLoc [" << new_frame->lidar_loc_pose_.translation().transpose() 
              << "], t=" << new_frame->timestamp_;
    return ProcessPGOFrame(new_frame);
}
```

**Lidar Loc触发的操作:**
1. 创建新的PGOFrame
2. 设置绝对位姿约束
3. 根据置信度设置权重
4. 更新LO可靠性状态
5. **触发图优化** (通过ProcessPGOFrame)
6. 发布融合结果

**权重策略:**
- 置信度 > 0.6: 完全信任 (权重=1.0)
- 置信度 ≤ 0.6: 降低权重 (最低0.5)

### 4.3 PGOFrame - 优化单元

```cpp
struct PGOFrame {
    double timestamp_;        // 时间戳
    FrameId frame_id_;       // 帧ID
    
    // 优化位姿
    SE3 opti_pose_;          // 最新优化位姿
    SE3 last_opti_pose_;     // 上一轮优化位姿(判断收敛)
    int opti_times_;         // 被优化次数
    
    // 激光定位约束 (绝对)
    bool lidar_loc_set_;     // 是否设置
    bool lidar_loc_valid_;   // 是否有效
    SE3 lidar_loc_pose_;     // 定位观测
    double confidence_;       // 置信度
    Vec6d lidar_loc_normalized_weight_;  // 归一化权重[0.5, 1.0]
    
    // 激光里程计约束 (相对)
    bool lidar_odom_set_;    // 是否设置
    bool lidar_odom_valid_;  // 是否有效
    SE3 lidar_odom_pose_;    // LO观测
    Vec6d lidar_odom_normalized_weight_;  // 归一化权重
    
    // DR约束 (相对)
    bool dr_set_;            // 是否设置
    bool dr_valid_;          // 是否有效
    SE3 dr_pose_;            // DR观测
    Vec3d dr_vel_b_;         // 速度(车体系)
    
    // 先验约束 (边缘化)
    bool prior_set_;         // 是否设置
    SE3 prior_pose_;         // 先验位姿
    Mat6d prior_cov_;        // 协方差
};
```

### 4.4 高频外推机制 - ExtrapolateLocResult

```cpp
bool PGO::ExtrapolateLocResult(LocalizationResult& output_result) {
    // 1. 确定最新时间
    double latest_time = impl_->result_.timestamp_;
    if (!dr_pose_queue.empty() && dr_pose_queue.back().timestamp_ > latest_time) {
        latest_time = dr_pose_queue.back().timestamp_;
    }
    if (!lo_pose_queue.empty() && lo_pose_queue.back().timestamp_ > latest_time) {
        latest_time = lo_pose_queue.back().timestamp_;
    }

    // 2. IMU断流检测
    if (!dr_pose_queue.empty() && 
        latest_time - dr_pose_queue.back().timestamp_ > imu_interruption_time_thd_) {
        imu_interruption_tag_ = true;
        LOG(ERROR) << "IMU断流, 断流时间: " << latest_time - dr_pose_queue.back().timestamp_;
    }

    // 3. 使用DR外推
    if (!dr_pose_queue.empty() && dr_pose_queue.back().timestamp_ > output_result.timestamp_) {
        double dr_extrap_time = dr_pose_queue.back().timestamp_ - output_result.timestamp_;
        
        // 计算允许的外推距离
        double dr_pose_inc_th = 5.0 * dr_extrap_time;
        if (!impl_->lidar_odom_valid_) {
            dr_pose_inc_th *= 2;  // LO失效时放宽
        }
        if (dr_pose_inc_th < 2.0) {
            dr_pose_inc_th = 2.0;
        }

        // 插值获取起始位姿
        SE3 interp_pose;
        NavState best_match;
        bool dr_interp_success = math::PoseInterp<NavState>(
            output_result.timestamp_, dr_pose_queue,
            [](const NavState& nav_state) { return nav_state.timestamp_; },
            [](const NavState& nav_state) { return nav_state.GetPose(); },
            interp_pose, best_match);
        
        if (dr_interp_success) {
            // 计算相对运动
            SE3 pose_incre = interp_pose.inverse() * dr_pose_queue.back().GetPose();
            
            // 应用外推
            output_result.pose_ = output_result.pose_ * pose_incre;
            output_result.timestamp_ = dr_pose_queue.back().timestamp_;
        }
    }

    // 4. 如果外推失败，使用LidarLoc自身外推
    if (output_result.timestamp_ < (latest_time - 0.05)) {
        SE3 interp_pose_final;
        TimedPose best_match_final;
        if (math::PoseInterp<TimedPose>(
                latest_time, impl_->lidar_loc_pose_queue_,
                [](const TimedPose& pose) { return pose.timestamp_; },
                [](const TimedPose& pose) { return pose.pose_; },
                interp_pose_final, best_match_final, 5.0)) {
            output_result.pose_ = interp_pose_final;
            output_result.timestamp_ = latest_time;
        }
    }

    return true;
}
```

**外推策略:**
1. **优先DR外推**: 高频、低延迟
2. **距离限制**: 防止发散 (5m/s × 时间)
3. **LO失效放宽**: 允许更大外推距离
4. **失败降级**: 使用LidarLoc历史外推

### 4.5 高频发布机制 - PubResult

```cpp
void PGO::PubResult() {
    if (high_freq_output_func_ && impl_->result_.valid_) {
        // 1. 检查激光定位分值
        if (last_lidar_loc_time_ > 0 && impl_->result_.timestamp_ > last_lidar_loc_time_) {
            if (impl_->result_.confidence_ < lidar_loc_score_thd_) {
                localization_unusual_count_++;
            } else {
                localization_unusual_count_ = 0;
            }
        }

        // 2. 异常标志
        if (localization_unusual_count_ > localization_unusual_thd_) {
            localization_unusual_tag_ = true;
        }

        // 3. 外推到最新时刻
        auto result = impl_->result_;
        ExtrapolateLocResult(result);

        // 4. 外推有效性校验
        if (impl_->lidar_loc_pose_queue_.size() > 5 && 
            impl_->result_.lidar_loc_smooth_flag_ &&
            (result.timestamp_ - impl_->lidar_loc_pose_queue_.back().timestamp_) < 0.3) {
            
            // 使用LidarLoc自身外推结果校验DR外推
            SE3 lidar_loc_extrap_pose;
            TimedPose best_match;
            bool interp_succ = math::PoseInterp<TimedPose>(
                result.timestamp_, impl_->lidar_loc_pose_queue_,
                [](const TimedPose& tp) { return tp.timestamp_; },
                [](const TimedPose& tp) { return tp.pose_; },
                lidar_loc_extrap_pose, best_match, 1.0);
            
            // 如果DR外推与LidarLoc外推差异过大，使用LidarLoc结果
            // (代码中此部分被注释)
        }

        // 5. 平滑处理
        if (!impl_->dr_pose_queue_.empty()) {
            smoother_->PushDRPose(impl_->dr_pose_queue_.back().GetPose());
        }
        smoother_->PushPose(result.pose_);
        result.pose_ = smoother_->GetPose();

        // 6. 发布结果
        high_freq_output_func_(result);

        // 7. 保存输出历史
        impl_->output_pose_queue_.emplace_back(result.timestamp_, result.pose_);
        while (impl_->output_pose_queue_.size() > 1000) {
            impl_->output_pose_queue_.pop_front();
        }

        // 8. 外推失败处理
        if (!extrap_success) {
            impl_->dr_pose_queue_.clear();  // 清空DR队列
        }
    }
}
```

**发布机制:**
1. **触发条件**: DR或LO更新时
2. **外推**: 使用DR/LO外推到最新时刻
3. **校验**: 用LidarLoc历史验证外推有效性
4. **平滑**: 使用Smoother平滑轨迹
5. **高频输出**: 回调发布融合结果

---

## 五、三者之间的协同关系

### 5.1 信息流向图

```
                    时刻 t
                      │
        ┌─────────────┼─────────────┐
        │             │             │
   ┌────▼────┐   ┌────▼────┐   ┌───▼────┐
   │   DR    │   │   LO    │   │  Loc   │
   │ ~200Hz  │   │ ~10Hz   │   │  ~2Hz  │
   └────┬────┘   └────┬────┘   └───┬────┘
        │             │             │
        └─────────────┼─────────────┘
                      │
              ┌───────▼────────┐
              │      PGO       │
              │  构建位姿图     │
              │  添加约束       │
              │  图优化         │
              └───────┬────────┘
                      │
              ┌───────▼────────┐
              │   外推+平滑     │
              └───────┬────────┘
                      │
              ┌───────▼────────┐
              │  融合输出       │
              │  ~200Hz        │
              └────────────────┘
```

### 5.2 约束关系

```
PGO Frame (时刻t)
    │
    ├─► 绝对约束
    │   └─ Lidar Loc: 全局一致位姿
    │       - 权重: 根据置信度调整
    │       - 频率: 低(1-5Hz)
    │
    └─► 相对约束 (从t-1到t)
        ├─ Lidar Odom: 帧间运动
        │   - 权重: 高(精度好)
        │   - 频率: 中(10-20Hz)
        │   - 可靠性检查
        │
        └─ DR: 帧间运动
            - 权重: 低(精度中)
            - 频率: 高(~200Hz)
            - 备用方案
```

### 5.3 可靠性传递链

```
1. LidarLoc检查LO-Loc一致性
   └─► 不一致 → lidar_loc_odom_error_normal_ = false

2. PGO接收Loc结果
   └─► 更新 lidar_odom_valid_ 标志
       └─► LO不可靠需要连续10帧正常才恢复

3. PGO使用LO状态
   └─► 检查 lidar_odom_reliable_ 标志
       └─► 不可靠 → lidar_odom_conflict_with_dr_ = true
           └─► 需要连续200帧正常才恢复

4. PGO外推选择
   └─► LO可靠 → 优先使用LO外推
   └─► LO不可靠 → 使用DR外推(放宽阈值)
```

### 5.4 优先级策略

| 场景 | LO状态 | DR状态 | 策略 |
|------|--------|--------|------|
| 正常行驶 | 可靠 | 可靠 | LO外推为主，DR辅助 |
| LO退化 | 不可靠 | 可靠 | DR外推，放宽阈值 |
| 静止 | - | - | 保持位姿不变 |
| 初始化 | - | - | 等待LidarLoc |
| Loc低分 | - | - | 连续异常→标记 |

---

## 六、时序分析

### 6.1 典型时序流程

```
时间线: ──────►

t=0.00s: DR(0)           ──► PGO: 等待
t=0.01s: LO(0)           ──► PGO: 等待
t=0.05s: DR(1)           ──► PGO: 等待
t=0.10s: LO(1) + Loc(0)  ──► PGO: 创建Frame(0) + 优化
                              └─► 输出: 融合结果(0)
t=0.15s: DR(2)           ──► PGO: 外推到t=0.15s
                              └─► 输出: 融合结果(0→0.15)
t=0.20s: LO(2) + DR(3)   ──► PGO: 外推到t=0.20s
                              └─► 输出: 融合结果(0→0.20)
t=0.30s: LO(3) + Loc(1)  ──► PGO: 创建Frame(1) + 优化
                              └─► 输出: 融合结果(1)
...
```

### 6.2 延迟分析

| 模块 | 处理延迟 | 输出频率 | 累积延迟 |
|------|----------|----------|----------|
| LIO | ~10ms | 10-20Hz | ~10ms |
| LidarLoc | ~50-100ms | 1-5Hz | ~60-110ms |
| PGO优化 | ~5ms | 触发时 | ~65-115ms |
| PGO外推 | <1ms | ~200Hz | <1ms |

**总延迟**: 
- **最优情况**: ~11ms (仅LO+外推)
- **常规情况**: ~66ms (LO+Loc+PGO+外推)
- **最差情况**: ~116ms (Loc延迟大)

---

## 七、关键参数配置

### 7.1 PGO参数

```yaml
pgo:
  # 噪声参数
  dr_pos_noise: 1.0              # DR位置噪声(米)
  dr_ang_noise: 0.5              # DR角度噪声(度)
  lidar_odom_pos_noise: 0.3      # LO位置噪声(米)
  lidar_odom_ang_noise: 1.0      # LO角度噪声(度)
  lidar_loc_pos_noise: 0.3       # Loc位置噪声(米)
  lidar_loc_ang_noise: 1.0       # Loc角度噪声(度)
  
  # 阈值参数
  lidar_loc_outlier_th: 5.0      # Loc异常阈值
  pgo_frame_converge_pos_th: 0.05  # 收敛位置阈值(米)
  pgo_frame_converge_ang_th: 1.0   # 收敛角度阈值(度)
  
  # 平滑参数
  smooth_factor: 0.01            # 平滑因子[0,1]
```

### 7.2 系统参数

```yaml
system:
  enable_lidar_loc_skip: false   # 是否允许Loc跳帧
  lidar_loc_skip_num: 4          # Loc跳帧数
  enable_lidar_odom_skip: false  # 是否允许LO跳帧
  lidar_odom_skip_num: 2         # LO跳帧数
  loc_on_kf: true                # 仅在关键帧定位
```

### 7.3 硬编码参数

```cpp
// PGO内部
const int PGO_MAX_FRAMES = 5;                        // 最大帧数
const int PGO_MAX_SIZE_OF_RELATIVE_POSE_QUEUE = 10000;  // 队列大小
const double PGO_DISTANCE_TH_LAST_FRAMES = 2.5;     // 滑窗距离阈值(米)
const double PGO_ANGLE_TH_LAST_FRAMES = 10°;        // 滑窗角度阈值

// 外推参数
const float imu_interruption_time_thd_ = 1.0;       // IMU断流阈值(秒)
const float lidar_loc_score_thd_ = 0.5;             // Loc分值阈值
const int localization_unusual_thd_ = 10;           // 异常次数阈值

// 可靠性恢复
const int lidar_odom_valid_cnt_init = 10;           // LO恢复计数
const int lidar_odom_conflict_cnt_init = 200;       // LO冲突计数
```

---

## 八、典型场景处理

### 场景1: 正常行驶

```
t0: Loc(pose0, conf=2.5) 
    → PGO: 创建Frame0, 权重=1.0
    → 优化: 融合LO+DR约束
    → 输出: pose0

t0~t1: DR持续更新
    → PGO: 用DR外推
    → 输出: pose0 + DR_delta

t1: LO(pose1) + Loc(pose1, conf=2.3)
    → PGO: 创建Frame1
    → 优化: 添加LO相对约束(Frame0→Frame1)
    → 输出: pose1_optimized

结果: 高频(200Hz)、平滑、全局一致
```

### 场景2: LO退化(长走廊)

```
t0: Loc检测到LO-Loc不一致
    → lidar_loc_odom_error_normal_ = false
    → PGO: lidar_odom_valid_ = false

t0~t1: DR持续更新
    → PGO: 仅使用DR外推(放宽阈值2倍)
    → 不使用LO约束

t1: Loc(conf=低) + LO恢复
    → 连续10帧正常后
    → PGO: lidar_odom_valid_ = true
    → 恢复使用LO

结果: 降级到DR外推，等待LO恢复
```

### 场景3: 初始化

```
t0: 系统启动
    → LIO: 收集IMU数据初始化
    → LidarLoc: 未初始化状态

t1: 第一帧点云到达
    → LIO: 处理点云(但可能不稳定)
    → LidarLoc: 功能点初始化
    → 等待Yaw搜索完成

t2: Loc初始化成功
    → PGO: 创建第一个Frame
    → 但相对位姿队列为空
    → 等待LO/DR

t3: LO + DR数据到达
    → PGO: 开始正常工作
    → 输出融合结果

结果: 渐进式初始化，等待各模块就绪
```

### 场景4: 停车

```
t0: DR检测到停车
    → dr_state.is_parking_ = true

t1: Loc接收到停车状态
    → localization_result_.is_parking_ = true

t2: PGO接收到Loc结果
    → is_parking_ = true
    → 直接输出parking_result_
    → 不进行优化和外推

结果: 保持位姿不变，节省计算
```

---

## 九、优势与局限

### 9.1 系统优势

✅ **多源融合**: 综合LO、DR、Loc的优点
✅ **高频输出**: IMU频率(~200Hz)的融合结果
✅ **鲁棒性**: 单源失效仍可工作
✅ **全局一致**: Loc提供无漂移定位
✅ **平滑性**: Smoother保证轨迹平滑
✅ **可靠性监控**: 实时检测各模块状态

### 9.2 系统局限

❌ **延迟**: Loc处理延迟~50-100ms
❌ **依赖性**: 需要预先建图
❌ **计算负载**: 三个模块并行计算
❌ **参数敏感**: 多达30+参数需要调优
❌ **退化场景**: 长走廊、空旷场地挑战大

---

## 十、总结

### 核心设计思想

1. **分层架构**: LIO→LidarLoc→PGO，各司其职
2. **多源互补**: LO精确、Loc全局、DR高频
3. **可靠性优先**: 持续监控+降级策略
4. **高频响应**: 外推机制保证实时性
5. **平滑一致**: 图优化+Smoother保证质量

### 数据流总结

```
传感器 → 预处理 → LIO(高频局部) → LidarLoc(低频全局) → PGO(融合外推) → 输出
  ↓                                      ↓                    ↓
 IMU → DR(高频递推) ──────────────────────┴────────────────────┘
```

### 关键技术点

- **位姿插值**: 在不同频率数据间同步
- **外推策略**: DR为主,LO为辅,有限距离
- **可靠性链**: Loc→LO→PGO层层传递
- **图优化**: 多约束融合求最优解
- **平滑处理**: 避免跳跃和抖动

这个系统通过精心设计的三层架构,实现了高频、准确、鲁棒的定位输出! 🎯
