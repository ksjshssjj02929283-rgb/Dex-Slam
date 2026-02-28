# AddPGOFrame 函数详解

## 概述
`AddPGOFrame` 是PGO系统的核心函数，负责接收新的PGOFrame、构建优化问题、执行图优化并维护滑动窗口。每次收到LidarLoc结果时触发一次。

---

## 完整流程图

```
AddPGOFrame(pgo_frame)
    │
    ├─► 步骤1: 时间戳检查
    │   └─► 防止时间回退
    │
    ├─► 步骤2: 插值分配相对位姿
    │   ├─► AssignLidarOdomPoseIfNeeded
    │   │   └─► 从LO队列插值
    │   └─► AssignDRPoseIfNeeded
    │       └─► 从DR队列插值
    │
    ├─► 步骤3: 有效性检查
    │   └─► 检查LidarLoc是否有效
    │
    ├─► 步骤4: 分配Frame ID
    │   └─► 更新current_frame_
    │
    ├─► 步骤5: 构建并求解优化问题
    │   ├─► BuildProblem
    │   │   ├─► AddVertex (添加顶点)
    │   │   ├─► AddLidarLocFactors (绝对约束)
    │   │   ├─► AddLidarOdomFactors (相对约束)
    │   │   └─► AddPriorFactors (先验约束)
    │   │
    │   └─► RunOptimization
    │       ├─► InitializeOptimization
    │       ├─► Optimize(5) (5次迭代)
    │       └─► 提取优化结果
    │
    ├─► 步骤6: 收集统计信息
    │   └─► CollectOptimizationStatistics
    │       ├─► 计算各种误差
    │       └─► 更新result_
    │
    └─► 步骤7: 滑动窗口维护
        └─► SlideWindowAdaptively
            ├─► 判断窗口大小
            └─► 边缘化最旧帧
```

---

## 详细步骤拆解

### 步骤1: 时间戳检查与增量计算

```cpp
void PGOImpl::AddPGOFrame(std::shared_ptr<PGOFrame> pgo_frame) {
    assert(pgo_frame != nullptr);
    
    // 1.1 时间戳检查
    if (last_frame_ != nullptr) {
        const double adjacent_dalta_t = pgo_frame->timestamp_ - last_frame_->timestamp_;
        if (adjacent_dalta_t < 0.) {
            LOG(WARNING) << "PGO received pgoframe, however timestamp rollback for " 
                        << adjacent_dalta_t << "senonds!";
            return;
        }
        // 记录与上一帧的时间间隔
        pgo_frame->lidar_loc_delta_t_ = adjacent_dalta_t;
    }
    
    // 1.2 调试输出
    if (debug_) {
        LOG(INFO) << "inserting pgo frame, it's timestamp is " 
                  << std::setprecision(18) << pgo_frame->timestamp_;
    }
}
```

**作用：**
- 防止时间回退导致的系统异常
- 计算相邻帧的时间间隔（用于统计和分析）
- 提供调试信息输出

**关键检查：**
- `adjacent_dalta_t < 0`: 时间回退，直接返回
- `last_frame_ != nullptr`: 确保不是第一帧

---

### 步骤2: 插值分配相对位姿观测

```cpp
// 2.1 尝试分配LidarOdom和DR位姿
bool interp_lio_success = AssignLidarOdomPoseIfNeeded(pgo_frame);
bool interp_dr_success = AssignDRPoseIfNeeded(pgo_frame);

// 2.2 至少需要一个成功
if (!interp_lio_success && !interp_dr_success) {
    LOG(ERROR) << "PGO received pgo frame, but assign relative pose failed!";
    return;
}
```

**为什么需要插值？**
- LidarLoc触发PGO的时刻 ≠ LO/DR的精确时刻
- 需要在LO/DR队列中插值获取对应时刻的位姿
- 保证时间对齐，提高优化精度

#### 2.1 AssignLidarOdomPoseIfNeeded - LO插值

```cpp
bool PGOImpl::AssignLidarOdomPoseIfNeeded(std::shared_ptr<PGOFrame> frame) {
    assert(frame != nullptr);
    SE3 interp_pose;
    Vec3d interp_vel_b;
    NavState best_match;
    bool lo_interp_done = false;

    // A. 在LO队列中插值
    lo_interp_done = math::PoseInterp<NavState>(
        frame->timestamp_,                          // 目标时间
        lidar_odom_pose_queue_,                     // LO队列
        [](const NavState& nav_state) { return nav_state.timestamp_; },
        [](const NavState& nav_state) { return nav_state.GetPose(); },
        interp_pose,                                // 输出: 插值位姿
        best_match);                                // 输出: 最近匹配

    // B. 更新LO状态信息
    if (lo_interp_done) {
        UpdateLidarOdomStatusInFrame(best_match, frame);
    }

    // C. 如果Frame还未设置LO，则设置之
    if (!frame->lidar_odom_set_) {
        if (lo_interp_done) {
            interp_vel_b = best_match.GetRot().matrix().transpose() * best_match.GetVel();
            frame->lidar_odom_set_ = true;
            frame->lidar_odom_valid_ = true;
            frame->lidar_odom_pose_ = interp_pose;
            frame->lidar_odom_vel_ = interp_vel_b;
            return true;
        } else {
            // LO可能比LidarLoc慢
            LOG(WARNING) << "PGOFrame (frame_id " << frame->frame_id_ 
                        << ") Interpolate on lidarOdom Failed!";
            return false;
        }
    } else {
        // 已经设置过了
        return true;
    }
}
```

**关键点：**

1. **插值算法**: `math::PoseInterp`
   - 在队列中找到目标时间前后的位姿
   - 使用SLERP(球面线性插值)进行位姿插值
   - 返回最接近的匹配帧

2. **状态更新**: `UpdateLidarOdomStatusInFrame`
   ```cpp
   void PGOImpl::UpdateLidarOdomStatusInFrame(NavState& lio_result, 
                                               std::shared_ptr<PGOFrame> frame) {
       // 更新置信度权重
       frame->lidar_odom_normalized_weight_ = lio_result.confidence_ * Vec6d::Ones();
       
       // 退化检测 (这里的逻辑尚未完善)
       Eigen::Array3d trans_confidence = {1.0, 1.0, 1.0};
       Eigen::Array3d rot_confidence = {1.0, 1.0, 1.0};
       
       if ((trans_confidence < kLidarOdomTransDegenThres).any()) {
           frame->lidar_odom_trans_degenerated = true;
       }
       if ((rot_confidence < kLidarOdomRotDegenThres).any()) {
           frame->lidar_odom_rot_degenerated = true;
       }
   }
   ```

3. **速度转换**:
   ```cpp
   interp_vel_b = best_match.GetRot().matrix().transpose() * best_match.GetVel();
   // 从世界系转到车体系
   ```

#### 2.2 AssignDRPoseIfNeeded - DR插值

```cpp
bool PGOImpl::AssignDRPoseIfNeeded(std::shared_ptr<PGOFrame> frame) {
    assert(frame != nullptr);
    SE3 interp_pose;
    Vec3d interp_vel_b;
    NavState best_match;
    bool dr_interp_done = false;

    // A. 在DR队列中插值
    dr_interp_done = math::PoseInterp<NavState>(
        frame->timestamp_, 
        dr_pose_queue_,
        [](const NavState& nav_state) { return nav_state.timestamp_; },
        [](const NavState& nav_state) { return nav_state.GetPose(); },
        interp_pose, 
        best_match);

    // B. 如果Frame的DR未设置，则设置之
    if (!frame->dr_valid_) {
        if (dr_interp_done) {
            interp_vel_b = best_match.GetRot().matrix().transpose() * best_match.GetVel();
            frame->dr_set_ = true;
            frame->dr_valid_ = true;
            frame->dr_pose_ = interp_pose;
            frame->dr_vel_b_ = interp_vel_b;
            return true;
        } else {
            // DR理论上应该比LidarLoc快
            LOG(WARNING) << "PGOFrame (frame_id" << frame->frame_id_ 
                        << ") Interpolate on DR Failed!";
            return false;
        }
    } else {
        return true;
    }
}
```

**DR与LO的区别：**
| 特性 | LidarOdom | DR |
|------|-----------|-----|
| 频率 | ~10-20Hz | ~200Hz |
| 精度 | 高 | 中 |
| 可靠性检查 | 有 | 无 |
| 退化检测 | 有 | 无 |
| 速度信息 | 有 | 有 |

---

### 步骤3: 有效性检查

```cpp
// 3.1 检查是否在地图中
is_in_map_ = pgo_frame->lidar_loc_set_ && pgo_frame->lidar_loc_valid_;

// 3.2 拒绝无效Frame
if (!is_in_map_) {
    LOG(ERROR) << "PGO received PGOFrame with lidar_loc_set_(" << pgo_frame->lidar_loc_set_
               << "), lidar_loc_valid_(" << pgo_frame->lidar_loc_valid_ << "); Reject It!";
    return;
}
```

**检查条件：**
- `lidar_loc_set_`: LidarLoc是否已设置
- `lidar_loc_valid_`: LidarLoc是否有效
- 两者都为true才接受这个Frame

**为什么严格？**
- PGO由LidarLoc触发
- 没有有效的全局观测，优化无意义
- 保证绝对约束的存在

---

### 步骤4: 分配Frame ID并加入窗口

```cpp
// 4.1 分配全局唯一ID
pgo_frame->frame_id_ = accumulated_frame_id_++;

// 4.2 设置为当前帧
current_frame_ = pgo_frame;

// 4.3 加入ID索引
frames_by_id_.emplace(pgo_frame->frame_id_, pgo_frame);

// 4.4 加入帧队列
frames_.emplace_back(pgo_frame);
```

**数据结构：**
```cpp
std::vector<std::shared_ptr<PGOFrame>> frames_;              // 按时间顺序
std::map<FrameId, std::shared_ptr<PGOFrame>> frames_by_id_; // ID索引
std::shared_ptr<PGOFrame> current_frame_;                    // 当前帧指针
FrameId accumulated_frame_id_;                               // 全局ID计数器
```

**作用：**
- `frames_`: 保持时间顺序，方便遍历
- `frames_by_id_`: 快速根据ID查找
- `current_frame_`: 指向最新帧，方便访问

---

### 步骤5: 运行优化 - RunOptimization

```cpp
void PGOImpl::RunOptimization() {
    // 5.1 构建优化问题
    BuildProblem();

    // 5.2 初始化优化器
    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(options_.verbose_);
    
    // 5.3 执行优化（5次迭代）
    optimizer_->Optimize(5);

    // 5.4 提取优化结果
    for (const auto& frame : frames_) {
        auto v = std::dynamic_pointer_cast<miao::VertexSE3>(
            optimizer_->GetVertex(frame->frame_id_));
        if (v == nullptr) {
            continue;
        }

        ++frame->opti_times_;                     // 优化次数+1
        frame->last_opti_pose_ = frame->opti_pose_;  // 保存上一次结果
        frame->opti_pose_ = SE3(v->Estimate().matrix());  // 更新优化位姿
    }
}
```

**优化流程：**
1. 构建图结构（顶点+边）
2. 初始化优化器状态
3. Levenberg-Marquardt优化（5次迭代）
4. 提取所有顶点的优化结果

**为什么5次迭代？**
- 平衡精度和效率
- PGO是增量式的，每次只优化新加入的观测
- 5次迭代通常足够收敛

#### 5.1 BuildProblem - 构建优化问题

```cpp
void PGOImpl::BuildProblem() {
    // 添加顶点
    AddVertex();

    // 添加因子（约束边）
    if (is_in_map_) {
        AddLidarLocFactors();    // 绝对约束
    }
    
    AddLidarOdomFactors();      // 相对约束
    AddPriorFactors();          // 先验约束
}
```

**优化问题结构：**
```
图结构:
  顶点(Vertices): 每个PGOFrame的位姿 (SE3)
  边(Edges): 各种约束
    - LidarLoc: 一元边 (绝对位姿)
    - LidarOdom: 二元边 (相对运动)
    - Prior: 一元边 (边缘化先验)
```

##### 5.1.1 AddVertex - 添加位姿顶点

```cpp
void PGOImpl::AddVertex() {
    // 创建SE3顶点
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(current_frame_->frame_id_);
    v->SetEstimate(current_frame_->opti_pose_);  // 初始值

    // 添加到优化器
    optimizer_->AddVertex(v);

    /// 重要: 在incremental模式下，optimizer可能修改顶点ID
    /// 例如替换掉被边缘化的顶点，复用其ID
    current_frame_->frame_id_ = v->GetId();

    // 保存顶点指针
    vertices_.emplace_back(v);
}
```

**Incremental优化模式：**
- 固定窗口大小（如5帧）
- 新帧加入时，最旧帧被边缘化
- 优化器可能复用被移除顶点的ID
- **必须更新frame_id以保持一致性**

##### 5.1.2 AddLidarLocFactors - 添加激光定位约束

```cpp
void PGOImpl::AddLidarLocFactors() {
    // 检查是否设置
    if (!current_frame_->lidar_loc_set_) {
        return;
    }

    // A. 构建观测
    SE3 loc_obs_pose = current_frame_->lidar_loc_pose_;
    
    // B. 构建协方差矩阵
    Mat6d loc_obs_cov = Mat6d::Zero();
    loc_obs_cov(0, 0) = lidar_loc_noise_[0] * lidar_loc_noise_[0];  // x
    loc_obs_cov(1, 1) = lidar_loc_noise_[1] * lidar_loc_noise_[1];  // y
    loc_obs_cov(2, 2) = lidar_loc_noise_[2] * lidar_loc_noise_[2];  // z
    loc_obs_cov(3, 3) = lidar_loc_noise_[3] * lidar_loc_noise_[3];  // roll
    loc_obs_cov(4, 4) = lidar_loc_noise_[4] * lidar_loc_noise_[4];  // pitch
    loc_obs_cov(5, 5) = lidar_loc_noise_[5] * lidar_loc_noise_[5];  // yaw
    
    // C. 计算信息矩阵（协方差的逆）
    Mat6d loc_obs_info = loc_obs_cov.inverse();
    
    // D. 应用归一化权重（基于置信度）
    Vec6d loc_obs_weight = current_frame_->lidar_loc_normalized_weight_;
    loc_obs_info(0, 0) *= loc_obs_weight[0];
    loc_obs_info(1, 1) *= loc_obs_weight[1];
    loc_obs_info(2, 2) *= loc_obs_weight[2];
    loc_obs_info(3, 3) *= loc_obs_weight[3];
    loc_obs_info(4, 4) *= loc_obs_weight[4];
    loc_obs_info(5, 5) *= loc_obs_weight[5];

    // E. 创建一元边（绝对位姿约束）
    auto e = std::make_shared<miao::EdgeSE3Prior>();
    e->SetVertex(0, optimizer_->GetVertex(current_frame_->frame_id_));
    e->SetMeasurement(loc_obs_pose);
    e->SetInformation(loc_obs_info);

    // F. 添加鲁棒核函数（Huber）
    auto rk = std::make_shared<miao::RobustKernelHuber>();
    rk->SetDelta(options_.lidar_loc_outlier_th);  // 异常值阈值
    e->SetRobustKernel(rk);

    // G. 添加到优化器
    optimizer_->AddEdge(e);
    lidar_loc_edges_.emplace_back(e);
}
```

**信息矩阵计算：**
```
Information = (Covariance)^-1 × Weight

其中:
  Covariance: 基础噪声模型
  Weight: 置信度调整 [0.5, 1.0]
```

**权重策略：**
```cpp
// 在ProcessLidarLoc中设置
if (loc_result.confidence_ > 0.6) {
    frame->lidar_loc_normalized_weight_ = Vec6d::Ones();  // 权重=1.0
} else {
    const double clap_weight = std::max(loc_result.confidence_, 0.5);
    frame->lidar_loc_normalized_weight_ = clap_weight * Vec6d::Ones();  // 权重∈[0.5,0.6]
}
```

**Huber鲁棒核函数：**
```
误差函数:
  ρ(e) = e^2                     if |e| ≤ δ
  ρ(e) = 2δ|e| - δ^2            if |e| > δ

作用: 降低异常值的影响
```

##### 5.1.3 AddLidarOdomFactors - 添加LO相对约束

```cpp
void PGOImpl::AddLidarOdomFactors() {
    size_t num = lo_relative_constraints_num_;  // 默认=2

    // 从当前帧向历史帧添加约束
    for (auto iter = frames_.rbegin(); iter != frames_.rend(); ++iter) {
        auto frame = *iter;
        
        // 跳过当前帧自己
        if (frame == current_frame_) {
            continue;
        }

        // 只与最近num帧建立约束
        if ((current_frame_->frame_id_ - frame->frame_id_) >= num) {
            continue;
        }

        auto pre_key_frame = frame;
        auto cur_key_frame = current_frame_;
        
        // 检查LO有效性
        if (!pre_key_frame->lidar_odom_valid_ || !cur_key_frame->lidar_odom_valid_) {
            continue;
        }

        // A. 计算相对观测
        SE3 lo_obs_pose = pre_key_frame->lidar_odom_pose_.inverse() * 
                          cur_key_frame->lidar_odom_pose_;

        // B. 构建协方差
        Mat6d lo_obs_cov = Mat6d::Zero();
        const Vec6d lo_noise = lidar_odom_rel_noise_;
        lo_obs_cov(0, 0) = lo_noise[0] * lo_noise[0];
        lo_obs_cov(1, 1) = lo_noise[1] * lo_noise[1];
        lo_obs_cov(2, 2) = lo_noise[2] * lo_noise[2];
        lo_obs_cov(3, 3) = lo_noise[3] * lo_noise[3];
        lo_obs_cov(4, 4) = lo_noise[4] * lo_noise[4];
        lo_obs_cov(5, 5) = lo_noise[5] * lo_noise[5];

        // C. 创建二元边（相对位姿约束）
        auto e = std::make_shared<miao::EdgeSE3>();
        auto v1 = optimizer_->GetVertex(pre_key_frame->frame_id_);
        auto v2 = optimizer_->GetVertex(cur_key_frame->frame_id_);

        if (v1 == nullptr || v2 == nullptr) {
            continue;
        }

        e->SetVertex(0, v1);
        e->SetVertex(1, v2);
        e->SetMeasurement(lo_obs_pose);

        Mat6d rel_obs_info = lo_obs_cov.inverse();
        e->SetInformation(rel_obs_info);

        // D. 添加鲁棒核（Cauchy）
        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.lidar_odom_ang_noise);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        lidar_odom_edges_.emplace_back(e);
    }
}
```

**约束连接策略：**
```
当前帧: Frame_N
历史帧: Frame_N-1, Frame_N-2

约束: 
  Frame_N-1 ←→ Frame_N  (直接相邻)
  Frame_N-2 ←→ Frame_N  (跨帧约束)

作用: 增强图的稳定性和鲁棒性
```

**为什么使用Cauchy核而非Huber？**
```
Cauchy: ρ(e) = δ^2 * log(1 + (e/δ)^2)
  - 对大误差更加宽容
  - 适用于可能存在较大相对误差的场景

Huber: ρ(e) = 2δ|e| - δ^2 (当|e|>δ)
  - 对大误差更严格
  - 适用于期望误差分布更集中的场景
```

##### 5.1.4 AddPriorFactors - 添加先验约束

```cpp
void PGOImpl::AddPriorFactors() {
    auto frame = current_frame_;

    // 只有被边缘化的第一帧才有先验约束
    if (frame->prior_set_ && frame->prior_valid_) {
        SE3 prior_obs_pose = frame->prior_pose_;

        // A. 创建一元边
        auto e = std::make_shared<miao::EdgeSE3Prior>();
        e->SetVertex(0, optimizer_->GetVertex(frame->frame_id_));
        e->SetMeasurement(prior_obs_pose);

        // B. 使用LidarLoc的噪声模型
        Vec6d prior_noise = lidar_loc_noise_;

        Mat6d prior_obs_cov = Mat6d::Zero();
        prior_obs_cov(0, 0) = prior_noise[0] * prior_noise[0];
        prior_obs_cov(1, 1) = prior_noise[1] * prior_noise[1];
        prior_obs_cov(2, 2) = prior_noise[2] * prior_noise[2];
        prior_obs_cov(3, 3) = prior_noise[3] * prior_noise[3];
        prior_obs_cov(4, 4) = prior_noise[4] * prior_noise[4];
        prior_obs_cov(5, 5) = prior_noise[5] * prior_noise[5];

        // C. 应用权重
        Mat6d prior_obs_info = prior_obs_cov.inverse();
        const Vec6d prior_obs_weight = frame->prior_normalized_weight_;
        prior_obs_info(0, 0) *= prior_obs_weight[0];
        prior_obs_info(1, 1) *= prior_obs_weight[1];
        prior_obs_info(2, 2) *= prior_obs_weight[2];
        prior_obs_info(3, 3) *= prior_obs_weight[3];
        prior_obs_info(4, 4) *= prior_obs_weight[4];
        prior_obs_info(5, 5) *= prior_obs_weight[5];
        e->SetInformation(prior_obs_info);

        // D. 不使用鲁棒核（先验是可信的）
        optimizer_->AddEdge(e);
        prior_edges_.emplace_back(e);
    }
}
```

**先验约束来源：**
- 从边缘化过程得到
- 代表被移除帧的累积信息
- 防止窗口外信息丢失

---

### 步骤6: 收集优化统计信息

```cpp
void PGOImpl::CollectOptimizationStatistics() {
    // 6.1 打印各类约束的统计信息
    if (debug_) {
        LOG(INFO) << "LidarLoc       -- " + print_info(lidar_loc_edges_, 
                                                       options_.lidar_loc_outlier_th);
        LOG(INFO) << "LidarOdom      -- " + print_info(lidar_odom_edges_);
        LOG(INFO) << "DR             -- " + print_info(dr_edges_);
        LOG(INFO) << "Marginal Prior -- " + print_info(prior_edges_);
    }

    // 6.2 更新最终结果
    if (frames_.size() < kMinNumRequiredForOptimization) {
        UpdateFinalResultByLastFrame();
    } else {
        UpdateFinalResultByWindow();
    }
    
    // 6.3 发布结果（如果有回调）
    if (output_func_) {
        output_func_(result_);
    }
}
```

#### 6.1 print_info - 约束统计

```cpp
template <typename T>
std::string print_info(const std::vector<T>& edges, double th = 0) {
    std::vector<double> chi2;
    
    // 收集所有边的卡方误差
    for (auto& edge : edges) {
        if (edge->Level() == 0) {  // 只统计active的边
            chi2.push_back(edge->Chi2());
        }
    }

    // 排序
    std::sort(chi2.begin(), chi2.end());
    
    // 计算统计量
    double ave_chi2 = std::accumulate(chi2.begin(), chi2.end(), 0.0) / chi2.size();
    
    // 格式化输出
    boost::format fmt("数量: %d, 均值: %f, 中位数: %f, 0.1分位: %f, 0.9分位: %f, 最大值: %f, 阈值: %f\n");
    if (!chi2.empty()) {
        return (fmt % chi2.size() 
                    % ave_chi2 
                    % chi2[chi2.size() / 2]           // 中位数
                    % chi2[int(chi2.size() * 0.1)]    // 10%分位
                    % chi2[int(chi2.size() * 0.9)]    // 90%分位
                    % chi2.back()                      // 最大值
                    % th).str();                       // 阈值
    }
    return std::string("");
}
```

**卡方误差(Chi2)含义：**
```
Chi2 = e^T × Ω × e

其中:
  e: 残差向量 (观测 - 估计)
  Ω: 信息矩阵 (协方差的逆)

物理意义: 
  - 观测与估计的加权平方误差
  - 越小越好
  - 大于阈值视为异常值
```

#### 6.2 UpdateFinalResultByWindow - 更新融合结果

```cpp
void PGOImpl::UpdateFinalResultByWindow() {
    auto& lf = frames_.back();  // 最新帧

    // A. 复制Frame信息到Result
    PGOFrameToResult(lf, result_);

    // B. 计算LO误差
    auto& ff = frames_.front();  // 最旧帧
    if (ff->lidar_odom_valid_ && lf->lidar_odom_valid_) {
        // 用LO从第一帧外推到最后一帧
        SE3 lo_pose_w = ff->opti_pose_ * ff->lidar_odom_pose_.inverse() * 
                        lf->lidar_odom_pose_;
        
        // 计算与优化位姿的误差
        Vec3d lo_error = lo_pose_w.translation() - lf->opti_pose_.translation();
        
        // 转到车体坐标系
        Vec3d lo_err_body = lf->opti_pose_.so3().inverse() * lo_error;
        
        // 分离纵向和横向误差
        result_.lidar_odom_error_vert_ = fabs(lo_err_body[0]);  // 纵向(前后)
        result_.lidar_odom_error_hori_ = fabs(lo_err_body[1]);  // 横向(左右)
    } else {
        result_.lidar_odom_error_vert_ = -999;  // 无效标记
        result_.lidar_odom_error_hori_ = -999;
    }

    // C. 设置有效标志
    result_.valid_ = true;
}
```

**PGOFrameToResult - 信息复制：**
```cpp
void PGOImpl::PGOFrameToResult(const PGOFramePtr& frame, LocalizationResult& result) {
    // 检查时间延迟
    if ((result.timestamp_ - frame->timestamp_) > 0.3) {
        return;  // 相差太多，放弃
    }

    // 基本信息
    result.timestamp_ = frame->timestamp_;
    result.pose_ = frame->opti_pose_;  // ⭐ 优化后的位姿

    // LidarLoc信息
    result.lidar_loc_valid_ = frame->lidar_loc_valid_;
    result.lidar_loc_inlier_ = frame->lidar_loc_inlier_;
    result.lidar_loc_delta_t_ = frame->lidar_loc_delta_t_;
    result.confidence_ = frame->confidence_;
    
    // 计算Loc误差
    Vec3d loc_error = frame->lidar_loc_pose_.translation() - result.pose_.translation();
    Vec3d loc_err_body = result.pose_.so3().inverse() * loc_error;
    result.lidar_loc_error_vert_ = fabs(loc_err_body[0]);
    result.lidar_loc_error_hori_ = fabs(loc_err_body[1]);

    // 相对位姿信息
    result.rel_pose_set_ = frame->lidar_odom_set_;
    result.rel_pose_ = frame->lidar_odom_pose_;
    result.vel_b_ = frame->dr_vel_b_;
    result.lidar_odom_delta_t_ = frame->lidar_odom_delta_t_;
    result.dr_delta_t_ = frame->dr_delta_t_;
}
```

---

### 步骤7: 滑动窗口维护

```cpp
void PGOImpl::SlideWindowAdaptively() {
    // 7.1 检查窗口大小
    if (frames_.size() < options_.PGO_MAX_FRAMES) {
        return;  // 窗口未满，不需要滑动
    }

    // 7.2 边缘化最旧帧
    auto frame_to_remove = *(frames_.begin());      // 第一帧
    auto frame_to_keep = *(++frames_.begin());      // 第二帧
    
    // 7.3 执行边缘化
    Marginalize(frame_to_remove, frame_to_keep);
    
    // 7.4 移除最旧帧
    frames_.erase(frames_.begin());
    frames_by_id_.erase(frame_to_remove->frame_id_);
}
```

**固定窗口策略：**
```
窗口大小: PGO_MAX_FRAMES = 5

示例:
  初始: [F1, F2, F3, F4, F5]
  新帧: F6 到达
  
  操作:
    1. 边缘化 F1 → F2获得先验
    2. 移除 F1
    3. 添加 F6
  
  结果: [F2, F3, F4, F5, F6]
```

#### Marginalize - 边缘化处理

```cpp
void PGOImpl::Marginalize(const PGOFramePtr& frame_to_remove, 
                          const PGOFramePtr& frame_to_keep) {
    /// TODO: 实现完整的边缘化算法
    /// 当前是简化版本
    
    // 将被移除帧的优化位姿作为下一帧的先验
    frame_to_keep->prior_pose_ = frame_to_keep->opti_pose_;
    frame_to_keep->prior_normalized_weight_ = Vec6d::Ones();
    frame_to_keep->prior_set_ = true;
    frame_to_keep->prior_valid_ = true;
}
```

**理想的边缘化：**
```
1. 构建Hessian矩阵:
   H = [H_mm  H_mr]
       [H_rm  H_rr]
   
   其中:
     H_mm: 被边缘化帧相关
     H_rr: 保留帧相关
     H_mr: 交叉项

2. Schur补:
   H_rr' = H_rr - H_rm × H_mm^-1 × H_mr

3. 得到先验:
   prior_cov = (H_rr')^-1
   prior_pose = frame_to_keep->opti_pose_

当前实现: 简化为直接使用优化位姿作为先验
```

---

## 数据流总结

### 输入数据
```
PGOFrame (from LidarLoc):
  ├─ timestamp_
  ├─ lidar_loc_pose_ (绝对位姿)
  ├─ lidar_loc_valid_
  ├─ confidence_
  └─ lidar_loc_normalized_weight_

LidarOdom队列:
  └─ NavState × N (位姿序列)

DR队列:
  └─ NavState × N (位姿序列)
```

### 处理过程
```
1. 插值 → 获取对齐的LO和DR位姿
2. 构图 → 添加顶点和约束边
3. 优化 → Levenberg-Marquardt求解
4. 提取 → 获取优化后的位姿
5. 统计 → 计算各种误差指标
6. 滑窗 → 边缘化最旧帧
```

### 输出数据
```
LocalizationResult:
  ├─ pose_ (融合优化后的位姿) ⭐
  ├─ confidence_
  ├─ lidar_loc_error_vert_ (Loc纵向误差)
  ├─ lidar_loc_error_hori_ (Loc横向误差)
  ├─ lidar_odom_error_vert_ (LO纵向误差)
  ├─ lidar_odom_error_hori_ (LO横向误差)
  └─ valid_
```

---

## 约束类型总结

| 约束类型 | 边类型 | 鲁棒核 | 作用 | 频率 |
|---------|--------|--------|------|------|
| **LidarLoc** | 一元边 | Huber | 提供全局位姿 | 每次触发 |
| **LidarOdom** | 二元边 | Cauchy | 帧间相对运动 | 向前2帧 |
| **Prior** | 一元边 | 无 | 边缘化先验 | 滑窗第一帧 |
| **DR** | 二元边 | 无 | 帧间相对运动 | (未启用) |

---

## 关键参数

### 噪声模型
```cpp
// 位置噪声 (米)
lidar_loc_pos_noise: 0.3
lidar_odom_pos_noise: 0.3
dr_pos_noise: 1.0

// 角度噪声 (弧度)
lidar_loc_ang_noise: 1.0 * DEG2RAD
lidar_odom_ang_noise: 1.0 * DEG2RAD
dr_ang_noise: 0.5 * DEG2RAD
```

### 异常值阈值
```cpp
lidar_loc_outlier_th: 5.0  // Huber核阈值
```

### 窗口参数
```cpp
PGO_MAX_FRAMES: 5           // 最大帧数
lo_relative_constraints_num_: 2  // LO约束连接数
```

### 优化参数
```cpp
max_iteration: 5            // 每次优化迭代次数
algorithm: LM               // Levenberg-Marquardt
incremental_mode: true      // 增量模式
```

---

## 典型场景分析

### 场景1: 正常添加Frame

```
输入: 
  PGOFrame(t=1.0s, loc_pose, confidence=2.3)
  
流程:
  1. 插值 → LO(t=1.0s), DR(t=1.0s)
  2. 检查 → lidar_loc_valid=true
  3. 构图 → 添加1个顶点 + 3条边(Loc+LO×2)
  4. 优化 → 5次LM迭代
  5. 提取 → opti_pose
  6. 统计 → 计算误差
  7. 滑窗 → frames_.size()=4, 无需滑动

输出:
  result_.pose_ = opti_pose
  result_.valid_ = true
```

### 场景2: 窗口已满，需要边缘化

```
输入:
  PGOFrame(t=2.5s)
  当前窗口: [F1, F2, F3, F4, F5]

流程:
  1~6. (同上)
  7. 滑窗:
     - 边缘化 F1 → F2.prior_pose_
     - 移除 F1
     - frames_ = [F2, F3, F4, F5, F6]

下次优化:
  F2会有prior约束，代表F1的累积信息
```

### 场景3: LO失效

```
输入:
  PGOFrame(t=1.5s)
  LO队列为空或插值失败

流程:
  1. 插值 → LO失败, DR成功
  2. 检查 → interp_lio_success=false, interp_dr_success=true
  3. 继续 → 只使用DR约束（如果启用）
  4~7. (同上)

结果:
  优化只依赖 LidarLoc + DR
  精度可能降低
```

---

## 性能考虑

### 计算复杂度
```
每次AddPGOFrame:
  - 插值: O(log N)  // 二分查找
  - 添加顶点: O(1)
  - 添加边: O(W)    // W=窗口大小
  - 优化: O(W^3)    // LM迭代
  - 统计: O(E)      // E=边数

总复杂度: O(W^3)  // W=5, 可接受
```

### 内存占用
```
窗口: 5 frames
每帧: 
  - 位姿: 7×8 = 56 bytes
  - 其他: ~200 bytes
  
边:
  - LidarLoc: 5个
  - LidarOdom: ~10个
  - Prior: 1个
  每条边: ~100 bytes
  
总计: ~5KB (可忽略)
```

---

## 总结

### AddPGOFrame的核心作用

1. **时间对齐**: 通过插值确保所有观测在同一时刻
2. **图构建**: 建立位姿图的顶点和约束边
3. **多源融合**: 综合LidarLoc、LO、DR的优势
4. **增量优化**: 高效的滑动窗口优化
5. **质量评估**: 实时计算各种误差指标

### 设计亮点

✅ **增量式优化**: 固定窗口大小，复杂度可控
✅ **鲁棒性**: 使用鲁棒核函数抑制异常值
✅ **自适应权重**: 根据置信度调整约束强度
✅ **多连接**: 跨帧约束增强稳定性
✅ **边缘化**: 保留历史信息，防止漂移

### 改进空间

⚠️ **边缘化**: 当前是简化实现，理想应该实现完整Schur补
⚠️ **DR约束**: 代码中未启用DR边，可能浪费了信息
⚠️ **退化检测**: LO退化检测逻辑尚未完善
⚠️ **参数自适应**: 噪声参数固定，可考虑自适应调整

这个PGO系统通过精心设计的图优化框架，实现了高质量的多源融合定位！🎯
