//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    config.incremental_mode_ = true;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    // 初始化 LIO-SAM 风格的 KD-Tree 搜索结构
    kf_poses_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    kdtree_history_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
        try {
            options_.history_search_time_diff_ =
                yaml.GetValue<double>("loop_closing", "history_search_time_diff");
        } catch (const YAML::Exception&) {
            LOG(WARNING) << "loop_closing.history_search_time_diff not set in yaml, using default: "
                         << options_.history_search_time_diff_;
        }
        try {
            options_.history_keyframe_search_num_ =
                yaml.GetValue<int>("loop_closing", "history_keyframe_search_num");
        } catch (const YAML::Exception&) {}
        try {
            options_.icp_filter_size_ =
                yaml.GetValue<float>("loop_closing", "icp_filter_size");
        } catch (const YAML::Exception&) {}
        try {
            options_.min_cur_cloud_size_ =
                yaml.GetValue<int>("loop_closing", "min_cur_cloud_size");
        } catch (const YAML::Exception&) {}
        try {
            options_.min_prev_cloud_size_ =
                yaml.GetValue<int>("loop_closing", "min_prev_cloud_size");
        } catch (const YAML::Exception&) {}
        try {
            options_.max_candidates_per_check_ =
                yaml.GetValue<int>("loop_closing", "max_candidates_per_check");
        } catch (const YAML::Exception&) {}
        try {
            options_.icp_fitness_score_th_ =
                yaml.GetValue<float>("loop_closing", "icp_fitness_score_th");
        } catch (const YAML::Exception&) {}
        try {
            options_.icp_max_correspondence_dist_ =
                yaml.GetValue<float>("loop_closing", "icp_max_correspondence_dist");
        } catch (const YAML::Exception&) {}
    }

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 清空上一次的调试数据
    loop_debug_data_.valid = false;

    // 检测回环候选（此时 kf_poses_cloud_ 不含当前帧，搜索在历史帧中进行）
    DetectLoopCandidates();

    // if (options_.verbose_) {
    //     LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    // }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    // 将当前帧位置加入 KD-Tree 点云（下一帧的搜索才能用到当前帧）
    Vec3d t = cur_kf_->GetOptPose().translation();
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(t.x());
    pt.y = static_cast<float>(t.y());
    pt.z = static_cast<float>(t.z());
    kf_poses_cloud_->push_back(pt);

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    // kf_poses_cloud_ 为空说明还没有历史帧可供搜索
    if (kf_poses_cloud_->empty()) {
        return;
    }

    uint64_t cur_id = cur_kf_->GetID();

    // 第一次进入时初始化冷却起点，直接返回（与原有逻辑保持一致）
    // 确保冷却期从第 0 帧开始计数，避免系统刚启动就触发回环检测
    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    // ---- LIO-SAM 风格：loopIndexContainer 防重复 ----
    // 如果当前帧已经参与过回环检测，跳过（避免对同一位置重复触发）
    if (loop_index_container_.count(cur_id) > 0) {
        LOG(INFO) << "lc: skip kf " << cur_id << " (already in loop_index_container)";
        return;
    }

    // 冷却期检查（继承 lightning-lm 的 loop_kf_gap_ 机制）
    if (cur_id - last_loop_kf_->GetID() <= static_cast<uint64_t>(options_.loop_kf_gap_)) {
        // LOG(INFO) << "lc: skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    // ---- LIO-SAM 风格：KD-Tree 半径搜索 ----
    // kf_poses_cloud_ 不含当前帧，索引 i 对应 all_keyframes_[i]
    Vec3d t_cur = cur_kf_->GetOptPose().translation();
    pcl::PointXYZ query;
    query.x = static_cast<float>(t_cur.x());
    query.y = static_cast<float>(t_cur.y());
    query.z = static_cast<float>(t_cur.z());

    std::vector<int> indices;
    std::vector<float> sq_dists;
    kdtree_history_poses_->setInputCloud(kf_poses_cloud_);
    kdtree_history_poses_->radiusSearch(query, options_.max_range_, indices, sq_dists);

    if (indices.empty()) {
        return;
    }

    // ---- LIO-SAM 风格：时间差过滤 ----
    // 只保留时间上足够早的历史帧，排除同一轨迹段的近邻帧
    double cur_time = cur_kf_->GetState().timestamp_;

    Keyframe::Ptr check_first = nullptr;  // 用于 min_id_interval_ 密集度控制
    for (int idx : indices) {
        auto kf = all_keyframes_[idx];

        // 时间差过滤（LIO-SAM 核心逻辑）
        double time_diff = std::abs(cur_time - kf->GetState().timestamp_);
        if (time_diff < options_.history_search_time_diff_) {
            continue;
        }

        // 防止候选帧之间 ID 间隔过小（保留 lightning-lm 的密集度控制）
        if (check_first &&
            std::abs(static_cast<int64_t>(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            continue;
        }

        LoopCandidate c(kf->GetID(), cur_id);
        c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
        candidates_.emplace_back(c);
        check_first = kf;
    }

    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;

        // 限制单次处理的候选数量（KD-Tree 按距离排序，优先保留最近的候选帧）
        if (static_cast<int>(candidates_.size()) > options_.max_candidates_per_check_) {
            candidates_.resize(options_.max_candidates_per_check_);
        }
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (lc.ndt_score_ > options_.ndt_score_th_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);

    // LIO-SAM 风格：将成功的回环对记录到 loop_index_container_，防止后续重复检测
    for (const auto& c : candidates_) {
        loop_index_container_[c.idx2_] = c.idx1_;
    }

    // LIO-SAM 风格：将通过 NDT 验证的回环结果写入三队列，供 AddLoopFactor 消费
    // 与检测/验证解耦，PoseOptimization 调用 AddLoopFactor 时才真正加入因子图
    for (const auto& c : candidates_) {
        loop_index_queue_.emplace_back(c.idx1_, c.idx2_);
        loop_pose_queue_.emplace_back(c.Tij_);
        loop_score_queue_.emplace_back(c.ndt_score_);
    }
}

// ---- LIO-SAM loopFindNearKeyframes 对应实现 ----
// 构建以 key 帧为中心坐标系的子地图，避免世界坐标系大值导致 VoxelGrid 整数溢出
void LoopClosing::LoopFindNearKeyframes(CloudPtr& near_keyframes, int key, int search_num) {
    near_keyframes->clear();
    int cloud_size = static_cast<int>(all_keyframes_.size());

    // 以中心帧的 LIO 位姿作为局部坐标系原点
    SE3 T_center = all_keyframes_[key]->GetLIOPose();
    SE3 T_center_inv = T_center.inverse();

    // 取 [key - search_num, key + search_num] 范围内所有关键帧（LIO-SAM 风格）
    for (int i = -search_num; i <= search_num; ++i) {
        int key_near = key + i;
        if (key_near < 0 || key_near >= cloud_size) continue;

        auto kf = all_keyframes_[key_near];
        CloudPtr cloud = kf->GetCloud();
        if (cloud->empty()) continue;

        // 变换到中心帧局部坐标系（相对位姿，坐标值接近 0，避免 VoxelGrid 溢出）
        SE3 T_rel = T_center_inv * kf->GetLIOPose();
        CloudPtr cloud_local(new PointCloudType);
        pcl::transformPointCloud(*cloud, *cloud_local, T_rel.matrix());
        *near_keyframes += *cloud_local;
    }

    if (near_keyframes->empty()) return;

    // VoxelGrid 降采样（对应 LIO-SAM downSizeFilterICP，icp_filter_size_ = 0.4m）
    near_keyframes = VoxelGrid(near_keyframes, options_.icp_filter_size_);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;

    auto kf1 = all_keyframes_.at(c.idx1_);
    auto kf2 = all_keyframes_.at(c.idx2_);

    // ---- 第一步：构建子地图（LIO-SAM loopFindNearKeyframes 风格）----
    //   target：以 kf1 为中心坐标系，前后各 history_keyframe_search_num_ 帧拼成子地图
    //   source：kf2 的原始点云（body 坐标系，坐标值接近 0，无溢出风险）
    CloudPtr prev_cloud(new PointCloudType);
    LoopFindNearKeyframes(prev_cloud, c.idx1_, options_.history_keyframe_search_num_);

    // source 直接取 kf2 的体坐标系点云（不做世界坐标系变换，与 target 坐标系对齐由 NDT 初始猜测完成）
    CloudPtr cur_cloud = kf2->GetCloud();
    if (!cur_cloud || cur_cloud->empty()) {
        LOG(WARNING) << "lc: kf2 cloud empty, skip";
        c.ndt_score_ = 0;
        return;
    }
    // 对 source 做 VoxelGrid 降采样，控制 NDT 计算量
    cur_cloud = VoxelGrid(cur_cloud, options_.icp_filter_size_);

    // ---- 第二步：最小点数检查（LIO-SAM size check）----
    if (static_cast<int>(cur_cloud->size()) < options_.min_cur_cloud_size_ ||
        static_cast<int>(prev_cloud->size()) < options_.min_prev_cloud_size_) {
        LOG(WARNING) << "lc: cloud too small, cur=" << cur_cloud->size()
                     << " prev=" << prev_cloud->size() << ", skip";
        c.ndt_score_ = 0;
        return;
    }

    // ---- 第三步：NDT 配准（仅大场景使用）+ 第四步准备 ----
    // 初始猜测：kf1 局部系中 kf2 的相对位姿 = kf1->GetLIOPose().inverse() * kf2->GetLIOPose()
    Mat4f T_ndt = (kf1->GetLIOPose().inverse() * kf2->GetLIOPose()).matrix().cast<float>();

    PointType min_pt, max_pt;
    pcl::getMinMax3D(*prev_cloud, min_pt, max_pt);
    float extent = std::max({max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z});

    if (extent >= 200.0f) {
        // 大场景（累计里程 >= 200m）：里程计误差大，需要多分辨率 NDT 粗对齐
        // 初始猜测偏差可能较大，从 10m 分辨率开始逐步精化
        // LOG(INFO) << "lc: extent=" << extent << "m >= 200m, running multi-res NDT";
        std::vector<double> res = {10.0, 5.0, 2.0, 1.0};
        CloudPtr output(new PointCloudType);
        for (auto& r : res) {
            pcl::NormalDistributionsTransform<PointType, PointType> ndt;
            ndt.setTransformationEpsilon(0.05);
            ndt.setStepSize(0.7);
            ndt.setMaximumIterations(40);
            ndt.setResolution(r);
            CloudPtr rough_map1 = VoxelGrid(prev_cloud, r * 0.1);
            CloudPtr rough_map2 = VoxelGrid(cur_cloud, r * 0.1);
            ndt.setInputTarget(rough_map1);
            ndt.setInputSource(rough_map2);
            ndt.align(*output, T_ndt);
            T_ndt = ndt.getFinalTransformation();
            c.ndt_score_ = ndt.getTransformationProbability();
        }
    } else {
        // 小/中场景（extent < 200m）：里程计误差小，初始猜测已足够精确
        // 跳过 NDT，直接用里程计位姿作为 GICP 初始猜测，ndt_score 将由 GICP 结果确定
        // LOG(INFO) << "lc: extent=" << extent << "m < 200m, skipping NDT, using LIO pose as GICP init";
    }

    // ---- 第四步：GICP 精匹配（替代标准 ICP，点到面模型对噪声和稀疏点云更鲁棒）----
    // 坐标系：target=kf1 局部系，source=kf2 body 系
    // 初始位姿：大场景=NDT 输出；小场景=LIO 里程计相对位姿（T_ndt 保持初始值）
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(options_.icp_max_correspondence_dist_);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0);
    gicp.setInputTarget(prev_cloud);
    gicp.setInputSource(cur_cloud);

    CloudPtr icp_output(new PointCloudType);
    gicp.align(*icp_output, T_ndt);

    float icp_fitness = gicp.getFitnessScore();
    bool icp_ok = gicp.hasConverged() && icp_fitness < options_.icp_fitness_score_th_;

    LOG(INFO) << "lc: kf " << c.idx1_ << " -> " << c.idx2_
              << ", ndt_score=" << c.ndt_score_
              << ", gicp_fitness=" << icp_fitness
              << ", gicp_ok=" << (icp_ok ? "yes" : "no")
              << ", prev_cloud=" << prev_cloud->size()
              << ", cur_cloud=" << cur_cloud->size();

    if (!icp_ok) {
        // GICP 精匹配失败，丢弃该候选（即使 NDT 通过）
        c.ndt_score_ = 0;
        return;
    }

    // GICP 成功。
    // 小场景跳过了 NDT，c.ndt_score_ 仍为 0，需根据 GICP fitness 换算等效质量分，
    // 以便后续 ComputeLoopCandidates 的阈值过滤和 AddLoopFactor 的自适应噪声逻辑正常工作。
    // 映射规则：fitness 越小 → 匹配越好 → 分数越高；fitness 刚达标时分数刚好超过阈值。
    if (extent < 200.0f) {
        c.ndt_score_ = options_.ndt_score_th_ *
                       (options_.icp_fitness_score_th_ / std::max(icp_fitness, 1e-6f));
    }
    Mat4f T_icp = gicp.getFinalTransformation();

    // ---- 第五步：提取相对位姿（使用 ICP 结果）----
    // T_icp 是 kf1 局部系中 kf2 的修正相对位姿，直接作为回环约束 c.Tij_
    Mat4d T = T_icp.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);
    c.Tij_ = SE3(q, t);

    // ---- 第六步：保存调试数据 ----
    // 仅在候选帧完全通过验证（GICP + NDT score 阈值）时才保存调试数据并触发发布。
    // 若 NDT score 不达标，后续 ComputeLoopCandidates 会将该候选丢弃，不应发布其调试点云。
    if (c.ndt_score_ <= options_.ndt_score_th_) {
        return;
    }

    // UI（Pangolin/RViz）期望 loop_submap 和 aligned_scan 已在 map（世界）坐标系下：
    //   - pangolin_window_impl.cc 以 SE3() 渲染 submap/aligned（不再叠加额外位姿）
    //   - rviz_publisher.cc 以 loop_aligned_pose_ 变换 aligned_scan：设为 SE3() 则等价于直接发布
    // 因此这里将两者从 kf1 局部坐标系变换到世界坐标系
    Mat4f T_kf1_world = kf1->GetLIOPose().matrix().cast<float>();

    CloudPtr prev_cloud_world(new PointCloudType);
    pcl::transformPointCloud(*prev_cloud, *prev_cloud_world, T_kf1_world);

    CloudPtr icp_output_world(new PointCloudType);
    pcl::transformPointCloud(*icp_output, *icp_output_world, T_kf1_world);

    loop_debug_data_.loop_submap = prev_cloud_world;
    loop_debug_data_.current_scan = cur_cloud;        // body 坐标系，UI 侧乘以 current_kf_pose
    loop_debug_data_.aligned_scan = icp_output_world;
    loop_debug_data_.loop_kf_pose = kf1->GetLIOPose();
    loop_debug_data_.current_kf_pose = kf2->GetLIOPose();
    loop_debug_data_.aligned_pose = SE3();             // aligned_scan 已在 map 系，pose 置 identity
    loop_debug_data_.loop_kf_id = c.idx1_;
    loop_debug_data_.current_kf_id = c.idx2_;
    loop_debug_data_.ndt_score = c.ndt_score_;
    loop_debug_data_.valid = true;

    LOG(INFO) << "lc: kf " << c.idx1_ << " -> " << c.idx2_
              << ", ndt_score=" << c.ndt_score_
              << ", prev_cloud=" << prev_cloud->size()
              << ", cur_cloud=" << cur_cloud->size();
}

void LoopClosing::PoseOptimization() {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    // 运动边：与前 1、前 2 帧的相对位姿约束（LIO 里程计提供）
    for (int i = 1; i < 3; i++) {
        int id = cur_kf_->GetID() - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
        }
    }

    // 高度约束（平面场景）
    if (options_.with_height_) {
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
    }

    // 消费回环队列，向因子图添加回环边（LIO-SAM addLoopFactor 对应实现）
    AddLoopFactor();

    if (optimizer_->GetEdges().empty()) {
        return;
    }

    if (candidates_.empty()) {
        return;
    }

    optimizer_->InitializeOptimization();
    optimizer_->SetVerbose(false);

    optimizer_->Optimize(20);

    // 外点剔除：Chi2 超过 Cauchy 核阈值的回环边标记为外点
    int cnt_outliers = 0;
    for (auto& e : edge_loops_) {
        if (e->GetRobustKernel() == nullptr) {
            continue;
        }

        if (e->Chi2() > e->GetRobustKernel()->Delta()) {
            e->SetLevel(1);
            cnt_outliers++;
        } else {
            e->SetRobustKernel(nullptr);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "loop outliers: " << cnt_outliers << "/" << edge_loops_.size();
    }

    // 将优化结果写回各关键帧 opt_pose_
    for (auto& vert : kf_vert_) {
        SE3 pose = vert->Estimate();
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    if (loop_cb_) {
        loop_cb_();
    }

    LOG(INFO) << "optimize finished, loops: " << edge_loops_.size();
}

// ---- LIO-SAM addLoopFactor 对应实现 ----
void LoopClosing::AddLoopFactor() {
    if (loop_index_queue_.empty()) {
        return;
    }

    for (int i = 0; i < static_cast<int>(loop_index_queue_.size()); ++i) {
        int idx1 = loop_index_queue_[i].first;   // 历史帧 ID
        int idx2 = loop_index_queue_[i].second;  // 当前帧 ID
        const SE3& Tij = loop_pose_queue_[i];
        double ndt_score = loop_score_queue_[i];

        // ---- NDT 得分自适应噪声模型（类比 LIO-SAM 用 ICP fitness score 作方差）----
        // NDT TransformationProbability 越高 = 配准越好 = 约束越强
        // 以 ndt_score_th_ 为基准：得分恰好达标时使用标准噪声；得分越高噪声越小
        double noise_scale = std::max(options_.ndt_score_th_ / ndt_score, 0.1);
        Mat6d info = Mat6d::Identity();
        double t_sigma = options_.loop_trans_noise_ * noise_scale;
        double r_sigma = options_.loop_rot_noise_ * noise_scale;
        info.block<3, 3>(0, 0) = Mat3d::Identity() / (t_sigma * t_sigma);
        info.block<3, 3>(3, 3) = Mat3d::Identity() / (r_sigma * r_sigma);

        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, optimizer_->GetVertex(idx1));
        e->SetVertex(1, optimizer_->GetVertex(idx2));
        e->SetMeasurement(Tij);
        e->SetInformation(info);

        // Cauchy 鲁棒核（保留 lightning-lm 对抗误检的机制）
        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        edge_loops_.emplace_back(e);
    }

    LOG(INFO) << "lc: added " << loop_index_queue_.size() << " loop factor(s)";

    loop_index_queue_.clear();
    loop_pose_queue_.clear();
    loop_score_queue_.clear();
}

}  // namespace lightning