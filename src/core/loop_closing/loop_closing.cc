//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <thread>
#include <algorithm>
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
#include "core/miao/core/types/edge_se3_prior.h"
#include "io/yaml_io.h"
#include "utils/timer.h"

#include <Eigen/Dense>

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
            options_.height_noise_ = yaml.GetValue<double>("loop_closing", "height_noise");
        } catch (const YAML::Exception&) {
            LOG(WARNING) << "loop_closing.height_noise not set in yaml, using default: "
                         << options_.height_noise_;
        }
        try {
            options_.online_mode_ = yaml.GetValue<bool>("loop_closing", "online_mode");
        } catch (const YAML::Exception&) {
            LOG(WARNING) << "loop_closing.online_mode not set in yaml, keeping current value: "
                         << options_.online_mode_;
        }
        try {
            options_.verbose_ = yaml.GetValue<bool>("loop_closing", "verbose");
        } catch (const YAML::Exception&) {
            LOG(WARNING) << "loop_closing.verbose not set in yaml, using default: "
                         << options_.verbose_;
        }
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
            options_.optimize_every_n_nodes_ =
                yaml.GetValue<int>("loop_closing", "optimize_every_n_nodes");
        } catch (const YAML::Exception&) {}
        try {
            options_.fix_first_node_ =
                yaml.GetValue<bool>("loop_closing", "fix_first_node");
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
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { 
            Timer::Evaluate([this, kf]() { HandleKF(kf); }, "HandleKF");
        });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }

    // 取硬件并发数的一半（最少 2 线程），为前端保留资源
    int num_pool_threads = std::max(2, static_cast<int>(std::thread::hardware_concurrency()) / 2);
    thread_pool_ = std::make_unique<common::ThreadPool>(num_pool_threads);
    LOG(INFO) << "LoopClosing ThreadPool created with " << num_pool_threads << " threads.";
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        Timer::Evaluate([this, kf]() { HandleKF(kf); }, "HandleKF");
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (kf == last_kf_) {
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // ── 在后端线程中安全地初始化当前帧的 opt_pose ──
    // MakeKF（前端线程）在所有模式下都只用 LIO pose 占位，opt_pose 的正式传播在这里完成。
    // 此时上一帧的 PoseOptimization 已执行完毕（串行），
    // prev_kf->GetOptPose() 是最新的优化结果，读取安全，无数据竞争。
    if (all_keyframes_.size() >= 2) {
        auto prev_kf = all_keyframes_[all_keyframes_.size() - 2];
        SE3 delta_lio = prev_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
        cur_kf_->SetOptPose(prev_kf->GetOptPose() * delta_lio);
    }



    // 检测回环候选（此时 kf_poses_cloud_ 不含当前帧，搜索在历史帧中进行）
    Timer::Evaluate([this]() { DetectLoopCandidates(); }, "DetectLoopCandidates");

    // if (options_.verbose_) {
    //     LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    // }

    // 计算回环位姿
    Timer::Evaluate([this]() { ComputeLoopCandidates(); }, "ComputeLoopCandidates");

    // 每帧无条件将当前帧加入因子图（顶点 + 运动边 + 高度约束 + anchor 先验）
    // 必须在 AddLoopFactor 之前调用，确保当前帧顶点已存在于图中，
    // 否则 AddLoopFactor 中 optimizer_->GetVertex(cur_id) 会返回 nullptr。
    AddOdomFactor(kf);

    // 将回环检测结果追加到因子图（消费 loop_index_queue_）
    // 此时当前帧顶点已由 AddOdomFactor 加入，回环边两端顶点均已在图中。
    AddLoopFactor();

    // 有新回环或累积满 N 帧时执行 PGO；否则只追加 KD-Tree 点
    if (ShouldRunOptimization()) {
        Timer::Evaluate([this]() { PoseOptimization(); }, "PoseOptimization");
    }

    // 将当前帧位置加入 KD-Tree 点云（供下一帧搜索使用）
    // PoseOptimization 内部会全量重建 kf_poses_cloud_（含当前帧），此时 size == all_keyframes_.size()，
    // 不会重复追加；未触发 PGO 时 kf_poses_cloud_ 不含当前帧，正常追加。
    if (kf_poses_cloud_->size() < all_keyframes_.size()) {
        Vec3d t = cur_kf_->GetOptPose().translation();
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(t.x());
        pt.y = static_cast<float>(t.y());
        pt.z = static_cast<float>(t.z());
        kf_poses_cloud_->push_back(pt);
    }

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

    // ---- 防重复检测 ----
    // loop_index_container_ 以 {min_id, max_id} 规范化存储，双向查重：
    // 只要 cur_id 出现在任意已有回环对中（作为 first 或 second），就跳过。
    {
        std::lock_guard<std::mutex> lk(loop_index_mutex_);
        bool found = std::any_of(loop_index_container_.begin(), loop_index_container_.end(),
            [cur_id](const std::pair<uint64_t, uint64_t>& p) {
                return p.first == cur_id || p.second == cur_id;
            });
        if (found) {
            LOG(INFO) << "lc: skip kf " << cur_id << " (already in loop_index_container)";
            return;
        }
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
        c.Tij_ = kf->GetOptPose().inverse() * cur_kf_->GetOptPose();
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

    // 多个候选帧的点云配准完全独立，投入 ThreadPool 并行执行，
    // WaitUntilIdle() 充当 WhenDone 的 barrier 语义，全部完成后再汇总结果。
    std::vector<std::shared_ptr<common::Task>> tasks;
    tasks.reserve(candidates_.size());
    for (auto& c : candidates_) {
        auto task = std::make_shared<common::Task>();
        task->SetWorkItem([this, &c] { ComputeForCandidate(c); });
        thread_pool_->Schedule(task);
        tasks.emplace_back(std::move(task));
    }
    // 等待所有配准任务完成
    thread_pool_->WaitUntilIdle();

    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& c : candidates_) {
        bool pass = c.ndt_score_ > options_.ndt_score_th_;
        if (options_.verbose_) {
            LOG(INFO) << "candi " << c.idx1_ << " -> " << c.idx2_
                    << " score=" << c.ndt_score_
                    << " fitness=" << c.icp_fitness_
                    << (pass ? " [PASS]" : " [FAIL]");
        }
        if (pass) {
            succ_candidates.emplace_back(c);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);

    // LIO-SAM 风格：将成功的回环对记录到 loop_index_container_，防止后续重复检测
    // 以 {min_id, max_id} 规范化存储，支持双向查重
    {
        std::lock_guard<std::mutex> lk(loop_index_mutex_);
        for (const auto& c : candidates_) {
            uint64_t a = c.idx1_, b = c.idx2_;
            if (a > b) std::swap(a, b);
            loop_index_container_.emplace(a, b);
        }
    }

    // LIO-SAM 风格：将通过 NDT 验证的回环结果写入三队列，供 AddLoopFactor 消费
    // 与检测/验证解耦，PoseOptimization 调用 AddLoopFactor 时才真正加入因子图
    for (const auto& c : candidates_) {
        loop_index_queue_.emplace_back(c.idx1_, c.idx2_);
        loop_pose_queue_.emplace_back(c.Tij_);
        loop_score_queue_.emplace_back(c.icp_fitness_);  // 存 raw fitness，供 AddLoopFactor 构建 LIO-SAM 风格噪声
    }

    // 本轮是否有新回环：在 AddLoopFactor 消费队列前快照（消费后队列清空）
    if (!candidates_.empty()) {
        has_new_loop_ = !loop_index_queue_.empty();
    }
}

// 构建以 opt 世界坐标系为基准的子地图，对应原始版本的 "build_in_world=true" 逻辑
// 每帧点云用 GetOptPose() 变换到世界系后拼合，供 ICP 作为 target
void LoopClosing::LoopFindNearKeyframes(CloudPtr& near_keyframes, int key, int search_num) {
    near_keyframes->clear();
    int cloud_size = static_cast<int>(all_keyframes_.size());

    // 取 [key - search_num, key + search_num] 范围内所有关键帧
    for (int i = -search_num; i <= search_num; ++i) {
        int key_near = key + i;
        if (key_near < 0 || key_near >= cloud_size) continue;

        auto kf = all_keyframes_[key_near];
        CloudPtr cloud = kf->GetCloud();
        if (cloud->empty()) continue;

        // 用 opt_pose 变换到世界坐标系（对应原始版本的 Twb = kf->GetLIOPose(), build_in_world=true）
        CloudPtr cloud_world(new PointCloudType);
        pcl::transformPointCloud(*cloud, *cloud_world, kf->GetOptPose().matrix());
        *near_keyframes += *cloud_world;
    }

    if (near_keyframes->empty()) return;

    // VoxelGrid 降采样
    near_keyframes = VoxelGrid(near_keyframes, options_.icp_filter_size_);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    if (options_.verbose_) {
        LOG(INFO) << "aligning " << c.idx1_ << " with " << c.idx2_;
    }

    auto kf1 = all_keyframes_.at(c.idx1_);
    auto kf2 = all_keyframes_.at(c.idx2_);

    // ---- 第一步：构建子地图（对齐原始版本逻辑）----
    //
    // 原始版本：build_submap(kf1, build_in_world=true)
    //   → kf1 前后各 N 帧的点云用 GetLIOPose() 变换到世界系拼合，作为 ICP target
    //
    // 当前版本（opt 坐标系，逻辑相同，用 GetOptPose() 替代 GetLIOPose()）：
    //   → LoopFindNearKeyframes 用 GetOptPose() 变换到 opt 世界系，作为 ICP target
    CloudPtr prev_cloud(new PointCloudType);
    LoopFindNearKeyframes(prev_cloud, c.idx1_, options_.history_keyframe_search_num_);

    // source：kf2 自身点云保持 body 系，ICP 初始猜测用 kf2->GetOptPose()（世界系位姿）
    // 这样对齐原始版本的逻辑：source 在 body 系，target 在世界系，ICP 直接求解修正后的世界位姿
    CloudPtr cur_cloud(new PointCloudType);
    {
        CloudPtr kf2_body = kf2->GetCloud();
        if (!kf2_body || kf2_body->empty()) {
            LOG(WARNING) << "lc: kf2 cloud empty, skip";
            c.ndt_score_ = 0;
            return;
        }
        cur_cloud = VoxelGrid(kf2_body, options_.icp_filter_size_);
    }

    // ---- 第二步：最小点数检查（LIO-SAM size check）----
    if (static_cast<int>(cur_cloud->size()) < options_.min_cur_cloud_size_ ||
        static_cast<int>(prev_cloud->size()) < options_.min_prev_cloud_size_) {
        LOG(WARNING) << "lc: cloud too small, cur=" << cur_cloud->size()
                     << " prev=" << prev_cloud->size() << ", skip";
        c.ndt_score_ = 0;
        return;
    }

    // ---- 第三步：NDT 粗对齐（仅大场景）----
    //
    // target（prev_cloud）在 opt 世界系下，source（cur_cloud）在 kf2 body 系，
    // ICP 初始猜测 = kf2->GetOptPose()（kf2 在世界系中的当前估计位姿）。
    // 对齐原始版本：Tw2 = kf2->GetLIOPose()，只是这里用 opt 替代 LIO。
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*prev_cloud, min_pt, max_pt);
    float extent = std::max({max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z});

    Mat4f T_init = kf2->GetOptPose().matrix().cast<float>();  // ICP 初始猜测 = kf2 在世界系的位姿

    if (extent >= 200.0f) {
        std::vector<double> res = {10.0, 5.0, 2.0, 1.0};
        CloudPtr output(new PointCloudType);
        for (auto& r : res) {
            pcl::NormalDistributionsTransform<PointType, PointType> ndt;
            ndt.setTransformationEpsilon(0.05);
            ndt.setStepSize(0.7);
            ndt.setMaximumIterations(40);
            ndt.setResolution(r);
            CloudPtr rough_map1 = VoxelGrid(prev_cloud, static_cast<float>(r * 0.1));
            CloudPtr rough_map2 = VoxelGrid(cur_cloud,  static_cast<float>(r * 0.1));
            ndt.setInputTarget(rough_map1);
            ndt.setInputSource(rough_map2);
            ndt.align(*output, T_init);
            T_init = ndt.getFinalTransformation();
            c.ndt_score_ = ndt.getTransformationProbability();
        }
    }

    // ---- 第四步：ICP/GICP 精匹配 ----
    //
    // target = opt 世界系下的 prev_cloud（LoopFindNearKeyframes 用 GetOptPose() 变换）
    // source = kf2 body 系点云，初始猜测 T_init = kf2->GetOptPose()（世界系位姿）
    // ICP 输出 = 修正后的 kf2 世界系位姿 T_kf2_corrected（对齐原始版本的 Tw2）
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
    gicp.setMaxCorrespondenceDistance(options_.icp_max_correspondence_dist_);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setRANSACIterations(0);
    gicp.setInputTarget(prev_cloud);
    gicp.setInputSource(cur_cloud);

    CloudPtr icp_output(new PointCloudType);
    gicp.align(*icp_output, T_init);

    float icp_fitness = gicp.getFitnessScore();
    bool icp_ok = gicp.hasConverged() && icp_fitness < options_.icp_fitness_score_th_;

    LOG(INFO) << "lc: kf " << c.idx1_ << " -> " << c.idx2_
              << ", ndt_score=" << c.ndt_score_
              << ", gicp_fitness=" << icp_fitness
              << ", gicp_ok=" << (icp_ok ? "yes" : "no")
              << ", prev=" << prev_cloud->size()
              << ", cur=" << cur_cloud->size();

    if (!icp_ok) {
        c.ndt_score_ = 0;
        return;
    }

    // ---- 第五步：约束提取（对齐原始版本逻辑，统一到 opt 坐标系）----
    //
    // 原始版本（LIO 坐标系）：
    //   c.Tij_ = kf1->GetLIOPose().inverse() * SE3(T_kf2_corrected_world)
    //
    // 当前版本（opt 坐标系，逻辑完全相同，只是把 LIO 替换为 opt）：
    //   ICP 输出 T_kf2_corrected = gicp.getFinalTransformation()
    //     = kf2 修正后在 opt 世界系中的位姿（source body → world）
    //   Tij = kf1->GetOptPose().inverse() * SE3(T_kf2_corrected)
    //       = kf1→kf2 在 opt 坐标系下的相对变换（因子图边测量值）
    Mat4f T_kf2_corrected = gicp.getFinalTransformation();
    Mat4d T = T_kf2_corrected.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t_vec = T.block<3, 1>(0, 3);
    c.Tij_ = kf1->GetOptPose().inverse() * SE3(q, t_vec);

    // ---- 第六步：噪声模型（对齐 LIO-SAM：直接用 fitness score 作方差）----
    //
    // LIO-SAM:  noiseScore = icp.getFitnessScore()
    //           Vector6 << noiseScore * 6
    // 我们保存 icp_fitness 到 ndt_score_（作为质量分的载体），并在 AddLoopFactor 中用它
    // 小场景跳过 NDT，ndt_score_ 赋为等效质量分（供阈值过滤）
    c.ndt_score_ = options_.ndt_score_th_ *
                   (options_.icp_fitness_score_th_ / std::max(icp_fitness, 1e-6f));
    // 同时保存 raw fitness score，供 AddLoopFactor 构建噪声模型
    c.icp_fitness_ = icp_fitness;

    // ---- 第七步：保存调试数据（仅 verbose 模式）----
    if (c.ndt_score_ <= options_.ndt_score_th_) {
        return;
    }

    if (!options_.verbose_) {
        return;
    }

    // 可视化：prev_cloud 已在 opt 世界系，icp_output 也在世界系（source body → world），
    // cur_cloud（body 系）用修正后的 kf2 位姿变换到世界系，三者均以 SE3() 渲染
    CloudPtr cur_cloud_world(new PointCloudType);
    pcl::transformPointCloud(*cur_cloud, *cur_cloud_world, T_kf2_corrected);

    // ComputeForCandidate 在 ThreadPool 中并行执行，需加锁保护 loop_debug_data_。
    // 同一帧可能有多个候选同时通过 ICP，将所有成功候选的点云累积追加（而非覆盖），
    // 使 UI 能一次性显示本轮所有有效回环的 submap 和 aligned scan。
    {
        std::lock_guard<std::mutex> lk(loop_debug_mutex_);
        if (loop_debug_data_ == nullptr) {
            // 第一个成功候选：创建新的调试数据对象
            loop_debug_data_ = std::make_unique<LoopDebugData>();
            loop_debug_data_->loop_submap  = std::make_shared<PointCloudType>(*prev_cloud);
            loop_debug_data_->aligned_scan = std::make_shared<PointCloudType>(*icp_output);
            loop_debug_data_->current_scan = std::make_shared<PointCloudType>(*cur_cloud_world);
        } else {
            // 后续成功候选：追加合并（所有候选均在同一世界坐标系下，可以直接合并）
            *loop_debug_data_->loop_submap  += *prev_cloud;
            *loop_debug_data_->aligned_scan += *icp_output;
            *loop_debug_data_->current_scan += *cur_cloud_world;
        }
        loop_debug_data_->loop_kf_pose    = SE3();
        loop_debug_data_->current_kf_pose = SE3();
        loop_debug_data_->aligned_pose    = SE3();
        // loop_kf_id / current_kf_id 记录最后一个成功候选的 ID（仅供日志参考）
        loop_debug_data_->loop_kf_id    = c.idx1_;
        loop_debug_data_->current_kf_id = c.idx2_;
        loop_debug_data_->ndt_score     = c.ndt_score_;
    }

    if (options_.verbose_) {
        LOG(INFO) << "lc: kf " << c.idx1_ << " -> " << c.idx2_
                  << ", ndt_score=" << c.ndt_score_
                  << ", prev_cloud=" << prev_cloud->size()
                  << ", cur_cloud=" << cur_cloud->size();
    }
}

bool LoopClosing::ShouldRunOptimization() {
    ++num_nodes_since_last_optimization_;

    bool should =
        has_new_loop_ ||
        (options_.optimize_every_n_nodes_ > 0 &&
         num_nodes_since_last_optimization_ >= options_.optimize_every_n_nodes_);

    if (should) {
        if (options_.verbose_) {
            LOG(INFO) << "ShouldRunOptimization: yes"
                      << " (has_new_loop=" << has_new_loop_
                      << ", nodes_since_last=" << num_nodes_since_last_optimization_
                      << ", every_n=" << options_.optimize_every_n_nodes_ << ")";
        }
        num_nodes_since_last_optimization_ = 0;
    }

    return should;
}

void LoopClosing::AddOdomFactor(Keyframe::Ptr kf) {
    uint64_t id = kf->GetID();

    // 顶点已存在（重复调用保护）
    if (optimizer_->GetVertex(id) != nullptr) {
        return;
    }

    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(id);
    v->SetEstimate(kf->GetOptPose());
    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    // ---- anchor：固定第一帧 ----
    // 增量模式不支持 SetFixed，用极大信息阵的先验边代替（information = I * 1e9 ≈ SetFixed）
    if (options_.fix_first_node_ && !first_node_fixed_ && kf_vert_.size() == 1) {
        auto e_prior = std::make_shared<miao::EdgeSE3Prior>();
        e_prior->SetVertex(0, v);
        e_prior->SetMeasurement(v->Estimate());
        e_prior->SetInformation(Mat6d::Identity() * 1e9);
        optimizer_->AddEdge(e_prior);
        first_node_fixed_ = true;
        LOG(INFO) << "AddOdomFactor: fixed first node (kf " << id << ") via SE3Prior";
    }

    // ---- 运动边：与前 1、前 2 帧的相对位姿约束 ----
    for (int i = 1; i < 3; i++) {
        int64_t prev_id = static_cast<int64_t>(id) - i;
        if (prev_id < 0) continue;
        auto prev_v = optimizer_->GetVertex(prev_id);
        if (prev_v == nullptr) continue;
        auto prev_kf = all_keyframes_.at(prev_id);
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, prev_v);
        e->SetVertex(1, v);
        e->SetMeasurement(prev_kf->GetOptPose().inverse() * kf->GetOptPose());
        e->SetInformation(info_motion_);
        optimizer_->AddEdge(e);
    }

    // ---- 高度约束（平面场景）----
    if (options_.with_height_) {
        auto e_h = std::make_shared<miao::EdgeHeightPrior>();
        e_h->SetVertex(0, v);
        e_h->SetMeasurement(0);
        e_h->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e_h);
    }
}

void LoopClosing::PoseOptimization() {
    // AddOdomFactor 已在每帧 HandleKF 中无条件调用，AddLoopFactor 也已在 ComputeLoopCandidates 后
    // 立即调用，此处因子图已完整连通，PoseOptimization 只负责 Optimize → 写回位姿 → 重建 KD-Tree。

    if (optimizer_->GetEdges().empty()) {
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

    // 将优化结果写回所有关键帧 opt_pose_（回环修正了历史帧，需全量写回）
    for (auto& vert : kf_vert_) {
        SE3 pose = vert->Estimate();
        all_keyframes_[vert->GetId()]->SetOptPose(pose);
    }

    // 全量重建 kf_poses_cloud_，保持 KD-Tree 与最新 opt_pose 一致
    RebuildHistoryPoses();

    if (loop_cb_ && has_new_loop_) {
        // 仅在本轮确实添加了回环边时触发重绘回调，
        // 纯定期 N 帧优化（无新回环）不触发，避免无意义的地图重建。
        loop_cb_();
    }

    has_new_loop_ = false;

    if (options_.verbose_) {
        LOG(INFO) << "PoseOptimization: done";
    }
}

void LoopClosing::RebuildHistoryPoses() {
    kf_poses_cloud_->clear();
    kf_poses_cloud_->reserve(all_keyframes_.size());
    for (const auto& kf : all_keyframes_) {
        Vec3d t = kf->GetOptPose().translation();
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(t.x());
        pt.y = static_cast<float>(t.y());
        pt.z = static_cast<float>(t.z());
        kf_poses_cloud_->push_back(pt);
    }
}

void LoopClosing::RunFinalOptimization() {
    // 在线模式：等待异步队列耗尽，确保所有帧都已进入因子图
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }

    if (kf_vert_.empty() || optimizer_->GetEdges().empty()) {
        LOG(WARNING) << "RunFinalOptimization: graph empty, skip";
        return;
    }

    LOG(INFO) << "RunFinalOptimization: running final PGO on " << kf_vert_.size() << " nodes";
    PoseOptimization();
    LOG(INFO) << "RunFinalOptimization: done";
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
        double icp_fitness = loop_score_queue_[i];

        // ---- LIO-SAM 风格噪声模型 ----
        // LIO-SAM:  noiseScore = icp.getFitnessScore()
        //           Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore
        //           correctionNoise = noiseModel::Diagonal::Variances(Vector6)
        // 直接用 icp_fitness 作为方差，fitness 越小 = 约束越强
        // 加下限避免数值问题（方差不能为 0）
        double variance = std::max(icp_fitness, 1e-6);
        Mat6d info = Mat6d::Identity() * (1.0 / variance);

        auto v0 = optimizer_->GetVertex(idx1);
        auto v1 = optimizer_->GetVertex(idx2);
        if (v0 == nullptr || v1 == nullptr) {
            LOG(WARNING) << "AddLoopFactor: vertex not found for kf " << idx1 << " or " << idx2 << ", skip";
            continue;
        }

        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, v0);
        e->SetVertex(1, v1);
        e->SetMeasurement(Tij);
        e->SetInformation(info);

        // Cauchy 鲁棒核（保留 lightning-lm 对抗误检的机制）
        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        {
            std::lock_guard<std::mutex> lk(edge_loops_mutex_);
            edge_loops_.emplace_back(e);
        }
    }

    LOG(INFO) << "lc: added " << loop_index_queue_.size() << " loop factor(s)";

    loop_index_queue_.clear();
    loop_pose_queue_.clear();
    loop_score_queue_.clear();
}

}  // namespace lightning