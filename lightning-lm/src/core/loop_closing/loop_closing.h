//
// Created by xiang on 25-4-21.
//

#ifndef LIGHTNING_LOOP_CLOSING_H
#define LIGHTNING_LOOP_CLOSING_H

#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "common/thread_pool.h"
#include "utils/async_message_process.h"

#include "core/graph/optimizer.h"
#include "core/types/edge_se3.h"
#include "core/miao/core/types/edge_se3_prior.h"
#include "voxel_map/voxel_map_manager.hpp"

#include <map>
#include <memory>
#include <set>
#include <mutex>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

namespace lightning {

/**
 * 基于grid ndt的回环检测
 */
class LoopClosing {
   public:
    struct Options {
        Options() {}

        bool verbose_ = false;       // 输出调试信息
        bool online_mode_ = false;  // 切换离线-在线模式

        int loop_kf_gap_ = 20;       // 每隔多少个关键帧检查一次
        int min_id_interval_ = 20;   // 被检查的关键帧ID间隔
        int closest_id_th_ = 50;     // 历史关键帧与当前帧的ID间隔（已由时间差过滤取代，保留兼容）
        double max_range_ = 30.0;    // KD-Tree 搜索半径（单位：米）
        double ndt_score_th_ = 1.0;  // NDT 配准通过阈值（TransformationProbability）

        // LIO-SAM 风格：时间差过滤阈值（秒）
        // 历史候选帧与当前帧的时间差必须超过该值，避免检测同一轨迹段的近邻帧
        double history_search_time_diff_ = 30.0;

        // ---- LIO-SAM 风格子地图构建参数 ----
        // 对应 LIO-SAM historyKeyframeSearchNum（前后各取 N 帧，共 2N+1 帧）
        int history_keyframe_search_num_ = 25;
        // 子地图 VoxelGrid 叶大小（对应 LIO-SAM mappingSurfLeafSize = 0.4m）
        float icp_filter_size_ = 0.4f;
        // 当前帧（source）最小点数，不足则放弃本次配准
        int min_cur_cloud_size_ = 300;
        // 历史子地图（target）最小点数，不足则放弃本次配准
        int min_prev_cloud_size_ = 1000;

        // 每次触发时最多处理的候选帧数（超出部分按距离最近优先截取）
        int max_candidates_per_check_ = 3;

        // ---- 位姿图优化触发策略（optimize_every_n_nodes）----
        // 每累积 N 个新节点执行一次优化（0 = 每帧都优化，即保持旧行为）
        // 有新回环时无条件触发一次优化（覆盖 N 帧计数）
        int optimize_every_n_nodes_ = 10;

        // 是否固定第一帧（anchor node）：通过强先验边约束初始位姿不漂移
        // 等效于 g2o/Cartographer 中的 SetFixed，但绕过增量模式限制（用极大信息阵代替）
        bool fix_first_node_ = true;

        // ---- ICP 精匹配参数（NDT 后进行细化）----
        // ICP fitness score 阈值（均方点距，越小越精，超出则认为配准失败）
        float icp_fitness_score_th_ = 0.3f;
        // ICP 最大点对点对应距离（米），NDT 粗对齐后 1.5m 足够
        float icp_max_correspondence_dist_ = 1.5f;

        /// 图优化权重
        double motion_trans_noise_ = 0.1;               // 运动边平移权重
        double motion_rot_noise_ = 3.0 * M_PI / 180.0;  // 运动边旋转权重

        double loop_trans_noise_ = 0.2;               // 回环边平移权重（基准值）
        double loop_rot_noise_ = 3.0 * M_PI / 180.0;  // 回环边旋转权重（基准值）

        double rk_loop_th_ = 5.2 / 5;  // 回环的 Cauchy 鲁棒核阈值

        bool with_height_ = true;
        double height_noise_ = 1.0;  // 回环边高度维权重（单位：米），仅当 with_height_ = true 时生效

        // ---- VoxelMap 回环精化参数 ----
        // 构建临时 VoxelMap 用的体素大小（比 LIO 更细以提高精化精度）
        double lc_voxel_size_ = 0.5;
        // IEKF 最大迭代次数
        int lc_vmap_max_iter_ = 5;
        // 迭代收敛阈值（平移+旋转范数）
        double lc_vmap_conv_th_ = 1e-4;
        // 最少有效点面匹配数（低于此值认为精化不可靠，保持 GICP 结果）
        int lc_vmap_min_match_ = 20;
    };

    LoopClosing(Options options = Options()) { options_ = options; }
    ~LoopClosing();

    void Init(const std::string yaml_path);

    /// 向回环中添加一个关键帧
    void AddKF(Keyframe::Ptr kf);

    /**
     * 建图结束时调用（SaveMap 前）：在线模式下等待异步队列耗尽，然后执行一次最终全局优化。
     * 阻塞直到完成，有回环时触发 loop_cb_。
     */
    void RunFinalOptimization();

    /// 如果检测到新地回环并发生了优化，则调用回调
    using LoopClosedCallback = std::function<void()>;
    void SetLoopClosedCB(LoopClosedCallback cb) { loop_cb_ = cb; }

    /// 获取回环边（用于可视化）
    /// 从 edge_loops_ 读取（因子图中实际加入的所有回环边，每条边独立存储，不会被覆盖），
    /// 用 edge_loops_mutex_ 保护，避免与 kf_thread_ 写入产生数据竞争。
    /// 注意：loop_index_container_ 是 map<cur_id→hist_id>，同一 cur_id 对应多个候选时会互相覆盖，
    ///       因此不能用它做可视化数据源。
    std::vector<std::pair<int, int>> GetLoopEdges() const {
        std::lock_guard<std::mutex> lk(edge_loops_mutex_);
        std::vector<std::pair<int, int>> edges;
        edges.reserve(edge_loops_.size());
        for (const auto& e : edge_loops_) {
            if (e->GetVertex(0) == nullptr || e->GetVertex(1) == nullptr) continue;
            edges.emplace_back(
                static_cast<int>(e->GetVertex(0)->GetId()),
                static_cast<int>(e->GetVertex(1)->GetId()));
        }
        return edges;
    }

    /// 获取最新的回环调试数据（用于可视化）
    struct LoopDebugData {
        CloudPtr loop_submap;          // 回环帧构建的submap（世界坐标系）
        CloudPtr current_scan;         // 当前帧的scan（body坐标系）
        CloudPtr aligned_scan;         // 配准后的scan（世界坐标系）
        SE3 loop_kf_pose;              // 回环关键帧的位姿
        SE3 current_kf_pose;           // 当前关键帧的位姿
        SE3 aligned_pose;              // 配准后的位姿
        int loop_kf_id;                // 回环关键帧ID
        int current_kf_id;             // 当前关键帧ID
        double ndt_score;              // NDT匹配得分

        LoopDebugData() : ndt_score(0.0) {}
    };

    /// 取走最新的回环调试数据（move 语义，调用后内部指针置为 nullptr）。
    /// 返回 nullptr 表示本轮尚无成功回环。
    std::unique_ptr<LoopDebugData> GetLoopDebugData() {
        std::lock_guard<std::mutex> lk(loop_debug_mutex_);
        return std::move(loop_debug_data_);
    }

   protected:
    void HandleKF(Keyframe::Ptr kf);

    void DetectLoopCandidates();

    /// 计算回环候选位姿
    void ComputeLoopCandidates();

    /// 计算单个回环候选（NDT 配准，结果写入 c.Tij_ / c.ndt_score_）
    void ComputeForCandidate(LoopCandidate& c);

    /**
     * 构建关键帧附近子地图（LIO-SAM loopFindNearKeyframes 对应实现）
     * @param near_keyframes  输出点云（世界坐标系，VoxelGrid 降采样后）
     * @param key             中心关键帧索引
     * @param search_num      前后各取帧数；0 = 只取自身（当前帧 source）
     */
    void LoopFindNearKeyframes(CloudPtr& near_keyframes, int key, int search_num);

    /**
     * 每帧关键帧到达时无条件调用：将该帧顶点加入因子图，并添加运动边 + 高度约束 + anchor 先验。
     * 与 AddLoopFactor / PoseOptimization 解耦，职责单一：只负责"入图"，不触发优化。
     */
    void AddOdomFactor(Keyframe::Ptr kf);

    /**
     * 消费回环队列，向 miao 优化器添加回环边（LIO-SAM addLoopFactor 对应实现）
     * 队列来源：ComputeLoopCandidates 成功后写入
     */
    void AddLoopFactor();

    /// 优化位姿（miao 增量 PGO，lightning-lm 原有逻辑保留）
    void PoseOptimization();

    /// 全量重建 kf_poses_cloud_，使 KD-Tree 与最新 opt_pose 保持一致
    void RebuildHistoryPoses();

    /**
     * 参考 Cartographer pose_graph_2d AddNode 末尾的 kRunOptimization 判断：
     *   - 有新回环 → true（重置计数器，无条件优化）
     *   - 距上次优化已累积 N 帧 → true（重置计数器，定期优化）
     *   - 否则 → false（只积累节点/边，不调用 Optimize）
     */
    bool ShouldRunOptimization();

    Options options_;

    Keyframe::Ptr last_kf_ = nullptr;
    Keyframe::Ptr last_loop_kf_ = nullptr;
    Keyframe::Ptr cur_kf_ = nullptr;
    std::vector<Keyframe::Ptr> all_keyframes_;
    std::vector<LoopCandidate> candidates_;

    AsyncMessageProcess<Keyframe::Ptr> kf_thread_;

    std::shared_ptr<miao::Optimizer> optimizer_ = nullptr;

    Mat6d info_motion_ = Mat6d::Identity();  // 关键帧间的运动信息阵
    Mat6d info_loops_ = Mat6d::Identity();   // 回环帧的信息矩阵

    std::vector<std::shared_ptr<miao::VertexSE3>> kf_vert_;
    std::vector<std::shared_ptr<miao::EdgeSE3>> edge_loops_;
    mutable std::mutex edge_loops_mutex_;  // 保护 edge_loops_ 的并发读写（kf_thread_ 写，主线程读）

    LoopClosedCallback loop_cb_;

    mutable std::unique_ptr<LoopDebugData> loop_debug_data_;  // 回环调试数据（nullptr 表示本轮无成功回环）
    mutable std::mutex loop_debug_mutex_;    // 保护 loop_debug_data_ 的并发写入（ComputeForCandidate 在线程池中并行执行）

    // Cartographer 风格线程池：ComputeLoopCandidates 中并行执行点云配准
    std::unique_ptr<common::ThreadPool> thread_pool_;

    // optimize_every_n_nodes 计数：距上次优化已累积的新节点数
    int num_nodes_since_last_optimization_ = 0;

    // 第一帧是否已通过强先验边固定（fix_first_node_ 选项生效后，只固定一次）
    bool first_node_fixed_ = false;

    // ---- LIO-SAM 风格的候选帧搜索结构 ----
    // 关键帧位置点云（与 all_keyframes_ 同步，但滞后一帧——不含当前帧，用于 KD-Tree 搜索）
    pcl::PointCloud<pcl::PointXYZ>::Ptr kf_poses_cloud_;
    // KD-Tree 空间索引，用于半径搜索历史关键帧
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_history_poses_;
    // 已检测到的回环对集合（以 {min_id, max_id} 规范化存储），支持双向查重：
    // 无论 A→B 还是 B→A 查询，只要该对已存在就跳过，避免对同一关键帧对重复触发回环。
    std::set<std::pair<uint64_t, uint64_t>> loop_index_container_;
    mutable std::mutex loop_index_mutex_;  // 保护 loop_index_container_ 的并发读写

    // ---- LIO-SAM 风格的回环约束队列（解耦检测与优化）----
    // 成功通过 NDT 验证的回环对，等待 AddLoopFactor 消费
    bool has_new_loop_ = false;  // 本轮是否有新回环（AddLoopFactor 消费前快照）
    std::vector<std::pair<int, int>> loop_index_queue_;  // (hist_id, cur_id)
    std::vector<SE3> loop_pose_queue_;                   // 对应的相对位姿 Tij
    std::vector<double> loop_score_queue_;               // 对应的 NDT 得分（用于计算噪声模型）
};

}  // namespace lightning

#endif  // LIGHTNING_LOOP_CLOSING_H
