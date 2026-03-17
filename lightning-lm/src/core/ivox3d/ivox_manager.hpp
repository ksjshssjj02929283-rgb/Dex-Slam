/**
 * @file ivox_manager.hpp
 * @brief IVox 增量地图统一管理接口
 *
 * 对外提供的唯一入口类，封装了：
 *  1. IVox 地图初始化 (Init)
 *  2. 地图增量更新 (UpdateMap)    — 含最近邻去重滤波，对齐 faster-lio 原始逻辑
 *  3. 点到平面对应关系查找 (FindCorrespondences) — kNN + 平面拟合 + 马氏距离检验
 *
 * 设计原则：
 *  - Header-only，无 .cpp 依赖（与 IVox 本身风格一致）
 *  - 接口与 VoxelMapManager 保持对称，方便 LaserMapping 统一调用
 *  - 内部辅助容器作为成员变量复用，避免每帧重复分配/释放
 *  - 内部使用 std::execution::par_unseq 并行化对应关系查找
 */
#pragma once

#include <execution>
#include <memory>
#include <vector>

#include "common/eigen_types.h"
#include "common/options.h"
#include "core/ivox3d/ivox3d.h"
#include "core/lightning_math.hpp"

namespace lightning {

// ============================================================================
// 配置参数
// ============================================================================
struct IVoxConfig {
    /// IVox 体素分辨率（米）
    float resolution = 0.5f;

    /// IVox 邻域查找类型（0=CENTER, 6=NEARBY6, 18=NEARBY18, 26=NEARBY26）
    IVox<3, IVoxNodeType::DEFAULT, PointType>::NearbyType nearby_type =
        IVox<3, IVoxNodeType::DEFAULT, PointType>::NearbyType::NEARBY18;

    /// IVox 最大容量（体素个数）
    std::size_t capacity = 1000000;

    /// 地图更新去重的滤波尺寸（通常等于 scan 下采样分辨率）
    float filter_size_map_min = 0.5f;

    /// kNN 查询点数
    int num_match_points = fasterlio::NUM_MATCH_POINTS;

    /// 平面拟合最少点数
    int min_num_match_points = fasterlio::MIN_NUM_MATCH_POINTS;

    /// 平面拟合阈值（与 fasterlio::ESTI_PLANE_THRESHOLD 保持一致，默认 0.1f）
    float esti_plane_threshold = 0.1f;
};

// ============================================================================
// IVoxManager 管理器
// ============================================================================
class IVoxManager {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType    = IVox<3, IVoxNodeType::DEFAULT, PointType>;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

    // -------------------------------------------------------------------------
    //  构造 / 初始化
    // -------------------------------------------------------------------------

    explicit IVoxManager(const IVoxConfig& config = IVoxConfig()) { Init(config); }

    /**
     * @brief 按配置初始化（或重置）IVox 地图
     * @param config IVoxConfig 配置参数
     */
    void Init(const IVoxConfig& config) {
        config_ = config;

        IVoxType::Options opts;
        opts.resolution_     = config_.resolution;
        opts.nearby_type_    = config_.nearby_type;
        opts.capacity_       = config_.capacity;
        opts.inv_resolution_ = 1.0f / config_.resolution;

        ivox_ = std::make_shared<IVoxType>(opts);
    }

    /// 获取配置
    const IVoxConfig& GetConfig() const { return config_; }

    /// 获取底层 IVox 指针（用于外部直接访问的场合，如首帧初始化）
    std::shared_ptr<IVoxType> GetIVox() const { return ivox_; }

    // -------------------------------------------------------------------------
    //  地图更新
    // -------------------------------------------------------------------------

    /**
     * @brief 增量更新 IVox 地图（EKF 收敛后调用）
     *
     * 对齐 faster-lio 原始逻辑，采用两阶段设计：
     *
     * **Phase 1（并行）**：对每个点独立判断是否需要加入地图
     *  - 判断仅依赖 nearest_points[i]（只读），无共享写，可 par_unseq
     *  - 结果写入 per-point tag：SKIP / ADD / ADD_NO_DEDUP
     *
     * **Phase 2（单线程）**：按 tag 分桶后批量写入 IVox（IVox::AddPoints 内部已做体素级并行）
     *
     * 注意：辅助容器（update_tags_、pts_to_add_ 等）为成员变量，避免每帧重复分配。
     * 接受 const std::vector<PointType>& 可直接传入 scan_down_world_->points，无需拷贝。
     *
     * @param points_world    当前帧下采样后的世界坐标点（直接传 scan_down_world_->points）
     * @param nearest_points  上一轮 FindCorrespondences 返回的 kNN 邻居（逐点对应）
     * @param flg_EKF_inited  EKF 是否已初始化（未初始化时跳过去重逻辑）
     */
    void UpdateMap(const PointVector& points_world,
                   const std::vector<PointVector>& nearest_points,
                   bool flg_EKF_inited) {
        const size_t cur_pts = points_world.size();
        if (cur_pts == 0) return;

        // 复用成员容器，避免每帧分配
        update_tags_.assign(cur_pts, AddTag::SKIP);
        update_idx_.resize(cur_pts);
        for (size_t i = 0; i < cur_pts; ++i) update_idx_[i] = i;

        // ------------------------------------------------------------------
        // Phase 1 (并行): 对每个点独立判断加入策略
        //   去重判断仅读 nearest_points[i]，无共享写，par_unseq 安全
        // ------------------------------------------------------------------
        std::for_each(std::execution::par_unseq, update_idx_.begin(), update_idx_.end(),
                      [&](const size_t i) {
            const PointType& pw = points_world[i];

            if (!nearest_points[i].empty() && flg_EKF_inited) {
                const PointVector& points_near = nearest_points[i];

                Eigen::Vector3f center =
                    ((pw.getVector3fMap() / config_.filter_size_map_min).array().floor() + 0.5f)
                    * config_.filter_size_map_min;

                Eigen::Vector3f dis_2_center = points_near[0].getVector3fMap() - center;

                // 邻居最近点在体素中心三轴偏移均 > 0.5*filter_size → 无需去重
                if (fabs(dis_2_center.x()) > 0.5f * config_.filter_size_map_min &&
                    fabs(dis_2_center.y()) > 0.5f * config_.filter_size_map_min &&
                    fabs(dis_2_center.z()) > 0.5f * config_.filter_size_map_min) {
                    update_tags_[i] = AddTag::ADD_NO_DEDUP;
                    return;
                }

                float dist = math::calc_dist(pw.getVector3fMap(), center);
                int check_n = std::min(static_cast<int>(points_near.size()), config_.num_match_points);
                for (int j = 0; j < check_n; ++j) {
                    if (math::calc_dist(points_near[j].getVector3fMap(), center) < dist + 1e-6f) {
                        return;  // 已有更优点，保持 SKIP
                    }
                }
                update_tags_[i] = AddTag::ADD;
            } else {
                update_tags_[i] = AddTag::ADD;  // 第一帧或 EKF 未初始化
            }
        });

        // ------------------------------------------------------------------
        // Phase 2 (单线程分桶 + IVox 内部体素级并行写入)
        //   复用成员容器，避免每帧 reserve/alloc
        // ------------------------------------------------------------------
        pts_to_add_.clear();
        pts_no_dedup_.clear();

        for (size_t i = 0; i < cur_pts; ++i) {
            if      (update_tags_[i] == AddTag::ADD)          pts_to_add_.emplace_back(points_world[i]);
            else if (update_tags_[i] == AddTag::ADD_NO_DEDUP) pts_no_dedup_.emplace_back(points_world[i]);
        }

        ivox_->AddPoints(pts_to_add_);
        ivox_->AddPoints(pts_no_dedup_);
    }

    // -------------------------------------------------------------------------
    //  对应关系查找
    // -------------------------------------------------------------------------

    /**
     * @brief 查找 IVox 点到平面对应关系（kNN + 平面拟合 + 马氏距离检验）
     *
     * 对每个下采样 body 点：
     *  1. body → world 坐标变换
     *  2. 在 IVox 中查找 kNN 邻居
     *  3. 用邻居做平面拟合（esti_plane）
     *  4. 马氏距离检验（点到平面距离 < 1/9 * body_norm）
     *
     * 有效对应关系写入 corr_pts / corr_norm：
     *  - corr_pts[k]  = [body_x, body_y, body_z, residual]
     *  - corr_norm[k] = [nx, ny, nz, d]（平面系数）
     *
     * nearest_points_out 同步更新，供下一帧 UpdateMap 使用（去重滤波依赖）。
     *
     * 注意：辅助容器（point_selected_、plane_coef_、corr_index_）为成员变量，避免每帧重复分配。
     *
     * @param scan_down_body     body 坐标系下采样点云
     * @param scan_down_world    [in/out] world 坐标系点云（会被本函数更新坐标）
     * @param R_wl               lidar→world 旋转（Eigen::Matrix3f）
     * @param t_wl               lidar→world 平移（Eigen::Vector3f）
     * @param corr_pts           [out] 有效匹配的 body 坐标 + 残差 [x,y,z,pd2]
     * @param corr_norm          [out] 有效匹配的平面系数 [nx,ny,nz,d]
     * @param residuals          [out] 每个下采样点的残差（大小 = scan_down_body.size()，无效点为 0）
     * @param nearest_points_out [out] 每个下采样点的 kNN 邻居（大小 = scan_down_body.size()）
     */
    void FindCorrespondences(const CloudPtr& scan_down_body,
                             CloudPtr& scan_down_world,
                             const Eigen::Matrix3f& R_wl,
                             const Eigen::Vector3f& t_wl,
                             std::vector<Vec4f>& corr_pts,
                             std::vector<Vec4f>& corr_norm,
                             std::vector<float>& residuals,
                             std::vector<PointVector>& nearest_points_out) {
        const int cnt_pts = static_cast<int>(scan_down_body->size());

        // 复用成员辅助容器
        nearest_points_out.resize(cnt_pts);
        residuals.resize(cnt_pts, 0.0f);
        point_selected_.assign(cnt_pts, false);
        plane_coef_.resize(cnt_pts);
        corr_index_.resize(cnt_pts);
        for (int i = 0; i < cnt_pts; ++i) corr_index_[i] = i;

        // 并行：坐标变换 + kNN + 平面拟合 + 马氏距离检验
        std::for_each(std::execution::par_unseq, corr_index_.begin(), corr_index_.end(),
                      [&](const size_t i) {
            const PointType& pb = scan_down_body->points[i];
            PointType& pw       = scan_down_world->points[i];

            Vec3f p_body = pb.getVector3fMap();
            pw.getVector3fMap() = R_wl * p_body + t_wl;
            pw.intensity        = pb.intensity;

            nearest_points_out[i].clear();
            ivox_->GetClosestPoint(pw, nearest_points_out[i], config_.num_match_points);
            point_selected_[i] = (static_cast<int>(nearest_points_out[i].size()) >= config_.min_num_match_points);

            if (point_selected_[i]) {
                point_selected_[i] =
                    math::esti_plane(plane_coef_[i], nearest_points_out[i], config_.esti_plane_threshold);
            }

            if (point_selected_[i]) {
                auto temp = pw.getVector4fMap();
                temp[3] = 1.0f;
                float pd2 = plane_coef_[i].dot(temp);
                if (p_body.norm() > 81.0f * pd2 * pd2) {
                    residuals[i] = pd2;
                } else {
                    point_selected_[i] = false;
                }
            }
        });

        // 收集有效匹配（复用 corr_pts/corr_norm 的已分配内存）
        corr_pts.resize(cnt_pts);
        corr_norm.resize(cnt_pts);
        int eff = 0;
        for (int i = 0; i < cnt_pts; ++i) {
            if (point_selected_[i]) {
                corr_norm[eff]   = plane_coef_[i];
                corr_pts[eff]    = scan_down_body->points[i].getVector4fMap();
                corr_pts[eff][3] = residuals[i];
                ++eff;
            }
        }
        corr_pts.resize(eff);
        corr_norm.resize(eff);
    }

    // -------------------------------------------------------------------------
    //  统计信息
    // -------------------------------------------------------------------------

    /// 地图中的有效体素数
    size_t NumValidGrids() const { return ivox_ ? ivox_->NumValidGrids() : 0; }

    /// 地图中的总点数
    size_t NumPoints() const { return ivox_ ? ivox_->NumPoints() : 0; }

   private:
    enum class AddTag : uint8_t { SKIP = 0, ADD = 1, ADD_NO_DEDUP = 2 };

    IVoxConfig                config_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;

    // UpdateMap 复用容器（避免每帧 alloc/free）
    std::vector<AddTag>   update_tags_;
    std::vector<size_t>   update_idx_;
    PointVector           pts_to_add_;
    PointVector           pts_no_dedup_;

    // FindCorrespondences 复用容器
    std::vector<bool>     point_selected_;
    std::vector<Vec4f>    plane_coef_;
    std::vector<size_t>   corr_index_;
};

}  // namespace lightning
