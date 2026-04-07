/**
 * @file voxel_map_manager.hpp
 * @brief VoxelMap 统一管理接口
 *
 * 对外提供的唯一入口类，封装了：
 *  1. 地图初始化 (initializeMap)
 *  2. 地图增量更新 (updateMap)
 *  3. 点到平面匹配 (findCorrespondences) — 替代 ivox_->GetClosestPoint
 *  4. 点测量协方差计算 (calcBodyCov / propagateCovToWorld)
 *  5. 可视化数据导出 (getUpdatedPlanes)
 *
 * 无 ROS 依赖，仅需 Eigen + OpenMP。
 */
#pragma once

#include "voxel_map/octo_tree.hpp"
#include "voxel_map/voxel_types.hpp"

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace lightning {
namespace voxel_map {

// ============================================================================
// 配置参数
// ============================================================================
struct VoxelMapConfig {
    double voxel_size = 1.0;                             // 顶层体素大小（米）
    int max_layer = 2;                                   // 八叉树最大层数
    std::vector<int> layer_point_size = {5, 5, 5};       // 每层触发平面拟合的最小点数
    int max_points_size = 50;                            // 每个体素最大点数
    int max_cov_points_size = 50;                        // 保留协方差更新的最大点数
    float planer_threshold = 0.01f;                      // 平面判定阈值（最小特征值）
    double sigma_num = 2.0;                              // 3σ 检验的 sigma 倍数
    double ranging_cov = 0.04;                           // 测距方差 (m²)
    double angle_cov = 0.1;                              // 角度方差 (rad²)
    int pub_max_voxel_layer = 1;                         // 可视化的最大体素层级
};

// ============================================================================
// VoxelMap 管理器
// ============================================================================
class VoxelMapManager {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit VoxelMapManager(const VoxelMapConfig& config = VoxelMapConfig());
    ~VoxelMapManager();

    /// 获取配置
    const VoxelMapConfig& GetConfig() const { return config_; }

    // ===== 核心接口 =====

    /**
     * @brief 首帧初始化地图
     * @param points_with_cov 带协方差的世界坐标系点（point 字段存储世界坐标）
     */
    void InitializeMap(const std::vector<pointWithCov>& points_with_cov);

    /**
     * @brief 增量更新地图（EKF 收敛后调用）
     * @param points_with_cov 带协方差的世界坐标系点
     */
    void UpdateMap(const std::vector<pointWithCov>& points_with_cov);

    /**
     * @brief 查找点到平面对应关系（替代 ivox_->GetClosestPoint + esti_plane）
     *
     * 对每个查询点，在 voxel_map 中查找最佳匹配平面，返回匹配结果。
     * 此函数内部使用 OpenMP 并行加速。
     *
     * @param query_points  查询点列表（需包含 point_world 和 cov）
     * @param ptpl_list     [out] 匹配成功的点-平面对
     * @param non_match     [out] 未匹配的点（可选，用于调试）
     */
    void FindCorrespondences(const std::vector<pointWithCov>& query_points, std::vector<ptpl>& ptpl_list,
                             std::vector<Eigen::Vector3d>* non_match = nullptr);

    // ===== 协方差工具 =====

    /**
     * @brief 计算 body 坐标系下的点测量协方差
     * @param point_body body 坐标系下的 3D 点
     * @return 3x3 协方差矩阵
     */
    Eigen::Matrix3d CalcBodyCov(const Eigen::Vector3d& point_body) const;

    /**
     * @brief 将 body 协方差传播到世界坐标系
     *
     *   Cov_world = R * Cov_body * R^T
     *             + [-Rp×] * Cov_rotation * [-Rp×]^T
     *             + Cov_translation
     *
     * @param body_cov     body 坐标系下的点协方差
     * @param point_body   body 坐标系下的点坐标
     * @param R            当前旋转矩阵
     * @param cov_rot      旋转分量的状态协方差 (3x3)
     * @param cov_trans    平移分量的状态协方差 (3x3)
     * @return 世界坐标系下的 3x3 协方差矩阵
     */
    static Eigen::Matrix3d PropagateCovToWorld(const Eigen::Matrix3d& body_cov, const Eigen::Vector3d& point_body,
                                               const Eigen::Matrix3d& R, const Eigen::Matrix3d& cov_rot,
                                               const Eigen::Matrix3d& cov_trans);

    // ===== 可视化接口 =====

    /**
     * @brief 获取需要更新可视化的平面列表
     * @return 平面信息列表（包含中心、法向、特征值、协方差 trace 等）
     * @note  调用后会清除 is_update 标志
     */
    std::vector<PlaneVis> GetUpdatedPlanesForVis();
    /// 带距离过滤的版本：只返回距 current_pos 在 radius 米以内的平面（radius<=0 表示不过滤）
    std::vector<PlaneVis> GetUpdatedPlanesForVis(const Eigen::Vector3d& current_pos, double radius);

    /**
     * @brief 获取地图中所有平面（用于全量发布）
     * @param max_layer 最大遍历层
     * @return 平面信息列表
     */
    std::vector<PlaneVis> GetAllPlanesForVis(int max_layer = -1);

    /**
     * @brief 获取体素数量
     */
    size_t NumVoxels() const { return voxel_map_.size(); }

   private:
    VoxelMapConfig config_;
    std::unordered_map<VOXEL_LOC, OctoTree*> voxel_map_;
};

}  // namespace voxel_map
}  // namespace lightning
