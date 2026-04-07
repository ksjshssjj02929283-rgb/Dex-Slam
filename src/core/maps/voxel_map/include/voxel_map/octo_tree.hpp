/**
 * @file octo_tree.hpp
 * @brief 自适应八叉树 + 平面拟合
 *
 * 从 VoxelMap (RA-L 2022) 提取，核心算法包括：
 *  - OctoTree: 自适应粗到精八叉树结构
 *  - init_plane / update_plane: 带协方差的平面拟合
 *  - build_single_residual: 点到平面匹配（递归搜索）
 *
 * 无 ROS 依赖，仅需 Eigen。
 */
#pragma once

#include "voxel_map/voxel_types.hpp"
#include <vector>

namespace lightning {
namespace voxel_map {

// 全局平面 ID 计数器
static int g_plane_id = 0;

class OctoTree {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<pointWithCov> temp_points_;  // 当前八叉树节点存储的所有点
    std::vector<pointWithCov> new_points_;   // 增量更新时的新点
    Plane* plane_ptr_;
    int max_layer_;
    int layer_;
    int octo_state_;  // 0: 叶节点（已拟合平面或到达最大层）, 1: 非叶节点
    OctoTree* leaves_[8];
    double voxel_center_[3];
    std::vector<int> layer_point_size_;
    float quater_length_;
    float planer_threshold_;
    int max_plane_update_threshold_;
    int update_size_threshold_;
    int all_points_num_;
    int new_points_num_;
    int max_points_size_;
    int max_cov_points_size_;
    bool init_octo_;
    bool update_cov_enable_;
    bool update_enable_;

    OctoTree(int max_layer, int layer, std::vector<int> layer_point_size, int max_point_size, int max_cov_points_size,
             float planer_threshold);

    ~OctoTree();

    /// 平面拟合（完整协方差计算）
    void init_plane(const std::vector<pointWithCov>& points, Plane* plane);

    /// 增量平面更新（仅更新 normal, center, radius）
    void update_plane(const std::vector<pointWithCov>& points, Plane* plane);

    /// 初始化八叉树
    void init_octo_tree();

    /// 递归分裂
    void cut_octo_tree();

    /// 增量更新（插入单个带协方差的点）
    void UpdateOctoTree(const pointWithCov& pv);
};

// ============================================================================
// 点到平面匹配（递归）
// ============================================================================
/**
 * @brief 在八叉树中递归搜索最佳匹配平面
 * @param pv            查询点（含 world 坐标和协方差）
 * @param current_octo  当前八叉树节点
 * @param current_layer 当前层级
 * @param max_layer     最大层级
 * @param sigma_num     3σ 检验的 sigma 倍数
 * @param is_success    [out] 是否找到匹配
 * @param prob          [out] 匹配概率
 * @param single_ptpl   [out] 匹配结果
 */
void build_single_residual(const pointWithCov& pv, const OctoTree* current_octo, int current_layer, int max_layer,
                           double sigma_num, bool& is_success, double& prob, ptpl& single_ptpl);

// ============================================================================
// 获取需要更新的平面（用于可视化）
// ============================================================================
/// 无距离过滤版本（遍历全量）
void GetUpdatePlane(const OctoTree* current_octo, int pub_max_voxel_layer, std::vector<Plane>& plane_list);

/// 带距离过滤版本：current_pos + radius_sq 用于在遍历时提前剔除远端体素
void GetUpdatePlane(const OctoTree* current_octo, int pub_max_voxel_layer, std::vector<Plane>& plane_list,
                    const Eigen::Vector3d& current_pos, double radius_sq);

}  // namespace voxel_map
}  // namespace lightning
