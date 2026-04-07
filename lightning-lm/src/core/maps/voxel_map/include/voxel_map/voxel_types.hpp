/**
 * @file voxel_types.hpp
 * @brief VoxelMap 核心数据类型定义
 *
 * 从 VoxelMap (RA-L 2022) 提取的纯数据结构，无 ROS 依赖。
 * 包含：pointWithCov, ptpl (点到平面匹配), VOXEL_LOC (体素坐标), Plane (平面参数)
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <unordered_map>
#include <vector>

namespace lightning {
namespace voxel_map {

#define HASH_P 116101
#define MAX_N 10000000000

// ============================================================================
// 带协方差的 3D 点
// ============================================================================
struct pointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;        // world 坐标系下的点坐标（用于 OctoTree 内部存储和平面拟合）
    Eigen::Vector3d point_world;  // world 坐标系下的点坐标（与 point 相同，保持兼容）
    Eigen::Vector3d point_body;   // body 坐标系下的点坐标（用于 Jacobian 计算）
    Eigen::Matrix3d cov;          // 3x3 协方差矩阵（world 坐标系）

    pointWithCov() : point(Eigen::Vector3d::Zero()), point_world(Eigen::Vector3d::Zero()),
                     point_body(Eigen::Vector3d::Zero()), cov(Eigen::Matrix3d::Zero()) {}
};

// ============================================================================
// 点到平面匹配结果
// ============================================================================
struct ptpl {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;                    // body 坐标系下的点（用于 Jacobian）
    Eigen::Vector3d point_world;              // world 坐标系下的点
    Eigen::Vector3d normal;                   // 匹配平面的法向量
    Eigen::Vector3d center;                   // 匹配平面的中心
    Eigen::Matrix<double, 6, 6> plane_cov;    // 平面的 6x6 协方差
    double d;                                 // 平面参数 d (n·x + d = 0)
    int layer;                                // 匹配发生的八叉树层级
};

// ============================================================================
// 平面参数
// ============================================================================
struct Plane {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d center       = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal       = Eigen::Vector3d::Zero();
    Eigen::Vector3d y_normal     = Eigen::Vector3d::Zero();
    Eigen::Vector3d x_normal     = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance   = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, 6, 6> plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    float radius            = 0;
    float min_eigen_value   = 1;
    float mid_eigen_value   = 1;
    float max_eigen_value   = 1;
    float d                 = 0;
    int points_size         = 0;
    bool is_plane           = false;
    bool is_init            = false;
    int id                  = 0;
    bool is_update          = false;
    int last_update_points_size = 0;
    bool update_enable      = true;
};

// ============================================================================
// 体素坐标（用于哈希表的 Key）
// ============================================================================
class VOXEL_LOC {
   public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC& other) const { return (x == other.x && y == other.y && z == other.z); }
};

// ============================================================================
// 可视化用的平面信息（轻量级，用于跨模块传递）
// ============================================================================
struct PlaneVis {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    Eigen::Vector3d x_normal;
    Eigen::Vector3d y_normal;
    float max_eigen_value = 0;
    float mid_eigen_value = 0;
    float min_eigen_value = 0;
    bool is_plane = false;
    double plane_cov_trace = 0;  // plane_cov 前3x3 对角线之和，用于着色
    int id = 0;
    float radius = 0;
};

}  // namespace voxel_map
}  // namespace lightning

// ============================================================================
// VOXEL_LOC 的哈希函数
// ============================================================================
namespace std {
template <>
struct hash<lightning::voxel_map::VOXEL_LOC> {
    int64_t operator()(const lightning::voxel_map::VOXEL_LOC& s) const {
        using namespace lightning::voxel_map;
        return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
    }
};
}  // namespace std
