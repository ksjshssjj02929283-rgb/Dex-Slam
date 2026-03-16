/**
 * @file voxel_map_manager.cpp
 * @brief VoxelMap 管理器实现
 *
 * 从 VoxelMap (RA-L 2022) 的 buildVoxelMap, updateVoxelMap,
 * BuildResidualListOMP, calcBodyCov 等函数提取并重构。
 */
#include "voxel_map/voxel_map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <execution>
#include <mutex>
#include <numeric>

namespace lightning {
namespace voxel_map {

// ============================================================================
// 构造 / 析构
// ============================================================================
VoxelMapManager::VoxelMapManager(const VoxelMapConfig& config) : config_(config) {}

VoxelMapManager::~VoxelMapManager() {
    for (auto& pair : voxel_map_) {
        if (pair.second != nullptr) {
            delete pair.second;
            pair.second = nullptr;
        }
    }
    voxel_map_.clear();
}

// ============================================================================
// 首帧初始化地图 (对应原 buildVoxelMap)
// ============================================================================
void VoxelMapManager::InitializeMap(const std::vector<pointWithCov>& points_with_cov) {
    for (size_t i = 0; i < points_with_cov.size(); i++) {
        const pointWithCov& p_v = points_with_cov[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / config_.voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_num_++;
        } else {
            OctoTree* octo_tree = new OctoTree(config_.max_layer, 0, config_.layer_point_size, config_.max_points_size,
                                               config_.max_cov_points_size, config_.planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = config_.voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * config_.voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * config_.voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * config_.voxel_size;
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_num_++;
            voxel_map_[position]->layer_point_size_ = config_.layer_point_size;
        }
    }
    // 初始化每个体素的八叉树
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        iter->second->init_octo_tree();
    }
}

// ============================================================================
// 增量更新地图 (对应原 updateVoxelMap)
// ============================================================================
void VoxelMapManager::UpdateMap(const std::vector<pointWithCov>& points_with_cov) {
    for (size_t i = 0; i < points_with_cov.size(); i++) {
        const pointWithCov& p_v = points_with_cov[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point[j] / config_.voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->UpdateOctoTree(p_v);
        } else {
            OctoTree* octo_tree = new OctoTree(config_.max_layer, 0, config_.layer_point_size, config_.max_points_size,
                                               config_.max_cov_points_size, config_.planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = config_.voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] = (0.5 + position.x) * config_.voxel_size;
            voxel_map_[position]->voxel_center_[1] = (0.5 + position.y) * config_.voxel_size;
            voxel_map_[position]->voxel_center_[2] = (0.5 + position.z) * config_.voxel_size;
            voxel_map_[position]->UpdateOctoTree(p_v);
        }
    }
}

// ============================================================================
// 查找点到平面对应关系 (对应原 BuildResidualListOMP)
// ============================================================================
void VoxelMapManager::FindCorrespondences(const std::vector<pointWithCov>& query_points,
                                          std::vector<ptpl>& ptpl_list,
                                          std::vector<Eigen::Vector3d>* non_match) {
    const size_t n = query_points.size();
    ptpl_list.clear();

    // 预分配：每个查询点的结果槽（避免 push_back 的竞争）
    std::vector<ptpl> all_ptpl_list(n);
    std::vector<uint8_t> useful_ptpl(n, 0);  // uint8_t 比 bool 更 cache-friendly

    // 并行查找 —— 使用 std::execution::par_unseq（与项目其余部分一致）
    std::vector<size_t> indices(n);
    for (size_t i = 0; i < n; ++i) indices[i] = i;

    std::for_each(std::execution::par_unseq, indices.begin(), indices.end(), [&](const size_t i) {
        const pointWithCov& pv = query_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_world[j] / config_.voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            OctoTree* current_octo = iter->second;
            ptpl single_ptpl;
            bool is_success = false;
            double prob = 0;
            build_single_residual(pv, current_octo, 0, config_.max_layer, config_.sigma_num, is_success, prob,
                                  single_ptpl);

            // Near Search：如果点位于体素边界附近，检查相邻体素
            if (!is_success) {
                VOXEL_LOC near_position = position;
                if (loc_xyz[0] > (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
                    near_position.x = near_position.x + 1;
                } else if (loc_xyz[0] < (current_octo->voxel_center_[0] - current_octo->quater_length_)) {
                    near_position.x = near_position.x - 1;
                }
                if (loc_xyz[1] > (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
                    near_position.y = near_position.y + 1;
                } else if (loc_xyz[1] < (current_octo->voxel_center_[1] - current_octo->quater_length_)) {
                    near_position.y = near_position.y - 1;
                }
                if (loc_xyz[2] > (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
                    near_position.z = near_position.z + 1;
                } else if (loc_xyz[2] < (current_octo->voxel_center_[2] - current_octo->quater_length_)) {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map_.find(near_position);
                if (iter_near != voxel_map_.end()) {
                    build_single_residual(pv, iter_near->second, 0, config_.max_layer, config_.sigma_num, is_success,
                                          prob, single_ptpl);
                }
            }

            if (is_success) {
                useful_ptpl[i] = 1;
                all_ptpl_list[i] = single_ptpl;
            }
        }
    });

    // 预计数 → 一次性分配 → 紧凑拷贝
    size_t match_count = 0;
    for (size_t i = 0; i < n; i++) {
        if (useful_ptpl[i]) ++match_count;
    }
    ptpl_list.reserve(match_count);
    for (size_t i = 0; i < n; i++) {
        if (useful_ptpl[i]) {
            ptpl_list.push_back(all_ptpl_list[i]);
        } else if (non_match) {
            non_match->push_back(query_points[i].point_world);
        }
    }
}

// ============================================================================
// 计算 body 坐标系下的点测量协方差 (对应原 calcBodyCov)
// ============================================================================
Eigen::Matrix3d VoxelMapManager::CalcBodyCov(const Eigen::Vector3d& point_body) const {
    float range = std::sqrt(point_body[0] * point_body[0] + point_body[1] * point_body[1] +
                            point_body[2] * point_body[2]);
    float range_var = config_.ranging_cov;

    Eigen::Matrix2d direction_var;
    direction_var << std::pow(std::sin(config_.angle_cov), 2), 0, 0, std::pow(std::sin(config_.angle_cov), 2);

    Eigen::Vector3d direction(point_body);
    direction.normalize();

    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;

    Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
    base_vector2.normalize();

    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);

    Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;

    Eigen::Matrix3d cov = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
    return cov;
}

// ============================================================================
// 协方差传播到世界坐标系
// ============================================================================
Eigen::Matrix3d VoxelMapManager::PropagateCovToWorld(const Eigen::Matrix3d& body_cov,
                                                     const Eigen::Vector3d& point_body, const Eigen::Matrix3d& R,
                                                     const Eigen::Matrix3d& cov_rot,
                                                     const Eigen::Matrix3d& cov_trans) {
    // Rp 在世界坐标系下
    Eigen::Vector3d Rp = R * point_body;

    // [Rp]× 反对称矩阵
    Eigen::Matrix3d Rp_cross;
    Rp_cross << 0, -Rp(2), Rp(1), Rp(2), 0, -Rp(0), -Rp(1), Rp(0), 0;

    // Cov_world = R * Cov_body * R^T + [Rp×] * Cov_rot * [Rp×]^T + Cov_trans
    Eigen::Matrix3d cov_world =
        R * body_cov * R.transpose() + Rp_cross * cov_rot * Rp_cross.transpose() + cov_trans;

    return cov_world;
}

// ============================================================================
// 获取需要更新的平面用于可视化
// ============================================================================
std::vector<PlaneVis> VoxelMapManager::GetUpdatedPlanesForVis() {
    std::vector<Plane> raw_planes;
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        GetUpdatePlane(iter->second, config_.pub_max_voxel_layer, raw_planes);
    }

    std::vector<PlaneVis> result;
    result.reserve(raw_planes.size());
    for (auto& p : raw_planes) {
        if (p.is_plane) {
            PlaneVis vis;
            vis.center = p.center;
            vis.normal = p.normal;
            vis.x_normal = p.x_normal;
            vis.y_normal = p.y_normal;
            vis.max_eigen_value = p.max_eigen_value;
            vis.mid_eigen_value = p.mid_eigen_value;
            vis.min_eigen_value = p.min_eigen_value;
            vis.is_plane = p.is_plane;
            vis.id = p.id;
            vis.radius = p.radius;
            vis.plane_cov_trace = p.plane_cov.block<3, 3>(0, 0).diagonal().sum();
            result.push_back(vis);
        }
    }
    return result;
}

std::vector<PlaneVis> VoxelMapManager::GetAllPlanesForVis(int max_layer) {
    if (max_layer < 0) max_layer = config_.max_layer;

    std::vector<Plane> raw_planes;
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        GetUpdatePlane(iter->second, max_layer, raw_planes);
    }

    std::vector<PlaneVis> result;
    result.reserve(raw_planes.size());
    for (auto& p : raw_planes) {
        PlaneVis vis;
        vis.center = p.center;
        vis.normal = p.normal;
        vis.x_normal = p.x_normal;
        vis.y_normal = p.y_normal;
        vis.max_eigen_value = p.max_eigen_value;
        vis.mid_eigen_value = p.mid_eigen_value;
        vis.min_eigen_value = p.min_eigen_value;
        vis.is_plane = p.is_plane;
        vis.id = p.id;
        vis.radius = p.radius;
        vis.plane_cov_trace = p.plane_cov.block<3, 3>(0, 0).diagonal().sum();
        result.push_back(vis);
    }
    return result;
}

/// 带距离过滤的重载：只返回距 current_pos 在 radius 米以内的平面
std::vector<PlaneVis> VoxelMapManager::GetUpdatedPlanesForVis(const Eigen::Vector3d& current_pos, double radius) {
    if (radius <= 0.0) {
        return GetUpdatedPlanesForVis();
    }

    const double radius_sq = radius * radius;
    std::vector<Plane> raw_planes;
    // 距离过滤下沉到 GetUpdatePlane：直接用体素中心剔除远端体素，不进入其子树
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        GetUpdatePlane(iter->second, config_.pub_max_voxel_layer, raw_planes, current_pos, radius_sq);
    }

    std::vector<PlaneVis> result;
    result.reserve(raw_planes.size());
    for (auto& p : raw_planes) {
        if (!p.is_plane) continue;
        PlaneVis vis;
        vis.center = p.center;
        vis.normal = p.normal;
        vis.x_normal = p.x_normal;
        vis.y_normal = p.y_normal;
        vis.max_eigen_value = p.max_eigen_value;
        vis.mid_eigen_value = p.mid_eigen_value;
        vis.min_eigen_value = p.min_eigen_value;
        vis.is_plane = p.is_plane;
        vis.id = p.id;
        vis.radius = p.radius;
        vis.plane_cov_trace = p.plane_cov.block<3, 3>(0, 0).diagonal().sum();
        result.push_back(vis);
    }
    return result;
}

}  // namespace voxel_map
}  // namespace lightning
