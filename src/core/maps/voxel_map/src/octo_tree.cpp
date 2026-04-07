/**
 * @file octo_tree.cpp
 * @brief OctoTree 实现
 *
 * 从 VoxelMap (RA-L 2022) voxel_map_util.hpp 提取并重构。
 * 去除了 ROS 依赖，保留核心算法不变。
 */
#include "voxel_map/octo_tree.hpp"
#include <cmath>

namespace lightning {
namespace voxel_map {

// ============================================================================
// OctoTree 构造
// ============================================================================
OctoTree::OctoTree(int max_layer, int layer, std::vector<int> layer_point_size, int max_point_size,
                   int max_cov_points_size, float planer_threshold)
    : max_layer_(max_layer),
      layer_(layer),
      layer_point_size_(layer_point_size),
      max_points_size_(max_point_size),
      max_cov_points_size_(max_cov_points_size),
      planer_threshold_(planer_threshold) {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_num_ = 0;
    all_points_num_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    update_cov_enable_ = true;
    max_plane_update_threshold_ = layer_point_size_[layer_];
    for (int i = 0; i < 8; i++) {
        leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
}

// ============================================================================
// OctoTree 析构（递归释放子节点）
// ============================================================================
OctoTree::~OctoTree() {
    for (int i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            delete leaves_[i];
            leaves_[i] = nullptr;
        }
    }
    if (plane_ptr_ != nullptr) {
        delete plane_ptr_;
        plane_ptr_ = nullptr;
    }
}

// ============================================================================
// 平面拟合（带协方差传播）
// ============================================================================
void OctoTree::init_plane(const std::vector<pointWithCov>& points, Plane* plane) {
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = points.size();
    plane->radius = 0;

    for (const auto& pv : points) {
        plane->covariance += pv.point * pv.point.transpose();
        plane->center += pv.point;
    }
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->covariance / plane->points_size - plane->center * plane->center.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal = evals.real();

    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;

    // 平面协方差计算（VoxelMap 的核心贡献）
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size;

    if (evalsReal(evalsMin) < planer_threshold_) {
        for (size_t i = 0; i < points.size(); i++) {
            Eigen::Matrix<double, 6, 3> J;
            Eigen::Matrix3d F;
            for (int m = 0; m < 3; m++) {
                if (m != (int)evalsMin) {
                    Eigen::Matrix<double, 1, 3> F_m =
                        (points[i].point - plane->center).transpose() /
                        ((plane->points_size) * (evalsReal[evalsMin] - evalsReal[m])) *
                        (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                         evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
                    F.row(m) = F_m;
                } else {
                    F.row(m) = Eigen::Matrix<double, 1, 3>::Zero();
                }
            }
            J.block<3, 3>(0, 0) = evecs.real() * F;
            J.block<3, 3>(3, 0) = J_Q;
            plane->plane_cov += J * points[i].cov * J.transpose();
        }

        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = std::sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                     plane->normal(2) * plane->center(2));
        plane->is_plane = true;

        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }

        if (!plane->is_init) {
            plane->id = g_plane_id++;
            plane->is_init = true;
        }
    } else {
        if (!plane->is_init) {
            plane->id = g_plane_id++;
            plane->is_init = true;
        }
        if (plane->last_update_points_size == 0) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        } else if (plane->points_size - plane->last_update_points_size > 100) {
            plane->last_update_points_size = plane->points_size;
            plane->is_update = true;
        }
        plane->is_plane = false;
        plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
        plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
        plane->min_eigen_value = evalsReal(evalsMin);
        plane->mid_eigen_value = evalsReal(evalsMid);
        plane->max_eigen_value = evalsReal(evalsMax);
        plane->radius = std::sqrt(evalsReal(evalsMax));
        plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                     plane->normal(2) * plane->center(2));
    }
}

// ============================================================================
// 增量平面更新（仅更新几何参数，不重算协方差）
// ============================================================================
void OctoTree::update_plane(const std::vector<pointWithCov>& points, Plane* plane) {
    Eigen::Matrix3d sum_ppt =
        (plane->covariance + plane->center * plane->center.transpose()) * plane->points_size;
    Eigen::Vector3d sum_p = plane->center * plane->points_size;

    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d pv = points[i].point;
        sum_ppt += pv * pv.transpose();
        sum_p += pv;
    }
    plane->points_size = plane->points_size + points.size();
    plane->center = sum_p / plane->points_size;
    plane->covariance = sum_ppt / plane->points_size - plane->center * plane->center.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal = evals.real();

    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;

    plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
    plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid), evecs.real()(2, evalsMid);
    plane->x_normal << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax), evecs.real()(2, evalsMax);
    plane->min_eigen_value = evalsReal(evalsMin);
    plane->mid_eigen_value = evalsReal(evalsMid);
    plane->max_eigen_value = evalsReal(evalsMax);
    plane->radius = std::sqrt(evalsReal(evalsMax));
    plane->d = -(plane->normal(0) * plane->center(0) + plane->normal(1) * plane->center(1) +
                 plane->normal(2) * plane->center(2));

    if (evalsReal(evalsMin) < planer_threshold_) {
        plane->is_plane = true;
    } else {
        plane->is_plane = false;
    }
    plane->is_update = true;
}

// ============================================================================
// 初始化八叉树
// ============================================================================
void OctoTree::init_octo_tree() {
    if (static_cast<int>(temp_points_.size()) > max_plane_update_threshold_) {
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane) {
            octo_state_ = 0;
            if (static_cast<int>(temp_points_.size()) > max_cov_points_size_) {
                update_cov_enable_ = false;
            }
            if (static_cast<int>(temp_points_.size()) > max_points_size_) {
                update_enable_ = false;
            }
        } else {
            octo_state_ = 1;
            cut_octo_tree();
        }
        init_octo_ = true;
        new_points_num_ = 0;
    }
}

// ============================================================================
// 递归分裂八叉树
// ============================================================================
void OctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) {
        octo_state_ = 0;
        return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point[0] > voxel_center_[0]) xyz[0] = 1;
        if (temp_points_[i].point[1] > voxel_center_[1]) xyz[1] = 1;
        if (temp_points_[i].point[2] > voxel_center_[2]) xyz[2] = 1;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] =
                new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_, max_cov_points_size_, planer_threshold_);
            leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_num_++;
    }
    for (int i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            if (static_cast<int>(leaves_[i]->temp_points_.size()) > leaves_[i]->max_plane_update_threshold_) {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane) {
                    leaves_[i]->octo_state_ = 0;
                } else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_num_ = 0;
            }
        }
    }
}

// ============================================================================
// 增量更新
// ============================================================================
void OctoTree::UpdateOctoTree(const pointWithCov& pv) {
    if (!init_octo_) {
        new_points_num_++;
        all_points_num_++;
        temp_points_.push_back(pv);
        if (static_cast<int>(temp_points_.size()) > max_plane_update_threshold_) {
            init_octo_tree();
        }
    } else {
        if (plane_ptr_->is_plane) {
            if (update_enable_) {
                new_points_num_++;
                all_points_num_++;
                if (update_cov_enable_) {
                    temp_points_.push_back(pv);
                } else {
                    new_points_.push_back(pv);
                }
                if (new_points_num_ > update_size_threshold_) {
                    if (update_cov_enable_) {
                        init_plane(temp_points_, plane_ptr_);
                    }
                    new_points_num_ = 0;
                }
                if (all_points_num_ >= max_cov_points_size_) {
                    update_cov_enable_ = false;
                    std::vector<pointWithCov>().swap(temp_points_);
                }
                if (all_points_num_ >= max_points_size_) {
                    update_enable_ = false;
                    plane_ptr_->update_enable = false;
                    std::vector<pointWithCov>().swap(new_points_);
                }
            }
        } else {
            if (layer_ < max_layer_) {
                if (!temp_points_.empty()) {
                    std::vector<pointWithCov>().swap(temp_points_);
                }
                if (!new_points_.empty()) {
                    std::vector<pointWithCov>().swap(new_points_);
                }
                int xyz[3] = {0, 0, 0};
                if (pv.point[0] > voxel_center_[0]) xyz[0] = 1;
                if (pv.point[1] > voxel_center_[1]) xyz[1] = 1;
                if (pv.point[2] > voxel_center_[2]) xyz[2] = 1;
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } else {
                    leaves_[leafnum] = new OctoTree(max_layer_, layer_ + 1, layer_point_size_, max_points_size_,
                                                    max_cov_points_size_, planer_threshold_);
                    leaves_[leafnum]->layer_point_size_ = layer_point_size_;
                    leaves_[leafnum]->voxel_center_[0] = voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] = voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] = voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } else {
                if (update_enable_) {
                    new_points_num_++;
                    all_points_num_++;
                    if (update_cov_enable_) {
                        temp_points_.push_back(pv);
                    } else {
                        new_points_.push_back(pv);
                    }
                    if (new_points_num_ > update_size_threshold_) {
                        if (update_cov_enable_) {
                            init_plane(temp_points_, plane_ptr_);
                        } else {
                            update_plane(new_points_, plane_ptr_);
                            new_points_.clear();
                        }
                        new_points_num_ = 0;
                    }
                    if (all_points_num_ >= max_cov_points_size_) {
                        update_cov_enable_ = false;
                        std::vector<pointWithCov>().swap(temp_points_);
                    }
                    if (all_points_num_ >= max_points_size_) {
                        update_enable_ = false;
                        plane_ptr_->update_enable = false;
                        std::vector<pointWithCov>().swap(new_points_);
                    }
                }
            }
        }
    }
}

// ============================================================================
// 点到平面匹配（递归搜索）
// ============================================================================
void build_single_residual(const pointWithCov& pv, const OctoTree* current_octo, int current_layer, int max_layer,
                           double sigma_num, bool& is_success, double& prob, ptpl& single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_world;

    if (current_octo->plane_ptr_->is_plane) {
        const Plane& plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center;
        double proj_x = p_world_to_center.dot(plane.x_normal);
        double proj_y = p_world_to_center.dot(plane.y_normal);
        (void)proj_x;
        (void)proj_y;

        float dis_to_plane = std::fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
                                       plane.normal(2) * p_w(2) + plane.d);
        float dis_to_center = (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
                               (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
                               (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));
        float range_dis = std::sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center;
            J_nq.block<1, 3>(0, 3) = -plane.normal;
            double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
            sigma_l += plane.normal.transpose() * pv.cov * plane.normal;

            if (dis_to_plane < sigma_num * std::sqrt(sigma_l)) {
                is_success = true;
                double this_prob = 1.0 / (std::sqrt(sigma_l)) * std::exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    single_ptpl.point = pv.point_body;      // body 坐标，用于 Jacobian
                    single_ptpl.point_world = pv.point_world; // world 坐标
                    single_ptpl.plane_cov = plane.plane_cov;
                    single_ptpl.normal = plane.normal;
                    single_ptpl.center = plane.center;
                    single_ptpl.d = plane.d;
                    single_ptpl.layer = current_layer;
                }
                return;
            } else {
                return;
            }
        } else {
            return;
        }
    } else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {
                    build_single_residual(pv, current_octo->leaves_[leafnum], current_layer + 1, max_layer, sigma_num,
                                          is_success, prob, single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
}

// ============================================================================
// 获取需要更新可视化的平面（无距离过滤）
// ============================================================================
void GetUpdatePlane(const OctoTree* current_octo, int pub_max_voxel_layer, std::vector<Plane>& plane_list) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    if (current_octo->plane_ptr_->is_update) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list);
                }
            }
        }
    }
}

// ============================================================================
// 获取需要更新可视化的平面（带距离过滤）
// current_pos + radius_sq：在遍历时用体素中心做提前剔除，避免深入远端子树
// ============================================================================
void GetUpdatePlane(const OctoTree* current_octo, int pub_max_voxel_layer, std::vector<Plane>& plane_list,
                    const Eigen::Vector3d& current_pos, double radius_sq) {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    // 用体素中心做距离预判：若体素中心已超出 radius，跳过整颗子树
    Eigen::Vector3d center(current_octo->voxel_center_[0], current_octo->voxel_center_[1],
                           current_octo->voxel_center_[2]);
    if ((center - current_pos).squaredNorm() > radius_sq) {
        return;
    }
    if (current_octo->plane_ptr_->is_update) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i], pub_max_voxel_layer, plane_list, current_pos, radius_sq);
                }
            }
        }
    }
}

}  // namespace voxel_map
}  // namespace lightning
