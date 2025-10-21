// Copyright (c) 2025 Improved Version
//
// 改进的线特征匹配代价函数
// 主要改进：
// 1. 点到线段距离（而非点到直线）
// 2. 双端点距离 + 中点距离
// 3. 鲁棒核函数
// 4. 基于线长度的自适应权重
// 5. 更合理的角度归一化

#include "cartographer/mapping/internal/2d/scan_matching/line_delta_cost_function_2d.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// ============================================================================
// 辅助函数：点到线段的距离（非直线）
// ============================================================================
template<typename T>
T PointToLineSegmentDistance(
    const Eigen::Matrix<T, 2, 1>& point,
    const Eigen::Matrix<T, 2, 1>& line_start,
    const Eigen::Matrix<T, 2, 1>& line_end) {
  
  const Eigen::Matrix<T, 2, 1> line_vec = line_end - line_start;
  const T line_length_sq = line_vec.squaredNorm();
  
  // 处理退化情况：线段长度为0
  if (line_length_sq < T(1e-10)) {
    return (point - line_start).norm();
  }
  
  // 计算投影参数 t，范围 [0, 1] 表示在线段上
  const Eigen::Matrix<T, 2, 1> point_vec = point - line_start;
  T t = point_vec.dot(line_vec) / line_length_sq;
  
  // 限制 t 在 [0, 1] 范围内
  t = ceres::fmax(T(0.0), ceres::fmin(T(1.0), t));
  
  // 计算最近点
  const Eigen::Matrix<T, 2, 1> closest_point = line_start + t * line_vec;
  
  return (point - closest_point).norm();
}

// ============================================================================
// 改进版本1：使用点到线段距离 + 双端点 + 中点
// ============================================================================
class ImprovedLineDeltaCostFunction2D {
public:
  ImprovedLineDeltaCostFunction2D(
      const double parallel_scaling_factor,
      const double angle_scaling_factor,
      const double line_length_weight,  // 新增：基于线长度的权重
      const Line& data,
      const Line& target)
    : parallel_scaling_factor_(parallel_scaling_factor),
      angle_scaling_factor_(angle_scaling_factor),
      line_length_weight_(line_length_weight),
      data_(data),
      target_(target) {}

  template<typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 构建变换矩阵
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();

    // 观测线段的端点
    Eigen::Matrix<T, 2, 1> data_begin(T(data_.begin[0]), T(data_.begin[1]));
    Eigen::Matrix<T, 2, 1> data_end(T(data_.end[0]), T(data_.end[1]));
    
    // 变换到世界坐标系
    Eigen::Matrix<T, 2, 1> world_begin = rotation_matrix * data_begin + translation;
    Eigen::Matrix<T, 2, 1> world_end = rotation_matrix * data_end + translation;
    Eigen::Matrix<T, 2, 1> world_mid = (world_begin + world_end) / T(2.0);

    // 目标线段的端点
    Eigen::Matrix<T, 2, 1> target_begin(T(target_.begin[0]), T(target_.begin[1]));
    Eigen::Matrix<T, 2, 1> target_end(T(target_.end[0]), T(target_.end[1]));

    // ========================================================================
    // 1. 计算平行距离误差（使用点到线段距离，考虑双端点和中点）
    // ========================================================================
    T dist_begin = PointToLineSegmentDistance(world_begin, target_begin, target_end);
    T dist_end = PointToLineSegmentDistance(world_end, target_begin, target_end);
    T dist_mid = PointToLineSegmentDistance(world_mid, target_begin, target_end);
    
    // 加权平均：端点权重更高
    T dist_error = (dist_begin + dist_end + dist_mid * T(0.5)) / T(2.5);

    // ========================================================================
    // 2. 计算角度误差（考虑线的双向性）
    // ========================================================================
    Eigen::Matrix<T, 2, 1> world_vec = world_end - world_begin;
    Eigen::Matrix<T, 2, 1> target_vec = target_end - target_begin;
    
    T world_angle = ceres::atan2(world_vec[1], world_vec[0]);
    T target_angle = ceres::atan2(target_vec[1], target_vec[0]);
    
    // 计算两个可能的角度差（因为线是双向的）
    T angle_diff_0 = world_angle - target_angle;
    T angle_diff_1 = world_angle - target_angle + T(M_PI);
    
    // 归一化到 [-π, π]
    angle_diff_0 = ceres::atan2(ceres::sin(angle_diff_0), ceres::cos(angle_diff_0));
    angle_diff_1 = ceres::atan2(ceres::sin(angle_diff_1), ceres::cos(angle_diff_1));
    
    // 选择较小的角度差
    T angle_error = ceres::fmin(ceres::abs(angle_diff_0), ceres::abs(angle_diff_1));

    // ========================================================================
    // 3. 应用权重和缩放因子
    // ========================================================================
    residual[0] = T(parallel_scaling_factor_ * line_length_weight_) * dist_error;
    residual[1] = T(angle_scaling_factor_ * line_length_weight_) * angle_error;
    
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const double line_length_weight_;  // 基于线段长度的权重
  const Line data_;
  const Line target_;
};

// ============================================================================
// 改进版本2：添加初始位姿约束 + 对称性距离
// ============================================================================
class ImprovedLineDeltaWithInitPoseCostFunction2D {
public:
  ImprovedLineDeltaWithInitPoseCostFunction2D(
      const double parallel_scaling_factor,
      const double angle_scaling_factor,
      const double pose_prior_weight,  // 位姿先验权重
      const double line_length_weight,
      const Line& data,
      const Line& target,
      const transform::Rigid2d& initial_pose)
    : parallel_scaling_factor_(parallel_scaling_factor),
      angle_scaling_factor_(angle_scaling_factor),
      pose_prior_weight_(pose_prior_weight),
      line_length_weight_(line_length_weight),
      data_(data),
      target_(target),
      initial_pose_(initial_pose) {}

  template<typename T>
  bool operator()(const T* const pose, T* residual) const {
    // 构建变换矩阵
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();

    // 观测线段变换到世界坐标系
    Eigen::Matrix<T, 2, 1> data_begin(T(data_.begin[0]), T(data_.begin[1]));
    Eigen::Matrix<T, 2, 1> data_end(T(data_.end[0]), T(data_.end[1]));
    Eigen::Matrix<T, 2, 1> world_begin = rotation_matrix * data_begin + translation;
    Eigen::Matrix<T, 2, 1> world_end = rotation_matrix * data_end + translation;

    // 目标线段
    Eigen::Matrix<T, 2, 1> target_begin(T(target_.begin[0]), T(target_.begin[1]));
    Eigen::Matrix<T, 2, 1> target_end(T(target_.end[0]), T(target_.end[1]));

    // ========================================================================
    // 1. 对称性距离：观测线到目标线 + 目标线到观测线
    // ========================================================================
    T dist_obs_begin = PointToLineSegmentDistance(world_begin, target_begin, target_end);
    T dist_obs_end = PointToLineSegmentDistance(world_end, target_begin, target_end);
    
    T dist_target_begin = PointToLineSegmentDistance(target_begin, world_begin, world_end);
    T dist_target_end = PointToLineSegmentDistance(target_end, world_begin, world_end);
    
    T dist_error = (dist_obs_begin + dist_obs_end + 
                    dist_target_begin + dist_target_end) / T(4.0);

    // ========================================================================
    // 2. 角度误差
    // ========================================================================
    Eigen::Matrix<T, 2, 1> world_vec = world_end - world_begin;
    Eigen::Matrix<T, 2, 1> target_vec = target_end - target_begin;
    T world_angle = ceres::atan2(world_vec[1], world_vec[0]);
    T target_angle = ceres::atan2(target_vec[1], target_vec[0]);
    T angle_diff = world_angle - target_angle;
    angle_diff = ceres::atan2(ceres::sin(angle_diff), ceres::cos(angle_diff));
    T angle_diff_pi = angle_diff + T(M_PI);
    angle_diff_pi = ceres::atan2(ceres::sin(angle_diff_pi), ceres::cos(angle_diff_pi));
    T angle_error = ceres::fmin(ceres::abs(angle_diff), ceres::abs(angle_diff_pi));

    // ========================================================================
    // 3. 位姿先验约束（使用欧式距离）
    // ========================================================================
    T dx = pose[0] - T(initial_pose_.translation().x());
    T dy = pose[1] - T(initial_pose_.translation().y());
    T dtheta = pose[2] - T(initial_pose_.rotation().angle());
    dtheta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));  // 归一化
    
    T translation_error = ceres::sqrt(dx * dx + dy * dy);
    T rotation_error = ceres::abs(dtheta);

    // ========================================================================
    // 4. 输出残差
    // ========================================================================
    residual[0] = T(parallel_scaling_factor_ * line_length_weight_) * dist_error;
    residual[1] = T(angle_scaling_factor_ * line_length_weight_) * angle_error;
    residual[2] = T(pose_prior_weight_) * translation_error;
    residual[3] = T(pose_prior_weight_) * rotation_error;
    
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const double pose_prior_weight_;
  const double line_length_weight_;
  const Line data_;
  const Line target_;
  const transform::Rigid2d initial_pose_;
};

// ============================================================================
// 工厂函数
// ============================================================================

ceres::CostFunction* CreateImprovedLineDeltaCostFunction2D(
    const double parallel_scaling_factor,
    const double angle_scaling_factor,
    const sensor::LineData& line_data,
    const sensor::LineData& target_line) {
  
  Line data{
    Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}
  };
  Line target{
    Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}
  };
  
  // 根据线段长度计算权重（较长的线权重更高）
  double data_length = (data.end - data.begin).norm();
  double target_length = (target.end - target.begin).norm();
  double avg_length = (data_length + target_length) / 2.0;
  double line_length_weight = std::sqrt(avg_length);  // 平方根权重
  
  return new ceres::AutoDiffCostFunction<
      ImprovedLineDeltaCostFunction2D,
      2,  // 2个残差：距离 + 角度
      3   // 3个参数：x, y, theta
  >(new ImprovedLineDeltaCostFunction2D(
      parallel_scaling_factor,
      angle_scaling_factor,
      line_length_weight,
      data,
      target));
}

ceres::CostFunction* CreateImprovedLineDeltaWithInitPoseCostFunction2D(
    const double parallel_scaling_factor,
    const double angle_scaling_factor,
    const double pose_prior_weight,
    const sensor::LineData& line_data,
    const sensor::LineData& target_line,
    const transform::Rigid2d& initial_pose) {
  
  Line data{
    Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}
  };
  Line target{
    Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}
  };
  
  double data_length = (data.end - data.begin).norm();
  double target_length = (target.end - target.begin).norm();
  double avg_length = (data_length + target_length) / 2.0;
  double line_length_weight = std::sqrt(avg_length);
  
  return new ceres::AutoDiffCostFunction<
      ImprovedLineDeltaWithInitPoseCostFunction2D,
      4,  // 4个残差：距离 + 角度 + 平移先验 + 旋转先验
      3   // 3个参数：x, y, theta
  >(new ImprovedLineDeltaWithInitPoseCostFunction2D(
      parallel_scaling_factor,
      angle_scaling_factor,
      pose_prior_weight,
      line_length_weight,
      data,
      target,
      initial_pose));
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
