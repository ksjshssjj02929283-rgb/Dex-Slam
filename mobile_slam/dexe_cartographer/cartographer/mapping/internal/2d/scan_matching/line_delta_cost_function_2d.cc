// Copyright (c) 2021 Geekplus Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer/mapping/internal/2d/scan_matching/line_delta_cost_function_2d.h"

#include <cmath>
#include <limits>

// ceres::CostFunction* CreateLineDeltaCostFunction2D(
//   const double parallel_scaling_factor, const double angle_scaling_factor,
//   const std::vector<sensor::LineData>& line_data, const LineMap2DList& line_map);
namespace cartographer
{
namespace mapping
{
namespace scan_matching
{

class LineDeltaWithAngleCostAndInitPoseFunction2D
{
public:
  LineDeltaWithAngleCostAndInitPoseFunction2D(
    const double parallel_scaling_factor, const double angle_scaling_factor,
    Line & data, Line & target, const transform::Rigid2d & initial_pose)
  : parallel_scaling_factor_(parallel_scaling_factor), angle_scaling_factor_(angle_scaling_factor),
    data_(data), target_(target), initial_pose_(initial_pose) {}

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    // 观测线的起止点
    Eigen::Matrix<T, 3, 1> point_begin((T(data_.begin[0])), (T(data_.begin[1])), T(1.0));
    Eigen::Matrix<T, 3, 1> point_end((T(data_.end[0])), (T(data_.end[1])), T(1.0));

    // 观测线在世界坐标下的起止点坐标
    const Eigen::Matrix<T, 3, 1> world_b = transform * point_begin;
    const Eigen::Matrix<T, 3, 1> world_e = transform * point_end;
    // 世界坐标系下，观测线的角度
    T world_angle = ceres::atan2(world_e[1] - world_b[1], world_e[0] - world_b[0]);
    Eigen::Rotation2D<T> world_rotation_0(world_angle);
    Eigen::Rotation2D<T> world_rotation_1(world_angle + T(M_PI));

    // target line 的起止点
    const Eigen::Matrix<T, 2, 1> target_begin((T(target_.begin[0])),
      (T(target_.begin[1])));
    const Eigen::Matrix<T, 2, 1> target_end((T(target_.end[0])),
      (T(target_.end[1])));
    // target line 的角度
    Eigen::Rotation2D<T> target_angle((T(target_.angle())));
    
    // 角度误差(误差有两个，delta 和 delta + M_PI，因为线有两个方向，所以有两个方向)
    const Eigen::Rotation2D<T> delta_angle_0 = world_rotation_0.inverse() * target_angle;
    const Eigen::Rotation2D<T> delta_angle_1 = world_rotation_1.inverse() * target_angle;
    // std::cout << "delta_angle_0.angle(): " << delta_angle_0.angle() << std::endl;
    // std::cout << "delta_angle_1.angle(): " << delta_angle_1.angle() << std::endl;
    // 误差是最小的那个
    const T delta_angle = ceres::fmin(
      ceres::abs(
        ceres::atan2(
          ceres::sin(delta_angle_0.angle()),
          ceres::cos(delta_angle_0.angle()))),
      ceres::abs(
        ceres::atan2(
          ceres::sin(delta_angle_1.angle()),
          ceres::cos(delta_angle_1.angle()))));
    // std::cout << "delta_angle: " << delta_angle << std::endl;

    // ========== 计算双端点到目标线段的距离 ==========
    const T target_dx = target_begin[0] - target_end[0];
    const T target_dy = target_begin[1] - target_end[1];
    const T target_length_sq = target_dx * target_dx + target_dy * target_dy;
    
    // 避免除零
    if (target_length_sq < T(1e-10)) {
      return false;
    }
    
    // 计算起点到目标线段的距离
    T u_begin = ((world_b[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
                 (world_b[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    // 限制投影参数在 [0, 1] 范围内
    u_begin = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u_begin));
    T closest_x_begin = target_begin[0] + u_begin * (target_end[0] - target_begin[0]);
    T closest_y_begin = target_begin[1] + u_begin * (target_end[1] - target_begin[1]);
    T dist_begin = ceres::sqrt((world_b[0] - closest_x_begin) * (world_b[0] - closest_x_begin) +
                               (world_b[1] - closest_y_begin) * (world_b[1] - closest_y_begin));
    
    // 计算终点到目标线段的距离
    T u_end = ((world_e[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
               (world_e[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    u_end = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u_end));
    T closest_x_end = target_begin[0] + u_end * (target_end[0] - target_begin[0]);
    T closest_y_end = target_begin[1] + u_end * (target_end[1] - target_begin[1]);
    T dist_end = ceres::sqrt((world_e[0] - closest_x_end) * (world_e[0] - closest_x_end) +
                             (world_e[1] - closest_y_end) * (world_e[1] - closest_y_end));
    
    // 使用双端点距离的平均值，而非单端点
    T delta_tra = (dist_begin + dist_end) / T(2.0);

    T dx = pose[0] - T(initial_pose_.translation().x());
    T dy = pose[1] - T(initial_pose_.translation().y());
    T translation_delta = ceres::sqrt(dx * dx + dy * dy);

    residual[0] = parallel_scaling_factor_ * delta_tra;
    // residual[1] = parallel_scaling_factor_ * delta_end;
    residual[1] = angle_scaling_factor_ * delta_angle;
    residual[2] = translation_delta * (parallel_scaling_factor_ / 10);
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const Line data_;
  const Line target_;
  transform::Rigid2d initial_pose_;
};

class LineDeltaWithAngleCostFunction2D
{
public:
  LineDeltaWithAngleCostFunction2D(
    const double parallel_scaling_factor, const double angle_scaling_factor,
    Line & data, Line & target)
  : parallel_scaling_factor_(parallel_scaling_factor), angle_scaling_factor_(angle_scaling_factor),
    data_(data), target_(target) {}

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    Eigen::Matrix<T, 3, 1> point_begin((T(data_.begin[0])), (T(data_.begin[1])), T(1.0));
    Eigen::Matrix<T, 3, 1> point_end((T(data_.end[0])), (T(data_.end[1])), T(1.0));

    const Eigen::Matrix<T, 3, 1> world_b = transform * point_begin;
    const Eigen::Matrix<T, 3, 1> world_e = transform * point_end;
    T world_angle = ceres::atan2(world_e[1] - world_b[1], world_e[0] - world_b[0]);
    Eigen::Rotation2D<T> world_rotation_0(world_angle);
    Eigen::Rotation2D<T> world_rotation_1(world_angle + T(M_PI));

    const Eigen::Matrix<T, 2, 1> target_begin((T(target_.begin[0])),
      (T(target_.begin[1])));
    const Eigen::Matrix<T, 2, 1> target_end((T(target_.end[0])),
      (T(target_.end[1])));
    Eigen::Rotation2D<T> target_angle((T(target_.angle())));
    const Eigen::Rotation2D<T> delta_angle_0 = world_rotation_0.inverse() * target_angle;
    const Eigen::Rotation2D<T> delta_angle_1 = world_rotation_1.inverse() * target_angle;
    // std::cout << "delta_angle_0.angle(): " << delta_angle_0.angle() << std::endl;
    // std::cout << "delta_angle_1.angle(): " << delta_angle_1.angle() << std::endl;
    const T delta_angle = ceres::fmin(
      ceres::abs(
        ceres::atan2(
          ceres::sin(delta_angle_0.angle()),
          ceres::cos(delta_angle_0.angle()))),
      ceres::abs(
        ceres::atan2(
          ceres::sin(delta_angle_1.angle()),
          ceres::cos(delta_angle_1.angle()))));  // 角度归一化
    // std::cout << "delta_angle: " << delta_angle << std::endl;

    const T target_dx = target_begin[0] - target_end[0];
    const T target_dy = target_begin[1] - target_end[1];
    const T target_length_sq = target_dx * target_dx + target_dy * target_dy;
    
    if (target_length_sq < T(1e-10)) {
      return false;
    }
    
    // 计算起点到目标线段的距离
    T u_begin = ((world_b[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
                 (world_b[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    u_begin = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u_begin));  // ✅ 限制在线段上
    T closest_x_begin = target_begin[0] + u_begin * (target_end[0] - target_begin[0]);
    T closest_y_begin = target_begin[1] + u_begin * (target_end[1] - target_begin[1]);
    T dist_begin = ceres::sqrt((world_b[0] - closest_x_begin) * (world_b[0] - closest_x_begin) +
                               (world_b[1] - closest_y_begin) * (world_b[1] - closest_y_begin));
    
    // 计算终点到目标线段的距离
    T u_end = ((world_e[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
               (world_e[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    u_end = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u_end));  // ✅ 限制在线段上
    T closest_x_end = target_begin[0] + u_end * (target_end[0] - target_begin[0]);
    T closest_y_end = target_begin[1] + u_end * (target_end[1] - target_begin[1]);
    T dist_end = ceres::sqrt((world_e[0] - closest_x_end) * (world_e[0] - closest_x_end) +
                             (world_e[1] - closest_y_end) * (world_e[1] - closest_y_end));
    
    T delta_tra = (dist_begin + dist_end) / T(2.0);

    residual[0] = parallel_scaling_factor_ * delta_tra;
    // residual[1] = parallel_scaling_factor_ * delta_end;
    residual[1] = angle_scaling_factor_ * delta_angle;
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const Line data_;
  const Line target_;
};

class LineDeltaCostFunction2D
{
public:
  LineDeltaCostFunction2D(
    const double parallel_scaling_factor, const double angle_scaling_factor,
    Line & data, Line & target)
  : parallel_scaling_factor_(parallel_scaling_factor), angle_scaling_factor_(angle_scaling_factor),
    data_(data), target_(target)
  {
  }

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);


    Eigen::Matrix<T, 3, 1> point_begin((T(data_.begin[0])), (T(data_.begin[1])), T(1.0));
    Eigen::Matrix<T, 3, 1> point_end((T(data_.end[0])), (T(data_.end[1])), T(1.0));

    const Eigen::Matrix<T, 3, 1> world_b = transform * point_begin;
    const Eigen::Matrix<T, 3, 1> world_e = transform * point_end;

    const Eigen::Matrix<T, 2, 1> target_begin((T(target_.begin[0])),
      (T(target_.begin[1])));
    const Eigen::Matrix<T, 2, 1> target_end((T(target_.end[0])),
      (T(target_.end[1])));

    const T target_dx = target_begin[0] - target_end[0];
    const T target_dy = target_begin[1] - target_end[1];
    const T target_length_sq = target_dx * target_dx + target_dy * target_dy;

    if (target_length_sq < T(1e-10)) {
      return false;
    }

    // 计算起点投影参数
    T u = ((world_b[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
           (world_b[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    u = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u));  // ✅ 限制在 [0, 1]

    // 计算终点投影参数
    T u1 = ((world_e[0] - target_begin[0]) * (target_end[0] - target_begin[0]) +
            (world_e[1] - target_begin[1]) * (target_end[1] - target_begin[1])) / target_length_sq;
    u1 = ceres::fmax(T(0.0), ceres::fmin(T(1.0), u1));  // ✅ 限制在 [0, 1]

    // 计算最近点坐标
    T cross_x_begin = target_begin[0] + u * (target_end[0] - target_begin[0]);
    T cross_y_begin = target_begin[1] + u * (target_end[1] - target_begin[1]);
    T delta_2 = (world_b[0] - cross_x_begin) * (world_b[0] - cross_x_begin) +
                (world_b[1] - cross_y_begin) * (world_b[1] - cross_y_begin);
    T delta_begin = ceres::sqrt(delta_2);

    T cross_x_end = target_begin[0] + u1 * (target_end[0] - target_begin[0]);
    T cross_y_end = target_begin[1] + u1 * (target_end[1] - target_begin[1]);
    T delta_3 = (world_e[0] - cross_x_end) * (world_e[0] - cross_x_end) +
                (world_e[1] - cross_y_end) * (world_e[1] - cross_y_end);
    T delta_end = ceres::sqrt(delta_3);
    residual[0] = parallel_scaling_factor_ * delta_begin;
    residual[1] = parallel_scaling_factor_ * delta_end;
    // std::cout << "residual[0]: " << residual[0] << std::endl;
    // std::cout << "residual[1]: " << residual[1] << std::endl;
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const Line data_;
  const Line target_;
};

ceres::CostFunction * CreateLineDeltaCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line)
{
  Line data{Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}};
  Line target{Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}};
  return new ceres::AutoDiffCostFunction<LineDeltaCostFunction2D, 2 /* residuals */,
           3 /* pose variables */>(
    new LineDeltaCostFunction2D(parallel_scaling_factor, angle_scaling_factor, data, target));
}

ceres::CostFunction * CreateLineDeltaWithAngleCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line,
  const transform::Rigid2d & initial_pose)
{
  Line data{Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}};
  Line target{Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}};
  return new ceres::AutoDiffCostFunction<LineDeltaWithAngleCostAndInitPoseFunction2D,
           3 /* residuals */,
           3 /* pose variables */>(
    new LineDeltaWithAngleCostAndInitPoseFunction2D(
      parallel_scaling_factor,
      angle_scaling_factor, data, target, initial_pose));
}

ceres::CostFunction * CreateLineDeltaWithoutInitPoseCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line)
{
  Line data{Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}};
  Line target{Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}};
  return new ceres::AutoDiffCostFunction<LineDeltaWithAngleCostFunction2D, 2 /* residuals */,
           3 /* pose variables */>(
    new LineDeltaWithAngleCostFunction2D(
      parallel_scaling_factor,
      angle_scaling_factor, data, target));
}

// ============================================================================
// 方案B: 改进版本 - 对称性距离 + 自适应权重 + 中点距离
// ============================================================================

// 辅助函数：点到线段的距离（确保投影点在线段上）
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
// 改进版本1: 对称性距离 + 双端点 + 中点 + 自适应权重
// ============================================================================
class ImprovedLineDeltaCostFunction2D {
public:
  ImprovedLineDeltaCostFunction2D(
      const double parallel_scaling_factor,
      const double angle_scaling_factor,
      const double line_length_weight,
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
    Eigen::Matrix<T, 2, 1> target_mid = (target_begin + target_end) / T(2.0);

    // ========================================================================
    // 1. 对称性距离：观测线到目标线 + 目标线到观测线
    // ========================================================================
    
    // 观测线的点到目标线段的距离
    T dist_obs_begin = PointToLineSegmentDistance(world_begin, target_begin, target_end);
    T dist_obs_end = PointToLineSegmentDistance(world_end, target_begin, target_end);
    T dist_obs_mid = PointToLineSegmentDistance(world_mid, target_begin, target_end);
    
    // 目标线的点到观测线段的距离（反向）
    T dist_target_begin = PointToLineSegmentDistance(target_begin, world_begin, world_end);
    T dist_target_end = PointToLineSegmentDistance(target_end, world_begin, world_end);
    T dist_target_mid = PointToLineSegmentDistance(target_mid, world_begin, world_end);
    
    // 对称性距离：端点权重1.0，中点权重0.5
    T dist_error = (dist_obs_begin + dist_obs_end + dist_obs_mid * T(0.5) +
                    dist_target_begin + dist_target_end + dist_target_mid * T(0.5)) / T(5.0);

    // ========================================================================
    // 2. 角度误差（考虑线的双向性）
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
  const double line_length_weight_;
  const Line data_;
  const Line target_;
};

// ============================================================================
// 改进版本2: 添加初始位姿约束 + 对称性距离
// ============================================================================
class ImprovedLineDeltaWithInitPoseCostFunction2D {
public:
  ImprovedLineDeltaWithInitPoseCostFunction2D(
      const double parallel_scaling_factor,
      const double angle_scaling_factor,
      const double pose_prior_weight,
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
    Eigen::Matrix<T, 2, 1> world_mid = (world_begin + world_end) / T(2.0);

    // 目标线段
    Eigen::Matrix<T, 2, 1> target_begin(T(target_.begin[0]), T(target_.begin[1]));
    Eigen::Matrix<T, 2, 1> target_end(T(target_.end[0]), T(target_.end[1]));
    Eigen::Matrix<T, 2, 1> target_mid = (target_begin + target_end) / T(2.0);

    // ========================================================================
    // 1. 对称性距离
    // ========================================================================
    T dist_obs_begin = PointToLineSegmentDistance(world_begin, target_begin, target_end);
    T dist_obs_end = PointToLineSegmentDistance(world_end, target_begin, target_end);
    T dist_obs_mid = PointToLineSegmentDistance(world_mid, target_begin, target_end);
    
    T dist_target_begin = PointToLineSegmentDistance(target_begin, world_begin, world_end);
    T dist_target_end = PointToLineSegmentDistance(target_end, world_begin, world_end);
    T dist_target_mid = PointToLineSegmentDistance(target_mid, world_begin, world_end);
    
    T dist_error = (dist_obs_begin + dist_obs_end + dist_obs_mid * T(0.5) +
                    dist_target_begin + dist_target_end + dist_target_mid * T(0.5)) / T(5.0);

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
    // 3. 位姿先验约束
    // ========================================================================
    T dx = pose[0] - T(initial_pose_.translation().x());
    T dy = pose[1] - T(initial_pose_.translation().y());
    T dtheta = pose[2] - T(initial_pose_.rotation().angle());
    dtheta = ceres::atan2(ceres::sin(dtheta), ceres::cos(dtheta));
    
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
// 工厂函数 - 方案B
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

// ============================================================================
// 方案C: 基于距离场的快速线特征匹配
// ============================================================================

// LineDistanceField 实现
LineDistanceField::LineDistanceField(
    const std::vector<sensor::LineData>& lines,
    double resolution,
    double max_distance)
    : resolution_(resolution), max_distance_(max_distance) {
  
  if (lines.empty()) {
    LOG(WARNING) << "Creating LineDistanceField with empty line list";
    distance_grid_ = Eigen::MatrixXd::Constant(1, 1, max_distance);
    origin_ = Eigen::Vector2d::Zero();
    return;
  }

  // 1. 计算边界框
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();

  for (const auto& line : lines) {
    const Eigen::Vector2d start = line.begin();
    const Eigen::Vector2d end = line.end();
    
    min_x = std::min({min_x, start.x(), end.x()});
    min_y = std::min({min_y, start.y(), end.y()});
    max_x = std::max({max_x, start.x(), end.x()});
    max_y = std::max({max_y, start.y(), end.y()});
  }

  // 扩展边界（考虑max_distance）
  min_x -= max_distance;
  min_y -= max_distance;
  max_x += max_distance;
  max_y += max_distance;

  origin_ = Eigen::Vector2d(min_x, min_y);

  // 2. 计算栅格尺寸
  int cols = static_cast<int>(std::ceil((max_x - min_x) / resolution_)) + 1;
  int rows = static_cast<int>(std::ceil((max_y - min_y) / resolution_)) + 1;

  // 限制栅格大小（避免内存溢出）
  const int kMaxGridSize = 10000;
  if (cols > kMaxGridSize || rows > kMaxGridSize) {
    LOG(WARNING) << "LineDistanceField grid size too large: " << cols << "x" << rows
                 << ", limiting to " << kMaxGridSize << "x" << kMaxGridSize;
    cols = std::min(cols, kMaxGridSize);
    rows = std::min(rows, kMaxGridSize);
  }

  // 3. 初始化距离场为最大值
  distance_grid_ = Eigen::MatrixXd::Constant(rows, cols, max_distance);

  // 4. 计算每个栅格点到最近线的距离
  LOG(INFO) << "Building LineDistanceField: " << rows << "x" << cols 
            << " grid, " << lines.size() << " lines";

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      // 栅格坐标转世界坐标
      Eigen::Vector2d point(
          origin_.x() + c * resolution_,
          origin_.y() + r * resolution_);

      // 找到最近的线
      double min_dist = max_distance;
      for (const auto& line : lines) {
        double dist = PointToLineSegmentDistance(
            point, line.begin(), line.end());
        min_dist = std::min(min_dist, dist);
      }

      distance_grid_(r, c) = min_dist;
    }
  }

  LOG(INFO) << "LineDistanceField built successfully";
}

// 点到线段的最短距离
double LineDistanceField::PointToLineSegmentDistance(
    const Eigen::Vector2d& point,
    const Eigen::Vector2d& line_start,
    const Eigen::Vector2d& line_end) {
  
  const Eigen::Vector2d line_vec = line_end - line_start;
  const double line_length_sq = line_vec.squaredNorm();

  if (line_length_sq < 1e-8) {
    // 线段退化为点
    return (point - line_start).norm();
  }

  // 投影参数 t
  const double t = (point - line_start).dot(line_vec) / line_length_sq;

  // 限制在线段上
  const double t_clamped = std::max(0.0, std::min(1.0, t));

  // 计算最近点
  const Eigen::Vector2d closest_point = line_start + t_clamped * line_vec;

  return (point - closest_point).norm();
}

// 世界坐标转栅格坐标（模板函数）
template <typename T>
Eigen::Matrix<T, 2, 1> LineDistanceField::WorldToGrid(
    const Eigen::Matrix<T, 2, 1>& point) const {
  return Eigen::Matrix<T, 2, 1>(
      (point.x() - T(origin_.x())) / T(resolution_),
      (point.y() - T(origin_.y())) / T(resolution_));
}

// 辅助函数：从Jet或double类型中提取标量值用于索引计算
template <typename T>
inline double ExtractValue(const T& value) {
  // 对于Jet类型，访问其a成员（标量部分）
  return value.a;
}

template <>
inline double ExtractValue<double>(const double& value) {
  return value;
}

// 查询距离（带双线性插值）
template <typename T>
T LineDistanceField::GetDistance(const Eigen::Matrix<T, 2, 1>& point) const {
  const Eigen::Matrix<T, 2, 1> grid_pos = WorldToGrid(point);
  
  const T x = grid_pos.x();
  const T y = grid_pos.y();

  // 获取四个相邻栅格索引 - 提取标量值用于索引
  const int x0 = static_cast<int>(std::floor(ExtractValue(x)));
  const int y0 = static_cast<int>(std::floor(ExtractValue(y)));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  // 边界检查
  if (x0 < 0 || y0 < 0 || 
      x1 >= distance_grid_.cols() || y1 >= distance_grid_.rows()) {
    return T(max_distance_);
  }

  // 双线性插值权重
  const T wx = x - T(x0);
  const T wy = y - T(y0);

  // 获取四个角的距离值
  const T d00 = T(distance_grid_(y0, x0));
  const T d10 = T(distance_grid_(y0, x1));
  const T d01 = T(distance_grid_(y1, x0));
  const T d11 = T(distance_grid_(y1, x1));

  // 双线性插值
  const T d0 = d00 * (T(1.0) - wx) + d10 * wx;
  const T d1 = d01 * (T(1.0) - wx) + d11 * wx;
  const T distance = d0 * (T(1.0) - wy) + d1 * wy;

  return distance;
}

// 查询距离和梯度
template <typename T>
void LineDistanceField::GetDistanceAndGradient(
    const Eigen::Matrix<T, 2, 1>& point,
    T* distance,
    Eigen::Matrix<T, 2, 1>* gradient) const {
  
  const Eigen::Matrix<T, 2, 1> grid_pos = WorldToGrid(point);
  
  const T x = grid_pos.x();
  const T y = grid_pos.y();

  const int x0 = static_cast<int>(std::floor(ExtractValue(x)));
  const int y0 = static_cast<int>(std::floor(ExtractValue(y)));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;

  // 边界检查
  if (x0 < 0 || y0 < 0 || 
      x1 >= distance_grid_.cols() || y1 >= distance_grid_.rows()) {
    *distance = T(max_distance_);
    *gradient = Eigen::Matrix<T, 2, 1>::Zero();
    return;
  }

  // 双线性插值权重
  const T wx = x - T(x0);
  const T wy = y - T(y0);

  // 四个角的距离值
  const T d00 = T(distance_grid_(y0, x0));
  const T d10 = T(distance_grid_(y0, x1));
  const T d01 = T(distance_grid_(y1, x0));
  const T d11 = T(distance_grid_(y1, x1));

  // 计算距离
  const T d0 = d00 * (T(1.0) - wx) + d10 * wx;
  const T d1 = d01 * (T(1.0) - wx) + d11 * wx;
  *distance = d0 * (T(1.0) - wy) + d1 * wy;

  // 计算梯度（中心差分）
  // ∂d/∂x
  T grad_x;
  if (x0 > 0 && x1 < distance_grid_.cols() - 1) {
    const T d_left = T(distance_grid_(y0, x0 - 1));
    const T d_right = T(distance_grid_(y0, x1 + 1));
    grad_x = (d_right - d_left) / (T(2.0) * T(resolution_));
  } else {
    grad_x = (d10 - d00) / T(resolution_);
  }

  // ∂d/∂y
  T grad_y;
  if (y0 > 0 && y1 < distance_grid_.rows() - 1) {
    const T d_bottom = T(distance_grid_(y0 - 1, x0));
    const T d_top = T(distance_grid_(y1 + 1, x0));
    grad_y = (d_top - d_bottom) / (T(2.0) * T(resolution_));
  } else {
    grad_y = (d01 - d00) / T(resolution_);
  }

  *gradient = Eigen::Matrix<T, 2, 1>(grad_x, grad_y);
}

// 显式实例化模板
template double LineDistanceField::GetDistance<double>(
    const Eigen::Matrix<double, 2, 1>& point) const;
template void LineDistanceField::GetDistanceAndGradient<double>(
    const Eigen::Matrix<double, 2, 1>& point,
    double* distance,
    Eigen::Matrix<double, 2, 1>* gradient) const;

// ============================================================================
// 基于距离场的代价函数
// ============================================================================

class LineDistanceFieldCostFunction2D {
 public:
  LineDistanceFieldCostFunction2D(
      const double weight,
      const sensor::LineData& line_data,
      const LineDistanceField* distance_field)
      : weight_(weight),
        line_start_(line_data.begin()),
        line_end_(line_data.end()),
        distance_field_(distance_field) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    const T cos_theta = ceres::cos(pose[2]);
    const T sin_theta = ceres::sin(pose[2]);

    // 变换线段的两个端点
    Eigen::Matrix<T, 2, 1> start_transformed;
    start_transformed.x() = cos_theta * T(line_start_.x()) - 
                            sin_theta * T(line_start_.y()) + pose[0];
    start_transformed.y() = sin_theta * T(line_start_.x()) + 
                            cos_theta * T(line_start_.y()) + pose[1];

    Eigen::Matrix<T, 2, 1> end_transformed;
    end_transformed.x() = cos_theta * T(line_end_.x()) - 
                          sin_theta * T(line_end_.y()) + pose[0];
    end_transformed.y() = sin_theta * T(line_end_.x()) + 
                          cos_theta * T(line_end_.y()) + pose[1];

    // 计算线段中点
    Eigen::Matrix<T, 2, 1> mid_transformed = 
        (start_transformed + end_transformed) * T(0.5);

    // 查询三个点的距离
    const T dist_start = distance_field_->GetDistance(start_transformed);
    const T dist_end = distance_field_->GetDistance(end_transformed);
    const T dist_mid = distance_field_->GetDistance(mid_transformed);

    // 加权平均（端点权重1.0，中点权重0.5）
    const T total_weight = T(2.5);  // 1.0 + 1.0 + 0.5
    const T avg_distance = (dist_start + dist_end + dist_mid * T(0.5)) / total_weight;

    // 残差 = 加权距离
    residual[0] = T(weight_) * avg_distance;

    return true;
  }

 private:
  const double weight_;
  const Eigen::Vector2d line_start_;
  const Eigen::Vector2d line_end_;
  const LineDistanceField* distance_field_;  // 不拥有所有权
};

// 工厂函数
ceres::CostFunction* CreateLineDistanceFieldCostFunction2D(
    const double weight,
    const sensor::LineData& line_data,
    const LineDistanceField& distance_field) {
  
  return new ceres::AutoDiffCostFunction<
      LineDistanceFieldCostFunction2D,
      1,   // 1个残差
      3>(  // 3个参数 [x, y, theta]
      new LineDistanceFieldCostFunction2D(
          weight,
          line_data,
          &distance_field));
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
