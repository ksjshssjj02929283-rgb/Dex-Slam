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
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
