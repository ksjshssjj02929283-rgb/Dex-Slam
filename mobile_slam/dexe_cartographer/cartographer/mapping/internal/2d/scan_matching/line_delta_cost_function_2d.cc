// Copyright (c) 2026 Dexforce Inc
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

#include <algorithm>
#include <cmath>

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{

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

    const T begin_dis = (world_b.head(2) - translation).norm();
    const T end_dis = (world_e.head(2) - translation).norm();
    const T target_dx = target_begin[0] - target_end[0];
    const T target_dy = target_begin[1] - target_end[1];
    T delta_tra = T(0.);
    if (begin_dis < end_dis) {
      T u = (world_b[0] - target_begin[0]) * (target_begin[0] - target_end[0]) +
        (world_b[1] - target_begin[1]) * (target_begin[1] - target_end[1]);
      u = u / ((target_dx * target_dx) + (target_dy * target_dy));
      T cross_x_begin = target_begin[0] + u * target_dx;
      T cross_y_begin = target_begin[1] + u * target_dy;
      T delta_2 = (world_b[0] - cross_x_begin) * (world_b[0] - cross_x_begin) +
        (world_b[1] - cross_y_begin) * (world_b[1] - cross_y_begin);
      delta_tra = ceres::sqrt(delta_2);  // begin dis
    } else {
      T u1 = (world_e[0] - target_begin[0]) * (target_begin[0] - target_end[0]) +
        (world_e[1] - target_begin[1]) * (target_begin[1] - target_end[1]);
      u1 = u1 / ((target_dx * target_dx) + (target_dy * target_dy));
      T cross_x_end = target_begin[0] + u1 * target_dx;
      T cross_y_end = target_begin[1] + u1 * target_dy;
      T delta_3 = (world_e[0] - cross_x_end) * (world_e[0] - cross_x_end) +
        (world_e[1] - cross_y_end) * (world_e[1] - cross_y_end);
      delta_tra = ceres::sqrt(delta_3);  // end dis
    }

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

namespace {

constexpr double kMinLengthConfidence = 0.5;
constexpr double kMaxLengthConfidence = 2.0;
constexpr double kMinValidLineLength = 1e-6;

// 把 Huber 直接作用在具有物理单位的单项原始残差上，再乘外部权重。
// 这样阈值不会随 line_weight、匹配线数量或线长置信度漂移。
template<typename T>
T ApplyHuberResidual(const T & residual, const double delta)
{
  if (delta <= 0.) {
    return residual;
  }
  const T absolute_residual = ceres::abs(residual);
  if (absolute_residual <= T(delta)) {
    return residual;
  }
  const T robust_magnitude =
    ceres::sqrt(T(2. * delta) * absolute_residual - T(delta * delta));
  return residual < T(0.) ? -robust_magnitude : robust_magnitude;
}

double ComputeLineLengthConfidence(
  const Line & data, const Line & target, const double reference_length)
{
  const double data_length = (data.end - data.begin).norm();
  const double target_length = (target.end - target.begin).norm();
  const double effective_length = std::min(data_length, target_length);
  if (effective_length < kMinValidLineLength) {
    return 0.;
  }
  const double normalized_weight = std::sqrt(effective_length / reference_length);
  return std::clamp(
    normalized_weight, kMinLengthConfidence, kMaxLengthConfidence);
}

}  // namespace

class ImprovedLineDeltaCostFunction2D
{
public:
  ImprovedLineDeltaCostFunction2D(
    const double parallel_scaling_factor, const double angle_scaling_factor,
    const double distance_huber_scale, const double angle_huber_scale,
    const double line_length_confidence, const double overlap_gap_weight,
    const Line & data, const Line & target)
  : parallel_scaling_factor_(parallel_scaling_factor),
    angle_scaling_factor_(angle_scaling_factor),
    distance_huber_scale_(distance_huber_scale),
    angle_huber_scale_(angle_huber_scale),
    line_length_confidence_(line_length_confidence),
    overlap_gap_weight_(overlap_gap_weight),
    data_(data),
    target_(target),
    data_length_((data.end - data.begin).norm()),
    target_length_((target.end - target.begin).norm())
  {
    if (target_length_ >= kMinValidLineLength) {
      target_direction_ = (target_.end - target_.begin) / target_length_;
    } else {
      target_direction_ = Eigen::Vector2d::UnitX();
    }
  }

  template<typename T>
  bool operator()(const T * const pose, T * residual) const
  {
    if (data_length_ < kMinValidLineLength ||
      target_length_ < kMinValidLineLength ||
      line_length_confidence_ <= 0.)
    {
      residual[0] = T(0.);
      residual[1] = T(0.);
      residual[2] = T(0.);
      return true;
    }

    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();

    Eigen::Matrix<T, 2, 1> data_begin(T(data_.begin[0]), T(data_.begin[1]));
    Eigen::Matrix<T, 2, 1> data_end(T(data_.end[0]), T(data_.end[1]));
    Eigen::Matrix<T, 2, 1> world_begin = rotation_matrix * data_begin + translation;
    Eigen::Matrix<T, 2, 1> world_end = rotation_matrix * data_end + translation;
    Eigen::Matrix<T, 2, 1> world_mid = (world_begin + world_end) / T(2.0);

    Eigen::Matrix<T, 2, 1> target_begin(T(target_.begin[0]), T(target_.begin[1]));
    Eigen::Matrix<T, 2, 1> target_direction;
    target_direction << T(target_direction_[0]), T(target_direction_[1]);
    Eigen::Matrix<T, 2, 1> target_normal(
      -target_direction[1], target_direction[0]);

    // 1. 法向残差：只使用线中点到目标直线的有符号距离。
    // 墙面端点常受视野和遮挡影响，不把端点差直接当作切向位移观测。
    const T normal_error = (world_mid - target_begin).dot(target_normal);

    // 2. 切向间隙：部分重叠时为 0，仅两段完全不重叠时惩罚间隙。
    const T projected_begin = (world_begin - target_begin).dot(target_direction);
    const T projected_end = (world_end - target_begin).dot(target_direction);
    const T observed_min = ceres::fmin(projected_begin, projected_end);
    const T observed_max = ceres::fmax(projected_begin, projected_end);
    const T gap_before = ceres::fmax(T(0.), -observed_max);
    const T gap_after =
      ceres::fmax(T(0.), observed_min - T(target_length_));
    const T overlap_gap = ceres::fmax(gap_before, gap_after);

    // 3. 无向角度残差：0.5*wrap(2*delta)，保留符号且天然满足 theta~theta+pi。
    Eigen::Matrix<T, 2, 1> world_vec = world_end - world_begin;
    T world_angle = ceres::atan2(world_vec[1], world_vec[0]);
    const T target_angle =
      ceres::atan2(target_direction[1], target_direction[0]);
    const T doubled_angle_error = T(2.) * (world_angle - target_angle);
    const T angle_error = T(0.5) * ceres::atan2(
      ceres::sin(doubled_angle_error), ceres::cos(doubled_angle_error));

    const T parallel_scale =
      T(parallel_scaling_factor_ * line_length_confidence_);
    const T angle_scale =
      T(angle_scaling_factor_ * line_length_confidence_);
    residual[0] =
      parallel_scale * ApplyHuberResidual(normal_error, distance_huber_scale_);
    residual[1] = parallel_scale * T(overlap_gap_weight_) *
      ApplyHuberResidual(overlap_gap, distance_huber_scale_);
    residual[2] =
      angle_scale * ApplyHuberResidual(angle_error, angle_huber_scale_);
    return true;
  }

private:
  const double parallel_scaling_factor_;
  const double angle_scaling_factor_;
  const double distance_huber_scale_;
  const double angle_huber_scale_;
  const double line_length_confidence_;
  const double overlap_gap_weight_;
  const Line data_;
  const Line target_;
  const double data_length_;
  const double target_length_;
  Eigen::Vector2d target_direction_;
};

ceres::CostFunction * CreateImprovedLineDeltaCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const double distance_huber_scale, const double angle_huber_scale,
  const double line_length_reference, const double overlap_gap_weight,
  const sensor::LineData & line_data, const sensor::LineData & target_line)
{
  Line data{Eigen::Vector2d{line_data.begin()[0], line_data.begin()[1]},
    Eigen::Vector2d{line_data.end()[0], line_data.end()[1]}};
  Line target{Eigen::Vector2d{target_line.begin()[0], target_line.begin()[1]},
    Eigen::Vector2d{target_line.end()[0], target_line.end()[1]}};
  const double line_length_confidence =
    ComputeLineLengthConfidence(data, target, line_length_reference);
  return new ceres::AutoDiffCostFunction<ImprovedLineDeltaCostFunction2D,
           3 /* residuals */, 3 /* pose variables */>(
    new ImprovedLineDeltaCostFunction2D(
      parallel_scaling_factor, angle_scaling_factor,
      distance_huber_scale, angle_huber_scale,
      line_length_confidence, overlap_gap_weight, data, target));
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
