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

#include "cartographer/mapping/internal/2d/scan_matching/point_delta_cost_function_2d.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
namespace {

class PointDeltaCostFunction2D
{
public:
  PointDeltaCostFunction2D(const double scaling_factor,
                           const sensor::PointCloud::PointType& point_src,
                           const sensor::PointCloud::PointType& point_target)
      : scaling_factor(scaling_factor),
        point_src_(point_src),
        point_target_(point_target) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    // Note that this is a 2D point. The third component is a scaling factor.
    const Eigen::Matrix<T, 3, 1> point((T(point_src_.position.x())),
                                       (T(point_src_.position.y())),
                                        T(1.));
    const Eigen::Matrix<T, 3, 1> world = transform * point;

    const Eigen::Matrix<T, 3, 1> point_target_normalized((T(point_target_.position.x())),
                                                       (T(point_target_.position.y())),
                                                        T(1.));
    *residual = (world - point_target_normalized).norm();

    *residual = scaling_factor * *residual;
  
    return true;
  }


private:
  PointDeltaCostFunction2D(const PointDeltaCostFunction2D&) = delete;
  PointDeltaCostFunction2D& operator=(const PointDeltaCostFunction2D&) =
      delete;

  const double scaling_factor;
  const sensor::PointCloud::PointType point_src_;
  const sensor::PointCloud::PointType point_target_;
};

} // namespace

ceres::CostFunction* CreatePointDeltaCostFunction2D(
    const double scaling_factor,
    const sensor::PointCloud::PointType& point_src,
    const sensor::PointCloud::PointType& point_target) {
  return new ceres::AutoDiffCostFunction<PointDeltaCostFunction2D,
                                         1 /* residuals */,
                                         3 /* pose variables */>(
      new PointDeltaCostFunction2D(scaling_factor, point_src, point_target));
}



class PointLineCostFunction2D
{
public:
  PointLineCostFunction2D(const double scaling_factor,
                           const sensor::PointCloud::PointType& point_src,
                           const sensor::PointCloud::PointType& point_a,
                           const sensor::PointCloud::PointType& point_b)
      : scaling_factor(scaling_factor),
        point_src_(point_src),
        point_a_(point_a),
        point_b_(point_b) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    // Note that this is a 2D point. The third component is a scaling factor.
    const Eigen::Matrix<T, 3, 1> cp((T(point_src_.position.x())),
                                    (T(point_src_.position.y())),
                                     T(1.));

		Eigen::Matrix<T, 3, 1> lpa(T(point_a_.position.x()), T(point_a_.position.y()), T(1.));
		Eigen::Matrix<T, 3, 1> lpb(T(point_b_.position.x()), T(point_b_.position.y()), T(1.));

    const Eigen::Matrix<T, 3, 1> lp = transform * cp;
		Eigen::Matrix<T, 3, 1> nu = (lp - lpa).cross(lp - lpb);
		Eigen::Matrix<T, 3, 1> de = lpa - lpb;

		residual[0] = nu.x() / de.norm();
		residual[1] = nu.y() / de.norm();
  
    return true;
  }


private:
  PointLineCostFunction2D(const PointLineCostFunction2D&) = delete;
  PointLineCostFunction2D& operator=(const PointLineCostFunction2D&) =
      delete;

  const double scaling_factor;
  const sensor::PointCloud::PointType point_src_;
  const sensor::PointCloud::PointType point_a_;
  const sensor::PointCloud::PointType point_b_;
};


ceres::CostFunction* CreatePointLineCostFunction2D(
    const double scaling_factor,
    const sensor::PointCloud::PointType& point_src,
    const sensor::PointCloud::PointType& point_a,
    const sensor::PointCloud::PointType& point_b) {
  return new ceres::AutoDiffCostFunction<PointLineCostFunction2D,
                                         2 /* residuals */,
                                         3 /* pose variables */>(
      new PointLineCostFunction2D(scaling_factor, point_src, point_a, point_b));
}


}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
