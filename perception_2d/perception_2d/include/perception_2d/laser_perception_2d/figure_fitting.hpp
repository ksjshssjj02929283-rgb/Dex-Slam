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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__FIGURE_FITTING_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__FIGURE_FITTING_HPP_
#include <limits>
#include <algorithm>
#include <vector>
#include "perception_2d/laser_perception_2d/point.hpp"
#include "perception_2d/laser_perception_2d/segment.hpp"
#include "Eigen/Core"
#include "Eigen/SVD"

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(
  const _Matrix_Type_ & a,
  double epsilon = std::numeric_limits<double>::epsilon())
{
  Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs()(0);
  return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(
    svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

inline Segment fitSegment(const PointSet & point_set)
{
  static int num_calls = 0;
  num_calls++;
  int N = point_set.num_points;
  assert(N >= 2);
  Eigen::MatrixXd input = Eigen::MatrixXd::Zero(N, 2);   // [x_i, y_i]
  Eigen::VectorXd output = Eigen::VectorXd::Ones(N);      // [-C]
  Eigen::VectorXd params = Eigen::VectorXd::Zero(2);     // [A ; B]
  PointIterator point = point_set.begin;
  for (int i = 0; i < N; ++i) {
    input(i, 0) = point->x;
    input(i, 1) = point->y;
    std::advance(point, 1);
  }
  // Find A and B coefficients from linear regression (assuming C = -1.0)
  params = pseudoInverse(input) * output;
  double A, B, C;
  A = params(0);
  B = params(1);
  C = -1.0;
  // Find end points
  Point p1 = *point_set.begin;
  Point p2 = *point_set.end;

  Segment segment(p1, p2);
  segment.point_sets.push_back(point_set);

  double D = (A * A + B * B);

  // Project end points on the line
  if (D > 0.0) {
    Point projected_p1, projected_p2;

    projected_p1.x = ( B * B * p1.x - A * B * p1.y - A * C) / D;
    projected_p1.y = (-A * B * p1.x + A * A * p1.y - B * C) / D;

    projected_p2.x = ( B * B * p2.x - A * B * p2.y - A * C) / D;
    projected_p2.y = (-A * B * p2.x + A * A * p2.y - B * C) / D;

    segment.first_point = projected_p1;
    segment.last_point = projected_p2;
  }
  return segment;
}

inline Segment fitSegment(const std::vector<PointSet> & point_sets)
{
  static int num_calls = 0;
  num_calls++;

  int N = 0;
  for (PointSet ps : point_sets) {
    N += ps.num_points;
  }
  assert(N >= 2);

  Eigen::MatrixXd input = Eigen::MatrixXd::Zero(N, 2);   // [x_i, y_i]
  Eigen::VectorXd output = Eigen::VectorXd::Ones(N);      // [-C]
  Eigen::VectorXd params = Eigen::VectorXd::Zero(2);     // [A ; B]

  int n = 0;
  for (PointSet ps : point_sets) {
    PointIterator point = ps.begin;
    for (int i = 0; i < ps.num_points; ++i) {
      input(i + n, 0) = point->x;
      input(i + n, 1) = point->y;
      std::advance(point, 1);
    }
    n += ps.num_points;
  }

  // Find A and B coefficients from linear regression (assuming C = -1.0)
  params = pseudoInverse(input) * output;

  double A, B, C;
  A = params(0);
  B = params(1);
  C = -1.0;

  // Find end points
  Point p1 = *point_sets.front().begin;
  Point p2 = *point_sets.back().end;

  Segment segment(p1, p2);
  segment.point_sets = point_sets;

  double D = (A * A + B * B);

  // Project end points on the line
  if (D > 0.0) {
    Point projected_p1, projected_p2;

    projected_p1.x = ( B * B * p1.x - A * B * p1.y - A * C) / D;
    projected_p1.y = (-A * B * p1.x + A * A * p1.y - B * C) / D;

    projected_p2.x = ( B * B * p2.x - A * B * p2.y - A * C) / D;
    projected_p2.y = (-A * B * p2.x + A * A * p2.y - B * C) / D;

    segment.first_point = projected_p1;
    segment.last_point = projected_p2;
  }
  return segment;
}

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__FIGURE_FITTING_HPP_
