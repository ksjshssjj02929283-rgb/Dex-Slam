/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>
#include <cmath>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/rotation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/translation_delta_cost_functor_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/tsdf_match_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_delta_cost_function_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/point_delta_cost_function_2d.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
  common::LuaParameterDictionary* const parameter_dictionary) {
  proto::CeresScanMatcherOptions2D options;
  options.set_occupied_space_weight(
    parameter_dictionary->GetDouble("occupied_space_weight"));
  options.set_translation_weight(
    parameter_dictionary->GetDouble("translation_weight"));
  options.set_rotation_weight(
    parameter_dictionary->GetDouble("rotation_weight"));
  options.set_line_weight(
    parameter_dictionary->GetDouble("line_weight"));
  options.set_point_weight(
    parameter_dictionary->GetDouble("point_weight"));
  options.set_max_correspondence_distance(
    parameter_dictionary->GetDouble("max_correspondence_distance"));
  options.set_local_weight_factor(
    parameter_dictionary->GetDouble("local_weight_factor"));
  *options.mutable_ceres_solver_options() =
    common::CreateCeresSolverOptionsProto(
      parameter_dictionary->GetDictionary("ceres_solver_options").get());
  return options;
}

CeresScanMatcher2D::CeresScanMatcher2D(
  const proto::CeresScanMatcherOptions2D& options)
  : options_(options),
    ceres_solver_options_(
      common::CreateCeresSolverOptions(options.ceres_solver_options())) {
  ceres_solver_options_.linear_solver_type = ceres::DENSE_QR;
}

CeresScanMatcher2D::~CeresScanMatcher2D() {}

void CeresScanMatcher2D::MatchForLocalization(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const MapData& local_map,
    const MapData& global_map,
    transform::Rigid2d* const pose_estimate,
    Eigen::Matrix3d* const covariance,
    ceres::Solver::Summary* const summary) const
{
  CHECK(pose_estimate != nullptr);
  CHECK(summary != nullptr);
  CHECK(!point_cloud.empty()) << "point_cloud must not be empty";

  double ceres_pose_estimate[3] = {
      initial_pose_estimate.translation().x(),
      initial_pose_estimate.translation().y(),
      initial_pose_estimate.rotation().angle()
  };

  // Two-pass optimization for better convergence
  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem problem;

    // Add local map constraints if available
    if (local_map.grid != nullptr && options_.local_weight_factor() > 1e-3) {
      AddGridConstraints(problem, point_cloud, *local_map.grid,
                        options_.occupied_space_weight() * options_.local_weight_factor(),
                        ceres_pose_estimate);
      
      if (local_map.line_map != nullptr && local_map.tracked_line_data != nullptr) {
        AddLineConstraints(problem, *local_map.tracked_line_data, *local_map.line_map,
                          options_.line_weight() * options_.local_weight_factor(),
                          ceres_pose_estimate);
      }
      
      if (local_map.cloud_ikdtree != nullptr) {
        AddPointToLineConstraints(problem, point_cloud, *local_map.cloud_ikdtree,
                                 options_.point_weight() * options_.local_weight_factor(),
                                 ceres_pose_estimate);
      }
    }

    // Add global map constraints
    if (global_map.grid != nullptr) {
      AddGridConstraints(problem, point_cloud, *global_map.grid,
                        options_.occupied_space_weight(),
                        ceres_pose_estimate);
      
      if (global_map.line_map != nullptr && global_map.tracked_line_data != nullptr) {
        AddLineConstraints(problem, *global_map.tracked_line_data, *global_map.line_map,
                          options_.line_weight(),
                          ceres_pose_estimate);
      }
      
      if (global_map.cloud_ikdtree != nullptr) {
        AddPointToLineConstraints(problem, point_cloud, *global_map.cloud_ikdtree,
                                 options_.point_weight(),
                                 ceres_pose_estimate);
      }
    }

    // Add highlight point constraints
    if (global_map.highlight_cloud_ikdtree != nullptr && !highlight_point_cloud.empty()) {
      AddHighlightPointConstraints(problem, highlight_point_cloud,
                                   *global_map.highlight_cloud_ikdtree,
                                   options_.point_weight() * 3.0,
                                   ceres_pose_estimate);
    }

    // Add regularization constraints
    CHECK_GT(options_.translation_weight(), 0.);
    problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.translation_weight(),
            initial_pose_estimate.translation()),
        nullptr, ceres_pose_estimate);

    CHECK_GT(options_.rotation_weight(), 0.);
    problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.rotation_weight(), ceres_pose_estimate[2]),
        nullptr, ceres_pose_estimate);

    // Solve the optimization problem
    ceres::Solve(ceres_solver_options_, &problem, summary);

    // Check if solution is valid
    if (!summary->IsSolutionUsable() || std::isinf(summary->final_cost) || 
        std::isnan(summary->final_cost)) {
      LOG(WARNING) << "Ceres solve failed: " << summary->BriefReport();
      ceres_pose_estimate[0] = initial_pose_estimate.translation().x();
      ceres_pose_estimate[1] = initial_pose_estimate.translation().y();
      ceres_pose_estimate[2] = initial_pose_estimate.rotation().angle();
      break;
    }
  }

  // Output pose estimate
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, 
      ceres_pose_estimate[2]);

  // Validate pose change
  double delta_dist = (pose_estimate->translation() - 
                       initial_pose_estimate.translation()).norm();
  double delta_yaw = std::abs(pose_estimate->rotation().angle() - 
                              initial_pose_estimate.rotation().angle());

  if (delta_dist > 0.2 || delta_yaw > 0.1) {
    LOG(ERROR) << "Pose change too large (dist: " << delta_dist 
               << ", yaw: " << delta_yaw << "), reverting to initial pose";
    *pose_estimate = initial_pose_estimate;
  }

  // Compute covariance if requested
  if (covariance != nullptr) {
    // Create a new problem for covariance computation
    ceres::Problem cov_problem;
    
    // Re-add all constraints (same as final iteration)
    if (global_map.grid != nullptr) {
      AddGridConstraints(cov_problem, point_cloud, *global_map.grid,
                        options_.occupied_space_weight(),
                        ceres_pose_estimate);
    }
    
    cov_problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.translation_weight(),
            initial_pose_estimate.translation()),
        nullptr, ceres_pose_estimate);
    
    cov_problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.rotation_weight(), ceres_pose_estimate[2]),
        nullptr, ceres_pose_estimate);

    if (!ComputeCovariance(cov_problem, ceres_pose_estimate, covariance)) {
      LOG(WARNING) << "Failed to compute covariance, using default values";
      *covariance = Eigen::Matrix3d::Identity() * 0.01;  // Default uncertainty
    }
  }
}

void CeresScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud,
    const sensor::PointCloud& highlight_point_cloud,
    const MapData& map,
    transform::Rigid2d* const pose_estimate,
    Eigen::Matrix3d* const covariance,
    ceres::Solver::Summary* const summary) const
{
  CHECK(pose_estimate != nullptr);
  CHECK(summary != nullptr);
  CHECK(!point_cloud.empty()) << "point_cloud must not be empty";
  CHECK(map.grid != nullptr) << "Map grid must not be null";

  double ceres_pose_estimate[3] = {
      initial_pose_estimate.translation().x(),
      initial_pose_estimate.translation().y(),
      initial_pose_estimate.rotation().angle()
  };

  // Two-pass optimization
  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem problem;

    // Add grid constraints
    AddGridConstraints(problem, point_cloud, *map.grid,
                      options_.occupied_space_weight(),
                      ceres_pose_estimate);

    // Add line constraints if available
    if (map.line_map != nullptr && map.tracked_line_data != nullptr) {
      AddLineConstraints(problem, *map.tracked_line_data, *map.line_map,
                        options_.line_weight(),
                        ceres_pose_estimate);
    }

    // Add point-to-line constraints if KD-tree available
    if (map.cloud_ikdtree != nullptr) {
      AddPointToLineConstraints(problem, point_cloud, *map.cloud_ikdtree,
                               options_.point_weight(),
                               ceres_pose_estimate);
    }

    // Add highlight point constraints if available
    if (map.highlight_cloud_ikdtree != nullptr && !highlight_point_cloud.empty()) {
      AddHighlightPointConstraints(problem, highlight_point_cloud,
                                   *map.highlight_cloud_ikdtree,
                                   options_.point_weight() * 3.0,
                                   ceres_pose_estimate);
    }

    // Add regularization constraints
    CHECK_GT(options_.translation_weight(), 0.);
    problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.translation_weight(),
            initial_pose_estimate.translation()),
        nullptr, ceres_pose_estimate);

    CHECK_GT(options_.rotation_weight(), 0.);
    problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.rotation_weight(), ceres_pose_estimate[2]),
        nullptr, ceres_pose_estimate);

    // Solve
    ceres::Solve(ceres_solver_options_, &problem, summary);

    // Check solution validity
    if (!summary->IsSolutionUsable() || std::isinf(summary->final_cost) || 
        std::isnan(summary->final_cost)) {
      LOG(WARNING) << "Ceres solve failed: " << summary->BriefReport();
      ceres_pose_estimate[0] = initial_pose_estimate.translation().x();
      ceres_pose_estimate[1] = initial_pose_estimate.translation().y();
      ceres_pose_estimate[2] = initial_pose_estimate.rotation().angle();
      break;
    }
  }

  // Output pose estimate
  *pose_estimate = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, 
      ceres_pose_estimate[2]);

  // Compute covariance if requested
  if (covariance != nullptr) {
    ceres::Problem cov_problem;
    
    AddGridConstraints(cov_problem, point_cloud, *map.grid,
                      options_.occupied_space_weight(),
                      ceres_pose_estimate);
    
    cov_problem.AddResidualBlock(
        TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.translation_weight(),
            initial_pose_estimate.translation()),
        nullptr, ceres_pose_estimate);
    
    cov_problem.AddResidualBlock(
        RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
            options_.rotation_weight(), ceres_pose_estimate[2]),
        nullptr, ceres_pose_estimate);

    if (!ComputeCovariance(cov_problem, ceres_pose_estimate, covariance)) {
      LOG(WARNING) << "Failed to compute covariance, using default values";
      *covariance = Eigen::Matrix3d::Identity() * 0.01;
    }
  }
}

// ========== Helper Methods ==========

void CeresScanMatcher2D::AddGridConstraints(
    ceres::Problem& problem,
    const sensor::PointCloud& point_cloud,
    const Grid2D& grid,
    double weight,
    double* ceres_pose_estimate) const
{
  const double normalized_weight = weight / std::sqrt(static_cast<double>(point_cloud.size()));
  
  switch (grid.GetGridType()) {
    case GridType::PROBABILITY_GRID:
      problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(normalized_weight, point_cloud, grid),
          nullptr, ceres_pose_estimate);
      break;
    case GridType::TSDF:
      problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(normalized_weight, point_cloud,
                                       static_cast<const TSDF2D&>(grid)),
          nullptr, ceres_pose_estimate);
      break;
  }
}

void CeresScanMatcher2D::AddLineConstraints(
    ceres::Problem& problem,
    const std::map<int, std::vector<sensor::LineData>>& tracked_line_data,
    const LineMap2DList& line_map,
    double weight,
    double* ceres_pose_estimate) const
{
  if (tracked_line_data.empty()) {
    return;
  }

  const double line_parallel_weight = weight / 2.0 / static_cast<double>(tracked_line_data.size());
  const double line_angle_weight = weight * 2.0 / static_cast<double>(tracked_line_data.size());

  for (const auto& iter : tracked_line_data) {
    // Skip new line features or features not in map
    if (iter.first < 0 || line_map.GetLineMapData2DList().count(iter.first) == 0) {
      continue;
    }

    const auto it = line_map.GetLineMapData2DList().find(iter.first);
    const sensor::LineData& target_line = it->second.line_data();

    if (iter.second.size() == 1) {
      // Single match - add directly
      problem.AddResidualBlock(
          CreateLineDeltaWithoutInitPoseCostFunction2D(
              line_parallel_weight, line_angle_weight,
              iter.second[0], target_line),
          nullptr, ceres_pose_estimate);
    } else {
      // Multiple matches - find closest and add those
      double min_dis = 10.0;
      for (const auto& line : iter.second) {
        Eigen::Vector3d matching_dis = line.GetMatchingDis();
        if (matching_dis[0] < min_dis) {
          min_dis = matching_dis[0];
        }
      }
      
      for (const auto& line : iter.second) {
        Eigen::Vector3d matching_dis = line.GetMatchingDis();
        if (std::abs(matching_dis[0] - min_dis) < 0.005) {
          problem.AddResidualBlock(
              CreateLineDeltaWithoutInitPoseCostFunction2D(
                  line_parallel_weight, line_angle_weight,
                  line, target_line),
              nullptr, ceres_pose_estimate);
        }
      }
    }
  }
}

void CeresScanMatcher2D::AddPointToLineConstraints(
    ceres::Problem& problem,
    const sensor::PointCloud& point_cloud,
    ikd_tree::iKdTree<pcl::PointXYZ>& cloud_ikdtree,
    double weight,
    const double* ceres_pose_estimate) const
{
  if (cloud_ikdtree.Root_Node == nullptr || cloud_ikdtree.validnum() < 10) {
    return;
  }

  const auto pose_estimate_temp = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
  const auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);

  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
  std::vector<sensor::PointCloud::PointType> p_src_vec;
  std::vector<sensor::PointCloud::PointType> p_a_vec;
  std::vector<sensor::PointCloud::PointType> p_b_vec;

  const size_t step = std::max(size_t(1), point_cloud.size() / 50);
  
  for (size_t i = 0; i < point_cloud.size(); i += step) {
    const auto& p_src = point_cloud[i];
    const Eigen::Vector3d p_transformed =
        pose_estimate_temp_3d.rotation().toRotationMatrix() * 
        p_src.position.cast<double>() + pose_estimate_temp_3d.translation();

    // KD-tree search
    pcl::PointXYZ query_point(p_transformed.x(), p_transformed.y(), p_transformed.z());
    std::vector<float> squared_distances(10, -1);
    ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
    cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

    // Skip if too far
    if (std::sqrt(squared_distances[9]) > 0.15) {
      continue;
    }

    // Compute local covariance
    std::vector<Eigen::Vector3f> nearCorners;
    Eigen::Vector3f center(0, 0, 0);
    for (int j = 0; j < 10; j++) {
      Eigen::Vector3f p_tmp(points_near[j].x, points_near[j].y, points_near[j].z);
      center += p_tmp;
      nearCorners.push_back(p_tmp);
    }
    center /= 10.0;

    Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
    for (int j = 0; j < 10; j++) {
      Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
      covMat += tmpZeroMean * tmpZeroMean.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);
    
    // Check if it's a line feature (largest eigenvalue >> smallest)
    if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0]) {
      Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
      unit_direction.head<2>() = saes.eigenvectors().col(1);
      
      sensor::PointCloud::PointType point_a, point_b;
      point_a.position = 0.1 * unit_direction + center;
      point_b.position = -0.1 * unit_direction + center;

      p_src_vec.push_back(p_src);
      p_a_vec.push_back(point_a);
      p_b_vec.push_back(point_b);
    }
  }

  // Add residuals
  if (!p_src_vec.empty()) {
    const double normalized_weight = weight / static_cast<double>(p_src_vec.size());
    for (size_t i = 0; i < p_src_vec.size(); i++) {
      problem.AddResidualBlock(
          CreatePointLineCostFunction2D(normalized_weight, 
                                       p_src_vec[i], p_a_vec[i], p_b_vec[i]),
          loss_function, const_cast<double*>(ceres_pose_estimate));
    }
  }
}

void CeresScanMatcher2D::AddHighlightPointConstraints(
    ceres::Problem& problem,
    const sensor::PointCloud& highlight_point_cloud,
    ikd_tree::iKdTree<pcl::PointXYZ>& highlight_cloud_ikdtree,
    double weight,
    const double* ceres_pose_estimate) const
{
  if (highlight_cloud_ikdtree.Root_Node == nullptr || 
      highlight_cloud_ikdtree.validnum() < 10) {
    return;
  }

  const auto pose_estimate_temp = transform::Rigid2d(
      {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
  const auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);

  ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);
  std::vector<sensor::PointCloud::PointType> src_points;
  std::vector<sensor::PointCloud::PointType> nearest_points;

  const size_t step = highlight_point_cloud.size() > 200 ? 2 : 1;

  for (size_t i = 0; i < highlight_point_cloud.size(); i += step) {
    const auto& p_src = highlight_point_cloud[i];
    const Eigen::Vector3d p_transformed =
        pose_estimate_temp_3d.rotation().toRotationMatrix() * 
        p_src.position.cast<double>() + pose_estimate_temp_3d.translation();

    // KD-tree search
    pcl::PointXYZ query_point(p_transformed.x(), p_transformed.y(), p_transformed.z());
    std::vector<float> squared_distances(1, -1);
    ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
    highlight_cloud_ikdtree.Nearest_Search(query_point, 1, points_near, squared_distances);

    // Skip if too far
    if (std::sqrt(squared_distances[0]) > options_.max_correspondence_distance()) {
      continue;
    }

    src_points.push_back(p_src);
    
    sensor::PointCloud::PointType p_tmp;
    p_tmp.position.x() = points_near[0].x;
    p_tmp.position.y() = points_near[0].y;
    p_tmp.position.z() = points_near[0].z;
    nearest_points.push_back(p_tmp);
  }

  // Add residuals
  if (!src_points.empty()) {
    const double normalized_weight = weight / static_cast<double>(src_points.size());
    for (size_t i = 0; i < src_points.size(); i++) {
      problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(normalized_weight, 
                                        src_points[i], nearest_points[i]),
          loss_function, const_cast<double*>(ceres_pose_estimate));
    }
  }
}

bool CeresScanMatcher2D::ComputeCovariance(
    const ceres::Problem& problem,
    const double* ceres_pose_estimate,
    Eigen::Matrix3d* covariance) const
{
  CHECK(covariance != nullptr);
  
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_pose = 
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
  
  ceres::Covariance::Options cov_options;
  cov_options.algorithm_type = ceres::DENSE_SVD;
  cov_options.null_space_rank = -1;  // Automatic determination
  
  ceres::Covariance covariance_estimator(cov_options);
  
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.emplace_back(
      std::make_pair(ceres_pose_estimate, ceres_pose_estimate));
  
  // Need to const_cast because Ceres API expects non-const pointer
  if (!covariance_estimator.Compute(covariance_blocks, const_cast<ceres::Problem*>(&problem))) {
    LOG(WARNING) << "Covariance computation failed";
    return false;
  }
  
  if (!covariance_estimator.GetCovarianceBlockInTangentSpace(
          ceres_pose_estimate, ceres_pose_estimate, cov_pose.data())) {
    LOG(WARNING) << "Failed to get covariance block in tangent space";
    return false;
  }
  
  *covariance = cov_pose;
  
  // Validate covariance (should be positive semi-definite)
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(*covariance);
  if (es.eigenvalues().minCoeff() < -1e-6) {
    LOG(WARNING) << "Covariance matrix is not positive semi-definite, "
                 << "min eigenvalue: " << es.eigenvalues().minCoeff();
    return false;
  }
  
  return true;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer