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

#include "scan_match/ceres_scan_matcher_2d.h"

#include <utility>
#include <vector>

#include "Eigen/Core"
#include "scan_match/ceres_solver_options.h"
#include "common/lua_parameter_dictionary.h"
#include "scan_match/grid_2d.h"
#include "scan_match/occupied_space_cost_function_2d.h"
#include "scan_match/rotation_delta_cost_functor_2d.h"
#include "scan_match/translation_delta_cost_functor_2d.h"
#include "scan_match/tsdf_match_cost_function_2d.h"
#include "scan_match/point_delta_cost_function_2d.h"
#include "transform/transform.h"
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
  options.set_point_weight(
    parameter_dictionary->GetDouble("point_weight"));
  options.set_max_correspondence_distance(
    parameter_dictionary->GetDouble("max_correspondence_distance"));
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

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const Grid2D & grid,
  const pcl::KdTreeFLANN<pcl::PointXYZ> & cloud_kdtree,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary,
  Eigen::Matrix3d & cov,
  const bool & compute_cov) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.point_weight(), 0.);

  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem problem;
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
        break;
      case GridType::TSDF:
        problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, static_cast<const TSDF2D &>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
        break;
    }
    CHECK_GT(options_.translation_weight(), 0.);

    problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.translation_weight(),
        initial_pose_estimate.translation()),
      nullptr /* loss function */, ceres_pose_estimate);

    CHECK_GT(options_.rotation_weight(), 0.);
    problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

    if (cloud_kdtree.getInputCloud()->empty()) {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<std::pair<int,int>> index_buff;
      size_t step = 1;
      if (point_cloud.size() > 200) {
        step = 2;
      }
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        // std::cout << p_src.position.transpose() << std::endl;
        // std::cout << pose_estimate_temp_3d.rotation().toRotationMatrix().matrix() << std::endl;
        // std::cout << pose_estimate_temp_3d.translation().transpose() << std::endl;
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        cloud_kdtree.nearestKSearch(query_point, 1, indices, squared_distances);
        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }
        // std::cout << "p_transformed: " << p_transformed.transpose() << std::endl;
        // std::cout << "distance: " << distance << std::endl;
        // std::cout << "nearest point: " << cloud_kdtree.getInputCloud()->points[indices[0]] << std::endl;
        index_buff.push_back(std::make_pair(i, indices[0]));
      }
      for (size_t i = 0; i < index_buff.size(); i++) {
        const auto& p_src = point_cloud[index_buff[i].first];
        const int& indices = index_buff[i].second;
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = cloud_kdtree.getInputCloud()->points[indices].x;
        p_dist.position.y() = cloud_kdtree.getInputCloud()->points[indices].y;
        p_dist.position.z() = cloud_kdtree.getInputCloud()->points[indices].z;
        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() / 
            static_cast<double>(index_buff.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }
    // std::cout << "point_count: " << index_buff.size() << std::endl;
    // auto temp_pose = transform::Rigid2d({ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
    // std::cout << "iter: "<< opti_counter<< ", pose: " << temp_pose << std::endl;
    ceres::Solve(ceres_solver_options_, &problem, summary);

    if (compute_cov == true) {
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_pose
        = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
      ceres::Covariance::Options cov_options;
      ceres::Covariance covariance(cov_options);
      std::vector<std::pair<const double*, const double*>> covariance_blocks;
      covariance_blocks.emplace_back(
                          std::make_pair(ceres_pose_estimate,
                                        ceres_pose_estimate));
      CHECK(covariance.Compute(covariance_blocks, &problem));
      covariance.GetCovarianceBlockInTangentSpace(
                                          ceres_pose_estimate,
                                          ceres_pose_estimate,
                                          cov_pose.data());
      cov = cov_pose;
    }
  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const sensor::PointCloud & highlight_point_cloud,
  const Grid2D & grid,
  const pcl::KdTreeFLANN<pcl::PointXYZ> & cloud_kdtree,
  const pcl::KdTreeFLANN<pcl::PointXYZ> & highlight_cloud_kdtree,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary,
  Eigen::Matrix3d & cov,
  const bool & compute_cov) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.point_weight(), 0.);

  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem problem;
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, grid),
          nullptr /* loss function */, ceres_pose_estimate);
        break;
      case GridType::TSDF:
        problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, static_cast<const TSDF2D &>(grid)),
          nullptr /* loss function */, ceres_pose_estimate);
        break;
    }
    CHECK_GT(options_.translation_weight(), 0.);

    problem.AddResidualBlock(
      TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.translation_weight(),
        initial_pose_estimate.translation()),
      nullptr /* loss function */, ceres_pose_estimate);

    CHECK_GT(options_.rotation_weight(), 0.);
    problem.AddResidualBlock(
      RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
        options_.rotation_weight(), ceres_pose_estimate[2]),
      nullptr /* loss function */, ceres_pose_estimate);

    if (cloud_kdtree.getInputCloud()->empty()) {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<std::pair<int,int>> index_buff;
      size_t step = 1;
      if (point_cloud.size() > 200) {
        step = 2;
      }
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        // std::cout << p_src.position.transpose() << std::endl;
        // std::cout << pose_estimate_temp_3d.rotation().toRotationMatrix().matrix() << std::endl;
        // std::cout << pose_estimate_temp_3d.translation().transpose() << std::endl;
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        cloud_kdtree.nearestKSearch(query_point, 1, indices, squared_distances);
        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }
        // std::cout << "p_transformed: " << p_transformed.transpose() << std::endl;
        // std::cout << "distance: " << distance << std::endl;
        // std::cout << "nearest point: " << cloud_kdtree.getInputCloud()->points[indices[0]] << std::endl;
        index_buff.push_back(std::make_pair(i, indices[0]));
      }
      for (size_t i = 0; i < index_buff.size(); i++) {
        const auto& p_src = point_cloud[index_buff[i].first];
        const int& indices = index_buff[i].second;
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = cloud_kdtree.getInputCloud()->points[indices].x;
        p_dist.position.y() = cloud_kdtree.getInputCloud()->points[indices].y;
        p_dist.position.z() = cloud_kdtree.getInputCloud()->points[indices].z;
        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() / 
            static_cast<double>(index_buff.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    if (highlight_cloud_kdtree.getInputCloud() == nullptr ||
        highlight_cloud_kdtree.getInputCloud()->empty())
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<std::pair<int,int>> index_buff;
      size_t step = 1;
      if (highlight_point_cloud.size() > 200) {
        step = 2;
      }

      for (size_t i = 0; i < highlight_point_cloud.size(); i += step) {
        const auto& p_src = highlight_point_cloud[i];
        // // std::cout << p_src.position.transpose() << std::endl;
        // // std::cout << pose_estimate_temp_3d.rotation().toRotationMatrix().matrix() << std::endl;
        // // std::cout << pose_estimate_temp_3d.translation().transpose() << std::endl;
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        highlight_cloud_kdtree.nearestKSearch(query_point, 1, indices, squared_distances);
        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }
        // // std::cout << "p_transformed: " << p_transformed.transpose() << std::endl;
        // // std::cout << "distance: " << distance << std::endl;
        // // std::cout << "nearest point: " << cloud_kdtree.getInputCloud()->points[indices[0]] << std::endl;
        index_buff.push_back(std::make_pair(i, indices[0]));
      }

      for (size_t i = 0; i < index_buff.size(); i++) {
        const auto& p_src = highlight_point_cloud[index_buff[i].first];
        const int& indices = index_buff[i].second;
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = highlight_cloud_kdtree.getInputCloud()->points[indices].x;
        p_dist.position.y() = highlight_cloud_kdtree.getInputCloud()->points[indices].y;
        p_dist.position.z() = highlight_cloud_kdtree.getInputCloud()->points[indices].z;
        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() * 3 / 
            static_cast<double>(index_buff.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    ceres::Solve(ceres_solver_options_, &problem, summary);
    if (compute_cov == true) {
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_pose
        = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
      ceres::Covariance::Options cov_options;
      ceres::Covariance covariance(cov_options);
      std::vector<std::pair<const double*, const double*>> covariance_blocks;
      covariance_blocks.emplace_back(
                          std::make_pair(ceres_pose_estimate,
                                        ceres_pose_estimate));
      CHECK(covariance.Compute(covariance_blocks, &problem));
      covariance.GetCovarianceBlockInTangentSpace(
                                          ceres_pose_estimate,
                                          ceres_pose_estimate,
                                          cov_pose.data());
      cov = cov_pose;
    }

  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                 const transform::Rigid2d& initial_pose_estimate,
                 const sensor::PointCloud& point_cloud,
                 const Grid2D& grid,
                 transform::Rigid2d* const pose_estimate,
                 ceres::Solver::Summary* const summary,
                 Eigen::Matrix3d & cov,
                 const bool & compute_cov) const {
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                   initial_pose_estimate.translation().y(),
                   initial_pose_estimate.rotation().angle()};
  ceres::Problem problem;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  switch (grid.GetGridType()) {
  case GridType::PROBABILITY_GRID:
    problem.AddResidualBlock(
      CreateOccupiedSpaceCostFunction2D(
        options_.occupied_space_weight() /
          std::sqrt(static_cast<double>(point_cloud.size())),
        point_cloud, grid),
      nullptr /* loss function */, ceres_pose_estimate);
    break;
  case GridType::TSDF:
    problem.AddResidualBlock(
      CreateTSDFMatchCostFunction2D(
        options_.occupied_space_weight() /
          std::sqrt(static_cast<double>(point_cloud.size())),
        point_cloud, static_cast<const TSDF2D&>(grid)),
      nullptr /* loss function */, ceres_pose_estimate);
    break;
  }
  CHECK_GT(options_.translation_weight(), 0.);
  problem.AddResidualBlock(
    TranslationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
      options_.translation_weight(), target_translation),
    nullptr /* loss function */, ceres_pose_estimate);
  CHECK_GT(options_.rotation_weight(), 0.);
  problem.AddResidualBlock(
    RotationDeltaCostFunctor2D::CreateAutoDiffCostFunction(
      options_.rotation_weight(), ceres_pose_estimate[2]),
    nullptr /* loss function */, ceres_pose_estimate);

  ceres::Solve(ceres_solver_options_, &problem, summary);

  if (compute_cov == true) {
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_pose
      = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    std::vector<std::pair<const double*, const double*>> covariance_blocks;
    covariance_blocks.emplace_back(
                        std::make_pair(ceres_pose_estimate,
                                       ceres_pose_estimate));
    CHECK(covariance.Compute(covariance_blocks, &problem));
    covariance.GetCovarianceBlockInTangentSpace(
                                        ceres_pose_estimate,
                                        ceres_pose_estimate,
                                        cov_pose.data());
    cov = cov_pose;
  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(
              const Eigen::Vector2d & target_translation,
              const transform::Rigid2d & initial_pose_estimate,
              const sensor::PointCloud & point_cloud,
              const Grid2D & grid,
              transform::Rigid2d * const pose_estimate,
              ceres::Solver::Summary * const summary) const {
  Eigen::Matrix3d cov;
  this->Match(
    target_translation, initial_pose_estimate, point_cloud, grid,
    pose_estimate, summary, cov, true);
  std::cout << "covariance: " << std::endl << cov.matrix() << std::endl;
}


}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
