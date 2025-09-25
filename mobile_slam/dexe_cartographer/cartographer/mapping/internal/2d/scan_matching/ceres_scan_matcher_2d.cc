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

// for localization
void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const sensor::PointCloud & highlight_point_cloud,
  const std::map<int, std::vector<sensor::LineData>> & local_tracked_line_data,
  const std::map<int, std::vector<sensor::LineData>> & global_tracked_line_data,
  const Grid2D & local_grid, const Grid2D & global_grid,
  const LineMap2DList & local_line_map, const LineMap2DList & global_line_map,
  ikd_tree::iKdTree<pcl::PointXYZ> & local_cloud_ikdtree,
  ikd_tree::iKdTree<pcl::PointXYZ> & global_cloud_ikdtree,
  ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
  transform::Rigid2d * const pose_estimate,
  Eigen::Matrix3d& cov,
  ceres::Solver::Summary * const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
    initial_pose_estimate.translation().y(),
    initial_pose_estimate.rotation().angle()};
  
  // std::cout << "initial_pose_estimate: " << initial_pose_estimate << std::endl;
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.line_weight(), 0.);
  CHECK_GT(options_.point_weight(), 0.);

  for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
    ceres::Problem problem;
    if (options_.local_weight_factor() > 1e-3) {
      // LOG(WARNING) << "Using local grid ";
      switch (local_grid.GetGridType()) {
        case GridType::PROBABILITY_GRID:
          problem.AddResidualBlock(
            CreateOccupiedSpaceCostFunction2D(
              options_.occupied_space_weight() * options_.local_weight_factor() /
              std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, local_grid),
            nullptr /* loss function */, ceres_pose_estimate);
          break;
        case GridType::TSDF:
          problem.AddResidualBlock(
            CreateTSDFMatchCostFunction2D(
              options_.occupied_space_weight() * options_.local_weight_factor() /
              std::sqrt(static_cast<double>(point_cloud.size())),
              point_cloud, static_cast<const TSDF2D &>(local_grid)),
            nullptr /* loss function */, ceres_pose_estimate);
          break;
      }
    }
    switch (global_grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        problem.AddResidualBlock(
          CreateOccupiedSpaceCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, global_grid),
          nullptr /* loss function */, ceres_pose_estimate);
        break;
      case GridType::TSDF:
        problem.AddResidualBlock(
          CreateTSDFMatchCostFunction2D(
            options_.occupied_space_weight() /
            std::sqrt(static_cast<double>(point_cloud.size())),
            point_cloud, static_cast<const TSDF2D &>(global_grid)),
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

    if (options_.local_weight_factor() > 1e-3) {
      // LOG(WARNING) << "Using local line ";
      // 对于每一个跟踪到的线特征
      double local_line_parallel_weight
                = options_.line_weight() / 2 / static_cast<double>(local_tracked_line_data.size());
      double local_line_angle_weight
                = options_.line_weight() * 2 / static_cast<double>(local_tracked_line_data.size());
      for (auto iter : local_tracked_line_data) {
        // 如果是新的线特征（id < 0）；或者地图中没有改线段的ID，则跳过
        if (iter.first < 0 ||
          local_line_map.GetLineMapData2DList().count(iter.first) == 0)
        {
          continue;
        }
        // 地图中匹配的线特征id
        // // std::cout << "iter.first: " << iter.first << std::endl;
        auto it = local_line_map.GetLineMapData2DList().find(iter.first);
        const sensor::LineData target_line = it->second.line_data();
        // // std::cout << "iter.second.size(): " << iter.second.size() << std::endl;
        if (static_cast<int>(iter.second.size()) == 1) {
          // 如果只匹配了一个线特征，则直接构建残差
          problem.AddResidualBlock(
            CreateLineDeltaWithoutInitPoseCostFunction2D(
              local_line_parallel_weight * options_.local_weight_factor(),
              local_line_angle_weight * options_.local_weight_factor(),
              iter.second[0], target_line),
            nullptr, ceres_pose_estimate);
        } else {
          // 如果有多个匹配的线段，则知道啊最近的线段，构建残差
          double min_dis = 10.0;
          for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
            Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
            if (matching_dis[0] < min_dis) {
              min_dis = matching_dis[0];
            }
          }
          for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
            Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
            if (fabs(matching_dis[0] - min_dis) < 0.005) {
              problem.AddResidualBlock(
                CreateLineDeltaWithoutInitPoseCostFunction2D(
                  local_line_parallel_weight * options_.local_weight_factor(),
                  local_line_angle_weight * options_.local_weight_factor(),
                  iter.second[i], target_line),
                nullptr, ceres_pose_estimate);
            }
          }
        }
      }
    }

    // 对于每一个跟踪到的线特征
    double global_line_parallel_weight
              = options_.line_weight() / 2 / static_cast<double>(global_tracked_line_data.size());
    double global_line_angle_weight
              = options_.line_weight() * 2 / static_cast<double>(global_tracked_line_data.size());
    for (auto iter : global_tracked_line_data) {
      // 如果是新的线特征（id < 0）；或者地图中没有改线段的ID，则跳过
      if (iter.first < 0 ||
        global_line_map.GetLineMapData2DList().count(iter.first) == 0)
      {
        continue;
      }
      // 地图中匹配的线特征id
      // // std::cout << "iter.first: " << iter.first << std::endl;
      auto it = global_line_map.GetLineMapData2DList().find(iter.first);
      const sensor::LineData target_line = it->second.line_data();
      // // std::cout << "iter.second.size(): " << iter.second.size() << std::endl;
      if (static_cast<int>(iter.second.size()) == 1) {
        // 如果只匹配了一个线特征，则直接构建残差
        problem.AddResidualBlock(
          CreateLineDeltaWithoutInitPoseCostFunction2D(
            global_line_parallel_weight, global_line_angle_weight,
            iter.second[0], target_line),
          nullptr, ceres_pose_estimate);
      } else {
        // 如果有多个匹配的线段，则知道啊最近的线段，构建残差
        double min_dis = 10.0;
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (matching_dis[0] < min_dis) {
            min_dis = matching_dis[0];
          }
        }
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (fabs(matching_dis[0] - min_dis) < 0.005) {
            problem.AddResidualBlock(
              CreateLineDeltaWithoutInitPoseCostFunction2D(
                global_line_parallel_weight,
                global_line_angle_weight,
                iter.second[i], target_line),
              nullptr, ceres_pose_estimate);
          }
        }
      }
    }

    if (options_.local_weight_factor() > 1e-3) {
      // LOG(WARNING) << "Using local cloud ";
      // local point constraint
      if (local_cloud_ikdtree.Root_Node == nullptr ||
          local_cloud_ikdtree.validnum() < 10)
      {
        LOG(WARNING) << "No local points in the submap.";
      } else {
        const auto pose_estimate_temp = transform::Rigid2d(
          {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);

        auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        std::vector<sensor::PointCloud::PointType> p_src_vec;
        std::vector<sensor::PointCloud::PointType> p_a_vec;
        std::vector<sensor::PointCloud::PointType> p_b_vec;
        size_t step = std::max(size_t(1), point_cloud.size() / 25);
        for (size_t i = 0; i < point_cloud.size(); i += step) {
          const auto& p_src = point_cloud[i];
          const Eigen::Vector3d p_transformed =
                                        pose_estimate_temp_3d.rotation().toRotationMatrix()
                                        * p_src.position.cast<double>()
                                        + pose_estimate_temp_3d.translation();

          // KD-tree查找最近的点
          pcl::PointXYZ query_point(p_transformed.x(),
                                    p_transformed.y(),
                                    p_transformed.z());

          std::vector<float> squared_distances(10, -1);
          ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
          local_cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

          // 如果最近点不是地图点，则跳过
          double distance = std::sqrt(squared_distances[19]);
          if (distance > 0.15) {
            continue;
          }

          std::vector<Eigen::Vector3f> nearCorners;
          Eigen::Vector3f center(0, 0, 0);
          for (int j = 0; j < 10; j++) {
            sensor::PointCloud::PointType p_tmp;
            p_tmp.position.x() = points_near[j].x;
            p_tmp.position.y() = points_near[j].y;
            p_tmp.position.z() = points_near[j].z;

            center = center + p_tmp.position;
            nearCorners.push_back(p_tmp.position);
          }
          center = center / 10.0;

          Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
          for (int j = 0; j < 10; j++) {
            Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
          }
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);

          // if is indeed line feature
          // note Eigen library sort eigenvalues in increasing order
          Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
          unit_direction.head<2>() = saes.eigenvectors().col(1);

          if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0])
          { 
            Eigen::Vector3f point_on_line = center;
            sensor::PointCloud::PointType point_a, point_b;
            point_a.position = 0.1 * unit_direction + point_on_line;
            point_b.position = -0.1 * unit_direction + point_on_line;

            p_src_vec.push_back(p_src);
            p_a_vec.push_back(point_a);
            p_b_vec.push_back(point_b);
          }
        }

        for (size_t i = 0; i < p_src_vec.size(); i++) {
          problem.AddResidualBlock(
            CreatePointLineCostFunction2D(
              options_.point_weight() /
              static_cast<double>(p_src_vec.size()) *
              options_.local_weight_factor(),
              p_src_vec[i], p_a_vec[i], p_b_vec[i]),
            loss_function /* loss function */, ceres_pose_estimate);
        }
      }
    }

    // global point constraint
    if (global_cloud_ikdtree.Root_Node == nullptr ||
        global_cloud_ikdtree.validnum() < 10)
    {
      LOG(WARNING) << "No global points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> p_src_vec;
      std::vector<sensor::PointCloud::PointType> p_a_vec;
      std::vector<sensor::PointCloud::PointType> p_b_vec;
      size_t step = std::max(size_t(1), point_cloud.size() / 25);
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<float> squared_distances(10, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        global_cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[19]);
        if (distance > 0.15) {
          continue;
        }
        std::vector<Eigen::Vector3f> nearCorners;
        Eigen::Vector3f center(0, 0, 0);
        for (int j = 0; j < 10; j++) {
          sensor::PointCloud::PointType p_tmp;
          p_tmp.position.x() = points_near[j].x;
          p_tmp.position.y() = points_near[j].y;
          p_tmp.position.z() = points_near[j].z;

          center = center + p_tmp.position;
          nearCorners.push_back(p_tmp.position);
        }
        center = center / 10.0;

        Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
        for (int j = 0; j < 10; j++) {
          Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
        unit_direction.head<2>() = saes.eigenvectors().col(1);
        
        if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0])
        { 
          Eigen::Vector3f point_on_line = center;
          sensor::PointCloud::PointType point_a, point_b;
          point_a.position = 0.1 * unit_direction + point_on_line;
          point_b.position = -0.1 * unit_direction + point_on_line;

          p_src_vec.push_back(p_src);
          p_a_vec.push_back(point_a);
          p_b_vec.push_back(point_b);
        }
      }

      for (size_t i = 0; i < p_src_vec.size(); i++) {
        problem.AddResidualBlock(
          CreatePointLineCostFunction2D(
            options_.point_weight() /
            static_cast<double>(p_src_vec.size()),
            p_src_vec[i], p_a_vec[i], p_b_vec[i]),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    if (highlight_cloud_ikdtree.Root_Node == nullptr ||
        highlight_cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> src_points;
      std::vector<sensor::PointCloud::PointType> nearest_points;
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
      
        std::vector<float> squared_distances(1, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        highlight_cloud_ikdtree.Nearest_Search(query_point, 1, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }

        src_points.push_back(p_src);

        sensor::PointCloud::PointType p_tmp;
        p_tmp.position.x() = points_near[0].x;
        p_tmp.position.y() = points_near[0].y;
        p_tmp.position.z() = points_near[0].z;
        nearest_points.push_back(p_tmp);
      }

      for (size_t i = 0; i < nearest_points.size(); i++) {
        const auto& p_src = src_points[i];
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = nearest_points[i].position.x();
        p_dist.position.y() = nearest_points[i].position.y();
        p_dist.position.z() = nearest_points[i].position.z();

        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() * 3 / 
            static_cast<double>(src_points.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }
    ceres::Solve(ceres_solver_options_, &problem, summary);
    
    if (!summary->IsSolutionUsable() || std::isinf(summary->final_cost) || std::isnan(summary->final_cost)) {
      LOG(WARNING) << "Ceres solve failed: " << summary->BriefReport();
      ceres_pose_estimate[0] = initial_pose_estimate.translation().x();
      ceres_pose_estimate[1] = initial_pose_estimate.translation().y();
      ceres_pose_estimate[2] = initial_pose_estimate.rotation().angle();
      break;
    }
    // if (1) {
    //   Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cov_pose
    //     = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>::Zero();
    //   ceres::Covariance::Options cov_options;
    //   ceres::Covariance covariance(cov_options);
    //   std::vector<std::pair<const double*, const double*>> covariance_blocks;
    //   covariance_blocks.emplace_back(
    //                       std::make_pair(ceres_pose_estimate,
    //                                     ceres_pose_estimate));
    //   CHECK(covariance.Compute(covariance_blocks, &problem));
    //   covariance.GetCovarianceBlockInTangentSpace(
    //                                       ceres_pose_estimate,
    //                                       ceres_pose_estimate,
    //                                       cov_pose.data());
    //   cov = cov_pose;
    // }
  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);

  double delta_dist = (pose_estimate->translation() - initial_pose_estimate.translation()).norm();
  double delta_yaw = std::abs(pose_estimate->rotation().angle() - initial_pose_estimate.rotation().angle());

  // LOG(INFO) << "ceres_pose_estimate pose validationt: \n delta dist: " << delta_dist << " delta yaw: " << delta_yaw << std::endl;

  if (delta_dist > 0.2 || delta_yaw > 0.1) {
    LOG(ERROR) << "ceres_pose_estimate pose is not good.";
    *pose_estimate = initial_pose_estimate;
  }
}

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const std::map<int, std::vector<sensor::LineData>> & line_data,
  const Grid2D & grid, const LineMap2DList & line_map,
  ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.line_weight(), 0.);
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

    // 对于每一个跟踪到的线特征
    double line_parallel_weight
              = options_.line_weight() / 2 / static_cast<double>(line_data.size());
    double line_angle_weight
              = options_.line_weight() * 2 / static_cast<double>(line_data.size());
    for (auto iter : line_data) {
      // 如果是新的线特征（id < 0）；或者地图中没有改线段的ID，则跳过
      if (iter.first < 0 ||
        line_map.GetLineMapData2DList().count(iter.first) == 0)
      {
        continue;
      }
      // 地图中匹配的线特征id
      // // std::cout << "iter.first: " << iter.first << std::endl;
      auto it = line_map.GetLineMapData2DList().find(iter.first);
      const sensor::LineData target_line = it->second.line_data();
      // // std::cout << "iter.second.size(): " << iter.second.size() << std::endl;
      if (static_cast<int>(iter.second.size()) == 1) {
        // 如果只匹配了一个线特征，则直接构建残差
        problem.AddResidualBlock(
          CreateLineDeltaWithoutInitPoseCostFunction2D(
            line_parallel_weight, line_angle_weight,
            iter.second[0], target_line),
          nullptr, ceres_pose_estimate);
      } else {
        // 如果有多个匹配的线段，则知道啊最近的线段，构建残差
        double min_dis = 10.0;
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (matching_dis[0] < min_dis) {
            min_dis = matching_dis[0];
          }
        }
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (fabs(matching_dis[0] - min_dis) < 0.005) {
            problem.AddResidualBlock(
              CreateLineDeltaWithoutInitPoseCostFunction2D(
                line_parallel_weight, line_angle_weight,
                iter.second[i], target_line),
              nullptr, ceres_pose_estimate);
          }
        }
      }
    }
    if (cloud_ikdtree.Root_Node == nullptr ||
        cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      
      // TODO: 使用sharp point进行残差计算，而不是使用普通的点云
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> p_src_vec;
      std::vector<sensor::PointCloud::PointType> p_a_vec;
      std::vector<sensor::PointCloud::PointType> p_b_vec;
      size_t step = std::max(size_t(1), point_cloud.size() / 50);
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());

        std::vector<float> squared_distances(10, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[19]);
        if (distance > 0.15) {
          continue;
        }
        std::vector<Eigen::Vector3f> nearCorners;
        Eigen::Vector3f center(0, 0, 0);
        for (int j = 0; j < 10; j++) {
          sensor::PointCloud::PointType p_tmp;
          p_tmp.position.x() = points_near[j].x;
          p_tmp.position.y() = points_near[j].y;
          p_tmp.position.z() = points_near[j].z;

          center = center + p_tmp.position;
          nearCorners.push_back(p_tmp.position);
        }
        center = center / 10.0;

        Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
        for (int j = 0; j < 10; j++) {
          Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
        unit_direction.head<2>() = saes.eigenvectors().col(1);
        
        if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0])
        { 
          Eigen::Vector3f point_on_line = center;
          sensor::PointCloud::PointType point_a, point_b;
          point_a.position = 0.1 * unit_direction + point_on_line;
          point_b.position = -0.1 * unit_direction + point_on_line;

          p_src_vec.push_back(p_src);
          p_a_vec.push_back(point_a);
          p_b_vec.push_back(point_b);
        }
      }

      for (size_t i = 0; i < p_src_vec.size(); i++) {
        problem.AddResidualBlock(
          CreatePointLineCostFunction2D(
            options_.point_weight() /
            static_cast<double>(p_src_vec.size()),
            p_src_vec[i], p_a_vec[i], p_b_vec[i]),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    ceres::Solve(ceres_solver_options_, &problem, summary);
  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const sensor::PointCloud & highlight_point_cloud,
  const std::map<int, std::vector<sensor::LineData>> & line_data,
  const Grid2D & grid, const LineMap2DList & line_map,
  ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
  ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.line_weight(), 0.);
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

    // 对于每一个跟踪到的线特征
    double line_parallel_weight
              = options_.line_weight() / 2 / static_cast<double>(line_data.size());
    double line_angle_weight
              = options_.line_weight() * 2 / static_cast<double>(line_data.size());
    for (auto iter : line_data) {
      // 如果是新的线特征（id < 0）；或者地图中没有改线段的ID，则跳过
      if (iter.first < 0 ||
        line_map.GetLineMapData2DList().count(iter.first) == 0)
      {
        continue;
      }
      // 地图中匹配的线特征id
      // // std::cout << "iter.first: " << iter.first << std::endl;
      auto it = line_map.GetLineMapData2DList().find(iter.first);
      const sensor::LineData target_line = it->second.line_data();
      // // std::cout << "iter.second.size(): " << iter.second.size() << std::endl;
      if (static_cast<int>(iter.second.size()) == 1) {
        // 如果只匹配了一个线特征，则直接构建残差
        problem.AddResidualBlock(
          CreateLineDeltaWithoutInitPoseCostFunction2D(
            line_parallel_weight, line_angle_weight,
            iter.second[0], target_line),
          nullptr, ceres_pose_estimate);
      } else {
        // 如果有多个匹配的线段，则知道啊最近的线段，构建残差
        double min_dis = 10.0;
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (matching_dis[0] < min_dis) {
            min_dis = matching_dis[0];
          }
        }
        for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
          Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
          if (fabs(matching_dis[0] - min_dis) < 0.005) {
            problem.AddResidualBlock(
              CreateLineDeltaWithoutInitPoseCostFunction2D(
                line_parallel_weight, line_angle_weight,
                iter.second[i], target_line),
              nullptr, ceres_pose_estimate);
          }
        }
      }
    }

    if (cloud_ikdtree.Root_Node == nullptr ||
        cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      
      // TODO: 使用sharp point进行残差计算，而不是使用普通的点云
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> p_src_vec;
      std::vector<sensor::PointCloud::PointType> p_a_vec;
      std::vector<sensor::PointCloud::PointType> p_b_vec;
      size_t step = std::max(size_t(1), point_cloud.size() / 50);
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<float> squared_distances(10, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[19]);
        if (distance > 0.15) {
          continue;
        }
        std::vector<Eigen::Vector3f> nearCorners;
        Eigen::Vector3f center(0, 0, 0);
        for (int j = 0; j < 10; j++) {
          sensor::PointCloud::PointType p_tmp;
          p_tmp.position.x() = points_near[j].x;
          p_tmp.position.y() = points_near[j].y;
          p_tmp.position.z() = points_near[j].z;

          center = center + p_tmp.position;
          nearCorners.push_back(p_tmp.position);
        }
        center = center / 10.0;

        Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
        for (int j = 0; j < 10; j++) {
          Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
        unit_direction.head<2>() = saes.eigenvectors().col(1);
        
        if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0])
        { 
          Eigen::Vector3f point_on_line = center;
          sensor::PointCloud::PointType point_a, point_b;
          point_a.position = 0.1 * unit_direction + point_on_line;
          point_b.position = -0.1 * unit_direction + point_on_line;

          p_src_vec.push_back(p_src);
          p_a_vec.push_back(point_a);
          p_b_vec.push_back(point_b);
        }
      }

      for (size_t i = 0; i < p_src_vec.size(); i++) {
        problem.AddResidualBlock(
          CreatePointLineCostFunction2D(
            options_.point_weight() /
            static_cast<double>(p_src_vec.size()),
            p_src_vec[i], p_a_vec[i], p_b_vec[i]),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    if (highlight_cloud_ikdtree.Root_Node == nullptr ||
        highlight_cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> src_points;
      std::vector<sensor::PointCloud::PointType> nearest_points;
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
      
        std::vector<float> squared_distances(1, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        highlight_cloud_ikdtree.Nearest_Search(query_point, 1, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }

        src_points.push_back(p_src);

        sensor::PointCloud::PointType p_tmp;
        p_tmp.position.x() = points_near[0].x;
        p_tmp.position.y() = points_near[0].y;
        p_tmp.position.z() = points_near[0].z;
        nearest_points.push_back(p_tmp);
      }

      for (size_t i = 0; i < nearest_points.size(); i++) {
        const auto& p_src = src_points[i];
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = nearest_points[i].position.x();
        p_dist.position.y() = nearest_points[i].position.y();
        p_dist.position.z() = nearest_points[i].position.z();

        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() * 3 / 
            static_cast<double>(src_points.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    ceres::Solve(ceres_solver_options_, &problem, summary);
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
  ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
  ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary) const
{
  double ceres_pose_estimate[3] = {initial_pose_estimate.translation().x(),
                                   initial_pose_estimate.translation().y(),
                                   initial_pose_estimate.rotation().angle()};
  CHECK_GT(options_.occupied_space_weight(), 0.);
  CHECK_GT(options_.line_weight(), 0.);
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

    if (cloud_ikdtree.Root_Node == nullptr ||
        cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      
      // TODO: 使用sharp point进行残差计算，而不是使用普通的点云
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> p_src_vec;
      std::vector<sensor::PointCloud::PointType> p_a_vec;
      std::vector<sensor::PointCloud::PointType> p_b_vec;
      size_t step = std::max(size_t(1), point_cloud.size() / 50);
      for (size_t i = 0; i < point_cloud.size(); i += step) {
        const auto& p_src = point_cloud[i];
        const Eigen::Vector3d p_transformed =
                                      pose_estimate_temp_3d.rotation().toRotationMatrix()
                                      * p_src.position.cast<double>()
                                      + pose_estimate_temp_3d.translation();
        // KD-tree查找最近的点
        pcl::PointXYZ query_point(p_transformed.x(),
                                  p_transformed.y(),
                                  p_transformed.z());
      
        std::vector<float> squared_distances(10, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        cloud_ikdtree.Nearest_Search(query_point, 10, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[19]);
        if (distance > 0.15) {
          continue;
        }
        std::vector<Eigen::Vector3f> nearCorners;
        Eigen::Vector3f center(0, 0, 0);
        for (int j = 0; j < 10; j++) {
          sensor::PointCloud::PointType p_tmp;
          p_tmp.position.x() = points_near[j].x;
          p_tmp.position.y() = points_near[j].y;
          p_tmp.position.z() = points_near[j].z;

          center = center + p_tmp.position;
          nearCorners.push_back(p_tmp.position);
        }
        center = center / 10.0;

        Eigen::Matrix2f covMat = Eigen::Matrix2f::Zero();
        for (int j = 0; j < 10; j++) {
          Eigen::Matrix<float, 2, 1> tmpZeroMean = (nearCorners[j] - center).head<2>();
          covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> saes(covMat);

        // if is indeed line feature
        // note Eigen library sort eigenvalues in increasing order
        Eigen::Vector3f unit_direction = Eigen::Vector3f::Zero();
        unit_direction.head<2>() = saes.eigenvectors().col(1);
        
        if (saes.eigenvalues()[1] > 3 * saes.eigenvalues()[0])
        { 
          Eigen::Vector3f point_on_line = center;
          sensor::PointCloud::PointType point_a, point_b;
          point_a.position = 0.1 * unit_direction + point_on_line;
          point_b.position = -0.1 * unit_direction + point_on_line;

          p_src_vec.push_back(p_src);
          p_a_vec.push_back(point_a);
          p_b_vec.push_back(point_b);
        }
      }

      for (size_t i = 0; i < p_src_vec.size(); i++) {
        problem.AddResidualBlock(
          CreatePointLineCostFunction2D(
            options_.point_weight() /
            static_cast<double>(p_src_vec.size()),
            p_src_vec[i], p_a_vec[i], p_b_vec[i]),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    if (highlight_cloud_ikdtree.Root_Node == nullptr ||
        highlight_cloud_ikdtree.validnum() < 10)
    {
      // LOG(WARNING) << "No points in the submap.";
    } else {
      const auto pose_estimate_temp = transform::Rigid2d(
        {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
      auto pose_estimate_temp_3d = transform::Embed3D(pose_estimate_temp);
      // 点云误差
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      std::vector<sensor::PointCloud::PointType> src_points;
      std::vector<sensor::PointCloud::PointType> nearest_points;
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
      
        std::vector<float> squared_distances(1, -1);
        ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_near;
        highlight_cloud_ikdtree.Nearest_Search(query_point, 1, points_near, squared_distances);

        // 如果最近点不是地图点，则跳过
        double distance = std::sqrt(squared_distances[0]);
        if (distance > options_.max_correspondence_distance()) {
          continue;
        }

        src_points.push_back(p_src);

        sensor::PointCloud::PointType p_tmp;
        p_tmp.position.x() = points_near[0].x;
        p_tmp.position.y() = points_near[0].y;
        p_tmp.position.z() = points_near[0].z;
        nearest_points.push_back(p_tmp);
      }

      for (size_t i = 0; i < nearest_points.size(); i++) {
        const auto& p_src = src_points[i];
        sensor::PointCloud::PointType p_dist;
        p_dist.position.x() = nearest_points[i].position.x();
        p_dist.position.y() = nearest_points[i].position.y();
        p_dist.position.z() = nearest_points[i].position.z();

        problem.AddResidualBlock(
          CreatePointDeltaCostFunction2D(
            options_.point_weight() * 3 / 
            static_cast<double>(src_points.size()),
            p_src, p_dist),
          loss_function /* loss function */, ceres_pose_estimate);
      }
    }

    ceres::Solve(ceres_solver_options_, &problem, summary);
  }

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(
  const Eigen::Vector2d & target_translation,
  const transform::Rigid2d & initial_pose_estimate,
  const sensor::PointCloud & point_cloud,
  const std::map<int, std::vector<sensor::LineData>> & line_data,
  const Grid2D & grid, const LineMap2DList & line_map,
  transform::Rigid2d * const pose_estimate,
  ceres::Solver::Summary * const summary) const
{
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

  // 对于每一个跟踪到的线特征
   double line_parallel_weight
              = options_.line_weight() / 2 / static_cast<double>(line_data.size());
  double line_angle_weight
              = options_.line_weight() * 2 / static_cast<double>(line_data.size());
  for (auto iter : line_data) {
    // 如果是新的线特征（id < 0）；或者地图中没有改线段的ID，则跳过
    if (iter.first < 0 ||
      line_map.GetLineMapData2DList().count(iter.first) == 0)
    {
      continue;
    }
    // 地图中匹配的线特征id
    // // std::cout << "iter.first: " << iter.first << std::endl;
    auto it = line_map.GetLineMapData2DList().find(iter.first);
    const sensor::LineData target_line = it->second.line_data();
    // // std::cout << "iter.second.size(): " << iter.second.size() << std::endl;
    if (static_cast<int>(iter.second.size()) == 1) {
      // 如果只匹配了一个线特征，则直接构建残差
      problem.AddResidualBlock(
        CreateLineDeltaWithoutInitPoseCostFunction2D(
          line_parallel_weight, line_angle_weight,
          iter.second[0], target_line),
        nullptr, ceres_pose_estimate);
    } else {
      // 如果有多个匹配的线段，则知道啊最近的线段，构建残差
      double min_dis = 10.0;
      for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
        Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
        if (matching_dis[0] < min_dis) {
          min_dis = matching_dis[0];
        }
      }
      for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
        Eigen::Vector3d matching_dis = iter.second[i].GetMatchingDis();
        if (fabs(matching_dis[0] - min_dis) < 0.005) {
          problem.AddResidualBlock(
            CreateLineDeltaWithoutInitPoseCostFunction2D(
              line_parallel_weight, line_angle_weight,
              iter.second[i], target_line),
            nullptr, ceres_pose_estimate);
        }
      }
    }
    // for (int i = 0; i < static_cast<int>(iter.second.size()); i++) {
    //   // std::cout << "target_line begin x: " << target_line.begin()[0]
    //             << " y:
    //                " << target_line.begin()[1] << std::endl; // std::cout "
    //                "<< " target_line end x : " << target_line.end()[0] << " y
    //       : " << target_line.end()[1] <<
    //         std::endl;
    //   Eigen::Matrix<double, 2, 1> translation(ceres_pose_estimate[0],
    //                                           ceres_pose_estimate[1]);
    //   Eigen::Rotation2D<double> rotation(ceres_pose_estimate[2]);
    //   Eigen::Matrix<double, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    //   Eigen::Matrix<double, 3, 3> transform;
    //   transform << rotation_matrix, translation, 0., 0., 1.;

    //   Eigen::Matrix<double, 3, 1> point_begin(iter.second[i].begin()[0],
    //                                           iter.second[i].begin()[1], 1.0);
    //   Eigen::Matrix<double, 3, 1> point_end(iter.second[i].end()[0],
    //                                         iter.second[i].end()[1], 1.0);
    //   const Eigen::Matrix<double, 3, 1> world_b = transform * point_begin;
    //   const Eigen::Matrix<double, 3, 1> world_e = transform * point_end;
    //   // std::cout << "data begin x: " << iter.second[i].begin()[0]
    //             << " y: " << iter.second[i].begin()[1] << std::endl;
    //   // std::cout << "data end x: " << iter.second[i].end()[0]
    //             << " y: " << iter.second[i].end()[1] << std::endl;
    //   // std::cout << "world data begin x: " << world_b[0] << " y: " << world_b[1]
    //             << std::endl;
    //   // std::cout << "world data end x: " << world_e[0] << " y: " << world_e[1]
    //             << std::endl;
    //   problem.AddResidualBlock(
    //       CreateLineDeltaWithoutInitPoseCostFunction2D(
    //           options_.line_weight() / 2, options_.line_weight() * 2,
    //           iter.second[i], target_line),
    //       nullptr, ceres_pose_estimate);
    // }
  }

  ceres::Solve(ceres_solver_options_, &problem, summary);

  //   // std::cout << summary->BriefReport() << "\n";
  //   // std::cout << ceres_pose_estimate[0] << " " << ceres_pose_estimate[1] << "
  //   " << ceres_pose_estimate[2] << std::endl; for (auto iter : line_data) {
  //     for (int i = 0; i < iter.second.size(); i++) {
  //       Eigen::Matrix<double, 2, 1> translation(ceres_pose_estimate[0],
  //       ceres_pose_estimate[1]); Eigen::Rotation2D<double>
  //       rotation(ceres_pose_estimate[2]); Eigen::Matrix<double, 2, 2>
  //       rotation_matrix = rotation.toRotationMatrix(); Eigen::Matrix<double,
  //       3, 3> transform; transform << rotation_matrix, translation, 0.,
  //       0., 1.;

  //       Eigen::Matrix<double, 3, 1> point_begin(iter.second[i].begin()[0],
  //       iter.second[i].begin()[1], 1.0); Eigen::Matrix<double, 3, 1>
  //       point_end(iter.second[i].end()[0], iter.second[i].end()[1], 1.0);
  //       const Eigen::Matrix<double, 3, 1> world_b = transform * point_begin;
  //       const Eigen::Matrix<double, 3, 1> world_e = transform * point_end;
  //       // std::cout << "data begin x: " << world_b[0] << " y: " << world_b[1]
  //       << std::endl; // std::cout << "data end x: " << world_e[0] << " y: " <<
  //       world_e[1] << std::endl;
  //     }
  //   }
  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

void CeresScanMatcher2D::Match(const Eigen::Vector2d& target_translation,
                 const transform::Rigid2d& initial_pose_estimate,
                 const sensor::PointCloud& point_cloud,
                 const Grid2D& grid,
                 transform::Rigid2d* const pose_estimate,
                 ceres::Solver::Summary* const summary) const {
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

  *pose_estimate = transform::Rigid2d(
    {ceres_pose_estimate[0], ceres_pose_estimate[1]}, ceres_pose_estimate[2]);
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer


      // 点云误差
      // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      // std::vector<std::pair<int,int>> index_buff;
      // size_t step = std::max(size_t(1), point_cloud.size() / 50);
      // 下面这段代码是点到点的误差，已经被点到线的误差替换掉了
      // for (size_t i = 0; i < point_cloud.size(); i += step) {
      //   const auto& p_src = point_cloud[i];
      //   // // std::cout << p_src.position.transpose() << std::endl;
      //   // // std::cout << pose_estimate_temp_3d.rotation().toRotationMatrix().matrix() << std::endl;
      //   // // std::cout << pose_estimate_temp_3d.translation().transpose() << std::endl;
      //   const Eigen::Vector3d p_transformed =
      //                                 pose_estimate_temp_3d.rotation().toRotationMatrix()
      //                                 * p_src.position.cast<double>()
      //                                 + pose_estimate_temp_3d.translation();
      //   // KD-tree查找最近的点
      //   pcl::PointXYZ query_point(p_transformed.x(),
      //                             p_transformed.y(),
      //                             p_transformed.z());
      
      //   std::vector<int> indices(1);
      //   std::vector<float> squared_distances(1);
      //   cloud_kdtree.nearestKSearch(query_point, 1, indices, squared_distances);
      //   // 如果最近点不是地图点，则跳过
      //   double distance = std::sqrt(squared_distances[0]);
      //   if (distance > options_.max_correspondence_distance()) {
      //     continue;
      //   }
      //   // // std::cout << "p_transformed: " << p_transformed.transpose() << std::endl;
      //   // // std::cout << "distance: " << distance << std::endl;
      //   // // std::cout << "nearest point: " << cloud_kdtree.getInputCloud()->points[indices[0]] << std::endl;
      //   index_buff.push_back(std::make_pair(i, indices[0]));
      // }
      // for (size_t i = 0; i < index_buff.size(); i++) {
      //   const auto& p_src = point_cloud[index_buff[i].first];
      //   const int& indices = index_buff[i].second;
      //   sensor::PointCloud::PointType p_dist;
      //   p_dist.position.x() = cloud_kdtree.getInputCloud()->points[indices].x;
      //   p_dist.position.y() = cloud_kdtree.getInputCloud()->points[indices].y;
      //   p_dist.position.z() = cloud_kdtree.getInputCloud()->points[indices].z;
      //   problem.AddResidualBlock(
      //     CreatePointDeltaCostFunction2D(
      //       options_.point_weight() / 
      //       static_cast<double>(index_buff.size()),
      //       p_src, p_dist),
      //     loss_function /* loss function */, ceres_pose_estimate);
      // }
