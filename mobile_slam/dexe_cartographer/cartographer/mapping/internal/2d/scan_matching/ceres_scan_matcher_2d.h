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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/ikd_tree/ikd_tree.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
class CeresScanMatcher2D {
 public:
  explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D& options);
  virtual ~CeresScanMatcher2D();

  CeresScanMatcher2D(const CeresScanMatcher2D&) = delete;
  CeresScanMatcher2D& operator=(const CeresScanMatcher2D&) = delete;
  
  // Structure to hold map data for matching
  struct MapData {
    const Grid2D* grid = nullptr;
    const LineMap2DList* line_map = nullptr;
    ikd_tree::iKdTree<pcl::PointXYZ>* cloud_ikdtree = nullptr;
    ikd_tree::iKdTree<pcl::PointXYZ>* highlight_cloud_ikdtree = nullptr;
    const std::map<int, std::vector<sensor::LineData>>* tracked_line_data = nullptr;
    double weight_factor = 1.0;  // Weight factor for this map (used for local vs global)
  };

  // Unified Match method for localization
  // Uses both local and global maps with configurable weights
  void MatchForLocalization(
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud,
      const sensor::PointCloud& highlight_point_cloud,
      const MapData& local_map,
      const MapData& global_map,
      transform::Rigid2d* const pose_estimate,
      Eigen::Matrix3d* const covariance,
      ceres::Solver::Summary* const summary) const;

  void Match(
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud,
      const sensor::PointCloud& highlight_point_cloud,
      const MapData& map,
      transform::Rigid2d* const pose_estimate,
      Eigen::Matrix3d* const covariance,
      ceres::Solver::Summary* const summary) const;

 private:
  // Helper method to add grid constraints
  void AddGridConstraints(
      ceres::Problem& problem,
      const sensor::PointCloud& point_cloud,
      const Grid2D& grid,
      double weight,
      double* ceres_pose_estimate) const;

  // Helper method to add line constraints
  void AddLineConstraints(
      ceres::Problem& problem,
      const std::map<int, std::vector<sensor::LineData>>& tracked_line_data,
      const LineMap2DList& line_map,
      double weight,
      double* ceres_pose_estimate) const;

  // Helper method to add point-to-line constraints from KD-tree
  void AddPointToLineConstraints(
      ceres::Problem& problem,
      const sensor::PointCloud& point_cloud,
      ikd_tree::iKdTree<pcl::PointXYZ>& cloud_ikdtree,
      double weight,
      const double* ceres_pose_estimate) const;

  // Helper method to add highlight point constraints
  void AddHighlightPointConstraints(
      ceres::Problem& problem,
      const sensor::PointCloud& highlight_point_cloud,
      ikd_tree::iKdTree<pcl::PointXYZ>& highlight_cloud_ikdtree,
      double weight,
      const double* ceres_pose_estimate) const;

  // Helper method to compute covariance from Ceres problem
  bool ComputeCovariance(
      const ceres::Problem& problem,
      const double* ceres_pose_estimate,
      Eigen::Matrix3d* covariance) const;

  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPINGcloud_kdtree