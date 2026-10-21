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

  // Aligns 'point_cloud' within the 'grid' given an
  // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
  // 'summary'.
  // grid, local line, global line, local cloud, global cloud, highlight_point_cloud
  void Match(const Eigen::Vector2d & target_translation,
             const transform::Rigid2d & initial_pose_estimate,
             const sensor::PointCloud & point_cloud,
             const sensor::PointCloud & highlight_point_cloud,
             const std::map<int, std::vector<sensor::LineData>> & local_tracked_line_data,
             const std::map<int, std::vector<sensor::LineData>> & global_tracked_line_data,
             const Grid2D & local_grid,
             const Grid2D & global_grid,
             const LineMap2DList & local_line_map,
             const LineMap2DList & global_line_map,
             ikd_tree::iKdTree<pcl::PointXYZ> & local_cloud_ikdtree,
             ikd_tree::iKdTree<pcl::PointXYZ> & global_cloud_ikdtree,
             ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
             transform::Rigid2d * const pose_estimate,
             Eigen::Matrix3d& cov,
             ceres::Solver::Summary * const summary) const;

  // gird, line, point cloud
  void Match(const Eigen::Vector2d & target_translation,
            const transform::Rigid2d & initial_pose_estimate,
            const sensor::PointCloud & point_cloud,
            const std::map<int, std::vector<sensor::LineData>> & line_data,
            const Grid2D & grid, const LineMap2DList & line_map,
            ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
            transform::Rigid2d * const pose_estimate,
            ceres::Solver::Summary * const summary) const;

  // gird, line, point cloud, highlight_point_cloud
  void Match(const Eigen::Vector2d & target_translation,
            const transform::Rigid2d & initial_pose_estimate,
            const sensor::PointCloud & point_cloud,
            const sensor::PointCloud & highlight_point_cloud,
            const std::map<int, std::vector<sensor::LineData>> & line_data,
            const Grid2D & grid, const LineMap2DList & line_map,
            ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
            ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
            transform::Rigid2d * const pose_estimate,
            ceres::Solver::Summary * const summary) const;

  // gird, point cloud, highlight_point_cloud
  void Match(const Eigen::Vector2d & target_translation,
            const transform::Rigid2d & initial_pose_estimate,
            const sensor::PointCloud & point_cloud,
            const sensor::PointCloud & highlight_point_cloud,
            const Grid2D & grid,
            ikd_tree::iKdTree<pcl::PointXYZ> & cloud_ikdtree,
            ikd_tree::iKdTree<pcl::PointXYZ> & highlight_cloud_ikdtree,
            transform::Rigid2d * const pose_estimate,
            ceres::Solver::Summary * const summary) const;

  // grid, line
  void Match(const Eigen::Vector2d & target_translation,
            const transform::Rigid2d & initial_pose_estimate,
            const sensor::PointCloud & point_cloud,
            const std::map<int, std::vector<sensor::LineData>> & line_data,
            const Grid2D & grid, const LineMap2DList & line_map,
            transform::Rigid2d * const pose_estimate,
            ceres::Solver::Summary * const summary) const;

  // grid
  void Match(const Eigen::Vector2d& target_translation,
            const transform::Rigid2d& initial_pose_estimate,
            const sensor::PointCloud& point_cloud,
            const Grid2D& grid,
            transform::Rigid2d* const pose_estimate,
            ceres::Solver::Summary* const summary) const;

  // 回退门（Match 末尾：修正量超限即丢弃匹配结果、退回初值）的最近一次观测量。
  // 单独暴露的理由：这道门一旦触发，输出位姿 ≡ 输入初值，于是下游一致性检查的
  // translation_error/rotation_error **恒等于 0**——匹配越差它越安静。
  // 不把它打出来，这个构造性盲区在日志和快照里都不可见。
  struct RevertInfo {
    double delta_dist = 0.0;   // 本次 Ceres 相对初值的平移修正量 (m)
    double delta_yaw = 0.0;    // 同上，旋转 (rad)
    bool reverted = false;     // 是否触发了回退（结果被丢弃）
  };
  const RevertInfo& last_revert_info() const { return last_revert_info_; }

 private:
  mutable RevertInfo last_revert_info_;
  const proto::CeresScanMatcherOptions2D options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPINGcloud_kdtree