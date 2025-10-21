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

#ifndef CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_
#define CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_

#include "cartographer/mapping/2d/line_map_2d.h"
#include "ceres/ceres.h"

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
struct Line
{
  Eigen::Vector2d begin;
  Eigen::Vector2d end;
  double angle() const
  {
    return atan2(end[1] - begin[1], end[0] - begin[0]);
  }
};

ceres::CostFunction * CreateLineDeltaCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line);
ceres::CostFunction * CreateLineDeltaWithAngleCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line,
  const transform::Rigid2d & initial_pose);
ceres::CostFunction * CreateLineDeltaWithoutInitPoseCostFunction2D(
  const double parallel_scaling_factor, const double angle_scaling_factor,
  const sensor::LineData & line_data, const sensor::LineData & target_line);

// ============================================================================
// 方案B: 改进版本函数声明
// ============================================================================

// 改进版本1: 对称性距离 + 双端点 + 中点 + 自适应权重
// 返回2个残差: [距离误差, 角度误差]
ceres::CostFunction* CreateImprovedLineDeltaCostFunction2D(
  const double parallel_scaling_factor, 
  const double angle_scaling_factor,
  const sensor::LineData& line_data, 
  const sensor::LineData& target_line);

// 改进版本2: 添加初始位姿约束 + 对称性距离
// 返回4个残差: [距离误差, 角度误差, 平移先验误差, 旋转先验误差]
ceres::CostFunction* CreateImprovedLineDeltaWithInitPoseCostFunction2D(
  const double parallel_scaling_factor, 
  const double angle_scaling_factor,
  const double pose_prior_weight,
  const sensor::LineData& line_data, 
  const sensor::LineData& target_line,
  const transform::Rigid2d& initial_pose);

// ============================================================================
// 方案C: 基于距离场的快速线特征匹配
// ============================================================================

// 线特征距离场类 - 预计算线到栅格的距离场
class LineDistanceField {
 public:
  // 构造函数: 从线特征列表构建距离场
  // lines: 线特征列表
  // resolution: 栅格分辨率(米/像素)
  // max_distance: 最大距离阈值(米)，超过此距离设为最大值
  LineDistanceField(const std::vector<sensor::LineData>& lines,
                    double resolution,
                    double max_distance);

  // 查询点到最近线的距离（带双线性插值）
  // point: 查询点的世界坐标
  // 返回: 到最近线的距离（米）
  template <typename T>
  T GetDistance(const Eigen::Matrix<T, 2, 1>& point) const;

  // 查询点到最近线的距离及梯度（解析梯度）
  // point: 查询点的世界坐标
  // distance: 输出距离
  // gradient: 输出梯度（指向远离线的方向）
  template <typename T>
  void GetDistanceAndGradient(
      const Eigen::Matrix<T, 2, 1>& point,
      T* distance,
      Eigen::Matrix<T, 2, 1>* gradient) const;

  // 获取距离场信息（用于调试和可视化）
  const Eigen::MatrixXd& GetDistanceGrid() const { return distance_grid_; }
  double GetResolution() const { return resolution_; }
  const Eigen::Vector2d& GetOrigin() const { return origin_; }

 private:
  // 计算点到线段的最短距离
  static double PointToLineSegmentDistance(
      const Eigen::Vector2d& point,
      const Eigen::Vector2d& line_start,
      const Eigen::Vector2d& line_end);

  // 世界坐标转栅格坐标
  template <typename T>
  Eigen::Matrix<T, 2, 1> WorldToGrid(const Eigen::Matrix<T, 2, 1>& point) const;

  Eigen::MatrixXd distance_grid_;  // 距离场栅格 [rows x cols]
  double resolution_;              // 分辨率(米/像素)
  Eigen::Vector2d origin_;         // 栅格原点的世界坐标
  double max_distance_;            // 最大距离阈值
};

// 方案C: 基于距离场的代价函数（最快）
// 返回1个残差: [加权距离误差]
ceres::CostFunction* CreateLineDistanceFieldCostFunction2D(
  const double weight,
  const sensor::LineData& line_data,
  const LineDistanceField& distance_field);

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER__MAPPING__INTERNAL__2D__SCAN_MATCHING__LINE_DELTA_COST_FUNCTION_2D_H_
