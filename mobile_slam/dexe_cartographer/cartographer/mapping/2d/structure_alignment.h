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

#ifndef CARTOGRAPHER_MAPPING_2D_STRUCTURE_ALIGNMENT_H_
#define CARTOGRAPHER_MAPPING_2D_STRUCTURE_ALIGNMENT_H_

#include "absl/types/optional.h"
#include "cartographer/mapping/2d/line_map_2d.h"

namespace cartographer {
namespace mapping {

// 保存地图前，从全局线特征地图估计「地图原点对齐结构化环境」的重置角度。
// 参数含义见各字段注释；由上层（ROS 节点配置）填充。
struct StructureAlignmentOptions {
  // 结构线段最小长度（米）：短于此值的线段不参与角度估计（墙通常较长）
  double min_line_length = 1.5;
  // 线段 RANSAC 内点数下限（拟合质量的硬门槛）
  int min_inliers = 8;
  // 线段被观测次数下限（稳定性的硬门槛）
  int min_observations = 5;
  // 通过硬门槛的线段中，按长度降序取前 N 条作为候选
  // （防止某方向上大量中等长度线段在投票时压过真正的长墙）
  int max_candidate_lines = 20;
  // 角度直方图分箱宽度（度）
  double histogram_bin_deg = 1.0;
  // 正交容差（度）：次方向须落在主方向 +90° ± 该容差内
  double orthogonality_tolerance_deg = 10.0;
  // 次方向支撑量 / 主方向支撑量 的最小比值；
  // 低于该比值认为不存在可靠的正交结构，放弃角度重置
  double min_secondary_support_ratio = 0.3;
};

// 从全局线特征地图 'line_map'（应为聚合 + culling 后的 LineMap2DList）估计
// 对齐结构化环境的新原点朝向。
//
// 成功时返回 new_yaw：新原点在旧地图坐标系下的朝向（弧度），满足
//   new_yaw ≡ θ_dominant (mod π/2) 且 |new_yaw| ≤ π/4
// （θ_dominant 为主结构方向；对主方向旋转对齐后，与之正交的次方向
//  自然对齐到另一坐标轴）。找不到可靠的正交结构方向时返回 nullopt。
absl::optional<double> EstimateAlignmentYaw(
    const LineMap2DList& line_map, const StructureAlignmentOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_STRUCTURE_ALIGNMENT_H_
