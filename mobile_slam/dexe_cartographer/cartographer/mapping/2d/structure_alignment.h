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
// 注意：histogram_bin_deg 有意不暴露到上层配置 —— 直方图只用于找峰/支撑量，
// 方向精化直接使用线段自身角度，分箱宽度不再是精度瓶颈。
struct StructureAlignmentOptions {
  // 结构线段最小长度（米）：短于此值的线段不参与角度估计（墙通常较长）。
  // 注意：进入估计前，线段已通过 LineDataCulling（长度范围 / 观测次数 /
  // RANSAC 内点）完成基础质量剔除，此处仅做面向结构方向的进一步筛选。
  double min_line_length = 0.3;
  // 通过长度门槛的线段中，按长度降序取前 N 条作为候选
  // （防止某方向上大量中等长度线段在投票时压过真正的长墙）
  int max_candidate_lines = 20;
  // 角度直方图分箱宽度（度）
  double histogram_bin_deg = 1.0;
  // 正交容差（度）：次方向须落在主方向 +90° ± 该容差内
  double orthogonality_tolerance_deg = 10.0;
  // 次方向支撑量 / 主方向支撑量 的最小比值；
  // 低于该比值认为不存在可靠的正交结构，放弃角度重置
  double min_secondary_support_ratio = 0.1;
};

// 从全局线特征地图 'line_map'（应为聚合 + culling 后的 LineMap2DList）估计
// 对齐结构化环境的新原点朝向。
//
// 成功时返回 new_yaw：新原点在旧地图坐标系下的朝向（弧度），满足
//   new_yaw ≡ r (mod π/2) 且 |new_yaw| ≤ π/4。
// r 是主/次两个正交结构方向的支撑加权最小二乘折中：两方向先各自用候选
// 线段角度迭代精化（收窄窗口剔除不对称尾部），再按各自支撑量加权融合，
// 使两族墙平分非严格正交带来的残余角（见 .cc 第 6 段）。
// 找不到可靠的正交结构方向时返回 nullopt。
absl::optional<double> EstimateAlignmentYaw(
    const LineMap2DList& line_map, const StructureAlignmentOptions& options);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_STRUCTURE_ALIGNMENT_H_
