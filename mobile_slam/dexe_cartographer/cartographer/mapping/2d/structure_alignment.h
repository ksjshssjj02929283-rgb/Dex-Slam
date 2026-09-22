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
// 注意：histogram_bin_deg 有意不暴露到上层配置 —— 直方图只用于粗找峰
// （定精化种子与屏蔽区间），方向值与支撑量都来自线段自身角度的精化，
// 分箱宽度不再是精度瓶颈。
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

// 估角过程中的诊断量。此前只打进日志，现在带出函数供调用方使用：
//   - 离线工具把它打进 [ALIGN_RESULT] 结果行，供 command_board 编排判断是否值得执行；
//   - pbstream_check ALIGN-01 区分「没对齐」与「已达算法最优但地图本身不正交」。
// 单独成 struct 而非塞进返回值，是为了让返回 nullopt（估角失败）时调用方
// 仍能拿到已算出的部分信息（如候选线段数）用于报错提示。
struct StructureAlignmentDiagnostics {
  // 通过长度门槛并参与估计的候选线段数（Top-N 截断后）
  int candidate_lines = 0;
  // 主/次结构方向（弧度，归一化到 [0, π)）；estimated=false 时无意义
  double primary_direction = 0.0;
  double secondary_direction = 0.0;
  // 主/次方向的支撑量 = 各自精化窗口内候选线段长度之和（米）
  double primary_support = 0.0;
  double secondary_support = 0.0;
  // δ：两族墙的正交误差 = wrap(θ2 − θ1 − π/2)（弧度，∈ [−π/2, π/2)）。
  // 位姿图漂移会让两族墙不严格正交；|δ|/2 是对齐后残余角的物理下界
  // （等支撑时），据此可区分「对齐质量差」与「已达算法最优」。
  double orthogonality_error = 0.0;
  // 对齐后两族各自保留残余角的支撑加权 RMS（弧度）
  double rms_axis_residual = 0.0;
  // 上面几项是否已算出（走到第 6 段才为 true；提前 nullopt 时为 false）
  bool estimated = false;
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
//
// 'diagnostics' 可为 nullptr（默认）；非空时无论成功与否都会被填写，
// 失败路径至少填 candidate_lines，estimated 保持 false。
absl::optional<double> EstimateAlignmentYaw(
    const LineMap2DList& line_map, const StructureAlignmentOptions& options,
    StructureAlignmentDiagnostics* diagnostics = nullptr);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_STRUCTURE_ALIGNMENT_H_
