/*
 * Copyright 2018 The Cartographer Authors
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

#ifndef CARTOGRAPHER_IO_INTERNAL_PBSTREAM_CHECK_H_
#define CARTOGRAPHER_IO_INTERNAL_PBSTREAM_CHECK_H_

#include <vector>

#include <google/protobuf/repeated_field.h>

#include "cartographer/mapping/proto/submap.pb.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"

namespace cartographer {
namespace io {

// 统计 pbstream 中各类数据的数量（含线特征），并判定该 pbstream 是否满足
// 定位加载的要求。判定阈值优先取 pbstream 自带的 trajectory_builder_options，
// 缺失时回退到 mapping/2d/line_map_2d.h 的类内默认值。
//
// 返回值即进程退出码：
//   0 = 满足定位要求（可能带 warning）
//   1 = 不满足定位要求
//   2 = 文件不存在 / 为空 / 无法打开
int pbstream_check(int argc, char* argv[]);

// 线特征剔除预测的纯逻辑部分，抽出来供单测直接覆盖。
namespace pbstream_check_internal {

// 与 mapping/2d/line_map_2d.h 的类内默认值保持一致。
// global_dynamic_submap_ 由 LocalMapUpdater2D 深拷贝构造，从不调用
// SetLineCullingParam，因此定位期实际生效的就是这组默认值 —— 与 pbstream 内
// 自带的建图期参数可能不同，两套判据的差值即"加载后会被额外剔除多少条"。
constexpr double kDefaultMinLineLength = 0.4;
constexpr double kDefaultMaxLineLength = 7.0;
constexpr int kDefaultLineCullingObs = 3;

// LineDataCulling() 中固定的内点下限，非配置项。
constexpr int kMinLineInliers = 3;

struct CullingCriteria {
  double min_length;
  double max_length;
  int culling_obs;  // 仅用于展示：observations_ 不持久化，加载来的线段豁免该判据
};

struct LineStats {
  int count = 0;
  std::vector<double> lengths;
  std::vector<int> inliers;
  int survive_pbstream = 0;  // 按 pbstream 自带参数存活数
  int survive_defaults = 0;  // 按 dynamic submap 类内默认存活数
};

// 复算 LineDataCulling() 中对「加载来的线段」仍然生效的那部分条件。
// observations_ 不随 pbstream 持久化，因此观测次数判据不参与（见
// mapping/2d/line_map_2d.cc 的 has_obs_record 分支）。
bool SurvivesCulling(double length, int inliers, const CullingCriteria& c);

void CollectLineStats(const mapping::proto::LineMap2DList& proto,
                      const CullingCriteria& pbstream_criteria,
                      LineStats* stats);

// 从 pbstream 自带的 trajectory_builder_options 中取剔除判据；缺失则回退默认，
// 并把 from_pbstream 置 false。
CullingCriteria ResolveCriteria(
    const ::google::protobuf::RepeatedPtrField<
        mapping::proto::TrajectoryBuilderOptionsWithSensorIds>& all_options,
    bool* from_pbstream);

struct AlignmentStats {
  bool evaluated = false;        // 是否估计出可靠的正交结构方向
  double residual_yaw_deg = 0.0; // 对齐结构化环境所需的残余旋转角（|new_yaw|，
                                 // 已折叠到 [0, 45°]）；evaluated=false 时无意义
  // δ：两族墙的正交误差。位姿图漂移会让两族墙不严格正交，此时**即使算法已达
  // 最优，残余角也不可能小于 |δ|/2**（等支撑时两族平分误差）。有了它才能把
  // 「没对齐」和「已达最优但地图本身不正交」区分开（单看 residual 分不出来）。
  double orthogonality_error_deg = 0.0;
  // 对齐后两族各自保留残余角的支撑加权 RMS
  double rms_axis_residual_deg = 0.0;
  // 本次估计所用的判据阈值。⚠️ 这几个值在建图节点走 lua 的 reset_map_origin_*，
  // 而本工具在核心库、拿不到 ros 侧的配置加载器，只能用结构体默认值。
  // 两者今天一致，但将来 lua 里一旦显式写了这些键就会分叉 —— 把判据打出来，
  // 让「两边用了不同尺子」这件事**可见**，而不是靠记忆。
  double min_line_length = 0.0;
  int max_candidate_lines = 0;
  double orthogonality_tolerance_deg = 0.0;
  double min_secondary_support_ratio = 0.0;
};

// ALIGN-01 的分级判定。单看残余角分不清三种情况，故按 δ 给出物理可达下界：
//   kNotAligned  ：残余 > kNotAlignedDeg，基本等于没做过对齐
//   kPass        ：残余 ≤ tolerance
//   kPassLimited ：残余 > tolerance，但 ≤ |δ|/2 + tolerance —— **算法已达最优**，
//                  再小受地图自身非正交性限制，不是对齐质量问题
//   kWarn        ：以上都不是，确属对齐质量不足
enum class AlignmentGrade { kNotEvaluable, kNotAligned, kPass, kPassLimited, kWarn };

// 残余角超过此值视为「根本没对齐」（典型未对齐样本为几十度）
constexpr double kNotAlignedDeg = 5.0;

AlignmentGrade GradeAlignment(const AlignmentStats& stats,
                              double tolerance_deg);

// 从保存的全局线地图 proto 判断地图是否对齐结构化环境：
// 把 proto 线段轻量重建为 LineMap2DList（不重放观测历史，保持保存时角度
// 原样），复用 mapping::EstimateAlignmentYaw 估计残余旋转角。
// 无可靠正交结构（估计返回 nullopt）时 evaluated=false。
AlignmentStats EvaluateStructureAlignment(
    const mapping::proto::LineMap2DList& proto);

}  // namespace pbstream_check_internal
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PBSTREAM_CHECK_H_
