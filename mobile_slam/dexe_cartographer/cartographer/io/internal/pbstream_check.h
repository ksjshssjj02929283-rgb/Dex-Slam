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

}  // namespace pbstream_check_internal
}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_INTERNAL_PBSTREAM_CHECK_H_
