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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_

#include <limits>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/motion_filter_options.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

proto::MotionFilterOptions CreateMotionFilterOptions(
    common::LuaParameterDictionary* parameter_dictionary);


/**
 * @brief Takes poses as input and filters them to get fewer poses.
 *        关键帧筛选器
 * 
 */
class MotionFilter {
 public:
  explicit MotionFilter(const proto::MotionFilterOptions& options);

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(common::Time time, const transform::Rigid3d& pose);
  // 重新开始计数：下一次 IsSimilar 必返回 false 并以该帧为新基准。
  // 用于 active submap 被清空后强制插入首帧，否则修正量小于阈值时 active 会一直为空。
  void Reset() { num_total_ = 0; }

 private:
  const proto::MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  common::Time last_time_;
  transform::Rigid3d last_pose_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_MOTION_FILTER_H_
