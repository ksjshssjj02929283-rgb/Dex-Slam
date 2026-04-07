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

#ifndef PERCEPTION_2D__TRANSFORM__TIMESTAMPED_TRANSFORM_HPP_
#define PERCEPTION_2D__TRANSFORM__TIMESTAMPED_TRANSFORM_HPP_

#include "perception_2d/common/time.hpp"
#include "perception_2d/transform/rigid_transform.hpp"

namespace perception_2d
{
namespace transform
{

struct TimestampedTransform
{
  common::Time time;
  transform::Rigid3d transform;
};

TimestampedTransform Interpolate(
  const TimestampedTransform & start,
  const TimestampedTransform & end,
  const common::Time time);

}  // namespace transform
}  // namespace perception_2d

#endif  // PERCEPTION_2D__TRANSFORM__TIMESTAMPED_TRANSFORM_HPP_
