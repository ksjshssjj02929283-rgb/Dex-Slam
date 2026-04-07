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

#ifndef PERCEPTION_2D__SENSOR__RANGEFINDER_POINT_HPP_
#define PERCEPTION_2D__SENSOR__RANGEFINDER_POINT_HPP_

#include <vector>

#include "Eigen/Core"
#include "perception_2d/transform/transform.hpp"
#include "glog/logging.h"

namespace perception_2d
{
namespace sensor
{

// Stores 3D position of a point observed by a rangefinder sensor.
struct RangefinderPoint
{
  Eigen::Vector3f position;
};

// Stores 3D position of a point with its relative measurement time.
// See point_cloud.h for more details.
struct TimedRangefinderPoint
{
  Eigen::Vector3f position;
  float time;
};

template<class T>
inline RangefinderPoint operator*(
  const transform::Rigid3<T> & lhs,
  const RangefinderPoint & rhs)
{
  RangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

template<class T>
inline TimedRangefinderPoint operator*(
  const transform::Rigid3<T> & lhs,
  const TimedRangefinderPoint & rhs)
{
  TimedRangefinderPoint result = rhs;
  result.position = lhs * rhs.position;
  return result;
}

inline bool operator==(
  const RangefinderPoint & lhs,
  const RangefinderPoint & rhs)
{
  return lhs.position == rhs.position;
}

inline bool operator==(
  const TimedRangefinderPoint & lhs,
  const TimedRangefinderPoint & rhs)
{
  return lhs.position == rhs.position && lhs.time == rhs.time;
}

inline RangefinderPoint ToRangefinderPoint(
  const TimedRangefinderPoint & timed_rangefinder_point)
{
  return {timed_rangefinder_point.position};
}

inline TimedRangefinderPoint ToTimedRangefinderPoint(
  const RangefinderPoint & rangefinder_point, const float time)
{
  return {rangefinder_point.position, time};
}

}  // namespace sensor
}  // namespace perception_2d

#endif  // PERCEPTION_2D__SENSOR__RANGEFINDER_POINT_HPP_
