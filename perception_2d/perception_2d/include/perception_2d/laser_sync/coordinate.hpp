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
#ifndef PERCEPTION_2D__LASER_SYNC__COORDINATE_HPP_
#define PERCEPTION_2D__LASER_SYNC__COORDINATE_HPP_

#include <math.h>
#include "perception_2d/laser_sync/data_type.hpp"

class Transform2D
{
public:
  static Pose2D transformTo(const Pose2D & src, const Pose2D & tar);
  static Pose2D transformFrom(const Pose2D & src, const Pose2D & tar);
  static Point2D rotate(const double & angle, const Point2D & point);
};

#endif  // PERCEPTION_2D__LASER_SYNC__COORDINATE_HPP_
