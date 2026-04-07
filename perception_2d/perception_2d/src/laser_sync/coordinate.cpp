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
#include "perception_2d/laser_sync/coordinate.hpp"

Pose2D Transform2D::transformTo(const Pose2D & src, const Pose2D & tar)
{
  Pose2D ret;
  ret.theta = tar.theta - src.theta;
  ret.theta = atan2(sin(ret.theta), cos(ret.theta));
  ret.position.x = tar.position.x - src.position.x;
  ret.position.y = tar.position.y - src.position.y;
  ret.position = rotate(-src.theta, ret.position);
  return ret;
}

Pose2D Transform2D::transformFrom(const Pose2D & src, const Pose2D & tar)
{
  Pose2D ret;
  ret.theta = tar.theta + src.theta;
  ret.theta = atan2(sin(ret.theta), cos(ret.theta));
  ret.position = rotate(src.theta, tar.position);
  ret.position.x += src.position.x;
  ret.position.y += src.position.y;
  return ret;
}

Point2D Transform2D::rotate(const double & angle, const Point2D & point)
{
  double cs = cos(angle);
  double ss = sin(angle);
  Point2D ret;
  ret.x = point.x * cs - point.y * ss;
  ret.y = point.x * ss + point.y * cs;
  return ret;
}
