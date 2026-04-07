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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__SEGMENT_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__SEGMENT_HPP_

#include <vector>
#include "perception_2d/laser_perception_2d/point.hpp"
#include "perception_2d/laser_perception_2d/point_set.hpp"

class Segment
{
public:
  explicit Segment(const Point & p1 = Point(), const Point & p2 = Point())
  {
    // Swap if not counter-clockwise
    if (p1.cross(p2) > 0.0) {
      first_point = p1, last_point = p2;
    } else {
      first_point = p2, last_point = p1;
    }
  }

  double length() const
  {
    return (last_point - first_point).length();
  }

  double lengthSquared() const
  {
    return (last_point - first_point).lengthSquared();
  }

  Point normal() const
  {
    return (last_point - first_point).perpendicular().normalized();
  }

  Point projection(const Point & p) const
  {
    Point a = last_point - first_point;
    Point b = p - first_point;
    return first_point + a.dot(b) * a / a.lengthSquared();
  }

  Point trueProjection(const Point & p) const
  {
    Point a = last_point - first_point;
    Point b = p - first_point;
//    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0) {
      return first_point;
    } else if (t > 1.0) {
      return last_point;
    } else {
      return first_point + a.dot(b) * a / a.lengthSquared();
    }
  }

  double distanceTo(const Point & p) const
  {
    return (p - projection(p)).length();
  }

  double trueDistanceTo(const Point & p) const
  {
    Point a = last_point - first_point;
    Point b = p - first_point;
    Point c = p - last_point;

    double t = a.dot(b) / a.lengthSquared();

    if (t < 0.0) {
      return b.length();
    } else if (t > 1.0) {
      return c.length();
    }

    Point projection = first_point + t * a;
    return (p - projection).length();
  }


  friend std::ostream & operator<<(std::ostream & out, const Segment & s)
  {
    out << "p1: " << s.first_point << ", p2: " << s.last_point;
    return out;
  }

  Point first_point;
  Point last_point;
  std::vector<PointSet> point_sets;
};

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__SEGMENT_HPP_
