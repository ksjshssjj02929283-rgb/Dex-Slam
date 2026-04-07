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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__POINT_SET_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__POINT_SET_HPP_

#include <list>
#include "perception_2d/laser_perception_2d/point.hpp"

typedef std::list<Point>::iterator PointIterator;

class PointSet
{
public:
  PointSet()
  : num_points(0), is_visible(false) {}

  PointIterator begin, end;    // The iterators point to the list of points existing somewhere else
  int num_points;
  bool is_visible;  // The point set is not occluded by any other point set
};

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__POINT_SET_HPP_
