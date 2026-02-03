// Copyright (c) 2021 Geekplus Inc
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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__POINT_SET_H_
#define CARTOGRAPHER__LASER_PERCEPTION__POINT_SET_H_

#include <list>
#include "cartographer/laser_perception/point.h"

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

#endif  // CARTOGRAPHER__LASER_PERCEPTION__POINT_SET_H_
