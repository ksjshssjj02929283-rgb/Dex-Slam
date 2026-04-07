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
#ifndef PERCEPTION_2D__LASER_SYNC__DATA_TYPE_HPP_
#define PERCEPTION_2D__LASER_SYNC__DATA_TYPE_HPP_

#include <stdlib.h>
#include <vector>
#include <list>
#include <algorithm>
#include <queue>

#ifndef STRUCT_POINT2D
#define STRUCT_POINT2D
struct Point2D
{
  double x;
  double y;
};
#endif

#ifndef STRUCT_POSE2D
#define STRUCT_POSE2D
// TODO(yaochao): for now add v,w in Pose2D (temporary)
struct Pose2D
{
  Point2D position;
  double theta;
  double v;
  double w;
};
#endif

#ifndef STRUCT_PATH2D
#define STRUCT_PATH2D
struct Path2D
{
  std::vector<Pose2D> poses;
};
#endif

#ifndef STRUCT_POLYGON2D
#define STRUCT_POLYGON2D
struct Polygon2D
{
  std::vector<Point2D> points;
  bool closed;
};
#endif

#ifndef STRUCT_CMDVW
#define STRUCT_CMDVW
class CmdVW
{
public:
  CmdVW()
  : v(0.0), w(0.0) {}
  double v, w;
};
#endif

#ifndef STRUCT_CMDVWTC
#define STRUCT_CMDVWTC
class CmdVWTC
{
public:
  CmdVW vw;
  double timeToTarget;
  double costToTarget;
  double distToSource;
};
#endif

#ifndef STRUCT_CMDVWPOSE
#define STRUCT_CMDVWPOSE
class CmdVWPOSE
{
public:
  CmdVWTC cmd;
  Pose2D tar;
  int pathIndex;
};
#endif

#ifndef CLASS_POSECELLDATA
#define CLASS_POSECELLDATA
class PoseCellData
{
public:
  /**
   * @brief  Constructor for a PoseCellData objects
   * @param  d The distance to the nearest obstacle, used for ordering in the priority queue
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @return
   */
  PoseCellData()
  {
  }
  PoseCellData(double d, double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy)
  : distance_(d), index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy)
  {
  }
  PoseCellData(const PoseCellData & b)
  {
    distance_ = b.distance_;
    index_ = b.index_;
    x_ = b.x_;
    y_ = b.y_;
    src_x_ = b.src_x_;
    src_y_ = b.src_y_;
  }

  double distance_;
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
};
#endif

#ifndef CLASS_INFLATIONHASH
#define CLASS_INFLATIONHASH
class InflationHashMap
{
public:
  InflationHashMap(int n, double maxDist, double minDist, double inflationDist)
  : hashNum_(n), originMaxDist_(maxDist), originMinDist_(minDist), inflationMaxDist_(inflationDist)
  {
    dataNum_ = 0;
    hashMap_.resize(n);
    distSpacing_ = (maxDist - minDist) / (static_cast<double>(n));
    insertnum = 0;
    researchNum = 0;
    popMinIndex = 0;
  }
  std::vector<std::queue<PoseCellData>> hashMap_;
  int hashNum_;
  int dataNum_;
  double originMaxDist_;
  double originMinDist_;
  double inflationMaxDist_;
  double distSpacing_;
  int popMinIndex;
  int insertnum;
  int researchNum;

  inline int getHashSize()
  {
    return hashNum_;
  }
  inline int getDataNum()
  {
    return dataNum_;
  }
  inline int getMinHashPosition()
  {
    for (int i = 0; i < hashNum_; i++) {
      researchNum++;
      if (hashMap_[i].size() > 0) {
        return i;
      }
    }
  }
  inline int calcHashPosition(double dist)
  {
    int index = static_cast<int>((dist - originMinDist_) / distSpacing_);
    if (index > hashNum_ - 1) {
      index = hashNum_ - 1;
    }
    if (index < 0) {
      index = 0;
    }
    return index;
  }

  inline bool insertData(
    const double & distance, const int & hashIndex,
    bool & seen, const int & index,
    const int & mx, const int & my, const int & src_x, const int & src_y)
  {
    if (!seen) {
      if (distance < inflationMaxDist_) {
        PoseCellData data(distance, index, mx, my, src_x, src_y);
        hashMap_[hashIndex].push(data);
        dataNum_++;
        seen = true;
        insertnum++;
        popMinIndex = std::min(hashIndex, popMinIndex);
      }
    }
    return true;
  }

  inline bool popData(PoseCellData & data)
  {
    if (dataNum_ <= 0) {
      return false;
    } else {
      if (hashMap_[popMinIndex].size() <= 0) {
        popMinIndex = getMinHashPosition();
      }
      int index = popMinIndex;
      data = hashMap_[index].front();
      hashMap_[index].pop();
      dataNum_--;
      return true;
    }
  }
};
#endif  // CLASS_INFLATIONHASH

#endif  // PERCEPTION_2D__LASER_SYNC__DATA_TYPE_HPP_
