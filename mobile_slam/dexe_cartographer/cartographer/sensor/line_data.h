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

#ifndef CARTOGRAPHER__SENSOR__LINE_DATA_H_
#define CARTOGRAPHER__SENSOR__LINE_DATA_H_
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/common/port.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer
{
namespace sensor
{
enum VerticalStyle
{
  NONE = 0,
  X = 1,
  Y = 2,
};

// LineData是内部的计算数据结构，LaneData只负责存储线段数据
class LineData
{
private:
  Eigen::Vector2d begin_, end_;
  double k_, b_;
  double angle_;
  double weight_;
  // double ttt;
  // mutable absl::optional<Eigen::Vector2d> normal_; // 先验法向
  mutable Eigen::Vector2d normal_;
  mutable bool has_normal_;
  VerticalStyle style_;
  Eigen::Vector3d matching_dis_;
  void CreateLineData(
    const Eigen::Vector2d & begin, const Eigen::Vector2d & end,
    double weight = 1.0, bool has_normal = false)
  {
    begin_ = begin;
    end_ = end;
    weight_ = weight;
    has_normal_ = has_normal;
    if (begin_.norm() > end_.norm()) {
      Eigen::Vector2d tmp = end_;
      end_ = begin_;
      begin_ = tmp;
    }
    if (fabs(begin_[0] - end_[0]) < 1e-5) {
      angle_ = M_PI_2;
      style_ = Y;
    } else if (fabs(begin_[1] - end_[1]) < 1e-5) {
      angle_ = 0;
      style_ = X;
    } else {
      angle_ = atan2(end_[1] - begin_[1], end_[0] - begin_[0]);
      k_ = (end_[1] - begin_[1]) / (end_[0] - begin_[0]);
      b_ = begin_[1] - begin_[0] * k_;
      style_ = NONE;
    }
    if (weight_ > 1.0) {
      weight_ = 1.0;
    } else if (weight_ < 0.0) {
      weight_ = 0.0;
    }
    matching_dis_ = Eigen::Vector3d::Zero();
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LineData(
    const Eigen::Vector2d & begin, const Eigen::Vector2d & end,
    double weight = 1.0)
  : begin_(begin), end_(end), weight_(weight), has_normal_(false)
  {
    CreateLineData(begin_, end_, weight_, has_normal_);
  }

  LineData(const LineData & other)
  {
    CreateLineData(other.begin_, other.end_, other.weight_, false);
    if (other.has_normal_ == true) {
      Eigen::Vector2d n;
      other.GetNormal(n);
      set_normal(n);
    }
  }

  ~LineData() {}

  Eigen::Vector2d begin() const {return begin_;}
  Eigen::Vector2d end() const {return end_;}
  double weight() const {return weight_;}
  double length() const {return (begin_ - end_).norm();}
  double k() const {return k_;}
  double b() const {return b_;}
  double angle() const {return angle_;}
  VerticalStyle style() const {return style_;}

  void set_normal(const Eigen::Vector2d & normal)
  {
    normal_ = normal.normalized();
    has_normal_ = true;
  }

  bool GetNormal(Eigen::Vector2d & res) const
  {
    if (has_normal_) {
      return false;
    }
    res = normal_;
    return true;
  }

  Eigen::Vector2d normal(const Eigen::Vector2d & pos) const
  {
    if (has_normal_) {
      return normal_;
    }
    Eigen::Vector2d p = begin_ - end_;
    Eigen::Vector2d a = pos - begin_;
    Eigen::Vector2d normal1 = Eigen::Vector2d(-p.y(), p.x());
    Eigen::Vector2d normal2 = Eigen::Vector2d(p.y(), -p.x());
    if (normal1.dot(a) > 0) {
      normal_ = normal1.normalized();
    } else {
      normal_ = normal2.normalized();
    }
    has_normal_ = true;
    return normal_;
  }

  void SetMatchingDis(const Eigen::Vector3d & dis) {matching_dis_ = dis;}
  Eigen::Vector3d GetMatchingDis() const {return matching_dis_;}

  void Transform(transform::Rigid2d origin)
  {
    begin_ = origin * begin_;
    end_ = origin * end_;
    if (fabs(begin_[0] - end_[0]) < 1e-5) {
      angle_ = M_PI_2;
      style_ = Y;
    } else if (fabs(begin_[1] - end_[1]) < 1e-5) {
      angle_ = 0;
      style_ = X;
    } else {
      angle_ = atan2(end_[1] - begin_[1], end_[0] - begin_[0]);
      k_ = (end_[1] - begin_[1]) / (end_[0] - begin_[0]);
      b_ = begin_[1] - begin_[0] * k_;
      style_ = NONE;
    }
  }
};

LineData TransformLineData(transform::Rigid2d origin, LineData data);
/**
 * @struct LaneData 线数据结构（只负责数据的存储传输）
 */
struct LaneData
{
  common::Time time;  ///< 车道线时间戳 */
  struct Line
  {
    Eigen::Vector2d begin;  ///< 线起点 */
    Eigen::Vector2d end;    ///< 线终点 */
  };
  Line line_data;  ///< map坐标系下的线数据 */
  // Eigen::Vector2d normal;
};

struct LaneDataWithNormal
{
  LaneData data;
  Eigen::Vector2d normal;
};

struct LaneDataWithWeight
{
  LaneData data;
  transform::Rigid3d pose_scan_to_tracking;
  double line_weight;
};

struct LaneDataWithOdom
{
  OdometryData odom;
  LaneDataWithWeight lane_data;
};

struct LaserLineData
{
  common::Time time;
  OdometryData odom;
  transform::Rigid3d pose_scan_to_tracking;
  std::vector<LaneDataWithWeight> data;
  /* data */
};

typedef std::vector<cartographer::sensor::LaneDataWithNormal> LineMapData;

// Converts 'lane_data' to a proto::LaneData.
proto::LaneData ToProto(const LaneData & lane_data);

// Converts 'proto' to an LaneData.
LaneData FromProto(const proto::LaneData & proto);

LineData FromProto(const proto::LineData & proto);
proto::LineData ToProto(const LineData & lane_data);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER__SENSOR__LINE_DATA_H_
