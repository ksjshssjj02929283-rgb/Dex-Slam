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

#include "cartographer/sensor/line_data.h"

namespace cartographer
{
namespace sensor
{
LineData TransformLineData(transform::Rigid2d origin, LineData data)
{
  return LineData(origin * data.begin(), origin * data.end());
}

// 3D变换全局函数，风格与PointCloud保持一致
LineData TransformLineData(const LineData& line_data, const transform::Rigid3d& transform)
{
  Eigen::Vector3d begin_3d(line_data.begin().x(), line_data.begin().y(), 0.0);
  Eigen::Vector3d end_3d(line_data.end().x(), line_data.end().y(), 0.0);
  
  Eigen::Vector3d begin_transformed = transform * begin_3d;
  Eigen::Vector3d end_transformed = transform * end_3d;
  
  return LineData(
      Eigen::Vector2d(begin_transformed.x(), begin_transformed.y()),
      Eigen::Vector2d(end_transformed.x(), end_transformed.y()),
      line_data.weight()
  );
}

proto::LineData ToProto(const LineData & lane_data)
{
  proto::LineData proto;
  // proto.set_timestamp(common::ToUniversal(lane_data.time));
  // proto::LineData& line  = *proto.mutable_line();
  *proto.mutable_begin() = transform::ToProto(lane_data.begin());
  *proto.mutable_end() = transform::ToProto(lane_data.end());
  return proto;
}

LineData FromProto(const proto::LineData & proto)
{
  return LineData(
    transform::ToEigen(proto.begin()),
    transform::ToEigen(proto.end()));
}


proto::LaneData ToProto(const LaneData & lane_data)
{
  proto::LaneData proto;
  proto.set_timestamp(common::ToUniversal(lane_data.time));
  proto::LaneData_Line & line = *proto.mutable_line();
  *line.mutable_begin() = transform::ToProto(lane_data.line_data.begin);
  *line.mutable_end() = transform::ToProto(lane_data.line_data.end);
  return proto;
}

LaneData FromProto(const proto::LaneData & proto)
{
  LaneData lane_data;
  lane_data.time = common::FromUniversal(proto.timestamp());
  lane_data.line_data.begin = transform::ToEigen(proto.line().begin());
  lane_data.line_data.end = transform::ToEigen(proto.line().end());
  return lane_data;
}

}  // namespace sensor
}  // namespace cartographer
