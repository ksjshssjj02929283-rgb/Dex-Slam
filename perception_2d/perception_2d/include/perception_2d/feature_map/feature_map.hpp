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
#ifndef PERCEPTION_2D__FEATURE_MAP__FEATURE_MAP_HPP_
#define PERCEPTION_2D__FEATURE_MAP__FEATURE_MAP_HPP_
#include <string>
#include <utility>
#include <vector>
#include <map>
#include "perception_2d/common/port.hpp"
#include "perception_2d/common/time.hpp"
#include "perception_2d/transform/rigid_transform.hpp"
#include "perception_2d/laser_perception_2d/visualization.hpp"

namespace perception_2d
{

struct LineData
{
  Eigen::Vector3f begin;
  Eigen::Vector3f end;
};

struct FeatureMsg
{
  int id;
  std::string target_name;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> line;
  double x;
  double y;
  double theta;
  double corner_struct_angle;
};

struct FeatureMap
{
  common::Time time;
  std::string frame_id;
  std::vector<FeatureMsg> feature_msgs;
};

struct PrePointMsg
{
  common::Time time;
  std::string target_name;
  std::string frame_id;
  std::vector<FeatureMsg> target_msgs;
  std::vector<LinesAndTextInfo> feature_map_msgs;
  std::vector<LinesAndTextInfo> target_map_msgs;
};

struct TargetMap
{
  PrePointMsg pre_point_msg;
  PrePointMsg vmark_msg;
  FeatureMap reflector_msg;
  // std::vector<FeatureMsg> reflector_msg;
};

struct FeatureMsgWithOrigin
{
  common::Time time;
  std::string target_name;
  std::string frame_id;
  // Eigen::Vector3d origin;
  transform::Rigid3d origin;
  std::vector<FeatureMsg> MsgVec;
};

}  // namespace perception_2d

#endif  // PERCEPTION_2D__FEATURE_MAP__FEATURE_MAP_HPP_
