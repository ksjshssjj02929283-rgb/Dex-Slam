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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__ANGLE_DECTECTOR_H_
#define CARTOGRAPHER__LASER_PERCEPTION__ANGLE_DECTECTOR_H_
#include <utility>
#include <list>
#include <vector>
#include "cartographer/laser_perception/line_detector.h"
#include "cartographer/sensor/sync_point_cloud.h"
namespace cartographer
{
namespace laser_perception
{

struct AngleStructCandidate
{
  std::pair<Eigen::Vector3f, Eigen::Vector3f> first_line;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> second_line;
};

struct AngleStruct
{
  Eigen::Vector3d origin;
  Eigen::Vector3d pose;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> first_line;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> second_line;
  double angle;
};

class AngleDectector
{
public:
  AngleDectector();
  std::vector<AngleStructCandidate> computeAllPossibleStruct(std::list<Segment> lines);
  double computeMinDisBetweenLines(AngleStructCandidate candidate);
  bool detectAngleStruct(const AngleStructCandidate & candidate, AngleStruct & angle_struct);
  bool detectAngle(
    const Eigen::Vector2d & inter_point,
    const AngleStructCandidate & candidate,
    double & pose_angle,
    double & angle);

private:
  double min_dis_between_lines_;
  double min_dis_inter_lines_;
};

}  // namespace laser_perception
}  // cartographer


#endif  // CARTOGRAPHER__LASER_PERCEPTION__ANGLE_DECTECTOR_H_
