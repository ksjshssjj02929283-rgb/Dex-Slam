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

#include "cartographer/laser_perception/angle_dectector.h"
#include <list>
#include <vector>
#include <utility>
namespace cartographer
{
namespace laser_perception
{

AngleDectector::AngleDectector()
{
  min_dis_between_lines_ = 0.1;
  min_dis_inter_lines_ = 0.05;
}
std::vector<AngleStructCandidate> AngleDectector::computeAllPossibleStruct(std::list<Segment> lines)
{
  std::vector<AngleStructCandidate> candidates;
  std::vector<AngleStructCandidate> final_candidates;
  std::vector<Segment> seg_v;
  for (const auto & it : lines) {
    seg_v.push_back(it);
  }
  for (size_t i = 0; i < seg_v.size() - 1; i++) {
    AngleStructCandidate line_;
    line_.first_line = std::make_pair(
      Eigen::Vector3f(seg_v[i].first_point.x, seg_v[i].first_point.y, 0),
      Eigen::Vector3f(seg_v[i].last_point.x, seg_v[i].last_point.y, 0));
    for (size_t j = i + 1; j < seg_v.size(); j++) {
      line_.second_line = std::make_pair(
        Eigen::Vector3f(seg_v[j].first_point.x, seg_v[j].first_point.y, 0),
        Eigen::Vector3f(seg_v[j].last_point.x, seg_v[j].last_point.y, 0));
      candidates.push_back(line_);
    }
  }
  // LOG(INFO) << "TMP_CANDIDATES SIZE = " << candidates.size();
  for (size_t i = 0; i < candidates.size(); i++) {
    double dis = computeMinDisBetweenLines(candidates[i]);
    // LOG(INFO) << "DIS = " << dis;
    if (dis < min_dis_between_lines_) {
      final_candidates.push_back(candidates[i]);
    }
  }
  return final_candidates;
}

double AngleDectector::computeMinDisBetweenLines(AngleStructCandidate candidate)
{
  std::vector<std::pair<double, std::pair<Eigen::Vector2d, Eigen::Vector2d>>> dis_vec;
  Eigen::Vector2d first_line_1(candidate.first_line.first[0], candidate.first_line.first[1]);
  Eigen::Vector2d first_line_2(candidate.first_line.second[0], candidate.first_line.second[1]);
  Eigen::Vector2d second_line_1(candidate.second_line.first[0], candidate.second_line.first[1]);
  Eigen::Vector2d second_line_2(candidate.second_line.second[0], candidate.second_line.second[1]);
  std::vector<double> dis_v;
  double dis_1 = sqrt(
    (first_line_1[0] - second_line_1[0]) * (first_line_1[0] - second_line_1[0]) +
    (first_line_1[1] - second_line_1[1]) * (first_line_1[1] - second_line_1[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_1[0] - second_line_2[0]) * (first_line_1[0] - second_line_2[0]) +
    (first_line_1[1] - second_line_2[1]) * (first_line_1[1] - second_line_2[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_2[0] - second_line_1[0]) * (first_line_2[0] - second_line_1[0]) +
    (first_line_2[1] - second_line_1[1]) * (first_line_2[1] - second_line_1[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_2[0] - second_line_2[0]) * (first_line_2[0] - second_line_2[0]) +
    (first_line_2[1] - second_line_2[1]) * (first_line_2[1] - second_line_2[1]));
  dis_v.push_back(dis_1);
  double min_dis;
  min_dis = dis_v[0];
  for (size_t i = 1; i < dis_v.size(); i++) {
    double tmp_dis = dis_v[i];
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
    }
  }
  return min_dis;
}

bool AngleDectector::detectAngleStruct(
  const AngleStructCandidate & candidate,
  AngleStruct & angle_struct)
{
  double detect_angle = 0;
  if (candidate.first_line.first[0] == candidate.first_line.second[0]) {
    if (candidate.second_line.first[0] == candidate.second_line.second[0]) {
      // LOG(INFO) << "the two lines are parallel";
      return false;
    }
    double k_2 = (candidate.second_line.second[1] - candidate.second_line.first[1]) /
      (candidate.second_line.second[0] - candidate.second_line.first[0]);
    double b_2 = candidate.second_line.first[1] - k_2 * candidate.second_line.first[0];
    double x = candidate.first_line.first[0];
    double y = k_2 * x + b_2;
    // LOG(INFO) << "k_1 = " << k_2 << " b_1 = " << b_2 << " ";
    // LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      // LOG(INFO) << "can't detect angleStruct ";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  } else if (candidate.second_line.first[0] == candidate.second_line.second[0]) {
    double k_1 = (candidate.first_line.second[1] - candidate.first_line.first[1]) /
      (candidate.first_line.second[0] - candidate.first_line.first[0]);
    double b_1 = candidate.first_line.first[1] - k_1 * candidate.first_line.first[0];
    double x = candidate.second_line.first[0];
    double y = k_1 * x + b_1;
    // LOG(INFO) << "k_1 = " << k_1 << " b_1 = " << b_1 << " ";
    // LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      // LOG(INFO) << "can't detect angleStruct ";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  } else {
    double k_2 = (candidate.second_line.second[1] - candidate.second_line.first[1]) /
      (candidate.second_line.second[0] - candidate.second_line.first[0]);
    double b_2 = candidate.second_line.first[1] - k_2 * candidate.second_line.first[0];
    double k_1 = (candidate.first_line.second[1] - candidate.first_line.first[1]) /
      (candidate.first_line.second[0] - candidate.first_line.first[0]);
    double b_1 = candidate.first_line.first[1] - k_1 * candidate.first_line.first[0];
    if (k_1 == k_2) {
      // LOG(INFO) << "the two lines are parallel";
      return false;
    }
    double x = (b_2 - b_1) / (k_1 - k_2);
    double y = k_1 * x + b_1;
    // LOG(INFO) << "k_1 = " << k_1 << " b_1 = " << b_1 << " ";
    // LOG(INFO) << "k_2 = " << k_2 << " b_2 = " << b_2 << " ";
    // LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      // LOG(INFO) << "can't detect angleStruct";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  }
  return true;
}

bool AngleDectector::detectAngle(
  const Eigen::Vector2d & inter_point,
  const AngleStructCandidate & candidate,
  double & pose_angle,
  double & angle)
{
  Eigen::Vector2d v_1, v_2;
  Eigen::Vector2d point_center, point_center_1, point_center_2;
  // LOG(INFO) << "INTER_POINT = " << inter_point;
  double dis_1 = sqrt(
    (inter_point[0] - candidate.first_line.first[0]) *
    (inter_point[0] - candidate.first_line.first[0]) +
    (inter_point[1] - candidate.first_line.first[1]) *
    (inter_point[1] - candidate.first_line.first[1]));
  double dis_2 = sqrt(
    (inter_point[0] - candidate.first_line.second[0]) *
    (inter_point[0] - candidate.first_line.second[0]) +
    (inter_point[1] - candidate.first_line.second[1]) *
    (inter_point[1] - candidate.first_line.second[1]));
  if (dis_1 > dis_2) {
    if (dis_2 > min_dis_inter_lines_) {
      // LOG(INFO) << "the dis between inter point and lines are too larget. dis = " << dis_2;
      return false;
    }
    v_1 << candidate.first_line.first[0] - inter_point[0],
      candidate.first_line.first[1] - inter_point[1];
    point_center_1 << candidate.first_line.first[0],
      candidate.first_line.first[1];
  } else {
    if (dis_1 > min_dis_inter_lines_) {
      // LOG(INFO) << "the dis between inter point and lines are too larget. dis = " << dis_1;
      return false;
    }
    v_1 << candidate.first_line.second[0] - inter_point[0],
      candidate.first_line.second[1] - inter_point[1];
    point_center_1 << candidate.first_line.second[0],
      candidate.first_line.second[1];
  }

  dis_1 = sqrt(
    (inter_point[0] - candidate.second_line.first[0]) *
    (inter_point[0] - candidate.second_line.first[0]) +
    (inter_point[1] - candidate.second_line.first[1]) *
    (inter_point[1] - candidate.second_line.first[1]));
  dis_2 = sqrt(
    (inter_point[0] - candidate.second_line.second[0]) *
    (inter_point[0] - candidate.second_line.second[0]) +
    (inter_point[1] - candidate.second_line.second[1]) *
    (inter_point[1] - candidate.second_line.second[1]));
  if (dis_1 > dis_2) {
    if (dis_2 > min_dis_inter_lines_) {
      // LOG(INFO) << "the dis between inter point and lines are too larget dis = " << dis_2;
      // return false;
    }
    v_2 << candidate.second_line.first[0] - inter_point[0],
      candidate.second_line.first[1] - inter_point[1];
    point_center_2 << candidate.second_line.first[0],
      candidate.second_line.first[1];
  } else {
    if (dis_1 > min_dis_inter_lines_) {
      // LOG(INFO) << "the dis between inter point and lines are too larget dis = " << dis_1;
      return false;
    }
    v_2 << candidate.second_line.second[0] - inter_point[0],
      candidate.second_line.second[1] - inter_point[1];
    point_center_2 << candidate.second_line.second[0],
      candidate.second_line.second[1];
  }
  // LOG(INFO) << "V_1 = " << v_1[0] << " " << v_1[1];
  // LOG(INFO) << "V_2 = " << v_2[0] << " " << v_2[1];
  Eigen::Vector2d v_3 = v_1.normalized() + v_2.normalized();
  Eigen::Vector2d v_4(-v_3[0], -v_3[1]);
  angle = acos(v_1.normalized().transpose() * v_2.normalized());
  if (angle < 1.047197 || angle > 3.14) {  // 2.0943
    LOG(INFO) << "the angle is beyond the range the angle :" << angle;
    return false;
  }
  angle = 6.2831852 - angle;
  point_center << (point_center_1[0] + point_center_2[0]) / 2,
  (point_center_1[1] + point_center_2[1]) / 2;
  // LOG(INFO) << "V_3 = " << v_3[0] << " " << v_3[1];
  if (v_4[0] == 0 && v_4[1] > 0) {
    pose_angle = 3.1415926 / 2;
  } else if (v_4[0] == 0 && v_4[1] < 0) {
    pose_angle = -3.1415925 / 2;
  } else if (v_4[0] < 0 && v_4[1] > 0) {
    pose_angle = 3.1415926 / 2 + atan(fabs(v_4[0] / v_4[1]));
  } else if (v_4[0] < 0 && v_4[1] < 0) {
    pose_angle = -3.1415925 / 2 - atan(fabs(v_4[0] / v_4[1]));
  } else {
    pose_angle = atan(v_4[1] / v_4[0]);
  }
  if (point_center.norm() < inter_point.norm()) {
    angle = 6.2831852 - angle;
    pose_angle = pose_angle + 3.1415926;
    if (pose_angle > 3.1415926) {
      pose_angle = pose_angle - 6.1231852;
    }
  }
  return true;
}

}  // namespace laser_perception
}  // cartographer
