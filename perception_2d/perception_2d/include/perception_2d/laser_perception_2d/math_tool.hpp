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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__MATH_TOOL_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__MATH_TOOL_HPP_
#include <opencv2/opencv.hpp>
#include <vector>
#include "perception_2d/laser_perception_2d/model_manager.hpp"


namespace perception_2d
{
double ppDistance(cv::Point2d p1, cv::Point2d p2);
double calculateIntersectionLength(
  double side1, double side2,
  double angle1, double angle2);
double chordLength(double radius1, double radius2, double angle);
double chordLength_point(Eigen::Vector3f point_1, Eigen::Vector3f point_2);
double pointToSegDist(
  cv::Point2d point, cv::Point2d point1, cv::Point2d point2,
  cv::Point2d & point_nearest);
double pointToCircleDist(
  cv::Point2d point, cv::Point2d center, double radian, double startRadius,
  double endRadius, cv::Point2d & point_nearest);
double pointToLine(
  cv::Point2d point, cv::Point2d point1, cv::Point2d point2,
  cv::Point2d & point_nearest, bool & in_line);
double icp(
  const std::vector<cv::Point2d> source,
  const std::vector<Line> lines,
  cv::Mat & r_final,
  cv::Mat & t_final,
  cv::TermCriteria criteria);
double icp_original(
  const std::vector<cv::Point2d> source,
  const std::vector<Line> lines,
  cv::Mat & r_final,
  cv::Mat & t_final,
  cv::TermCriteria criterian,
  std::vector<cv::Point2d> & vis_input_point);
void getRTMatrixSVD(
  const cv::Point2d * a,
  const cv::Point2d * b,
  int count,
  cv::Mat & r,
  cv::Mat & t);
cv::Point2d getPointsCenter(const std::vector<cv::Point2d> & points);

}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__MATH_TOOL_HPP_
