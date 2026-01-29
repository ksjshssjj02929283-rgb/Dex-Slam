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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__MATH_TOOL_H_
#define CARTOGRAPHER__LASER_PERCEPTION__MATH_TOOL_H_
#include <vector>
#include "opencv2/opencv.hpp"
#include "cartographer/laser_perception/model_manager.h"
namespace cartographer
{
namespace laser_perception
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
  cv::TermCriteria criteria,
  std::vector<cv::Point2d> & vis_input_point,
  double shape_match_degree);
double icp_tmp(
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
double icp_test(
  const std::vector<cv::Point2d> source,
  const std::vector<Line> lines,
  cv::Mat & r_final,
  cv::Mat & t_final,
  cv::TermCriteria criteria,
  std::vector<cv::Point2d> & vis_input_point,
  double shape_match_close_degree);
void getRTMatrixSVD(
  const cv::Point2d * a,
  const cv::Point2d * b,
  int count,
  cv::Mat & r,
  cv::Mat & t);
cv::Point2d getPointsCenter(const std::vector<cv::Point2d> & points);

}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__MATH_TOOL_H_
