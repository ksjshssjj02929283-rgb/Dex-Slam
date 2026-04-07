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
#include <math.h>
#include <utility>
#include <vector>
#include "perception_2d/laser_perception_2d/math_tool.hpp"

namespace perception_2d
{
// 计算两点间的距离
double ppDistance(cv::Point2d p1, cv::Point2d p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}

// 已知两边，求点到弦长的距离
double calculateIntersectionLength(
  double side1, double side2,
  double angle1, double angle2)
{
  double side3 = chordLength(side1, side2, angle1);
  double tan_a = 2 * side1 * side2 * sin(angle1) / (side1 * side1 + side3 * side3 - side2 * side2);
  double length = side1 / (sin(angle2) + cos(angle2) / tan_a);
  return length;
}

// 弦长公式
double chordLength(double radius1, double radius2, double angle)
{
  double length = sqrt(radius1 * radius1 + radius2 * radius2 - 2 * radius1 * radius2 * cos(angle));
  return length;
}

double chordLength_point(Eigen::Vector3f point_1, Eigen::Vector3f point_2)
{
  double length = sqrt(
    (point_1[0] - point_2[0]) * (point_1[0] - point_2[0]) +
    (point_1[1] - point_2[1]) * (point_1[1] - point_2[1]));
  return length;
}

// 获得点到直线段的最近点和最近距离
double pointToSegDist(
  cv::Point2d point, cv::Point2d point1, cv::Point2d point2,
  cv::Point2d & point_nearest)
{
  double cross = (point2.x - point1.x) * (point.x - point1.x) + (point2.y - point1.y) *
    (point.y - point1.y);
  point_nearest = point1;
  if (cross <= 0) {
    return sqrt(
      (point.x - point1.x) * (point.x - point1.x) + (point.y - point1.y) *
      (point.y - point1.y));
  }

  double d2 = (point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) *
    (point2.y - point1.y);
  point_nearest = point2;
  if (cross >= d2) {
    return sqrt(
      (point.x - point2.x) * (point.x - point2.x) + (point.y - point2.y) *
      (point.y - point2.y));
  }

  double r = cross / d2;
  cv::Point2d point3;
  point3.x = point1.x + (point2.x - point1.x) * r;
  point3.y = point1.y + (point2.y - point1.y) * r;

  point_nearest = point3;
  return sqrt(
    (point.x - point3.x) * (point.x - point3.x) + (point.y - point3.y) *
    (point.y - point3.y));
}

double pointToLine(
  cv::Point2d point, cv::Point2d point1, cv::Point2d point2,
  cv::Point2d & point_nearest, bool & in_line)
{
  in_line = true;
  double cross = (point2.x - point1.x) * (point.x - point1.x) + (point2.y - point1.y) *
    (point.y - point1.y);
  point_nearest = point1;
  if (cross <= 0) {in_line = false;}
  double d2 = (point2.x - point1.x) * (point2.x - point1.x) + (point2.y - point1.y) *
    (point2.y - point1.y);
  point_nearest = point2;
  if (cross >= d2) {in_line = false;}
  double r = cross / d2;
  cv::Point2d point3;
  point3.x = point1.x + (point2.x - point1.x) * r;
  point3.y = point1.y + (point2.y - point1.y) * r;

  point_nearest = point3;
  return sqrt(
    (point.x - point3.x) * (point.x - point3.x) + (point.y - point3.y) *
    (point.y - point3.y));
}

// 获得点到圆弧的最近点和最近距离
double pointToCircleDist(
  cv::Point2d point, cv::Point2d center, double radian, double startRadius,
  double endRadius, cv::Point2d & point_nearest)
{
  cv::Point2d startPoint = cv::Point2d(
    center.x + radian * cos(startRadius), center.y + radian * sin(
      startRadius));
  cv::Point2d endPoint =
    cv::Point2d(center.x + radian * cos(endRadius), center.y + radian * sin(endRadius));

  double pc_dis = ppDistance(point, center) - radian;
  double pp_dis;
  if (ppDistance(point, startPoint) < ppDistance(point, endPoint)) {
    point_nearest = startPoint;
    pp_dis = ppDistance(point, startPoint);
  } else {
    point_nearest = endPoint;
    pp_dis = ppDistance(point, endPoint);
  }
  double theta = atan2(point.y - center.y, point.x - center.x);

  double radius1 = endRadius - startRadius;
  double radius2 = theta - startRadius;

  radius1 = (static_cast<int>(radius1 * 1000) % 6280) / 1000;
  radius2 = (static_cast<int>(radius2 * 1000) % 6280) / 1000;

  radius1 = (radius1 < 0) ? radius1 + 6.24 : radius1;
  radius2 = (radius2 < 0) ? radius2 + 6.24 : radius1;

  if (radius1 > radius2) {
    point_nearest = cv::Point2d(center.x + cos(theta), center.y + sin(theta));
    return pc_dis;
  } else {
    return pp_dis;
  }
}

// 获取点集中心
cv::Point2d getPointsCenter(const std::vector<cv::Point2d> & points)
{
  cv::Point2d midPoint;
  double x = 0;
  double y = 0;
  for (int j = 0; j < static_cast<int>(points.size()); j++) {
    x += points[j].x;
    y += points[j].y;
  }
  midPoint.x = x / points.size();
  midPoint.y = y / points.size();
  return midPoint;
}

// 获得变换矩阵
void getRTMatrixSVD(
  const cv::Point2d * a,
  const cv::Point2d * b,
  int count,
  cv::Mat & r,
  cv::Mat & t)
{
  double H[4] = {0.f, 0.f, 0.f, 0.f};
  cv::Mat h = cv::Mat(2, 2, CV_64F, H);
  double U[4] = {0.f, 0.f, 0.f, 0.f};
  cv::Mat u = cv::Mat(2, 2, CV_64F, U);
  double W[4] = {0.f, 0.f, 0.f, 0.f};
  cv::Mat w = cv::Mat(2, 2, CV_64F, W);
  double V[4] = {0.f, 0.f, 0.f, 0.f};
  cv::Mat v = cv::Mat(2, 2, CV_64F, V);
  cv::Point2d mean_a = cv::Point2d(0.f, 0.f);
  cv::Point2d mean_b = cv::Point2d(0.f, 0.f);
  for (int i = 0; i < count; i++) {
    mean_a.x += a[i].x;
    mean_a.y += a[i].y;
    mean_b.x += b[i].x;
    mean_b.y += b[i].y;
  }
  // printf("mean_a_x:%f,mean_a_y:%f,mean_b_x:%f,mean_b_y:%f\r\n",mean_a.x,mean_a.y,mean_b.x,mean_b.y);
  mean_a.x /= static_cast<double>(count);
  mean_a.y /= static_cast<double>(count);
  mean_b.x /= static_cast<double>(count);
  mean_b.y /= static_cast<double>(count);
  for (int i = 0; i < count; i++) {
    double AX = (a[i].x - mean_a.x);
    double AY = (a[i].y - mean_a.y);
    double BX = (b[i].x - mean_b.x);
    double BY = (b[i].y - mean_b.y);
    H[0] += AX * BX;
    H[1] += AX * BY;
    H[2] += AY * BX;
    H[3] += AY * BY;
  }
  cv::SVD::compute(h, w, u, v);

  r.at<double>(0, 0) = V[0] * U[0] + V[2] * U[1];
  r.at<double>(0, 1) = V[0] * U[2] + V[2] * U[3];
  r.at<double>(1, 0) = V[1] * U[0] + V[3] * U[1];
  r.at<double>(1, 1) = V[1] * U[2] + V[3] * U[3];

  // special reflection case
  if (cv::determinant(r) < 0) {
    r.at<double>(0, 0) = V[0] * U[0] - V[2] * U[1];
    r.at<double>(0, 1) = V[0] * U[2] - V[2] * U[3];
    r.at<double>(1, 0) = V[1] * U[0] - V[3] * U[1];
    r.at<double>(1, 1) = V[1] * U[2] - V[3] * U[3];
  }

  // T = -R*meanA+meanB
  t.at<double>(0, 0) =
    -r.at<double>(0, 0) * mean_a.x - r.at<double>(0, 1) * mean_a.y + mean_b.x;
  t.at<double>(1, 0) =
    -r.at<double>(1, 0) * mean_a.x - r.at<double>(1, 1) * mean_a.y + mean_b.y;
}

// 最近点迭代
double icp(
  const std::vector<cv::Point2d> source,
  const std::vector<Line> lines,
  cv::Mat & r_final,
  cv::Mat & t_final,
  cv::TermCriteria criteria)
{
  double prev_err = FLT_MAX;
  double err = FLT_MAX;
  std::vector<cv::Point2d> input_point(source);
  std::vector<cv::Point2d> match_point(source);
  // LOG(INFO) << "maxCount = " << criteria.maxCount;
  // LOG(INFO) << "source size = " << source.size();
  for (int k = 0; k < criteria.maxCount; k++) {
    double R[4];
    cv::Mat r = cv::Mat(2, 2, CV_64F, R);
    double T[2];
    cv::Mat t = cv::Mat(2, 1, CV_64F, T);
    err = 0.0;
    for (size_t i = 0; i < input_point.size(); i++) {
      // getClosePoint(input_point[i], match_point[i]);
      double nearestDist = DBL_MAX;
      double tmpDist = DBL_MAX;
      cv::Point2d tmpPoint;
      cv::Point2d nearestPoint;
      for (auto line : lines) {
        // getClosePoint(input_point[i], match_point[i]);
        if (line.type == "line") {
          tmpDist = pointToSegDist(input_point[i], line.line.first, line.line.second, tmpPoint);
        } else if (line.type == "circle") {
          tmpDist = pointToCircleDist(
            input_point[i], line.line.first,
            line.line.second.x, -line.line.second.y / 2, line.line.second.y / 2, tmpPoint);
        }
        if (nearestDist > tmpDist) {
          nearestDist = tmpDist;
          nearestPoint = tmpPoint;
        }
      }
      match_point[i] = nearestPoint;
    }

    getRTMatrixSVD(&input_point[0], &match_point[0], input_point.size(), r, t);
    for (size_t i = 0; i < input_point.size(); i++) {
      double x = input_point[i].x;
      double y = input_point[i].y;
      double X = (R[0] * x + R[1] * y + T[0]);
      double Y = (R[2] * x + R[3] * y + T[1]);
      input_point[i].x = X;
      input_point[i].y = Y;
      err += ppDistance(input_point[i], match_point[i]);
    }
    if (fabs(err - prev_err) < criteria.epsilon) {
      break;
    } else {
      prev_err = err;
    }
  }
  getRTMatrixSVD(
    &input_point[0],
    &source[0],
    static_cast<int>(input_point.size()),
    r_final,
    t_final);
  double averageErr = err / input_point.size();
  return averageErr;
}

bool getPointSetCenter(
  std::vector<std::pair<int, std::vector<cv::Point2d>>> pointSetList,
  std::vector<cv::Point2d> & pointSetCenter)
{
  pointSetCenter.clear();
  for (int i = 0; i < static_cast<int>(pointSetList.size()); i++) {
    if (pointSetList[i].first != 2) {
      continue;
    }
    double x = 0;
    double y = 0;
    cv::Point2d midPoint;
    for (int j = 0; j < static_cast<int>(pointSetList[i].second.size()); j++) {
      x += pointSetList[i].second[j].x;
      y += pointSetList[i].second[j].y;
    }
    midPoint.x = x / pointSetList[i].second.size();
    midPoint.y = y / pointSetList[i].second.size();
    pointSetCenter.push_back(midPoint);
  }
  return true;
}

// 最近点迭代
double icp_original(
  const std::vector<cv::Point2d> source,
  const std::vector<Line> lines,
  cv::Mat & r_final,
  cv::Mat & t_final,
  cv::TermCriteria criteria,
  std::vector<cv::Point2d> & vis_input_point)
{
  double prev_err = FLT_MAX;
  double err = 0.0;
  std::vector<cv::Point2d> input_point(source);
  std::vector<cv::Point2d> match_point(source);
  for (int k = 0; k < criteria.maxCount; k++) {
    double R[4];
    cv::Mat r = cv::Mat(2, 2, CV_64F, R);
    double T[2];
    cv::Mat t = cv::Mat(2, 1, CV_64F, T);
    err = 0.0;
    for (size_t i = 0; i < input_point.size(); i++) {
      // getClosePoint(input_point[i], match_point[i]);
      double nearestDist = DBL_MAX;
      double tmpDist = DBL_MAX;
      cv::Point2d tmpPoint;
      cv::Point2d nearestPoint;
      for (auto line : lines) {
        // getClosePoint(input_point[i], match_point[i]);
        if (line.type == "line") {
          tmpDist = pointToSegDist(input_point[i], line.line.first, line.line.second, tmpPoint);
        } else if (line.type == "circle") {
          tmpDist = pointToCircleDist(
            input_point[i], line.line.first,
            line.line.second.x, -line.line.second.y / 2, line.line.second.y / 2, tmpPoint);
        }
        if (nearestDist > tmpDist) {
          nearestDist = tmpDist;
          nearestPoint = tmpPoint;
        }
      }
      match_point[i] = nearestPoint;
      vis_input_point.push_back(nearestPoint);
    }

    getRTMatrixSVD(&input_point[0], &match_point[0], input_point.size(), r, t);
    for (size_t i = 0; i < input_point.size(); i++) {
      double x = input_point[i].x;
      double y = input_point[i].y;
      double X = (R[0] * x + R[1] * y + T[0]);
      double Y = (R[2] * x + R[3] * y + T[1]);
      input_point[i].x = X;
      input_point[i].y = Y;
      err += ppDistance(input_point[i], match_point[i]);
    }
    if (fabs(err - prev_err) < criteria.epsilon) {
      break;
    } else {
      prev_err = err;
    }
  }
  getRTMatrixSVD(
    &input_point[0],
    &source[0],
    static_cast<int>(input_point.size()),
    r_final,
    t_final);
  double averageErr = err / input_point.size();
  return averageErr;
}

}  // namespace perception_2d
