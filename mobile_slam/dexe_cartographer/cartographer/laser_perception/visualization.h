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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__VISUALIZATION_H_
#define CARTOGRAPHER__LASER_PERCEPTION__VISUALIZATION_H_
#include <utility>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"

enum Color
{
  red = 0,
  green = 1,
  blue = 2
};

struct LinesAndTextInfo
{
  std::vector<std::pair<cv::Point2d, cv::Point2d>> lines;
  std::string text;
  cv::Point2d text_position;
  Color color;
  std::string frame;
};

struct PointsAndTextInfo
{
  std::vector<cv::Point2d> points;
  std::string text;
  cv::Point2d text_position;
  Color color;
  std::string frame;
};

class Visualization
{
public:
  Visualization();
  void linesAndTextVisual(LinesAndTextInfo lines_text_info);
  void pointsAndTextVisual(PointsAndTextInfo points_text_info);
  void pointsVisual(PointsAndTextInfo points_info);

private:
//  ros::Publisher lines_marker_pub_;
//  ros::Publisher points_marker_pub_;
//  ros::Publisher reflector_pub_;
//  ros::NodeHandle n_;
};


#endif  // CARTOGRAPHER__LASER_PERCEPTION__VISUALIZATION_H_
