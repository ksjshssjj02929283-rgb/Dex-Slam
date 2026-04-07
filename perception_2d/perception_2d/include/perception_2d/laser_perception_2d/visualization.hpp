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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__VISUALIZATION_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__VISUALIZATION_HPP_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <utility>

namespace perception_2d
{
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

}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__VISUALIZATION_HPP_
