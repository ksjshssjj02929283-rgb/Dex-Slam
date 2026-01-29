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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__UTIL_H_
#define CARTOGRAPHER__LASER_PERCEPTION__UTIL_H_
#include <iostream>
#include <fstream>
// #include <fcntl.h>
#include <cstdio>
#include <vector>
#include <mutex>
#include <utility>
#include "cartographer/laser_perception/point.h"
#include "opencv2/opencv.hpp"

void swap(int * a, int * b);

void perm(int list[], int k, int m, std::vector<int> & combination);

void Combination(
  int nLen, int m, int out[], int outLen, bool order,
  std::vector<int> & combination);

std::vector<int> getCombination(int nLen, int m, bool order);

// Point transformPoint(const Point& point, const tf::StampedTransform& transform);

void kfMeasure(cv::KalmanFilter & KF, const cv::Point3d & position);

void kfPredict(cv::KalmanFilter & KF, cv::Point3d & predicPoint);

#endif  // CARTOGRAPHER__LASER_PERCEPTION__UTIL_H_
