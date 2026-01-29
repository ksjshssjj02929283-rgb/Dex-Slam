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

// #include <tf/transform_datatypes.h>
#include "cartographer/laser_perception/util.h"
#include <utility>
#include <vector>

void swap(int * a, int * b)
{
  int m;
  m = *a;
  *a = *b;
  *b = m;
}

void perm(int list[], int k, int m, std::vector<int> & combination)
{
  int i;
  if (k > m) {
    for (i = 0; i <= m; i++) {
      printf("%d ", list[i]);
      combination.push_back(list[i]);
    }
    printf("\n");
  } else {
    for (i = k; i <= m; i++) {
      swap(&list[k], &list[i]);
      perm(list, k + 1, m, combination);
      swap(&list[k], &list[i]);
    }
  }
}

void Combination(
  int nLen, int m, int out[], int outLen, bool order,
  std::vector<int> & combination)
{
  if (m == 0) {
    if (!order) {
      for (int j = 0; j < outLen; j++) {
        combination.push_back(out[j]);
        // std::cout << out[j] << "\t";
      }
      // std::cout << std::endl;
    } else {
      std::cout << std::endl;
      perm(out, 0, outLen - 1, combination);
    }

    return;
  }

  for (int i = nLen; i >= m; --i) {  // 从后往前依次选定一个
    out[m - 1] = i - 1;  // 选定一个后
    Combination(i - 1, m - 1, out, outLen, order, combination);  // 从前i-1个里面选取m-1个进行递归
  }
}

std::vector<int> getCombination(int nLen, int m, bool order)
{
  std::vector<int> combination;
  if (m > nLen) {
    return combination;
  }

  int * out = new int[m];
  Combination(nLen, m, out, m, order, combination);
  delete[] out;
  return combination;
}

// Point transformPoint(const Point& point, const tf::StampedTransform& transform) {
//   tf::Vector3 v(point.x, point.y, 0);
//   v = transform * v;

//   return {v.x(), v.y()};
// }

void kfMeasure(cv::KalmanFilter & KF, const cv::Point3d & position)
{
  cv::Mat measurement = cv::Mat(3, 1, CV_32F);
  measurement.at<float>(0) = static_cast<float>(position.x);
  measurement.at<float>(1) = static_cast<float>(position.y);
  measurement.at<float>(2) = static_cast<float>(position.z);
  KF.correct(measurement);
}

void kfPredict(cv::KalmanFilter & KF, cv::Point3d & predicPoint)
{
  cv::Mat prediction = KF.predict();
  predicPoint.x = prediction.at<float>(0);
  predicPoint.y = prediction.at<float>(1);
  predicPoint.z = prediction.at<float>(2);
}
