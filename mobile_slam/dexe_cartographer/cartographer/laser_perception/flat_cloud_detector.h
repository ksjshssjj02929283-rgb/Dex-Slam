/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER__LASER_PERCEPTION__FLAT_CLOUD_DETECTOR_H_
#define CARTOGRAPHER__LASER_PERCEPTION__FLAT_CLOUD_DETECTOR_H_

#include <cmath>
#include <algorithm>
#include <vector>

#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/sync_point_cloud.h"

namespace cartographer {
namespace laser_perception {

class FlatCloudDetector
{
public:

void DetectFlatCloud(const sensor::PointCloudWithOrigin& cloud_in,
                     sensor::PointCloud& flat_cloud,
                     sensor::PointCloud& less_flat_cloud)
{

  std::vector<Eigen::Vector3f> sync_points_base_link;
  int cloudSize = cloud_in.sync_points_base_link.size();

  const auto& point_cloud = cloud_in.sync_points_base_link;

  float startOri = - std::atan2(point_cloud[0].y(), point_cloud[0].x());
  float endOri = - std::atan2(point_cloud[cloudSize - 1].y(),
                             point_cloud[cloudSize - 1].x())
                 + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  int count = cloudSize;

  int scanStartInd = 5;
  int scanEndInd = point_cloud.size() - 6;

  std::vector<float> cloudCurvature(cloudSize);
  std::vector<int> cloudSortInd(cloudSize);
  std::vector<int> cloudNeighborPicked(cloudSize);
  std::vector<int> cloudLabel(cloudSize);
  for (int i = 5; i < cloudSize - 5; i++)
  { 
    float diffX = point_cloud[i - 5].x() + point_cloud[i - 4].x() + point_cloud[i - 3].x() + point_cloud[i - 2].x() + point_cloud[i - 1].x() - 10 * point_cloud[i].x() + point_cloud[i + 1].x() + point_cloud[i + 2].x() + point_cloud[i + 3].x() + point_cloud[i + 4].x() + point_cloud[i + 5].x();
    float diffY = point_cloud[i - 5].y() + point_cloud[i - 4].y() + point_cloud[i - 3].y() + point_cloud[i - 2].y() + point_cloud[i - 1].y() - 10 * point_cloud[i].y() + point_cloud[i + 1].y() + point_cloud[i + 2].y() + point_cloud[i + 3].y() + point_cloud[i + 4].y() + point_cloud[i + 5].y();

    cloudCurvature[i] = diffX * diffX + diffY * diffY;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;
  }

  auto comp = [&cloudCurvature](int i,int j) {  
      return cloudCurvature[i] < cloudCurvature[j];  
  };

  for (int j = 0; j < 6; j++) {
    int sp = scanStartInd + (scanEndInd - scanStartInd) * j / 6; 
    int ep = scanStartInd + (scanEndInd - scanStartInd) * (j + 1) / 6 - 1;

    std::sort(cloudSortInd.begin() + sp, cloudSortInd.begin() + ep + 1, comp);

    int smallestPickedNum = 0;
    for (int k = sp; k <= ep; k++) {
      int ind = cloudSortInd[k];

      if (cloudNeighborPicked[ind] == 0 &&
          cloudCurvature[ind] < 0.1)
      {
        cloudLabel[ind] = -1;
        sensor::PointCloud::PointType point;
        point.position = point_cloud[ind];
        flat_cloud.push_back(point);
        smallestPickedNum++;
        // if (smallestPickedNum >= 4) {
        //   break;
        // }

        cloudNeighborPicked[ind] = 1;
        for (int l = 1; l <= 5; l++) {
          float diffX = point_cloud[ind + l].x() - point_cloud[ind + l - 1].x();
          float diffY = point_cloud[ind + l].y() - point_cloud[ind + l - 1].y();
          if (diffX * diffX + diffY * diffY > 0.05) {
            break;
          }
          cloudNeighborPicked[ind + l] = 1;
        }

        for (int l = -1; l >= -5; l--) {
          float diffX = point_cloud[ind + l].x() - point_cloud[ind + l + 1].x();
          float diffY = point_cloud[ind + l].y() - point_cloud[ind + l + 1].y();
          if (diffX * diffX + diffY * diffY > 0.05) {
            break;
          }
          cloudNeighborPicked[ind + l] = 1;
        }
      }
    }

    for (int k = sp; k <= ep; k++) {
      if (cloudCurvature[k] < 0.1) {
        sensor::PointCloud::PointType point;
        point.position = point_cloud[k];
        less_flat_cloud.push_back(point);
      }
    }
  }
}


};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__FLAT_CLOUD_DETECTOR_H_
