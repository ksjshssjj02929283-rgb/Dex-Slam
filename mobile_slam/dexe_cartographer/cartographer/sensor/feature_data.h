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

#ifndef CARTOGRAPHER__SENSOR__FEATURE_DATAsrc
#define CARTOGRAPHER__SENSOR__FEATURE_DATA_H_
#include <mutex>
#include <vector>
#include <memory>

#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/line_data.h"


namespace cartographer
{

enum FeatureTpye
{
  LASER_LINE = 0,
  VIS_LINE = 1,
  REFLECTOR = 2,
  QR_CODE = 3,
  VIS_TAG = 4,
  END = 5,
};
namespace sensor
{
std::vector<sensor::LineData> SyncLaneData(
  const OdometryData & odom,
  const std::vector<sensor::LaneDataWithOdom> & lane_data);
std::vector<sensor::LandmarkPointData> SyncLandmarkData(
  const OdometryData & odom,
  const std::vector<sensor::LandmarkPointDataWithOdom> & landmark_data);

struct FeatureData
{
  std::vector<sensor::LineData> line_data;  // 雷达线特征
  std::vector<sensor::LandmarkPointData> reflector_data;

  bool HaveData() const
  {
    return line_data.size() != 0 || reflector_data.size() != 0;
  }

  void Clear()
  {
    line_data.clear();
    reflector_data.clear();
  }
};

struct UnhandleFeatureData
{
  std::vector<sensor::LaneDataWithOdom> unhandle_laser_line_data;
  std::vector<sensor::LandmarkPointDataWithOdom> reflector_data;
};

class ActiveFeatureData
{
public:
  ActiveFeatureData()
  {
  }
  ~ActiveFeatureData() {}
  void Clear()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    unhandle_laser_lane_data_.clear();
    reflector_data_.clear();
  }

  void PushBackLaserLane(const sensor::LaneDataWithOdom & data)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    unhandle_laser_lane_data_.push_back(data);
    if (unhandle_laser_lane_data_.size() > 100) {
      unhandle_laser_lane_data_.erase(
        unhandle_laser_lane_data_.begin(),
        unhandle_laser_lane_data_.begin() +
        static_cast<int>(unhandle_laser_lane_data_.size() / 2));
    }
  }

  void PushBackReflectorData(
    const sensor::LandmarkPointDataWithOdom & data)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    reflector_data_.push_back(data);
    if (reflector_data_.size() > 100) {
      reflector_data_.erase(
        reflector_data_.begin(),
        reflector_data_.begin() + 50);
    }
  }

  std::vector<sensor::LaneDataWithOdom> GetLaserLaneData()
  {
    return unhandle_laser_lane_data_;
  }

  std::vector<sensor::LandmarkPointDataWithOdom> GetReflectorData()
  {
    return reflector_data_;
  }

private:
  // std::
  std::mutex mutex_;
  std::vector<sensor::LaneDataWithOdom> unhandle_laser_lane_data_;
  std::vector<sensor::LandmarkPointDataWithOdom> reflector_data_;
};

}  // namespace sensor
}  // namespace cartographer


#endif  // CARTOGRAPHER__SENSOR__FEATURE_DATA_H_
