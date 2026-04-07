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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__RANGE_DATA_COLLATOR_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__RANGE_DATA_COLLATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "perception_2d/sensor/timed_point_cloud_data.hpp"

namespace perception_2d
{

// Synchronizes TimedPointCloudData from different sensors. Input needs only be
// monotonous in 'TimedPointCloudData::time', output is monotonous in per-point
// timing. Up to one message per sensor is buffered, so a delay of the period of
// the slowest sensor may be introduced, which can be alleviated by passing
// subdivisions.
class RangeDataCollator
{
public:
  explicit RangeDataCollator(
    const std::vector<std::string> & expected_range_sensor_ids)
  : expected_sensor_ids_(expected_range_sensor_ids.begin(),
      expected_range_sensor_ids.end()) {}

  sensor::TimedPointCloudOriginData AddRangeData(
    const std::string & sensor_id,
    const sensor::PointCloudDataWithIntensities & timed_point_cloud_data);
  // const sensor::TimedPointCloudData& timed_point_cloud_data);

private:
  sensor::TimedPointCloudOriginData CropAndMerge();

  const std::set<std::string> expected_sensor_ids_;
  // Store at most one message for each sensor.
  // std::map<std::string, sensor::TimedPointCloudData> id_to_pending_data_;
  std::map<std::string, sensor::PointCloudDataWithIntensities> id_to_pending_data_;
  common::Time current_start_ = common::Time::min();
  common::Time current_end_ = common::Time::min();
};

}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__RANGE_DATA_COLLATOR_HPP_
