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

#include "perception_2d/laser_perception_2d/range_data_collator.hpp"

#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include "glog/logging.h"

namespace perception_2d
{

sensor::TimedPointCloudOriginData RangeDataCollator::AddRangeData(
  const std::string & sensor_id,
  const sensor::PointCloudDataWithIntensities & timed_point_cloud_data)
{
  // const sensor::TimedPointCloudData& timed_point_cloud_data) {
  CHECK_NE(expected_sensor_ids_.count(sensor_id), 0);
  // TODO(gaschler): These two cases can probably be one.
  if (id_to_pending_data_.count(sensor_id) != 0) {
    current_start_ = current_end_;
    // When we have two messages of the same sensor, move forward the older of
    // the two (do not send out current).
    current_end_ = id_to_pending_data_.at(sensor_id).points.time;
    auto result = CropAndMerge();
    id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
    return result;
  }
  id_to_pending_data_.emplace(sensor_id, timed_point_cloud_data);
  if (expected_sensor_ids_.size() != id_to_pending_data_.size()) {
    return {};
  }
  current_start_ = current_end_;
  // We have messages from all sensors, move forward to oldest.
  common::Time oldest_timestamp = common::Time::max();
  for (const auto & pair : id_to_pending_data_) {
    oldest_timestamp = std::min(oldest_timestamp, pair.second.points.time);
  }
  current_end_ = oldest_timestamp;
  return CropAndMerge();
}

sensor::TimedPointCloudOriginData RangeDataCollator::CropAndMerge()
{
  sensor::TimedPointCloudOriginData result{current_end_, {}, {}};
  bool warned_for_dropped_points = false;
  for (auto it = id_to_pending_data_.begin();
    it != id_to_pending_data_.end(); )
  {
    sensor::PointCloudDataWithIntensities & data = it->second;
    sensor::TimedPointCloud & ranges = it->second.points.ranges;
    std::vector<float> intensities = it->second.intensities;

    auto overlap_begin = ranges.begin();
    auto intensities_begin = intensities.begin();
    while (overlap_begin < ranges.end() &&
      data.points.time + common::FromSeconds((*overlap_begin).time) <
      current_start_)
    {
      ++overlap_begin;
      ++intensities_begin;
    }
    auto overlap_end = overlap_begin;
    auto intensities_end = intensities_begin;
    while (overlap_end < ranges.end() &&
      data.points.time + common::FromSeconds((*overlap_end).time) <=
      current_end_)
    {
      ++overlap_end;
      ++intensities_end;
    }
    if (ranges.begin() < overlap_begin && !warned_for_dropped_points) {
      LOG(WARNING) << "Dropped " << std::distance(ranges.begin(), overlap_begin) <<
        " earlier points.";
      warned_for_dropped_points = true;
    }

    // Copy overlapping range.
    if (overlap_begin < overlap_end) {
      std::size_t origin_index = result.origins.size();
      result.origins.push_back(data.points.origin);
      const float time_correction =
        static_cast<float>(common::ToSeconds(data.points.time - current_end_));
      auto intensities_it = intensities_begin;
      for (auto overlap_it = overlap_begin; overlap_it != overlap_end;
        ++overlap_it)
      {
        sensor::TimedPointCloudOriginData::RangeMeasurement point{*overlap_it,
          *intensities_it,
          origin_index};
        intensities_it++;
        // current_end_ + point_time[3]_after == in_timestamp +
        // point_time[3]_before
        point.point_time.time += time_correction;
        result.ranges.push_back(point);
      }
    }

    // Drop buffered points until overlap_end.
    if (overlap_end == ranges.end()) {
      it = id_to_pending_data_.erase(it);
    } else if (overlap_end == ranges.begin()) {
      ++it;
    } else {
      data = sensor::PointCloudDataWithIntensities{
        sensor::TimedPointCloudData{
          data.points.time, data.points.origin,
          sensor::TimedPointCloud(overlap_end, ranges.end())},
        std::vector<float>(intensities_end, intensities.end())};
      ++it;
    }
  }

  std::sort(
    result.ranges.begin(), result.ranges.end(),
    [](const sensor::TimedPointCloudOriginData::RangeMeasurement & a,
    const sensor::TimedPointCloudOriginData::RangeMeasurement & b) {
      return a.point_time.time < b.point_time.time;
    });
  return result;
}

}  // namespace perception_2d
