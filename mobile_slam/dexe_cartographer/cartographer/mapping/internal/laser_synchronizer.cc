/*
 * Copyright 2025 The Cartographer Authors
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

#include "cartographer/mapping/internal/laser_synchronizer.h"

#include <algorithm>
#include <cmath>

namespace cartographer {
namespace mapping {

LaserSynchronizer::LaserSynchronizer(
    const std::vector<std::string>& expected_sensor_ids,
    double max_time_diff_seconds,
    size_t buffer_size)
    : expected_sensor_ids_(expected_sensor_ids),
      max_time_diff_seconds_(max_time_diff_seconds),
      buffer_size_(buffer_size) {
  
  // 初始化每个传感器的缓冲区
  for (const std::string& sensor_id : expected_sensor_ids_) {
    sensor_buffers_[sensor_id] = std::deque<sensor::TimedPointCloudData>();
    LOG(INFO) << "Initialized buffer for laser sensor: " << sensor_id;
  }
}

bool LaserSynchronizer::AddLaserData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) {
  
  // 检查sensor_id是否在预期列表中
  if (std::find(expected_sensor_ids_.begin(), expected_sensor_ids_.end(), sensor_id) 
      == expected_sensor_ids_.end()) {
    LOG(WARNING) << "Unexpected sensor ID: " << sensor_id;
    return false;
  }

  // 添加数据到对应的缓冲区
  auto& buffer = sensor_buffers_[sensor_id];
  buffer.push_back(timed_point_cloud_data);

  // 按时间排序（确保时间顺序）
  std::sort(buffer.begin(), buffer.end(),
            [](const sensor::TimedPointCloudData& a, const sensor::TimedPointCloudData& b) {
              return a.time < b.time;
            });

  // 限制缓冲区大小
  while (buffer.size() > buffer_size_) {
    buffer.pop_front();
  }

  // 更新最新同步时间
  latest_sync_time_ = timed_point_cloud_data.time;

  // 移除过期数据
  const common::Time cutoff_time = 
      timed_point_cloud_data.time - common::FromSeconds(max_time_diff_seconds_ * 10);
  RemoveOldData(cutoff_time);

  return true;
}

LaserSynchronizer::SynchronizedLaserData LaserSynchronizer::GetSynchronizedData() {
  SynchronizedLaserData result;
  
  if (!CanSynchronize()) {
    return result;  // is_valid = false
  }

  // 找到所有传感器中最新的时间作为同步时间
  common::Time sync_time = common::Time::min();
  for (const std::string& sensor_id : expected_sensor_ids_) {
    const auto& buffer = sensor_buffers_.at(sensor_id);
    if (!buffer.empty()) {
      sync_time = std::max(sync_time, buffer.back().time);
    }
  }

  result.sync_time = sync_time;

  // 为每个传感器找到最接近同步时间的数据
  bool all_sensors_have_data = true;
  for (const std::string& sensor_id : expected_sensor_ids_) {
    const sensor::TimedPointCloudData* closest_data = 
        FindClosestData(sensor_id, sync_time);
    
    if (closest_data != nullptr) {
      // 检查时间差是否在允许范围内
      const double time_diff = std::abs(common::ToSeconds(closest_data->time - sync_time));
      if (time_diff <= max_time_diff_seconds_) {
        result.laser_data_map[sensor_id] = *closest_data;
      } else {
        // LOG(WARNING) << "Time difference too large for sensor " << sensor_id 
        //              << ": " << time_diff << " seconds";
        all_sensors_have_data = false;
        break;
      }
    } else {
      LOG(WARNING) << "No data found for sensor: " << sensor_id;
      all_sensors_have_data = false;
      break;
    }
  }

  result.is_valid = all_sensors_have_data;
  // // 打印每个有效激光数据的时间戳
  // if (result.is_valid) {
  //   for (const auto& pair : result.laser_data_map) {
  //     LOG(INFO) << "Sensor: " << pair.first << ", Data time: " << common::ToUniversal(pair.second.time);
  //   }
  // }

  // if (result.is_valid) {
  //   LOG(INFO) << "Successfully synchronized " << expected_sensor_ids_.size() 
  //             << " laser sensors at time: " << sync_time;
  // }

  return result;
}

absl::optional<common::Time> LaserSynchronizer::GetLatestTime() const {
  return latest_sync_time_;
}

bool LaserSynchronizer::CanSynchronize() const {
  // 检查所有传感器都至少有一帧数据
  for (const std::string& sensor_id : expected_sensor_ids_) {
    const auto it = sensor_buffers_.find(sensor_id);
    if (it == sensor_buffers_.end() || it->second.empty()) {
      return false;
    }
  }
  return true;
}

void LaserSynchronizer::Clear() {
  for (auto& pair : sensor_buffers_) {
    pair.second.clear();
  }
  latest_sync_time_.reset();
  LOG(INFO) << "Cleared all laser synchronizer buffers";
}

const sensor::TimedPointCloudData* LaserSynchronizer::FindClosestData(
    const std::string& sensor_id, const common::Time& target_time) const {
  
  const auto it = sensor_buffers_.find(sensor_id);
  if (it == sensor_buffers_.end() || it->second.empty()) {
    return nullptr;
  }

  const auto& buffer = it->second;
  const sensor::TimedPointCloudData* closest_data = nullptr;
  double min_time_diff = std::numeric_limits<double>::max();

  // 查找时间最接近的数据
  for (const auto& data : buffer) {
    const double time_diff = std::abs(common::ToSeconds(data.time - target_time));
    if (time_diff < min_time_diff) {
      min_time_diff = time_diff;
      closest_data = &data;
    }
  }

  return closest_data;
}

void LaserSynchronizer::RemoveOldData(const common::Time& cutoff_time) {
  for (auto& pair : sensor_buffers_) {
    auto& buffer = pair.second;
    
    // 移除所有早于cutoff_time的数据
    auto it = std::remove_if(buffer.begin(), buffer.end(),
                            [&cutoff_time](const sensor::TimedPointCloudData& data) {
                              return data.time < cutoff_time;
                            });
    buffer.erase(it, buffer.end());
  }
}

}  // namespace mapping
}  // namespace cartographer
