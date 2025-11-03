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

#include "cartographer/preprocess/laser_synchronizer.h"

#include <algorithm>
#include <cmath>

namespace cartographer {
namespace preprocess {

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

bool LaserSynchronizer::AddOdometryData(const sensor::OdometryData& odometry_data) {
  // 添加里程计数据到缓冲区
  odom_buffer_.push_back(odometry_data);

  // 限制缓冲区大小（保持足够长的历史记录，至少buffer_size_ * 10）
  const size_t max_odom_buffer_size = buffer_size_ * 10;
  while (odom_buffer_.size() > max_odom_buffer_size) {
    odom_buffer_.pop_front();
  }

  return true;
}

LaserSynchronizer::SynchronizedLaserData LaserSynchronizer::GetSynchronizedData() {
  SynchronizedLaserData result;
  
  if (!CanSynchronize()) {
    return result;  // is_valid = false
  }

  // 以第一个传感器为基准，使用其第一帧数据（FIFO模式）
  const std::string& reference_sensor_id = expected_sensor_ids_[0];
  auto& reference_buffer = sensor_buffers_.at(reference_sensor_id);
  // 步骤1: 检查reference buffer长度是否 >= 2
  if (reference_buffer.size() < 2) {
    return result;  // is_valid = false, 数据不足，不进行同步
  }

  // 步骤2: Trim reference buffer到只剩2个数据
  while (reference_buffer.size() > 2) {
    reference_buffer.pop_front();
  }

  // 步骤3: 使用front数据尝试同步
  const sensor::TimedPointCloudData& reference_data = reference_buffer.front();
  const common::Time sync_time = reference_data.time;

  // 检查里程计数据的时间同步性
  // 要求：1. 雷达时间不能比最新里程计数据晚超过0.1s
  //      2. 雷达数据前0.1s必须有里程计数据（用于去畸变）
  if (!odom_buffer_.empty()) {
    const common::Time oldest_odom_time = odom_buffer_.front().time;
    const common::Time latest_odom_time = odom_buffer_.back().time;
    
    // 检查1: 雷达时间不能比最新里程计晚超过0.1s
    const double time_diff_with_latest = common::ToSeconds(sync_time - latest_odom_time);
    if (time_diff_with_latest > kMaxLaserOdomTimeDiff) {
      LOG(WARNING) << "Laser data is too late compared to latest odometry data. "
                   << "Laser time: " << common::ToUniversal(sync_time)
                   << ", Latest odom time: " << common::ToUniversal(latest_odom_time)
                   << ", Time diff: " << time_diff_with_latest << "s (max allowed: "
                   << kMaxLaserOdomTimeDiff << "s). Sync failed.";
      return result;  // is_valid = false, 同步失败，不pop
    }
    
    // 检查2: 雷达数据前0.1s必须有里程计数据
    const common::Time required_odom_time = sync_time - common::FromSeconds(kMaxLaserOdomTimeDiff);
    if (oldest_odom_time > required_odom_time) {
      LOG(WARNING) << "No odometry data available 0.1s before laser data. "
                   << "Laser time: " << common::ToUniversal(sync_time)
                   << ", Required odom time: " << common::ToUniversal(required_odom_time)
                   << ", Oldest odom time: " << common::ToUniversal(oldest_odom_time)
                   << ". Sync failed.";
      return result;  // is_valid = false, 同步失败，不pop
    }
  } else {
    // 如果没有里程计数据，同步失败
    LOG(WARNING) << "No odometry data available for synchronization. Sync failed.";
    return result;  // is_valid = false, 同步失败，不pop
  }
  
  result.sync_time = sync_time;
  result.laser_data_map[reference_sensor_id] = reference_data;

  // 为其他传感器找到与基准时间最接近的数据
  for (size_t i = 1; i < expected_sensor_ids_.size(); ++i) {
    const std::string& sensor_id = expected_sensor_ids_[i];
    const sensor::TimedPointCloudData* closest_data = 
        FindClosestData(sensor_id, sync_time);
    
    if (closest_data == nullptr) {
      LOG(WARNING) << "No data found for sensor: " << sensor_id 
                   << ". Sync failed.";
      return result;  // is_valid = false, 同步失败，不pop
    }
    
    // 检查时间差是否在允许范围内
    const double time_diff = std::abs(common::ToSeconds(closest_data->time - sync_time));
    if (time_diff > max_time_diff_seconds_) {
      LOG(WARNING) << "Time difference too large for sensor " << sensor_id 
                   << ": " << time_diff << " seconds (max allowed: " 
                   << max_time_diff_seconds_ << "). Sync failed.";
      return result;  // is_valid = false, 同步失败，不pop
    }
    
    result.laser_data_map[sensor_id] = *closest_data;
  }

  // 确保收到了所有期望的传感器数据
  if (result.laser_data_map.size() != expected_sensor_ids_.size()) {
    LOG(WARNING) << "Not all expected sensors have data. Expected: " 
                 << expected_sensor_ids_.size() << ", Got: " << result.laser_data_map.size()
                 << ". Sync failed.";
    result.laser_data_map.clear();
    return result;  // is_valid = false, 同步失败，不pop
  }

  // 步骤4: 同步成功，标记为valid并pop front数据
  result.is_valid = true;
  reference_buffer.pop_front();
  
  // 打印同步信息（可选）
  // LOG(INFO) << "Successfully synchronized " << expected_sensor_ids_.size() 
  //           << " laser sensors with reference: " << reference_sensor_id 
  //           << " at time: " << common::ToUniversal(sync_time);
  // for (const auto& pair : result.laser_data_map) {
  //   const double time_diff = std::abs(common::ToSeconds(pair.second.time - sync_time));
  //   LOG(INFO) << "  Sensor: " << pair.first 
  //             << ", Time: " << common::ToUniversal(pair.second.time)
  //             << ", Diff: " << time_diff << "s";
  // }

  return result;
}

absl::optional<common::Time> LaserSynchronizer::GetLatestTime() const {
  return latest_sync_time_;
}

bool LaserSynchronizer::CanSynchronize() const {
  // 检查所有传感器都至少有2帧数据
  for (const std::string& sensor_id : expected_sensor_ids_) {
    const auto it = sensor_buffers_.find(sensor_id);
    if (it == sensor_buffers_.end() || it->second.size() < 2) {
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

}  // namespace preprocess
}  // namespace cartographer
