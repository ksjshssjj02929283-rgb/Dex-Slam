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

#ifndef CARTOGRAPHER_PREPROCESS_LASER_SYNCHRONIZER_H_
#define CARTOGRAPHER_PREPROCESS_LASER_SYNCHRONIZER_H_

#include <map>
#include <string>
#include <vector>
#include <deque>
#include <memory>

#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "glog/logging.h"

namespace cartographer {
namespace preprocess {

/**
 * @brief 激光雷达同步器类，用于将多个激光雷达数据按时间同步
 */
class LaserSynchronizer {
 public:
  struct SynchronizedLaserData {
    common::Time sync_time;  // 同步时间（最新雷达数据的时间）
    std::map<std::string, sensor::TimedPointCloudData> laser_data_map;  // 各雷达同步后的数据
    bool is_valid = false;  // 是否有效（所有雷达都有数据）
  };

  /**
   * @brief 构造函数
   * @param expected_sensor_ids 预期的激光雷达传感器ID列表
   * @param max_time_diff_seconds 允许的最大时间差（秒）
   * @param buffer_size 每个传感器的最大缓冲区大小
   */
  explicit LaserSynchronizer(
      const std::vector<std::string>& expected_sensor_ids,
      double max_time_diff_seconds = 0.05,
      size_t buffer_size = 10);

  ~LaserSynchronizer() = default;

  // 禁止拷贝和赋值
  LaserSynchronizer(const LaserSynchronizer&) = delete;
  LaserSynchronizer& operator=(const LaserSynchronizer&) = delete;

  /**
   * @brief 添加激光雷达数据
   * @param sensor_id 传感器ID
   * @param timed_point_cloud_data 时间点云数据
   * @return 是否成功添加
   */
  bool AddLaserData(const std::string& sensor_id,
                    const sensor::TimedPointCloudData& timed_point_cloud_data);

  /**
   * @brief 添加里程计数据
   * @param odometry_data 里程计数据
   * @return 是否成功添加
   */
  bool AddOdometryData(const sensor::OdometryData& odometry_data);

  /**
   * @brief 获取同步后的激光雷达数据
   * @return 同步后的数据结构，如果无法同步则返回无效数据
   */
  SynchronizedLaserData GetSynchronizedData();

  /**
   * @brief 获取最新帧的时间
   * @return 最新一帧雷达数据的时间
   */
  absl::optional<common::Time> GetLatestTime() const;

  /**
   * @brief 检查是否有足够的数据进行同步
   * @return 是否可以进行同步
   */
  bool CanSynchronize() const;

  /**
   * @brief 清空所有缓冲区
   */
  void Clear();

 private:
  /**
   * @brief 查找与给定时间最接近的数据
   * @param sensor_id 传感器ID
   * @param target_time 目标时间
   * @return 最接近的数据，如果没有找到则返回nullptr
   */
  const sensor::TimedPointCloudData* FindClosestData(
      const std::string& sensor_id, const common::Time& target_time) const;

  /**
   * @brief 移除过期的数据
   * @param cutoff_time 截止时间，早于此时间的数据将被移除
   */
  void RemoveOldData(const common::Time& cutoff_time);

  // 激光数据最早不能早于里程计数据的最大时间差（秒）
  static constexpr double kMaxLaserOdomTimeDiff = 0.1;

  const std::vector<std::string> expected_sensor_ids_;
  const double max_time_diff_seconds_;
  const size_t buffer_size_;

  // 每个传感器的数据缓冲区
  std::map<std::string, std::deque<sensor::TimedPointCloudData>> sensor_buffers_;
  
  // 里程计数据缓冲区
  std::deque<sensor::OdometryData> odom_buffer_;
  
  // 最新的同步时间
  absl::optional<common::Time> latest_sync_time_;
};

}  // namespace preprocess
}  // namespace cartographer

#endif  // CARTOGRAPHER_PREPROCESS_LASER_SYNCHRONIZER_H_
