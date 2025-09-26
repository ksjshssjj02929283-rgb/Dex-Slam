/*
 * @Author: Buddy_X guo_linfeng@foxmail.com
 * @Date: 2023-05-26 14:39:30
 * @LastEditors: Buddy_X guo_linfeng@foxmail.com
 * @LastEditTime: 2023-05-30 17:54:11
 * @FilePath: /0_dcm_ws/src/cartographer/cartographer/sensor/timed_point_cloud_data.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright 2017 The Cartographer Authors
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

#ifndef CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
#define CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_

#include "Eigen/Core"
#include "common/time.h"
#include "sensor/point_cloud.h"
#include "transform/rigid_transform.h"
namespace cartographer {
namespace sensor {

struct TimedPointCloudData {
  common::Time time;
  Eigen::Vector3f origin;
  transform::Rigid3d sensor_to_tracking;
  TimedPointCloud ranges;
  // 'intensities' has to be same size as 'ranges', or empty.
  std::vector<float> intensities;
};

struct TimedPointCloudOriginData {
  struct RangeMeasurement {
    TimedRangefinderPoint point_time;  // 该点的观测时间
    float intensity;  // 反射率
    size_t origin_index;  // 原始索引
  };
  common::Time time;
  std::vector<Eigen::Vector3f> origins;  // 原点坐标
  std::vector<transform::Rigid3d> sensor_to_trackings;  // 坐标变换
  std::vector<RangeMeasurement> ranges;  // 激光数据
};

// Converts 'timed_point_cloud_data' to a proto::TimedPointCloudData.
proto::TimedPointCloudData ToProto(
    const TimedPointCloudData& timed_point_cloud_data);

// Converts 'proto' to TimedPointCloudData.
TimedPointCloudData FromProto(const proto::TimedPointCloudData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_TIMED_POINT_CLOUD_DATA_H_
