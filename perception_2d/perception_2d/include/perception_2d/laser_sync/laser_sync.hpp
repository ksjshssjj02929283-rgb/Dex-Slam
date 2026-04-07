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
#ifndef PERCEPTION_2D__LASER_SYNC__LASER_SYNC_HPP_
#define PERCEPTION_2D__LASER_SYNC__LASER_SYNC_HPP_


#include <iostream>
#include <string>
#include <vector>
#include "perception_2d/sensor/sync_timed_point_cloud.hpp"
#include "perception_2d/laser_sync/data_type.hpp"
#include "perception_2d/laser_sync/coordinate.hpp"
// #include "ros/ros.hpp"
// #include "sensor_msgs/LaserScan.hpp"
// #include "sensor_msgs/PointCloud.hpp"

namespace perception_2d
{

struct LaserWithPose
{
  LaserWithPose(
    std::string id, const sensor::SyncTimedPointCloudData & _scan,
    const bool & _updated = true)
  : scan(_scan), sensor_id(id), updated(_updated) {}
  sensor::SyncTimedPointCloudData scan;
  std::string sensor_id;
  bool updated = false;
};


class LaserSync
{
public:
  LaserSync();
  /**
    * 添加里程计数据
    * @param Odometry odometry
    */
  void AddOdometryData(const sensor::OdometryData & odometry);
  /**
    * 获取去畸变点云数据
    * @param SyncPointCloudWithOdom& sync_point_cloud 去畸变后多传感器融合点云数据
    * @return 是否得到去畸变点云
    */
  bool GetSyncPointCloud(
    const std::string & sensor_id, const sensor::SyncTimedPointCloudData & range_data,
    sensor::SyncPointCloudWithOdom & sync_point_cloud);
  // bool GetCodeDataWithOdom(const sensor::CodeData& code_data,
  // sensor::CodeDataWithOdom& code_data_with_odom);
  // bool GetLaneDataWithOdom(const sensor::LaneData& lane_datam,
  // sensor::LaneDataWithOdom& lane_data_with_odom);
  bool GetOdomData(const common::Time & time, sensor::OdometryData & odom);
  bool FiltePointCloud(
    const sensor::SyncPointCloudWithOdom & input_point_cloud,
    const double & max_range,
    const double & min_range,
    sensor::RangeDataWithOdom & range_data_with_odom);
//    bool ConvertPointCloudToLaser(const sensor_msgs::PointCloud &cloud,
//                                             sensor_msgs::LaserScan &scan);

private:
  void UpdateLaserData(
    const std::string & sensor_id,
    const sensor::SyncTimedPointCloudData & scan);
  bool SyncPointCloud(sensor::SyncPointCloudWithOdom & sync_point_cloud);
  bool GetFinalSyncPointCloud(sensor::SyncPointCloudWithOdom & sync_point_cloud);
  Pose2D GetRelativePose(common::Time stamp);
  Pose2D PredictPose(const Pose2D & cur, const double & duration);
  double CalcSinXDividX(const double & x)
  {
    if (fabs(x) < 1e-4) {
      return 1;
    }
    return sin(x) / x;
  }

private:
  bool odom_updated_ = false;
  sensor::OdometryData odom_;
  Pose2D odom_pose_;
  std::vector<LaserWithPose> scans_;
};
}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_SYNC__LASER_SYNC_HPP_
