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

#ifndef AMCL_DEAD_RECKONING_DEAD_RECKONING_H_
#define AMCL_DEAD_RECKONING_DEAD_RECKONING_H_

#include <deque>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"


#include <Eigen/Core>
#include <Eigen/Geometry>

// #include "cartographer/common/time.h"
// #include "cartographer/mapping/imu_tracker.h"
// #include "cartographer/mapping/pose_extrapolator_interface.h"
// #include "cartographer/sensor/imu_data.h"
// #include "cartographer/sensor/odometry_data.h"
// #include "cartographer/transform/rigid_transform.h"

namespace localization {

// struct TimedPose {
//   common::Time time;
//   transform::Rigid3d pose;
// };

struct TimedPose {
  double time;
  Eigen::Isometry3d pose;
};

class DeadReckoning
{
public:
  DeadReckoning() = default;
  ~DeadReckoning() = default;

  DeadReckoning(const DeadReckoning&) = delete;
  DeadReckoning& operator=(const DeadReckoning&) = delete;

  // void AddPose(const transform::Rigid3d& pose);
  // void AddImuData(const sensor::ImuData& imu_data);
  // void AddOdometryData(const sensor::OdometryData& odometry_data);

  void AddPose(const Eigen::Isometry3d& pose, const double time);
  void AddOdomData(const nav_msgs::msg::Odometry& odom_data);

  const Eigen::Isometry3d GetCurrentPose() const {return curr_pose_;}

  // transform::Rigid3d GetPose(common::Time time) override;

private:
  // void TrimImuData();
  void TrimOdometryData();

  void PredictPose(const Eigen::Isometry3d& odom_pose);
  // common::Time GetLastPoseTime() const override;
  // common::Time GetLastExtrapolatedTime() const override;

private:
  // std::deque<sensor::OdometryData> odometry_data_buff_;
  // std::deque<TimedPose> laser_pose_buff_;
  // std::deque<TimedPose> timed_pose_buff_;

  std::deque<TimedPose> pose_data_buff_;
  std::deque<TimedPose> odom_data_buff_;

  std::mutex pose_mutex_;

  Eigen::Isometry3d curr_pose_;

  bool is_pose_update_ = true;
  bool is_first_frame_ = true;
  Eigen::Isometry3d last_odom_pose_;
};

}  // namespace localization

#endif // AMCL_DEAD_RECKONING_DEAD_RECKONING_H_
