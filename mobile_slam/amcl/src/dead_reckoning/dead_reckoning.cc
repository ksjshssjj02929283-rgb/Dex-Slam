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

#include "dead_reckoning/dead_reckoning.h"

#include <algorithm>

#include "glog/logging.h"

namespace localization
{

void DeadReckoning::AddPose(
  const Eigen::Isometry3d& pose,
  const double time)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  TimedPose timed_pose{time, pose};
  pose_data_buff_.push_back(timed_pose);
  TrimOdometryData();
  while (pose_data_buff_.size() > 0) {
    pose_data_buff_.pop_front();
  }

  LOG(INFO) << "odom_data_buff_ size: " << odom_data_buff_.size();
  LOG(INFO) << "pose time : " << std::setprecision(16) << timed_pose.time;
  LOG(INFO) << "odom time : " << std::setprecision(16) << odom_data_buff_.front().time;
  curr_pose_ = timed_pose.pose;
  is_pose_update_ = true;

  // 把当前时间之前的odom都给清空
  for (auto it = odom_data_buff_.begin(); it != odom_data_buff_.end(); ++it) {
    PredictPose(it->pose);
  }
  // 因为里程计的时间要晚于位姿的时间，所以加入pose后，会导致pose的时间要早于odom时间
}

void DeadReckoning::AddOdomData(
  const nav_msgs::msg::Odometry& odom_data)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  Eigen::Isometry3d odom_pose = Eigen::Isometry3d::Identity();
  odom_pose.translate(Eigen::Vector3d(odom_data.pose.pose.position.x,
                                      odom_data.pose.pose.position.y,
                                      odom_data.pose.pose.position.z));
  odom_pose.rotate(Eigen::Quaterniond(odom_data.pose.pose.orientation.w,
                                      odom_data.pose.pose.orientation.x,
                                      odom_data.pose.pose.orientation.y,
                                      odom_data.pose.pose.orientation.z));
  double timestamp = odom_data.header.stamp.sec + odom_data.header.stamp.nanosec * 1e-9;
  TimedPose time_odom{timestamp, odom_pose};
  odom_data_buff_.push_back(time_odom);
  PredictPose(odom_data_buff_.back().pose);
}

void DeadReckoning::TrimOdometryData()
{
  while (odom_data_buff_.size() > 1 &&
         odom_data_buff_.front().time < pose_data_buff_.front().time)
  {
    odom_data_buff_.pop_front();
  }
}

void DeadReckoning::PredictPose(const Eigen::Isometry3d& odom_pose)
{
  if (is_pose_update_ || is_first_frame_) {
    last_odom_pose_ = odom_pose;
    is_pose_update_ = false;
    is_first_frame_ = false;
    return;
  }

  Eigen::Isometry3d delta_odom =
    last_odom_pose_.inverse() * odom_pose;

  curr_pose_ = curr_pose_ * delta_odom;

  last_odom_pose_ = odom_pose;
}

} // namespace localization
