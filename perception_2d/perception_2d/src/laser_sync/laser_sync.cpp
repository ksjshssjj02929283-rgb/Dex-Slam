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
#include "perception_2d/laser_sync/laser_sync.hpp"
#include <string>

namespace perception_2d
{
LaserSync::LaserSync()
{
}

void LaserSync::AddOdometryData(const sensor::OdometryData & odometry)
{
  odom_ = odometry;
  Eigen::Vector3d eular = odometry.pose.rotation().matrix().eulerAngles(0, 1, 2);
  odom_pose_.theta = eular[2];
  odom_pose_.position.x = odometry.pose.translation()[0];
  odom_pose_.position.y = odometry.pose.translation()[1];
  odom_pose_.v = odometry.linear_velocity[0];
  odom_pose_.w = odometry.angular_velocity[2];
  odom_updated_ = true;
  // static int i = 0;
  // LOG(INFO) << "I = " << i ;
  // i++;
}

void LaserSync::UpdateLaserData(
  const std::string & sensor_id,
  const sensor::SyncTimedPointCloudData & scan)
{
  int index = 0;
  for (index = 0; index < static_cast<int>(scans_.size()); index++) {
    // std::cout << "scan sensor_id = " << scans_[index].sensor_id << std::endl;
    // std::cout << "sensor_id = " << sensor_id << std::endl;
    if (scans_[index].sensor_id == sensor_id) {
      break;
    }
  }
  if (index >= static_cast<int>(scans_.size())) {
    scans_.push_back(LaserWithPose(sensor_id, scan));
    std::cout << "updated new scan message " << sensor_id << std::endl;
  } else {
    scans_[index] = LaserWithPose(sensor_id, scan);
  }
}

Pose2D LaserSync::PredictPose(const Pose2D & cur, const double & duration)
{
  Pose2D nxt = cur;
  double dist = cur.v * duration;
  double th = cur.w * duration;
  // LOG(INFO) << "duration = " << duration;
  nxt.position.x = dist * CalcSinXDividX(th);
  nxt.position.y = dist * CalcSinXDividX(th / 2) * sin(th / 2);
  nxt.theta = th;
//  LOG(WARNING)<<"predict odom: dist: "<<dist<<" th:"<<th<<" "<<duration<<" "<<cur.v<<" "<<cur.w;
  return Transform2D::transformFrom(cur, nxt);
}

Pose2D LaserSync::GetRelativePose(common::Time stamp)
{
//  LOG(WARNING)<<"odom time: "<<stamp<<" last odom theta: "<<odom_pose_.theta;
  // LOG(INFO) << "stamp = " << stamp;
  // LOG(INFO) << "odom time = " << odom_.time;
  return PredictPose(odom_pose_, common::ToSeconds(stamp - odom_.time));
}

bool LaserSync::SyncPointCloud(sensor::SyncPointCloudWithOdom & sync_point_cloud)
{
  if (!odom_updated_) {
    std::cout << "no odom data" << std::endl;
    // std::cout << "false 3" << std::endl;
    return false;
  }

  // common::ToSeconds(stamp - odom_.time)
  for (size_t index = 0; index < scans_.size(); index++) {
    common::Time time_stamp = scans_[index].scan.time;
    double duration = common::ToSeconds(time_stamp - odom_.time);
    if (duration > 1.0) {
      LOG(ERROR) << "time duration of laser and odom is too large ! :" << duration << " s";
      return false;
    }
  }
  // std::cout << "size = " << scans_.size() << std::endl;
  for (int i = 0; i < static_cast<int>(scans_.size()); i++) {
    if (!scans_[i].updated) {
      // std::cout << "scan[" << i << "]: " << scans_[i].sensor_id << "not updated " << std::endl;
      return false;
    }
  }
  sync_point_cloud.sync_point_clouds.clear();
  sync_point_cloud.sync_point_clouds.resize(scans_.size());
  for (int index = 0; index < static_cast<int>(scans_.size()); index++) {
    sensor::SyncTimedPointCloudData scan_ = scans_[index].scan;
    // Pose2D trans;
    // trans.position.x = scan_.origin[0];
    // trans.position.y = scan_.origin[1];
    // trans.theta = scan_.origin[2];
    common::Time time_stamp = scan_.time;

    sensor::PointCloudWithOrigin tmp_point_cloud;
    // tmp_point_cloud.origin = scan_.origin;
    tmp_point_cloud.frame_id = scans_[index].sensor_id;
    // tmp_point_cloud.origin = Eigen::Vector3d(
    //   scan_.origin.translation()[0],
    //   scan_.origin.translation()[1],
    //   transform::GetYaw(scan_.origin));
    tmp_point_cloud.origin = scan_.origin;
    // 对于每一个点
    for (int i = 0; i < static_cast<int>(scan_.point_cloud.points.size()); i++) {
      sensor::TimedRangefinderPoint point = scan_.point_cloud.points[i];
      // LOG(INFO) << "POINT TIME = " << point.time ;
      Pose2D p_b; // point laser link    point base link
      common::Time stamp = time_stamp + common::FromSeconds(point.time);
      p_b.position.x = point.position[0];
      p_b.position.y = point.position[1];
      Pose2D cur_odom = GetRelativePose(stamp);
      Pose2D p_o = Transform2D::transformFrom(cur_odom, p_b);
      Pose2D p_origin = Transform2D::transformFrom(odom_pose_, p_b);
      Eigen::Vector3f pt(p_o.position.x, p_o.position.y, point.position[2]);
      Eigen::Vector3f pt_origin(p_origin.position.x, p_origin.position.y, 0);
      // std::cout << "pt = " << pt.transpose() << std::endl;
      // std::cout << "pt_origin = " << pt_origin << std::endl;
      // Eigen::Vector3f pt_origin(p_b.position.x, p_b.position.y, 0);
      tmp_point_cloud.sync_points_base_link.push_back(pt);
      tmp_point_cloud.origin_points_odom.push_back(pt_origin);
      tmp_point_cloud.intensities.push_back(scan_.point_cloud.intensities[i]);
    }
    sync_point_cloud.sync_point_clouds[index] = tmp_point_cloud;
  }

  sync_point_cloud.odom = odom_;
  sync_point_cloud.stamp = odom_.time;
  for (auto & scan : scans_) {
    scan.updated = false;
  }

  return true;
}

bool LaserSync::GetFinalSyncPointCloud(sensor::SyncPointCloudWithOdom & sync_point_cloud)
{
  if (!odom_updated_) {
    return false;
  }

  Pose2D cur_odom = odom_pose_;
  // cur_odom.position.x =
  for (int index = 0; index < static_cast<int>(sync_point_cloud.sync_point_clouds.size());
    index++)
  {
    sensor::PointCloudWithOrigin tmp_point_cloud;
    for (int i = 0;
      i < static_cast<int>(sync_point_cloud.sync_point_clouds[index].sync_points_base_link.size());
      i++)
    {
      Pose2D p_o, p_b /*, pt_or, pt_br*/;
      p_o.position.x = sync_point_cloud.sync_point_clouds[index].sync_points_base_link[i][0];
      p_o.position.y = sync_point_cloud.sync_point_clouds[index].sync_points_base_link[i][1];

      p_b = Transform2D::transformTo(cur_odom, p_o);
      // LOG(INFO) << "CUR_ODOM X = " << cur_odom.position.x;
      // LOG(INFO) << "CUR_ODOM Y = " << cur_odom.position.y;
      // LOG(INFO) << "odom_pose_ time = " << odom_.time ;
      // LOG(INFO) << "odom_pose_ x = " << odom_.pose.translation()[0];
      // LOG(INFO) << "odom_pose_ y = " << odom_.pose.translation()[1];
      // LOG(INFO) << "p_o x = " << p_o.position.x;
      // LOG(INFO) << "p_o y = " << p_o.position.y;
      // pt_br = Transform2D::transformTo(cur_odom, pt_or);
      float p_o_z = sync_point_cloud.sync_point_clouds[index].sync_points_base_link[i][2];
      Eigen::Vector3f pt(p_b.position.x, p_b.position.y, p_o_z);

      tmp_point_cloud.sync_points_base_link.push_back(pt);
      // tmp_point_cloud.intensities.push_back(sync_point_cloud.sync_point_clouds[index].intensities[i]);
    }
    tmp_point_cloud.frame_id = sync_point_cloud.sync_point_clouds[index].frame_id;
    tmp_point_cloud.origin_points_odom =
      sync_point_cloud.sync_point_clouds[index].origin_points_odom;
    tmp_point_cloud.intensities = sync_point_cloud.sync_point_clouds[index].intensities;
    tmp_point_cloud.origin = sync_point_cloud.sync_point_clouds[index].origin;
    sync_point_cloud.sync_point_clouds[index] = tmp_point_cloud;
  }

  return true;
}

bool LaserSync::GetSyncPointCloud(
  const std::string & sensor_id, const sensor::SyncTimedPointCloudData & range_data,
  sensor::SyncPointCloudWithOdom & sync_point_cloud)
{
  UpdateLaserData(sensor_id, range_data);

  if (!SyncPointCloud(sync_point_cloud)) {
    // std::cout << "false 1" << std::endl;
    return false;
  }

  if (!GetFinalSyncPointCloud(sync_point_cloud)) {
    std::cout << "not get the FinalSyncPointClout yet" << std::endl;
    return false;
  }


  return true;
}


bool LaserSync::GetOdomData(const common::Time & time, sensor::OdometryData & odom)
{
  if (!odom_updated_) {
    return false;
  }


  Pose2D cur_odom = GetRelativePose(time);

  Eigen::Quaterniond q = transform::RollPitchYaw(0, 0, cur_odom.theta);
  Eigen::Vector3d t(cur_odom.position.x, cur_odom.position.y, 0);
  transform::Rigid3d r(t, q);
  // sensor::Odometry tmp_odom
  odom.time = time;
  odom.pose = r;
  odom.angular_velocity << 0, 0, cur_odom.w;
  odom.linear_velocity << cur_odom.v, 0, 0;
  odom.pose_covariance = odom_.pose_covariance;
  return true;
}

bool LaserSync::FiltePointCloud(
  const sensor::SyncPointCloudWithOdom & input_point_cloud,
  const double & max_range,
  const double & min_range,
  sensor::RangeDataWithOdom & range_data_with_odom)
{
  for (int i = 0; i < static_cast<int>(input_point_cloud.sync_point_clouds.size()); i++) {
    sensor::SyncRangeData range_data;
    Eigen::Vector3f origin_in_base_link = Eigen::Vector3f(
      static_cast<float>(input_point_cloud.sync_point_clouds[i].origin.translation()[0]),
      static_cast<float>(input_point_cloud.sync_point_clouds[i].origin.translation()[1]),
      static_cast<float>(input_point_cloud.sync_point_clouds[i].origin.translation()[2]));
    for (int index = 0;
      index < static_cast<int>(input_point_cloud.sync_point_clouds[i].sync_points_base_link.size());
      index++)
    {
      const Eigen::Vector3f hit =
        input_point_cloud.sync_point_clouds[i].sync_points_base_link[index];
      const Eigen::Vector3f delta = hit -
        Eigen::Vector3f(
        static_cast<float>(origin_in_base_link[0]),
        static_cast<float>(origin_in_base_link[1]),
        static_cast<float>(origin_in_base_link[2]));
      // Eigen::Vector3f hit_in_base_link = hit;
      sensor::RangefinderPoint hit_in_base_link;
      hit_in_base_link.position = hit;
      const float range = delta.norm();
      if (range >= min_range) {
        if (range <= max_range) {
          range_data.hits.push_back(hit_in_base_link);
        } else {
          hit_in_base_link.position = Eigen::Vector3f(
            static_cast<float>(origin_in_base_link[0]),
            static_cast<float>(origin_in_base_link[1]),
            static_cast<float>(origin_in_base_link[2])) + max_range / range * delta;
          range_data.misses.push_back(hit_in_base_link);
        }
      }
    }
    range_data.origin = origin_in_base_link;
    for (int index = 0;
      index < static_cast<int>(input_point_cloud.sync_point_clouds[i].origin_points_odom.size());
      index++)
    {
      sensor::RangefinderPoint point;
      point.position = input_point_cloud.sync_point_clouds[i].origin_points_odom[index];
      range_data.origin_points_odom.push_back(point);
    }
    // range_data.origin_points_odom = input_point_cloud.sync_point_clouds[i].origin_points_odom;
    range_data_with_odom.range_datas.push_back(range_data);
  }
  range_data_with_odom.odom = input_point_cloud.odom;
  range_data_with_odom.stamp = input_point_cloud.stamp;
  return true;
}

}  // namespace perception_2d
