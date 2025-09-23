#include "cartographer/preprocess/data_preprocessor.h"

#include "cartographer/sensor/range_data.h"

namespace cartographer {
namespace preprocess {

DataPreprocessor::DataPreprocessor(const proto::DataPreprocessorOptions& options)
: options_(options)
{
}

absl::optional<sensor::TimedRangeData>
DataPreprocessor::GetProcessedRangeData()
{
  if (accumlated_range_buff_.empty()) {
    return absl::nullopt;
  }
  sensor::TimedRangeData range_data = accumlated_range_buff_.front();
  accumlated_range_buff_.pop_front();
  return absl::optional<sensor::TimedRangeData>(range_data);
}

// 外插的精度不够，全部使用内插进行去畸变计算

void DataPreprocessor::AddSyncedPointCloud(const sensor::TimedPointCloudOriginData& cloud_in)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  cloud_data_buff_.push_back(cloud_in);

  if (!options_.use_odometry()) {
    // accumulated_range_data_.time = cloud_in.time;
    // accumulated_range_data_.data.returns.InsertCloud(cloud_in.ranges);
    // accumulated_range_data_.data.origin = Eigen::Vector3f::Zero();

    // return;
  }
  // 1. 数据同步
  // 要求有上一帧雷达到这一帧雷达之间的odom数据
  if (odom_data_buff_.empty() ||
      odom_data_buff_.front().time > cloud_data_buff_.front().time)
  {
    cloud_data_buff_.pop_front();
    return;
  }
  if (odom_data_buff_.back().time < cloud_data_buff_.front().time) {
    return;
  }
  // 2. 处理数据
  sensor::TimedPointCloudOriginData curr_cloud = cloud_data_buff_.back();
  while (cloud_data_buff_.size() > 1) {
    cloud_data_buff_.pop_front();
  }
  // 2.1 获取第一个点和最后一点的时间戳
  const common::Time& time = curr_cloud.time;
  const common::Time time_first_point =
      time + common::FromSeconds(curr_cloud.ranges.front().point_time.time);
  // 2.2 第一个点的数据早于里程计数据，无法去畸变，放弃这一帧数据
  if (time_first_point < odom_data_buff_.front().time) {
    last_laser_time_ = curr_cloud.time;
    return;
  }

  // 3. 去畸变
  // 3.1 从第一个点开始，计算每个点采集时雷达的位姿（在odom坐标系下）
  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(curr_cloud.ranges.size());

  for (size_t i = 0; i < curr_cloud.ranges.size(); i++) {
    common::Time time_point = time + common::FromSeconds(curr_cloud.ranges[i].point_time.time);
    range_data_poses.push_back(
      GetPointPose(time_point, odom_data_buff_).cast<float>());
    // std::cout << range_data_poses.back().translation().x() << " "
    //           << range_data_poses.back().translation().y() << " "
    //           << transform::GetYaw(range_data_poses.back().rotation()) << std::endl;
  }

  auto undistort_range_data = sensor::TimedRangeData{{}, {{}, {}, {}}};

  // 3.3 在odom坐标系下去畸变
  for (size_t i = 0; i < curr_cloud.ranges.size(); ++i) {
    // a 获取该点观测数据
    const sensor::TimedRangefinderPoint& hit =
        curr_cloud.ranges[i].point_time;
    if (hit.position.x() > -0.3 && hit.position.x() < 0.3 &&
        hit.position.y() > -0.3 && hit.position.y() < 0.3)
    {
      continue;
    }
    bool is_miss = false;
    const float range = hit.position.head<2>().norm();
    if (range > options_.max_range()) {
      is_miss = true;
    }
    // b 获取该点在odom坐标系下的位姿
    const Eigen::Vector3f origin_in_odom =
        range_data_poses[i] *
        curr_cloud.origins.at(curr_cloud.ranges[i].origin_index);
    // c 去畸变后的点在odom系下的坐标
    sensor::RangefinderPoint hit_in_odom;
    if (options_.use_deskew()) {
      hit_in_odom = range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    } else {
      hit_in_odom = range_data_poses.back() * sensor::ToRangefinderPoint(hit);
    }

    if (is_miss) {
      const Eigen::Vector3f delta = hit_in_odom.position - origin_in_odom;
      hit_in_odom.position =
          origin_in_odom +
          options_.missing_data_ray_length() / delta.norm() * delta;
      undistort_range_data.data.misses.push_back(hit_in_odom);
      continue;
    }
    // d 保留在min_range和max_range之间的点，并计算器位姿，添加到accumulated_range_data_中

    undistort_range_data.data.returns.push_back(hit_in_odom, curr_cloud.ranges[i].intensity);
  }
  undistort_range_data.time = time;
  undistort_range_data.data.origin = range_data_poses.back().translation();

  AccumulateRangeData(undistort_range_data, range_data_poses.back());
}

void DataPreprocessor::AccumulateRangeData(
    const sensor::TimedRangeData& range_data_in,
    const transform::Rigid3f& range_in_odom)
{
  accumulated_range_data_.time = range_data_in.time;
  accumulated_range_data_.data.returns.InsertCloud(range_data_in.data.returns);
  accumulated_range_data_.data.misses.InsertCloud(range_data_in.data.misses);
  accumulated_range_data_.data.origin = range_data_in.data.origin;

  transform::Rigid3f last_odom_pose = GetPointPose(last_laser_time_, odom_data_buff_);
  transform::Rigid3f curr_odom_pose = GetPointPose(range_data_in.time, odom_data_buff_);

  odom_delta_pose_ = last_odom_pose.inverse() * curr_odom_pose;
  // last_odom_pose_ = curr_odom_pose;
  num_accumulated_++;
  if (num_accumulated_ >= options_.num_accumulated_range_data()) {
    // 将激光点从odom转换到雷达坐标系下
    sensor::TimedRangeData temp_range;
    temp_range.time = accumulated_range_data_.time;
    for (size_t i = 0; i < accumulated_range_data_.data.returns.size(); i++) {
      auto hit_in_origin = range_in_odom.inverse() * accumulated_range_data_.data.returns[i];
      temp_range.data.returns.push_back(hit_in_origin, accumulated_range_data_.data.returns.intensities()[i]);
    }
    for (auto& miss : accumulated_range_data_.data.misses) {
      auto miss_in_origin = range_in_odom.inverse() * miss;
      temp_range.data.misses.push_back(miss_in_origin);
    }
    temp_range.data.origin = Eigen::Vector3f::Zero();

    accumlated_range_buff_.push_back(std::move(temp_range));
    num_accumulated_ = 0;
    last_laser_time_ = accumulated_range_data_.time;
    accumulated_range_data_ = sensor::TimedRangeData{{}, {{}, {}, {}}};
  }
}

/**
 * @brief 获得该点在odom下的位姿
 * 
 * @param time 
 * @return transform::Rigid3f 
 */
transform::Rigid3f DataPreprocessor::GetPointPose(
  const common::Time& time,
  const std::deque<sensor::OdometryData>& odom_data_buff) const
{
  // 健壮性检查
  if (odom_data_buff.empty()) {
    return transform::Rigid3f::Identity();
  }
  if (odom_data_buff.size() == 1) {
    return odom_data_buff.back().pose.cast<float>();
  }

  const common::Time latest_odom_time = odom_data_buff.back().time;

  // 命中最新时刻
  if (time == latest_odom_time) {
    return odom_data_buff.back().pose.cast<float>();
  }

  // 内插
  if (time < latest_odom_time) {
    // 查找 p0, p1
    size_t i1 = 1;
    for (; i1 < odom_data_buff.size(); ++i1) {
      if (odom_data_buff[i1].time >= time) break;
    }
    // 若 time 早于最早样本，则走外插到最早（或直接返回最早姿态）
    if (i1 == 0) {
      return odom_data_buff.front().pose.cast<float>();
    }
    const auto& p0 = odom_data_buff[i1 - 1];
    const auto& p1 = odom_data_buff[i1];

    const double denom = std::max(common::ToSeconds(p1.time - p0.time), 1e-9);
    double ratio = common::ToSeconds(time - p0.time) / denom;
    ratio = std::clamp(ratio, 0.0, 1.0);

    // 平移线性插值
    const Eigen::Vector3d t =
        (1.0 - ratio) * p0.pose.translation() + ratio * p1.pose.translation();
    // 旋转球面线性插值
    Eigen::Quaterniond q = p0.pose.rotation().slerp(ratio, p1.pose.rotation());
    q.normalize();

    return transform::Rigid3d(t, q).cast<float>();
  }

  // 外插（time > latest_time）
  {
    const auto& last = odom_data_buff.back();
    const double dt = common::ToSeconds(time - last.time);

    const Eigen::Vector3d w = last.twist.angular; // 角速度
    // 确认线速度是在哪个坐标系：如是机体系，需左乘姿态；如是世界系，直接用
    Eigen::Vector3d v = last.twist.linear;
    v = last.pose.rotation() * v;  // 如线速度在机体系，保留；否则去掉

    const Eigen::Vector3d t = last.pose.translation() + dt * v;
    const Eigen::Quaterniond dq =
        transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(w * dt));
    Eigen::Quaterniond q = last.pose.rotation() * dq;
    q.normalize();

    return transform::Rigid3d(t, q).cast<float>();
  }
}

void DataPreprocessor::AddOdometryData(const sensor::OdometryData& odometry_data)
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  if (!odom_data_buff_.empty() &&
    odom_data_buff_.back().time > odometry_data.time)
  {
    return;
  }
  odom_data_buff_.push_back(odometry_data);
  TrimOdomData();
}

void DataPreprocessor::TrimOdomData()
{
  // 最少需要保留上一帧激光到这一帧激光之间的odom数据
  while (odom_data_buff_.front().time < (last_laser_time_ - common::FromSeconds(0.1))) {
    odom_data_buff_.pop_front();
  }
}
 
} // namespace preprocess
} // namespace cartographer
