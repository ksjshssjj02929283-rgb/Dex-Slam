/**
 * @file sensor_bridge.h
 * @author ZheHe
 * @brief 1.将ros格式的odom和imu消息转换成carto格式；2.处理
 * 雷达，odom，imu，gps，路标点等数据，并通过trajectory_builder_->AddSensorData()将传感器数据传入
 * @version 0.1
 * @date 2023-05-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "absl/types/optional.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"

namespace cartographer_ros {

// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge {
public:
    explicit SensorBridge(int num_subdivisions_per_laser_scan,
                          const std::string& tracking_frame,
                          double lookup_transform_timeout_sec,
                          tf2_ros::Buffer* tf_buffer,
                          ::cartographer::mapping::TrajectoryBuilderInterface* trajectory_builder);

    SensorBridge(const SensorBridge&) = delete;
    SensorBridge& operator=(const SensorBridge&) = delete;

    std::unique_ptr<::cartographer::sensor::OdometryData> ToOdometryData(
        const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void HandleOdometryMessage(const std::string& sensor_id, const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void HandleNavSatFixMessage(const std::string& sensor_id, const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
    void HandleLandmarkMessage(const std::string& sensor_id,
                               const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg);

    std::unique_ptr<::cartographer::sensor::ImuData> ToImuData(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    void HandleImuMessage(const std::string& sensor_id, const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
    // noted by hz: 处理雷达数据（包含 将雷达msg转换为carto数据类型 + 雷达数据处理的回调）
    void HandleLaserScanMessage(const std::string& sensor_id, const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
    void HandleMultiEchoLaserScanMessage(const std::string& sensor_id,
                                         const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);
    void HandlePointCloud2Message(const std::string& sensor_id,
                                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

    const TfBridge& tf_bridge() const;

private:
    void HandleLaserScan(const std::string& sensor_id,
                         ::cartographer::common::Time start_time,
                         const std::string& frame_id,
                         const ::cartographer::sensor::PointCloudWithIntensities& points);
    void HandleRangefinder(const std::string& sensor_id,
                           ::cartographer::common::Time time,
                           const std::string& frame_id,
                           const ::cartographer::sensor::TimedPointCloud& ranges);

    const int num_subdivisions_per_laser_scan_;
    std::map<std::string, cartographer::common::Time> sensor_to_previous_subdivision_time_;
    const TfBridge tf_bridge_;
    // noted by hz: SLAM算法的接口
    ::cartographer::mapping::TrajectoryBuilderInterface* const trajectory_builder_;

    absl::optional<::cartographer::transform::Rigid3d> ecef_to_local_frame_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_SENSOR_BRIDGE_H
