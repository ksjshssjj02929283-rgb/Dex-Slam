//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

#include "livox_ros_driver2/msg/custom_msg.hpp"

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"

namespace lightning {

namespace loc {
class Localization;
}

class RVizPublisher;

class LocSystem {
   public:
    struct Options {
        bool online_mode_ = true;  // true=在线订阅传感器，false=离线由外部驱动
        bool with_rviz_ = true;    // 是否启用RViz可视化
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    /// 初始化，地图路径由外部参数（gflags/launch）传入
    bool Init(const std::string& yaml_path, const std::string& map_path);

    /// 设置初始化位姿
    void SetInitPose(const SE3& pose);

    /// 开始/停止定位（控制是否处理传感器输入）
    void StartLocalization();
    void StopLocalization();

    /// 状态与节点句柄
    bool IsMapLoaded() const { return map_loaded_; }
    bool IsLocalizationStarted() const { return loc_started_; }
    rclcpp::Node::SharedPtr GetNode() const { return node_; }

    /// 处理IMU
    void ProcessIMU(const lightning::IMUPtr& imu);

    /// 处理点云
    void ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud);
    void ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud);

    /// 实时模式下的spin
    void Spin();

   private:
    /// TF 查询工具函数
    bool GetPoseInFrame(const std::string& reference_frame, const std::string& pose_frame, const tf2::TimePoint& time,
                        const tf2::Duration& timeout, SE3& result);
    bool EnsureBaseToLidarTF();
    void TryPublishMapToOdomTF(const SE3& T_map_base, double timestamp);

   private:
    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;   // 定位接口
    std::shared_ptr<RVizPublisher> rviz_pub_ = nullptr;  // RViz 可视化

    std::atomic_bool loc_started_ = false;  // 是否开启定位
    std::atomic_bool map_loaded_ = false;   // 地图是否已载入

    std::string map_name_;        // 地图名（从 map_path 提取）
    std::ofstream loc_traj_out_;  // TUM 格式轨迹文件（定位）

    /// 实时模式下的ros2 node, subscribers
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = nullptr;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /// TF 配置
    std::string map_frame_ = "map";
    std::string lidar_frame_ = "lidar";
    std::string base_link_frame_ = "base_link";
    std::string odom_frame_ = "odom";
    bool extend_to_base_ = true;
    bool publish_map_to_odom_tf_ = false;

    /// TF 数据
    SE3 T_base_lidar_;  // lidar -> base_link 外参（从 TF 读取）
    bool T_base_lidar_valid_ = false;

    std::string imu_topic_;
    std::string cloud_topic_;
    std::string livox_topic_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_ = nullptr;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_ = nullptr;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_ = nullptr;
};

};  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H