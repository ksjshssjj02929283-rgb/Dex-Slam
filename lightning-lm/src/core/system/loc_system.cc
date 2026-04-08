//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"
#include "wrapper/rviz_publisher.h"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    if (loc_) {
        loc_->Finish();
    }
}

bool LocSystem::Init(const std::string &yaml_path, const std::string &map_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = options_.online_mode_;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);

    if (map_path.empty()) {
        LOG(ERROR) << "LocSystem::Init failed: empty map_path from gflags/launch";
        return false;
    }

    // 创建 ROS2 节点（online/offline 均需要，用于 TF 时间戳和 RViz 发布）
    node_ = std::make_shared<rclcpp::Node>("lightning_slam");

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating subscribers ... ";

        imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
        cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
        livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

        rclcpp::QoS qos(10);

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
            });

        LOG(INFO) << "online loc node has been created.";
    }

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) { tf_broadcaster_->sendTransform(pose); });
    }

    if (options_.with_rviz_) {
        LOG(INFO) << "loc with RViz visualization";
        RVizPublisher::Options rviz_opt;
        rviz_opt.pub_current_scan = yaml.GetValue<bool>("rviz", "pub_current_scan");
        rviz_opt.pub_local_map = yaml.GetValue<bool>("rviz", "pub_local_map");
        rviz_opt.pub_global_map = yaml.GetValue<bool>("rviz", "pub_global_map");
        rviz_opt.pub_dynamic_map = true;
        rviz_opt.pub_trajectory = yaml.GetValue<bool>("rviz", "pub_trajectory");
        rviz_opt.pub_keyframes = false;
        rviz_opt.pub_loop_edges = false;
        rviz_opt.local_map_downsample = yaml.GetValue<int>("rviz", "local_map_downsample");
        rviz_opt.global_map_downsample = yaml.GetValue<int>("rviz", "global_map_downsample");
        rviz_pub_ = std::make_shared<RVizPublisher>(node_, rviz_opt);
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
        map_loaded_ = true;
        loc_started_ = false;
        
        // 将 RViz Publisher 传递给 Localization
        if (rviz_pub_) {
            loc_->SetRVizPublisher(rviz_pub_);
        }
    } else {
        map_loaded_ = false;
        loc_started_ = false;
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
}

void LocSystem::StartLocalization() {
    if (map_loaded_) {
        loc_started_ = true;
    }
}

void LocSystem::StopLocalization() { loc_started_ = false; }

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning