//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "common/map_layout.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"
#include "wrapper/rviz_publisher.h"

#include <tf2/time.h>
#include <chrono>
#include <filesystem>
#include <iomanip>

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

    /// 读取 TF 配置
    try {
        map_frame_ = yaml.GetValue<std::string>("tf_frames", "map_frame");
        lidar_frame_ = yaml.GetValue<std::string>("tf_frames", "lidar_frame");
        base_link_frame_ = yaml.GetValue<std::string>("tf_frames", "base_link_frame");
        odom_frame_ = yaml.GetValue<std::string>("tf_frames", "odom_frame");
        extend_to_base_ = yaml.GetValue<bool>("tf_frames", "extend_to_base");
        publish_map_to_odom_tf_ = yaml.GetValue<bool>("tf_frames", "publish_map_to_odom_tf");
        LOG(INFO) << "TF frames: map=" << map_frame_ << ", lidar=" << lidar_frame_ << ", base_link=" << base_link_frame_
                  << ", odom=" << odom_frame_;
        LOG(INFO) << "TF options: extend_to_base=" << extend_to_base_
                  << ", publish_map_to_odom=" << publish_map_to_odom_tf_;
    } catch (const std::exception &e) {
        LOG(WARNING) << "Failed to read tf_frames config, using defaults: " << e.what();
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating subscribers ... ";

        imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
        cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
        livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

        rclcpp::QoS qos(10);
        rclcpp::QoS imu_qos(rclcpp::KeepLast(2000));  // 200Hz IMU: large buffer to survive executor delays

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

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

    if (publish_map_to_odom_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    }
    if (publish_map_to_odom_tf_ || extend_to_base_) {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

        /// 读取 RViz Frame ID 配置
        try {
            rviz_opt.map_frame_ = yaml.GetValue<std::string>("tf_frames", "map_frame");
            rviz_opt.base_link_frame_ = yaml.GetValue<std::string>("tf_frames", "base_link_frame");
        } catch (const std::exception &e) {
            LOG(WARNING) << "Failed to read tf_frames for RViz, using defaults: " << e.what();
        }

        rviz_pub_ = std::make_shared<RVizPublisher>(node_, rviz_opt);
    }

    bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
        map_loaded_ = true;
        loc_started_ = false;

        // 提取地图名，打开定位轨迹文件（兼容尾随 '/' 与 slam/ 分层子目录）
        map_name_ = MapNameFromPath(map_path);
        const char *home = std::getenv("HOME");
        if (home && !map_name_.empty()) {
            std::string traj_dir = std::string(home) + "/.dexe/localization/" + map_name_ + "/";
            std::filesystem::create_directories(traj_dir);
            loc_traj_out_.open(traj_dir + "localization_trajectory.txt", std::ios::out | std::ios::trunc);
            if (loc_traj_out_.is_open()) {
                LOG(INFO) << "localization trajectory will be saved to: " << traj_dir << "localization_trajectory.txt";
            } else {
                LOG(WARNING) << "failed to open localization trajectory file: " << traj_dir
                             << "localization_trajectory.txt";
            }
        }

        // 将 RViz Publisher 传递给 Localization
        if (rviz_pub_) {
            loc_->SetRVizPublisher(rviz_pub_);
        }

        loc_->SetLocResultCallback([this](const loc::LocalizationResult &res) -> loc::LocalizationResult {
            loc::LocalizationResult result = res;

            if (extend_to_base_ && !T_base_lidar_valid_) {
                EnsureBaseToLidarTF();
            }

            if (T_base_lidar_valid_) {
                result.pose_ = res.pose_ * T_base_lidar_.inverse();
                if (publish_map_to_odom_tf_) {
                    TryPublishMapToOdomTF(result.pose_, result.timestamp_);
                }
            }

            return result;
        });
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

        // 写 TUM 格式定位轨迹
        if (loc_traj_out_.is_open()) {
            auto state = loc_->GetState();
            if (state.timestamp_ > 0) {
                auto pose = state.GetPose();
                auto t = pose.translation();
                auto q = pose.unit_quaternion();
                char tum_buf[256];
                std::snprintf(tum_buf, sizeof(tum_buf), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", state.timestamp_,
                              t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
                loc_traj_out_ << tum_buf;
            }
        }
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_) {
        loc_->ProcessLivoxLidarMsg(cloud);

        // 写 TUM 格式定位轨迹
        if (loc_traj_out_.is_open()) {
            auto state = loc_->GetState();
            if (state.timestamp_ > 0) {
                auto pose = state.GetPose();
                auto t = pose.translation();
                auto q = pose.unit_quaternion();
                char tum_buf[256];
                std::snprintf(tum_buf, sizeof(tum_buf), "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", state.timestamp_,
                              t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w());
                loc_traj_out_ << tum_buf;
            }
        }
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        spin(node_);
    }
}

bool LocSystem::GetPoseInFrame(const std::string &reference_frame, const std::string &pose_frame,
                               const tf2::TimePoint &time, const tf2::Duration &timeout, SE3 &result) {
    if (!tf_buffer_) {
        LOG(WARNING) << "TF buffer not initialized, cannot lookup " << reference_frame << "<-" << pose_frame << " TF";
        return false;
    }

    try {
        auto tf_stamped = tf_buffer_->lookupTransform(reference_frame, pose_frame, time, timeout);
        result = TransformToSE3(tf_stamped);
        return true;
    } catch (const tf2::TransformException &e) {
        LOG(WARNING) << "Failed to lookup " << pose_frame << "'s pose in " << reference_frame << " : " << e.what();
        return false;
    }
}

bool LocSystem::EnsureBaseToLidarTF() {
    if (T_base_lidar_valid_) {
        return true;
    }

    SE3 result;
    if (GetPoseInFrame(base_link_frame_, lidar_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1), result)) {
        T_base_lidar_ = result;
        T_base_lidar_valid_ = true;
        LOG(INFO) << "Successfully loaded " << lidar_frame_ << "'s pose in " << base_link_frame_
                  << " frame, t=" << T_base_lidar_.translation().transpose()
                  << ", q=" << T_base_lidar_.unit_quaternion().coeffs().transpose();
        return true;
    }

    LOG_EVERY_N(WARNING, 50) << base_link_frame_ << " <- " << lidar_frame_
                             << " TF not available yet, pose not extended to " << base_link_frame_;
    return false;
}

void LocSystem::TryPublishMapToOdomTF(const SE3 &T_map_base, double timestamp) {
    if (!tf_broadcaster_) {
        return;
    }

    SE3 T_odom_base;
    tf2::TimePoint tf_time(std::chrono::nanoseconds(static_cast<int64_t>(timestamp * 1e9)));
    if (!GetPoseInFrame(odom_frame_, base_link_frame_, tf_time, tf2::durationFromSec(0.), T_odom_base)) {
        LOG_EVERY_N(ERROR, 100) << "Failed to lookup " << base_link_frame_ << "'s pose in " << odom_frame_
                                << " frame, cannot publish " << map_frame_ << "-" << odom_frame_ << " TF";
        return;
    }

    SE3 T_map_odom = T_map_base * T_odom_base.inverse();
    tf_broadcaster_->sendTransform(SE3ToTransform(timestamp, T_map_odom, map_frame_, odom_frame_));
}

}  // namespace lightning