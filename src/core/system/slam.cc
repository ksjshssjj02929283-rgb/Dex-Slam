//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"
#include "wrapper/rviz_publisher.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (yaml["rviz"] && yaml["rviz"]["enable"]) {
        options_.with_rviz_ = yaml["rviz"]["enable"].as<bool>();
    }

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui_->Init();

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_loop_closing_) {
            /// 当发生回环时，触发一次后端异步重绘（使用优化后位姿重建地图并回传给前端）
            lc_->SetLoopClosedCB([this]() { g2p5_->RedrawGlobalMapBackend(); });
        }

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (options_.online_mode_) {
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);
        // qos.best_effort();

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
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
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", false);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", false);
            });

        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); });

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

        if (options_.with_rviz_) {
            LOG(INFO) << "slam with RViz visualization";
            RVizPublisher::Options rviz_opt;
            if (yaml["rviz"]) {
                rviz_opt.pub_current_scan = yaml["rviz"]["pub_current_scan"].as<bool>(true);
                rviz_opt.pub_local_map = yaml["rviz"]["pub_local_map"].as<bool>(true);
                rviz_opt.pub_trajectory = yaml["rviz"]["pub_trajectory"].as<bool>(true);
                rviz_opt.pub_keyframes = yaml["rviz"]["pub_keyframes"].as<bool>(true);
                rviz_opt.pub_loop_edges = yaml["rviz"]["pub_loop_edges"].as<bool>(true);
                rviz_opt.local_map_downsample = yaml["rviz"]["local_map_downsample"].as<int>(5);
            }
            rviz_pub_ = std::make_shared<RVizPublisher>(node_, rviz_opt);
        }

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    map_name_ = request->map_id;
    std::string save_path = "./data/" + map_name_ + "/";

    SaveMap(save_path);
    response->response = 0;
}

void SlamSystem::SaveMap(const std::string& path) {
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    LOG(INFO) << "slam map saving to " << save_path;

    // ---- Step 1: 对最后一帧强制执行一次回环检测 + 全局优化 ----
    if (options_.with_loop_closing_ && lc_) {
        LOG(INFO) << "SaveMap: running final loop closure check and global optimization...";
        lc_->FinalizeAndWait();
        LOG(INFO) << "SaveMap: final optimization done";
    }

    // ---- Step 2: 保存地图文件 ----
    if (!std::filesystem::exists(save_path)) {
        std::filesystem::create_directories(save_path);
    } else {
        std::filesystem::remove_all(save_path);
        std::filesystem::create_directories(save_path);
    }

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    auto global_map = lio_->GetGlobalMap(!options_.with_loop_closing_);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    TiledMap::Options tm_options;
    tm_options.map_path_ = save_path;

    TiledMap tm(tm_options);
    SE3 start_pose = lio_->GetAllKeyframes().front()->GetOptPose();
    tm.ConvertFromFullPCD(global_map, start_pose, save_path);

    pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    // ---- Step 3: 保存 2D 栅格地图（重建 + 可选噪声过滤 + 写 map.png/map.yaml）----
    if (options_.with_gridmap_ && g2p5_) {
        g2p5_->SaveGridMap(save_path);
    }

    LOG(INFO) << "map saved";
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (running_ == false) {
        return;
    }
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    // 发布实时 scan 和 pose（每帧都发布）
    if (rviz_pub_) {
        auto state = lio_->GetState();
        auto scan = lio_->GetScanUndist();
        if (scan && scan->size() > 0) {
            rviz_pub_->UpdateCurrentScan(scan, state.GetPose());
            rviz_pub_->UpdateCurrentPose(state.GetPose());
            rviz_pub_->UpdateNavState(state);
        }
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
        
        // 更新UI的回环调试信息
        if (options_.with_loop_closing_) {
            // 更新回环边
            ui_->UpdateLoopEdges(lc_->GetLoopEdges());
            
            // 更新回环调试点云
            auto loop_debug_data = lc_->GetLoopDebugData();
            if (loop_debug_data.valid) {
                ui_->UpdateLoopDebug(
                    loop_debug_data.loop_submap,
                    loop_debug_data.current_scan,
                    loop_debug_data.aligned_scan,
                    loop_debug_data.loop_kf_pose,
                    loop_debug_data.current_kf_pose,
                    loop_debug_data.aligned_pose,
                    loop_debug_data.ndt_score);
            }
        }
    }

    if (rviz_pub_) {
        rviz_pub_->UpdateSingleKeyframe(cur_kf_);
        rviz_pub_->UpdateCurrentScan(cur_kf_->GetCloud(), cur_kf_->GetLIOPose());
        rviz_pub_->UpdateFrontendPath(cur_kf_->GetLIOPose());
        rviz_pub_->UpdateKeyframes(lio_->GetAllKeyframes());
        if (options_.with_loop_closing_) {
            rviz_pub_->UpdateLoopEdges(lc_->GetLoopEdges());
            
            // 更新回环调试点云（包含submap、原始scan和配准结果）
            auto loop_debug_data = lc_->GetLoopDebugData();
            if (loop_debug_data.valid) {
                rviz_pub_->UpdateLoopDebugClouds(
                    loop_debug_data.loop_submap,
                    loop_debug_data.current_scan,
                    loop_debug_data.aligned_scan,
                    loop_debug_data.loop_kf_pose,
                    loop_debug_data.current_kf_pose,
                    loop_debug_data.aligned_pose,
                    loop_debug_data.ndt_score);
            }
        }

        // 构建局部地图（最近10个关键帧）
        auto all_kfs = lio_->GetAllKeyframes();
        if (all_kfs.size() > 0) {
            std::map<int, CloudPtr> local_map;
            int start_idx = std::max(0, (int)all_kfs.size() - 10);
            for (size_t i = start_idx; i < all_kfs.size(); ++i) {
                local_map[i] = all_kfs[i]->GetCloud();
            }
            rviz_pub_->UpdateLocalMap(local_map);
        }
    }

    // 发布 TF
    if (tf_broadcaster_ != nullptr) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = node_->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";

        auto pose = cur_kf_->GetLIOPose();
        auto t = pose.translation();
        auto q = pose.unit_quaternion();

        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf_msg);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (running_ == false) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    // 发布实时 scan 和 pose（每帧都发布）
    if (rviz_pub_) {
        auto state = lio_->GetState();
        auto scan = lio_->GetScanUndist();
        if (scan && scan->size() > 0) {
            rviz_pub_->UpdateCurrentScan(scan, state.GetPose());
            rviz_pub_->UpdateCurrentPose(state.GetPose());
            rviz_pub_->UpdateNavState(state);
        }
    }

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
        
        // 更新UI的回环调试信息
        if (options_.with_loop_closing_) {
            // 更新回环边
            ui_->UpdateLoopEdges(lc_->GetLoopEdges());
            
            // 更新回环调试点云
            auto loop_debug_data = lc_->GetLoopDebugData();
            if (loop_debug_data.valid) {
                ui_->UpdateLoopDebug(
                    loop_debug_data.loop_submap,
                    loop_debug_data.current_scan,
                    loop_debug_data.aligned_scan,
                    loop_debug_data.loop_kf_pose,
                    loop_debug_data.current_kf_pose,
                    loop_debug_data.aligned_pose,
                    loop_debug_data.ndt_score);
            }
        }
    }

    if (rviz_pub_) {
        rviz_pub_->UpdateSingleKeyframe(cur_kf_);
        rviz_pub_->UpdateCurrentScan(cur_kf_->GetCloud(), cur_kf_->GetLIOPose());
        rviz_pub_->UpdateFrontendPath(cur_kf_->GetLIOPose());
        rviz_pub_->UpdateKeyframes(lio_->GetAllKeyframes());
        if (options_.with_loop_closing_) {
            rviz_pub_->UpdateLoopEdges(lc_->GetLoopEdges());
            
            // 更新回环调试点云（包含submap、原始scan和配准结果）
            auto loop_debug_data = lc_->GetLoopDebugData();
            if (loop_debug_data.valid) {
                rviz_pub_->UpdateLoopDebugClouds(
                    loop_debug_data.loop_submap,
                    loop_debug_data.current_scan,
                    loop_debug_data.aligned_scan,
                    loop_debug_data.loop_kf_pose,
                    loop_debug_data.current_kf_pose,
                    loop_debug_data.aligned_pose,
                    loop_debug_data.ndt_score);
            }
        }

        // 构建局部地图（最近10个关键帧）
        auto all_kfs = lio_->GetAllKeyframes();
        if (all_kfs.size() > 0) {
            std::map<int, CloudPtr> local_map;
            int start_idx = std::max(0, (int)all_kfs.size() - 10);
            for (size_t i = start_idx; i < all_kfs.size(); ++i) {
                local_map[i] = all_kfs[i]->GetCloud();
            }
            rviz_pub_->UpdateLocalMap(local_map);
        }
    }

    // 发布 TF
    if (tf_broadcaster_ != nullptr) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = node_->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link";

        auto pose = cur_kf_->GetLIOPose();
        auto t = pose.translation();
        auto q = pose.unit_quaternion();

        tf_msg.transform.translation.x = t.x();
        tf_msg.transform.translation.y = t.y();
        tf_msg.transform.translation.z = t.z();
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();

        tf_broadcaster_->sendTransform(tf_msg);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        spin(node_);
    }
}

}  // namespace lightning
