#include "wrapper/rviz_publisher.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "voxel_map/voxel_map_vis.hpp"

namespace lightning {

RVizPublisher::RVizPublisher(rclcpp::Node::SharedPtr node, Options options)
    : node_(node), options_(options) {

    if (options_.pub_current_scan) {
        scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/cloud/current_scan", 10);
    }

    if (options_.pub_local_map) {
        local_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/cloud/local_map", 10);
    }

    if (options_.pub_global_map) {
        global_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/cloud/global_map", 10);
    }

    if (options_.pub_dynamic_map) {
        dynamic_map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lightning/cloud/dynamic_map", 10);
    }

    if (options_.pub_trajectory) {
        frontend_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/lightning/path/frontend", 10);
        backend_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/lightning/path/backend", 10);
        realtime_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
            "/lightning/path/realtime", 10);
        current_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lightning/pose/current", 10);

        frontend_path_.header.frame_id = "map";
        backend_path_.header.frame_id = "map";
        realtime_path_.header.frame_id = "map";
    }

    if (options_.pub_keyframes) {
        keyframes_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lightning/keyframes", 10);
    }

    if (options_.pub_loop_edges) {
        loop_edges_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lightning/loop_edges", 10);
    }

    // 回环调试点云发布器（始终创建，用于调试）
    loop_submap_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightning/loop/submap", 10);
    loop_scan_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightning/loop/scan", 10);
    loop_aligned_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightning/loop/aligned", 10);

    bbs_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lightning/bbs/cloud", 10);

    if (options_.pub_voxel_map) {
        voxel_map_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/lightning/voxel_map/planes", 10);
    }

    if (options_.pub_gridmap) {
        grid_map_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/lightning/grid_map", rclcpp::QoS(1).transient_local());
    }

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RVizPublisher::PublishTimerCallback, this));
}

void RVizPublisher::UpdateCurrentScan(CloudPtr cloud, const SE3& pose) {
    std::lock_guard<std::mutex> lock(mtx_scan_);
    current_scan_ = cloud;
    current_scan_pose_ = pose;
    scan_updated_ = true;
}

void RVizPublisher::UpdateLocalMap(const std::map<int, CloudPtr>& cloud_map) {
    std::lock_guard<std::mutex> lock(mtx_map_);
    local_map_ = cloud_map;
    map_updated_ = true;
}

void RVizPublisher::UpdateGlobalMap(const std::map<int, CloudPtr>& cloud_map) {
    std::lock_guard<std::mutex> lock(mtx_map_);
    global_map_ = cloud_map;
    global_map_updated_ = true;
}

void RVizPublisher::UpdateDynamicMap(const std::map<int, CloudPtr>& cloud_map) {
    std::lock_guard<std::mutex> lock(mtx_map_);
    // 需要深拷贝，因为动态地图可能会被清空
    dynamic_map_.clear();
    for (const auto& [id, cloud] : cloud_map) {
        if (cloud) {
            CloudPtr c(new PointCloudType());
            *c = *cloud;
            dynamic_map_[id] = c;
        }
    }
    dynamic_map_updated_ = true;
}

void RVizPublisher::UpdateKeyframes(const std::vector<Keyframe::Ptr>& kfs) {
    std::lock_guard<std::mutex> lock(mtx_kf_);
    keyframes_ = kfs;
}

void RVizPublisher::UpdateSingleKeyframe(Keyframe::Ptr kf) {
    std::lock_guard<std::mutex> lock(mtx_kf_);
    keyframes_.push_back(kf);
}

void RVizPublisher::UpdateLoopEdges(const std::vector<std::pair<int, int>>& edges) {
    std::lock_guard<std::mutex> lock(mtx_kf_);
    loop_edges_ = edges;
}

void RVizPublisher::UpdateLoopDebugClouds(CloudPtr loop_submap, CloudPtr current_scan, CloudPtr aligned_scan,
                                          const SE3& loop_pose, const SE3& current_pose, const SE3& aligned_pose,
                                          double ndt_score) {
    std::lock_guard<std::mutex> lock(mtx_loop_debug_);
    
    if (loop_submap && !loop_submap->empty() && 
        current_scan && !current_scan->empty() &&
        aligned_scan && !aligned_scan->empty()) {
        // 深拷贝点云
        loop_submap_ = CloudPtr(new PointCloudType());
        loop_scan_ = CloudPtr(new PointCloudType());
        loop_aligned_ = CloudPtr(new PointCloudType());
        *loop_submap_ = *loop_submap;
        *loop_scan_ = *current_scan;
        *loop_aligned_ = *aligned_scan;
        
        loop_submap_pose_ = loop_pose;
        loop_scan_pose_ = current_pose;
        loop_aligned_pose_ = aligned_pose;
        loop_ndt_score_ = ndt_score;
        loop_debug_updated_ = true;
        
        LOG(INFO) << "RViz: Updated loop debug clouds, submap: " << loop_submap_->size() 
                  << " pts, scan: " << loop_scan_->size() 
                  << " pts, aligned: " << loop_aligned_->size()
                  << " pts, NDT score: " << ndt_score;
    }
}

void RVizPublisher::UpdateBBSResult(CloudPtr src_cloud, const SE3& pose, double score_pct,
                                    double elapsed_ms, const std::string& trigger_msg) {
    std::lock_guard<std::mutex> lock(mtx_bbs_);
    if (src_cloud && !src_cloud->empty()) {
        // 将 BBS 源点云按解算位姿变换到世界坐标系，与 Pangolin 中 bbs_cloud_ui_ 行为一致
        bbs_cloud_ = CloudPtr(new PointCloudType());
        pcl::transformPointCloud(*src_cloud, *bbs_cloud_, pose.matrix());
        bbs_updated_ = true;

        LOG(INFO) << "RViz: Updated BBS cloud: " << bbs_cloud_->size()
                  << " pts, score=" << score_pct << ", time=" << elapsed_ms
                  << "ms, trigger=" << trigger_msg;
    }
}

void RVizPublisher::UpdateFrontendPath(const SE3& pose) {
    std::lock_guard<std::mutex> lock(mtx_path_);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "map";

    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    frontend_path_.poses.push_back(pose_msg);
    if (frontend_path_.poses.size() > 10000) {
        frontend_path_.poses.erase(frontend_path_.poses.begin());
    }
}

void RVizPublisher::UpdateBackendPath(const SE3& pose) {
    std::lock_guard<std::mutex> lock(mtx_path_);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "map";

    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    backend_path_.poses.push_back(pose_msg);
    if (backend_path_.poses.size() > 10000) {
        backend_path_.poses.erase(backend_path_.poses.begin());
    }
}

void RVizPublisher::UpdateCurrentPose(const SE3& pose) {
    std::lock_guard<std::mutex> lock(mtx_path_);
    current_pose_ = pose;
    pose_updated_ = true;
    
    // 更新实时轨迹
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "map";

    auto t = pose.translation();
    auto q = pose.unit_quaternion();
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
   pose_msg.pose.orientation.z = q.z();

   if (++realtime_path_cnt_ % 5 == 0) {
       realtime_path_.poses.push_back(pose_msg);
   }
   // 限制路径点数，避免Path消息超过FastDDS 64KB传输上限被丢弃
   if (realtime_path_.poses.size() > 10000) {
       realtime_path_.poses.erase(realtime_path_.poses.begin());
   }
}

void RVizPublisher::UpdateNavState(const NavState& state) {
    std::lock_guard<std::mutex> lock(mtx_path_);
    current_nav_state_ = state;
    current_pose_ = state.GetPose();
    pose_updated_ = true;
}

void RVizPublisher::PublishTimerCallback() {
    auto now = node_->now();

    // Publish current scan (变换到 map 坐标系)
    if (options_.pub_current_scan && scan_updated_) {
        std::lock_guard<std::mutex> lock(mtx_scan_);
        if (current_scan_ && current_scan_->size() > 0 && scan_pub_) {
            // 将 scan 从 body 坐标系变换到 map 坐标系
            CloudPtr scan_in_map(new PointCloudType);
            pcl::transformPointCloud(*current_scan_, *scan_in_map, current_scan_pose_.matrix().cast<float>());
            
            auto msg = ToRosCloud(scan_in_map, "map");
            msg.header.stamp = now;
            scan_pub_->publish(msg);
        }
        scan_updated_ = false;
    }

    // Publish local map (每个关键帧的点云变换到 map 坐标系)
    if (options_.pub_local_map && map_updated_) {
        std::lock_guard<std::mutex> lock(mtx_map_);
        if (!local_map_.empty() && local_map_pub_) {
            CloudPtr merged(new PointCloudType);
            
            // 注意：local_map_ 中存储的是 {keyframe_id: cloud_in_body} 的对应关系
            // 需要从 keyframes_ 中获取对应的位姿，将点云变换到 map 坐标系
            std::lock_guard<std::mutex> lock_kf(mtx_kf_);
            for (const auto& [id, cloud] : local_map_) {
                if (cloud && id < (int)keyframes_.size() && keyframes_[id]) {
                    CloudPtr cloud_in_map(new PointCloudType);
                    auto pose = keyframes_[id]->GetOptPose();
                    pcl::transformPointCloud(*cloud, *cloud_in_map, pose.matrix().cast<float>());
                    *merged += *cloud_in_map;
                }
            }

            // Downsample
            if (options_.local_map_downsample > 1 && merged->size() > 0) {
                CloudPtr downsampled(new PointCloudType);
                for (size_t i = 0; i < merged->size(); i += options_.local_map_downsample) {
                    downsampled->push_back(merged->points[i]);
                }
                merged = downsampled;
            }

            auto msg = ToRosCloud(merged, "map");
            msg.header.stamp = now;
            local_map_pub_->publish(msg);
        }
        map_updated_ = false;
    }

    // Publish global map (每个关键帧的点云变换到 map 坐标系)
    if (options_.pub_global_map && global_map_updated_) {
        std::lock_guard<std::mutex> lock(mtx_map_);
        if (!global_map_.empty() && global_map_pub_) {
            CloudPtr merged(new PointCloudType);
            
            std::lock_guard<std::mutex> lock_kf(mtx_kf_);
            const bool use_kf_pose = !keyframes_.empty();
            for (const auto& [id, cloud] : global_map_) {
                if (!cloud) {
                    continue;
                }

                if (use_kf_pose && id < (int)keyframes_.size() && keyframes_[id]) {
                    CloudPtr cloud_in_map(new PointCloudType);
                    auto pose = keyframes_[id]->GetOptPose();
                    pcl::transformPointCloud(*cloud, *cloud_in_map, pose.matrix().cast<float>());
                    *merged += *cloud_in_map;
                } else if (!use_kf_pose) {
                    // 定位模式下 map_->GetStaticCloud() 已经是 map 坐标系，直接拼接发布。
                    *merged += *cloud;
                }
            }

            // Downsample
            if (options_.global_map_downsample > 1 && merged->size() > 0) {
                CloudPtr downsampled(new PointCloudType);
                for (size_t i = 0; i < merged->size(); i += options_.global_map_downsample) {
                    downsampled->push_back(merged->points[i]);
                }
                merged = downsampled;
            }

            auto msg = ToRosCloud(merged, "map");
            msg.header.stamp = now;
            global_map_pub_->publish(msg);
        }
        global_map_updated_ = false;
    }

    // Publish dynamic map (每个关键帧的点云变换到 map 坐标系)
    if (options_.pub_dynamic_map && dynamic_map_updated_) {
        std::lock_guard<std::mutex> lock(mtx_map_);
        if (!dynamic_map_.empty() && dynamic_map_pub_) {
            CloudPtr merged(new PointCloudType);
            
            std::lock_guard<std::mutex> lock_kf(mtx_kf_);
            const bool use_kf_pose = !keyframes_.empty();
            for (const auto& [id, cloud] : dynamic_map_) {
                if (!cloud) {
                    continue;
                }

                if (use_kf_pose && id < (int)keyframes_.size() && keyframes_[id]) {
                    CloudPtr cloud_in_map(new PointCloudType);
                    auto pose = keyframes_[id]->GetOptPose();
                    pcl::transformPointCloud(*cloud, *cloud_in_map, pose.matrix().cast<float>());
                    *merged += *cloud_in_map;
                } else if (!use_kf_pose) {
                    // 定位模式下 map_->GetDynamicCloud() 已经是 map 坐标系，直接拼接发布。
                    *merged += *cloud;
                }
            }

            // Downsample
            if (options_.dynamic_map_downsample > 1 && merged->size() > 0) {
                CloudPtr downsampled(new PointCloudType);
                for (size_t i = 0; i < merged->size(); i += options_.dynamic_map_downsample) {
                    downsampled->push_back(merged->points[i]);
                }
                merged = downsampled;
            }

            auto msg = ToRosCloud(merged, "map");
            msg.header.stamp = now;
            dynamic_map_pub_->publish(msg);
        }
        dynamic_map_updated_ = false;
    }

    // Publish trajectories
    if (options_.pub_trajectory) {
        std::lock_guard<std::mutex> lock(mtx_path_);
        if (frontend_path_pub_ && !frontend_path_.poses.empty()) {
            frontend_path_.header.stamp = now;
            frontend_path_pub_->publish(frontend_path_);
        }
        if (backend_path_pub_ && !backend_path_.poses.empty()) {
            backend_path_.header.stamp = now;
            backend_path_pub_->publish(backend_path_);
        }
        if (realtime_path_pub_ && !realtime_path_.poses.empty()) {
            realtime_path_.header.stamp = now;
            realtime_path_pub_->publish(realtime_path_);
        }
        
        // Publish current pose
        if (current_pose_pub_ && pose_updated_) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = now;
            pose_msg.header.frame_id = "map";
            
            auto t = current_pose_.translation();
            auto q = current_pose_.unit_quaternion();
            pose_msg.pose.position.x = t.x();
            pose_msg.pose.position.y = t.y();
            pose_msg.pose.position.z = t.z();
            pose_msg.pose.orientation.w = q.w();
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            
            current_pose_pub_->publish(pose_msg);
            pose_updated_ = false;
        }
    }

    // Publish keyframes
    if (options_.pub_keyframes && keyframes_pub_) {
        std::lock_guard<std::mutex> lock(mtx_kf_);
        if (!keyframes_.empty()) {
            visualization_msgs::msg::MarkerArray markers;
            for (size_t i = 0; i < keyframes_.size(); ++i) {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = now;
                marker.ns = "keyframes";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;

                auto t = keyframes_[i]->GetOptPose().translation();
                marker.pose.position.x = t.x();
                marker.pose.position.y = t.y();
                marker.pose.position.z = t.z();
                marker.pose.orientation.w = 1.0;

                marker.scale.x = 0.3;
                marker.scale.y = 0.3;
                marker.scale.z = 0.3;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;

                markers.markers.push_back(marker);
            }
            keyframes_pub_->publish(markers);
        }
    }

    // Publish loop edges
    if (options_.pub_loop_edges && loop_edges_pub_) {
        std::lock_guard<std::mutex> lock(mtx_kf_);
        if (!loop_edges_.empty() && !keyframes_.empty()) {
            visualization_msgs::msg::MarkerArray markers;
            for (size_t i = 0; i < loop_edges_.size(); ++i) {
                auto [id1, id2] = loop_edges_[i];
                if (id1 >= (int)keyframes_.size() || id2 >= (int)keyframes_.size()) continue;

                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = now;
                marker.ns = "loop_edges";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                marker.action = visualization_msgs::msg::Marker::ADD;

                geometry_msgs::msg::Point p1, p2;
                auto t1 = keyframes_[id1]->GetOptPose().translation();
                auto t2 = keyframes_[id2]->GetOptPose().translation();
                p1.x = t1.x(); p1.y = t1.y(); p1.z = t1.z();
                p2.x = t2.x(); p2.y = t2.y(); p2.z = t2.z();

                marker.points.push_back(p1);
                marker.points.push_back(p2);

                marker.scale.x = 0.05;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.color.a = 0.8;

                markers.markers.push_back(marker);
            }
            loop_edges_pub_->publish(markers);
        }
    }

    // 发布回环调试点云
    if (loop_debug_updated_) {
        std::lock_guard<std::mutex> lock(mtx_loop_debug_);
        
        if (loop_submap_ && !loop_submap_->empty()) {
            // submap已经在世界坐标系下，直接发布
            auto msg = ToRosCloud(loop_submap_, "map");
            msg.header.stamp = now;
            loop_submap_pub_->publish(msg);
        }
        
        if (loop_scan_ && !loop_scan_->empty()) {
            // 将当前scan从body坐标系转换到map坐标系
            CloudPtr scan_in_map(new PointCloudType());
            pcl::transformPointCloud(*loop_scan_, *scan_in_map, loop_scan_pose_.matrix());
            auto msg = ToRosCloud(scan_in_map, "map");
            msg.header.stamp = now;
            loop_scan_pub_->publish(msg);
        }
        
        if (loop_aligned_ && !loop_aligned_->empty()) {
            // 将配准后的scan从body坐标系转换到map坐标系（使用配准后的位姿）
            CloudPtr aligned_in_map(new PointCloudType());
            pcl::transformPointCloud(*loop_aligned_, *aligned_in_map, loop_aligned_pose_.matrix());
            auto msg = ToRosCloud(aligned_in_map, "map");
            msg.header.stamp = now;
            loop_aligned_pub_->publish(msg);
            
            LOG(INFO) << "RViz: Published loop debug clouds with NDT score: " << loop_ndt_score_;
        }
        
        loop_debug_updated_ = false;
    }

    // 发布 BBS 重定位源点云（已变换到世界坐标系）
    if (bbs_updated_ && bbs_cloud_pub_) {
        std::lock_guard<std::mutex> lock(mtx_bbs_);
        if (bbs_cloud_ && !bbs_cloud_->empty()) {
            auto msg = ToRosCloud(bbs_cloud_, "map");
            msg.header.stamp = now;
            bbs_cloud_pub_->publish(msg);
        }
        bbs_updated_ = false;
    }

    // 发布 VoxelMap 平面（MarkerArray）
    if (options_.pub_voxel_map && voxel_map_updated_ && voxel_map_pub_) {
        std::lock_guard<std::mutex> lock(mtx_voxel_map_);
        auto marker_array = voxel_map::toMarkerArray(voxel_map_planes_, "map", "voxel_plane");
        // 为所有 marker 补充时间戳
        for (auto& m : marker_array.markers) {
            m.header.stamp = now;
        }
        voxel_map_pub_->publish(marker_array);
        voxel_map_updated_ = false;
    }

    // 发布 2D 栅格地图（OccupancyGrid）
    if (options_.pub_gridmap && gridmap_updated_ && grid_map_pub_) {
        std::lock_guard<std::mutex> lock(mtx_gridmap_);
        grid_map_.header.stamp = now;
        grid_map_pub_->publish(grid_map_);
        gridmap_updated_ = false;
    }
}

void RVizPublisher::UpdateGridMap(nav_msgs::msg::OccupancyGrid grid) {
    std::lock_guard<std::mutex> lock(mtx_gridmap_);
    grid_map_ = std::move(grid);
    grid_map_.header.frame_id = "map";
    gridmap_updated_ = true;
}

void RVizPublisher::UpdateVoxelMap(const std::vector<voxel_map::PlaneVis>& planes) {
    std::lock_guard<std::mutex> lock(mtx_voxel_map_);
    voxel_map_planes_ = planes;
    voxel_map_updated_ = true;
}

sensor_msgs::msg::PointCloud2 RVizPublisher::ToRosCloud(CloudPtr cloud, const std::string& frame_id) {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame_id;
    return msg;
}

}  // namespace lightning
