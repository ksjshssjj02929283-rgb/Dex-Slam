#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "common/eigen_types.h"
#include "common/keyframe.h"
#include "common/point_def.h"
#include "common/nav_state.h"
#include "voxel_map/voxel_types.hpp"

namespace lightning {

class RVizPublisher {
public:
    struct Options {
        bool pub_current_scan = true;
        bool pub_local_map = true;
        bool pub_global_map = false;
        bool pub_dynamic_map = false;  // 动态地图（定位系统用）
        bool pub_trajectory = true;
        bool pub_keyframes = true;
        bool pub_loop_edges = true;
        bool pub_voxel_map = true;     // VoxelMap 平面可视化
        bool pub_gridmap = true;       // 2D 栅格地图（g2p5 输出）

        int local_map_downsample = 5;
        int global_map_downsample = 10;
        int dynamic_map_downsample = 5;
        double scan_rate = 10.0;
        double map_rate = 1.0;
    };

    RVizPublisher(rclcpp::Node::SharedPtr node, Options options);
    ~RVizPublisher() = default;

    void UpdateCurrentScan(CloudPtr cloud, const SE3& pose);
    void UpdateLocalMap(const std::map<int, CloudPtr>& cloud_map);
    void UpdateGlobalMap(const std::map<int, CloudPtr>& cloud_map);  // 全局地图
    void UpdateDynamicMap(const std::map<int, CloudPtr>& cloud_map);  // 动态地图
    void UpdateKeyframes(const std::vector<Keyframe::Ptr>& kfs);
    void UpdateSingleKeyframe(Keyframe::Ptr kf);  // 单个关键帧更新
    void UpdateLoopEdges(const std::vector<std::pair<int, int>>& edges);
    void UpdateLoopDebugClouds(CloudPtr loop_submap, CloudPtr current_scan, CloudPtr aligned_scan,
                               const SE3& loop_pose, const SE3& current_pose, const SE3& aligned_pose,
                               double ndt_score);  // 回环调试点云（包含配准结果）
    void UpdateBBSResult(CloudPtr src_cloud, const SE3& pose, double score_pct,
                         double elapsed_ms, const std::string& trigger_msg);  // BBS 重定位源点云
    void UpdateFrontendPath(const SE3& pose);
    void UpdateBackendPath(const SE3& pose);
    void UpdateCurrentPose(const SE3& pose);  // 实时位姿
    void UpdateNavState(const NavState& state);  // 完整状态（位姿+速度+偏置）

    /// 更新 VoxelMap 平面数据，下一定时器周期发布 MarkerArray
    void UpdateVoxelMap(const std::vector<voxel_map::PlaneVis>& planes);

    /// 更新 2D 栅格地图，下一定时器周期发布 OccupancyGrid
    void UpdateGridMap(nav_msgs::msg::OccupancyGrid grid);

private:
    void PublishTimerCallback();
    sensor_msgs::msg::PointCloud2 ToRosCloud(CloudPtr cloud, const std::string& frame_id);

    rclcpp::Node::SharedPtr node_;
    Options options_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr frontend_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr backend_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr realtime_path_pub_;  // 实时轨迹
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframes_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_edges_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_submap_pub_;      // 回环submap
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_scan_pub_;        // 回环当前scan
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_aligned_pub_;     // 配准后的scan
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bbs_cloud_pub_;        // BBS 源点云（变换到世界系）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_map_pub_; // VoxelMap 平面
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;           // 2D 栅格地图

    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_scan_;
    std::mutex mtx_map_;
    std::mutex mtx_path_;
    std::mutex mtx_kf_;
    std::mutex mtx_loop_debug_;  // 回环调试数据锁

    CloudPtr current_scan_;
    SE3 current_scan_pose_;
    SE3 current_pose_;
    NavState current_nav_state_;
    std::map<int, CloudPtr> local_map_;
    std::map<int, CloudPtr> global_map_;
    std::map<int, CloudPtr> dynamic_map_;
    nav_msgs::msg::Path frontend_path_;
    nav_msgs::msg::Path backend_path_;
    nav_msgs::msg::Path realtime_path_;  // 实时轨迹
    std::vector<Keyframe::Ptr> keyframes_;
    std::vector<std::pair<int, int>> loop_edges_;
    
    // 回环调试数据
    CloudPtr loop_submap_;
    CloudPtr loop_scan_;
    CloudPtr loop_aligned_;
    SE3 loop_submap_pose_;
    SE3 loop_scan_pose_;
    SE3 loop_aligned_pose_;
    double loop_ndt_score_;

    // BBS 重定位数据
    CloudPtr bbs_cloud_;
    std::mutex mtx_bbs_;
    bool bbs_updated_ = false;

    bool scan_updated_ = false;
    bool map_updated_ = false;
    bool global_map_updated_ = false;
    bool dynamic_map_updated_ = false;
   bool pose_updated_ = false;
   int realtime_path_cnt_ = 0;  // 实时轨迹降采样计数器
   bool loop_debug_updated_ = false;  // 回环调试数据更新标志
    bool voxel_map_updated_ = false;   // VoxelMap 平面更新标志
    bool gridmap_updated_ = false;     // 2D 栅格地图更新标志

    std::mutex mtx_voxel_map_;                           // VoxelMap 平面数据锁
    std::vector<voxel_map::PlaneVis> voxel_map_planes_;  // 缓存的平面数据

    std::mutex mtx_gridmap_;                             // 2D 栅格地图数据锁
    nav_msgs::msg::OccupancyGrid grid_map_;              // 缓存的栅格地图
};

}  // namespace lightning
