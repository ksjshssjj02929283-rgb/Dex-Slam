#ifndef FASTER_LIO_LASER_MAPPING_H
#define FASTER_LIO_LASER_MAPPING_H

#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/eigen_types.h"
#include "common/imu.h"
#include "common/keyframe.h"
#include "common/options.h"
#include "core/lio/eskf.hpp"
#include "core/lio/imu_processing.h"
#include "core/maps/ivox3d/ivox3d.h"
#include "core/maps/ivox3d/ivox_manager.hpp"
#include "pointcloud_preprocess.h"
#include "voxel_map/voxel_map_manager.hpp"

#include "livox_ros_driver2/msg/custom_msg.hpp"

namespace lightning {

namespace ui {
class PangolinWindow;
}

class RVizPublisher;

/**
 * laser mapping
 * 目前有个问题：点云在缓存之后，实际处理的并不是最新的那个点云（通常是buffer里的前一个），这是因为bag里的点云用的开始时间戳，导致
 * 点云的结束时间要比IMU多0.1s左右。为了同步最近的IMU，就只能处理缓冲队列里的那个点云，而不是最新的点云
 */
class LaserMapping {
   public:
    struct Options {
        Options() {}

        bool is_in_slam_mode_ = true;  // 是否在slam模式下

        /// 关键帧阈值
        double kf_dis_th_ = 0.5;
        double kf_angle_th_ = 15 * M_PI / 180.0;
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
    using IVoxManagerType = IVoxManager;
    using VoxelMapType = voxel_map::VoxelMapManager;

    LaserMapping(Options options = Options());
    ~LaserMapping();

    /// init without ros
    bool Init(const std::string &config_yaml);

    bool Run();

    // callbacks of lidar and imu
    /// 处理ROS2的点云
    void ProcessPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr &msg);

    /// 处理livox的点云
    void ProcessPointCloud2(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);

    /// 如果已经做了预处理，也可以直接处理点云
    void ProcessPointCloud2(CloudPtr cloud);

    void ProcessIMU(const lightning::IMUPtr &msg_in);

    /// 保存前端的地图
    void SaveMap();

    void SetUI(std::shared_ptr<ui::PangolinWindow> ui) { ui_ = ui; }

    /// 设置 RViz 发布器（用于 voxel map 平面发布）
    void SetRVizPublisher(std::shared_ptr<RVizPublisher> pub) { rviz_publisher_ = pub; }

    /// 获取关键帧
    Keyframe::Ptr GetKeyframe() const { return last_kf_; }

    /// 获取激光的状态
    NavState GetState() const { return state_point_; }

    /// 获取IMU状态
    NavState GetIMUState() {
        std::unique_lock<std::mutex> lock(mtx_imu_buffer_);
        if (p_imu_->IsIMUInited()) {
            return kf_imu_.GetX();
        } else {
            NavState s;
            s.pose_is_ok_ = false;
            return s;
        }
    }

    /// IMU 是否已完成初始化
    bool IsIMUInited() const { return p_imu_ && p_imu_->IsIMUInited(); }

    CloudPtr GetScanUndist() const { return scan_undistort_; }

    /// 获取最新的点云
    CloudPtr GetRecentCloud();

    std::vector<Keyframe::Ptr> GetAllKeyframes() { return all_keyframes_; }

    /**
     * 计算全局地图
     * @param use_lio_pose
     * @return
     */
    CloudPtr GetGlobalMap(bool use_lio_pose, bool use_voxel = true, float res = 0.1);

    /// 获取 VoxelMap 指针（用于外部可视化等）
    std::shared_ptr<VoxelMapType> GetVoxelMap() const { return voxel_map_; }

   private:
    // sync lidar with imu
    bool SyncPackages();

    void ObsModel(NavState &s, ESKF::CustomObservationModel &obs);

    inline void PointBodyToWorld(const PointType &pi, PointType &po) {
        Vec3d p_global(state_point_.rot_ * (state_point_.offset_R_lidar_ * pi.getVector3fMap().cast<double>() +
                                            state_point_.offset_t_lidar_) +
                       state_point_.pos_);

        po.x = p_global(0);
        po.y = p_global(1);
        po.z = p_global(2);
        po.intensity = pi.intensity;
    }

    void MapIncremental();

    bool LoadParamsFromYAML(const std::string &yaml);

    /// 创建关键帧
    void MakeKF();

   private:
    Options options_;
    int lidar_type_ = 0;  // 0: not init, 1: livox (lidar acc in G), 2: velodyne, 3: ouster

    /// modules
    voxel_map::VoxelMapConfig vmap_config_;
    std::shared_ptr<VoxelMapType> voxel_map_ = nullptr;  // localmap in voxel_map
    IVoxConfig ivox_config_;
    IVoxType::Options ivox_options_;                              // 保留用于从 yaml 配置参数后一次性传给 IVoxManager
    std::shared_ptr<IVoxManagerType> ivox_manager_ = nullptr;     // IVox 地图管理器
    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;  // point cloud preprocess
    std::shared_ptr<ImuProcess> p_imu_ = nullptr;                 // imu process

    bool use_ivox_ = true;       // 是否启用 IVox 约束
    bool use_voxel_map_ = true;  // 是否启用 VoxelMap 约束
    bool pub_voxel_map_ = true;  // 是否发布 VoxelMap 平面（可视化）

    /// local map related
    double filter_size_map_min_ = 0;

    /// params
    std::vector<double> extrinT_{3, 0.0};  // lidar-imu translation
    std::vector<double> extrinR_{9, 0.0};  // lidar-imu rotation
    std::string map_file_path_;

    std::vector<Keyframe::Ptr> all_keyframes_;
    Keyframe::Ptr last_kf_ = nullptr;
    int kf_id_ = 0;

    /// point clouds data
    CloudPtr scan_undistort_{new PointCloudType()};   // scan after undistortion
    CloudPtr scan_down_body_{new PointCloudType()};   // downsampled scan in body
    CloudPtr scan_down_world_{new PointCloudType()};  // downsampled scan in world
    pcl::VoxelGrid<PointType> voxel_scan_;            // voxel filter for current scan

    /// VoxelMap matching data
    std::vector<voxel_map::pointWithCov> pv_list_;  // points with covariance for voxel_map
    std::vector<voxel_map::ptpl> ptpl_list_;        // point-to-plane match results (VoxelMap)

    /// IVox matching data (point-to-plane via kNN + esti_plane)
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    std::vector<PointVector> nearest_points_;  // kNN neighbors for each down-sampled point (used for map update dedup)
    std::vector<Vec4f> corr_pts_;              // [px,py,pz, residual] for valid IVox matches
    std::vector<Vec4f> corr_norm_;             // [nx,ny,nz, d] for valid IVox matches
    std::vector<float> residuals_;             // per-point residual for IVox (used for median stats)

    /// IEKF 预缓存：body_cov 和 crossmat 在 IEKF 循环外仅计算一次（VoxelMap 需要）
    std::vector<Eigen::Matrix3d> body_cov_list_;     // body 坐标系下的点测量协方差
    std::vector<Eigen::Vector3d> body_points_list_;  // body 坐标系下的点坐标缓存
    bool iekf_first_iter_ = true;                    // 标记是否为 IEKF 首次迭代（VoxelMap hash 查找）

    std::mutex mtx_imu_buffer_;    // 保护 imu_buffer_, last_timestamp_imu_, kf_imu_
    std::mutex mtx_lidar_buffer_;  // 保护 lidar_buffer_, time_buffer_, last_timestamp_lidar_, lidar_pushed_
    std::deque<double> time_buffer_;

    std::deque<PointCloudType::Ptr> lidar_buffer_;
    std::deque<lightning::IMUPtr> imu_buffer_;

    /// options
    bool keep_first_imu_estimation_ = false;  // 在没有建立地图前，是否要使用前几帧的IMU状态
    double timediff_lidar_wrt_imu_ = 0.0;
    double last_timestamp_lidar_ = 0;
    double lidar_end_time_ = 0;
    double last_timestamp_imu_ = -1.0;
    double first_lidar_time_ = 0.0;
    bool lidar_pushed_ = false;

    bool enable_skip_lidar_ = false;  // 雷达是否需要跳帧（interval > 1 时启用）
    int lidar_process_interval_ = 1;  // 每隔 N 帧处理一次（1=逐帧处理，N=每N帧处理1帧）
    int skip_lidar_cnt_ = 0;

    /// statistics and flags ///
    int scan_count_ = 0;
    int publish_count_ = 0;
    bool flg_first_scan_ = true;
    bool flg_EKF_inited_ = false;
    double lidar_mean_scantime_ = 0.0;
    int scan_num_ = 0;
    int effect_feat_num_ = 0, frame_num_ = 0;

    double last_lidar_time_ = 0;

    ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
    MeasureGroup measures_;  // sync IMU and lidar scan

    ESKF kf_;      // 点云时刻的IMU状态
    ESKF kf_imu_;  // imu 最新时刻的eskf状态

    NavState state_point_;  // ekf current state

    SE3 T_lidar_body_;  // lidar to body transform

    Vec3d pos_lidar_;  // lidar position after eskf update
    SO3 euler_cur_;    // rotation in euler angles
    bool extrinsic_est_en_ = true;
    bool use_aa_ = false;  // use anderson acceleration?

    std::shared_ptr<ui::PangolinWindow> ui_ = nullptr;
    std::shared_ptr<RVizPublisher> rviz_publisher_ = nullptr;  // RViz 发布器（用于 voxel map 平面）
};

}  // namespace lightning

#endif  // FASTER_LIO_LASER_MAPPING_H