#ifndef AMCL_CORE_HPP_
#define AMCL_CORE_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <thread>
#include <deque>
#include <vector>
#include <memory>
#include <map>
#include <optional>
#include <queue>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/sensors/amcl_pose.h"
#include "amcl/sensors/amcl_cloud.h"
#include "amcl/utils/portable_utils.hpp"
#include "amcl/core/amcl_param.hpp"

#include "scan_match/grid_2d.h"
#include "scan_match/probability_grid.h"
#include "scan_match/submap_2d.h"
#include "scan_match/value_conversion_tables.h"
#include "scan_match/scan_matching_options.h"
#include "scan_match/fast_correlative_scan_matcher_2d.h"
#include "scan_match/real_time_correlative_scan_matcher_2d.h"
#include "scan_match/ceres_scan_matcher_2d.h"
#include "sensor/point_cloud.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace carto = cartographer;

namespace amcl
{

inline double normalized_angle(const double angle)
{
  double res = angle;
  while (res > M_PI) {
    res -= 2.0 * M_PI;
  }
  while (res <= -M_PI) {
    res += 2.0 * M_PI;
  }
  return res;
}

#define NEW_UNIFORM_SAMPLING 1

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

typedef struct
{
  double time;
  Eigen::Vector3d pose;
  Eigen::Matrix3d cov;
} PoseWithCovariance;


enum class LocalizationState
{
  LOCALIZATION_FAILED = -2,  // 定位失败
  INITIALIZATION_FAILED = -1,  // 系统初始化中
  NOT_INITAILIED = 0,  // 系统初始化中
  INITIALIZING = 1,  // 系统初始化中
  INITIALIZED  = 2,  // 系统初始化完成
  RELOCALIZING = 3,  // 重定位中
  LOCALIZED    = 4  // 重定位成功
};


class PoseEstimator2D
{
public:
  explicit PoseEstimator2D(const std::shared_ptr<AmclParam> param);
  ~PoseEstimator2D();

  void SetMap(map_t* map);
  void SetMapFolder(const std::string& map_folder) { map_folder_ = map_folder; }
  void SetMapName(const std::string& map_name) { map_name_ = map_name; }
  void SetInitialPose(const amcl_hyp_t& initial_pose);
  void CancelRelocalization();

  void AddOdomData(const AMCLOdomData& odom_data);
  void AddLaserData(const AMCLLaserData& laser_data);

  bool HasSensorLaser(const std::string& sensor_frame) {
    return frame_to_laser_id_.find(sensor_frame) != frame_to_laser_id_.end();
  }

  bool NeedSetInitPoseInFilter();

  void AddSensorLaser(const std::string& sensor_frame,
                      const pf_vector_t& sensor_pose);

  bool GlobalLocalization();

  std::optional<PoseWithCovariance> GetCurrentPose();
  pf_sample_set_t* GetParticleSet() const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetMapCloud() const;
  pcl::PointCloud<pcl::PointXYZ>::Ptr GetScanMatchedCloud();

  LocalizationState GetLocalizationState() const { return loc_state_;};


  bool IsInitialized() const { return is_initialized_; }
  bool IsMapSet() const { return is_map_set_; }
private:
  void FreeMapDependentMemory();
  void ResetParticleFilter();
  void ResetOdomModel();
  void ResetLaserModel();
  void ResetPoseUpdateModel();

  bool IsKeyFrame();
  
  void PreprocessLaserData(AMCLLaserData& laser_data);
  void Process(AMCLLaserData& laser_data);
  bool IsPoseValid(const carto::transform::Rigid2d& pose,
                   const carto::transform::Rigid2d& target,
                   const double dist_th = 0.5,
                   const double yaw_th = 0.765);
  bool ApplyInitialPose();

  // void ApplyGlobalLocalization(const amcl_hyp_t& initial_pose);

  // bool CheckPose();

  static pf_vector_t UniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
  static std::vector<std::pair<int, int>> free_space_indices_;
#endif

  std::unique_ptr<carto::mapping::Submap2D>
    CreateGridFromMap(const map_t* map);

  std::unique_ptr<carto::mapping::Submap2D>
    CreateGridFromYaml(const std::string& yaml_filename);

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    CreateCloudMap(const std::string& cloud_filename);

  bool InitScanMatcher();

  std::optional<AMCLOdomData> GetCurrOdomData(const double& time);

  carto::sensor::PointCloud GetHighlightPointCloud(const carto::sensor::PointCloud& cloud);
  carto::sensor::PointCloud ToPointCloud(const AMCLLaserData& laser_data);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ToPCLPointCloud(const AMCLLaserData& laser_data);
private:
  LocalizationState loc_state_ = LocalizationState::NOT_INITAILIED;

  // map data
  map_t* map_;

  // Particle filter
  pf_t *particle_filter_;
  int resample_count_ = 0;

  std::shared_ptr<AmclParam> param_;

  AMCLOdom* amcl_odom_;
  AMCLLaser* amcl_laser_;
  AMCLCloud* amcl_cloud_;
  AMCLPose* amcl_pose_;

  std::mutex data_mutex_;
  AMCLLaserData curr_laser_data_;
  AMCLCloudData curr_cloud_data_;
  AMCLOdomData curr_odom_data_;
  AMCLPoseData curr_pose_data_;

  std::unique_ptr<AMCLOdomData> last_odom_data_;

  std::vector<AMCLLaser*> amcl_lasers_;
  std::vector<AMCLCloud*> amcl_clouds_;
  std::vector<AMCLPose*> amcl_poses_;
  std::map<std::string, int> frame_to_laser_id_;
  std::map<std::string, pf_vector_t> laser_pose_;

  bool need_update_ = false;

  std::deque<AMCLOdomData> odom_data_buff_;
  std::deque<AMCLLaserData> laser_data_buff_;

  bool has_new_pose_ = false;
  PoseWithCovariance current_pose_;
  PoseWithCovariance last_pose_;

  // scan matching
  std::unique_ptr<carto::mapping::Submap2D> map_grid_;
  carto::mapping::ValueConversionTables conversion_tables_;
  std::unique_ptr<carto::mapping::scan_matching::FastCorrelativeScanMatcher2D>
  fast_correlative_scan_matcher_;
  std::unique_ptr<carto::mapping::scan_matching::CeresScanMatcher2D>
  ceres_scan_matcher_;
  std::unique_ptr<carto::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>
  real_time_correlative_scan_matcher_;

  std::shared_ptr<carto::sensor::PointCloud> curr_point_cloud_;
  std::shared_ptr<carto::sensor::PointCloud> highlight_point_cloud_;
  std::shared_ptr<carto::mapping::scan_matching::ScanMatchingOptions> options_;
  std::shared_ptr<carto::sensor::PointCloud> scan_matched_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> map_cloud_kdtree_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr highlight_map_cloud_;
  std::unique_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> highlight_map_cloud_kdtree_;

  amcl_hyp_t initial_pose_hyp_;

  bool has_new_initial_pose_ = false;
  bool need_global_localization_ = false;

  std::deque<PoseWithCovariance> pose_mean_queue_;

  bool is_initialized_ = false;
  bool is_map_set_ = false;
  std::string map_name_ = "";
  std::string map_folder_ = "";

  bool need_set_filter_initial_pose_ = false;
  uint ekf_frame_cnt_ = 0;
  std::shared_ptr<gtsam::ExtendedKalmanFilter<gtsam::Pose2>> ekf_;
  gtsam::Symbol last_meas_symol_;
  gtsam::Pose2 last_fusion_pose_;
  gtsam::Pose2 fusion_pose_;

  gtsam::Pose2 last_odom_pose2_;
  gtsam::Pose2 curr_odom_pose2_;

  std::shared_ptr<carto::transform::Rigid2d> last_local_pose_;
  carto::transform::Rigid3d delta_pose_by_odom_;
};

} // namespace amcl

#endif // AMCL_CORE_HPP_
