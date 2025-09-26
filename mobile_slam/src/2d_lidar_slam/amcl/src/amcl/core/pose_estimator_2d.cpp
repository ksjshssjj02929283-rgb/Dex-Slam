#include "glog/logging.h"

#include "amcl/core/pose_estimator_2d.hpp"
#include "common/config.h"
#include "sensor/voxel_filter.h"

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

#include "tic_toc.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

namespace amcl
{
static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

std::vector<std::pair<int, int>> PoseEstimator2D::free_space_indices_;

PoseEstimator2D::PoseEstimator2D(const std::shared_ptr<AmclParam> param)
: map_(nullptr),
  particle_filter_(nullptr),
  resample_count_(0),
  param_(param),
  amcl_odom_(nullptr),
  amcl_laser_(nullptr),
  amcl_cloud_(nullptr),
  amcl_pose_(nullptr),
  map_cloud_(nullptr),
  has_new_initial_pose_(false),
  is_initialized_(false)
{
  amcl_lasers_.clear();
  amcl_clouds_.clear();
  amcl_poses_.clear();
  frame_to_laser_id_.clear();
  laser_pose_.clear();

  auto file_resolver =
    absl::make_unique<cartographer::common::ConfigurationFileResolver>(
      std::vector<std::string>{cartographer::common::kConfigurationFilesDirectory});
  const std::string code =
    file_resolver->GetFileContentOrDie("scan_matching.lua");
  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
    code, std::move(file_resolver));

  options_ = std::make_shared<carto::mapping::scan_matching::ScanMatchingOptions>(
              carto::mapping::scan_matching::CreateScanMatchingOptions(&lua_parameter_dictionary));

  initial_pose_hyp_.pf_pose_mean = pf_vector_zero();
  initial_pose_hyp_.pf_pose_cov = pf_matrix_zero();
  // time_file_ = std::ofstream("/home/ubt/data/time.txt", std::ios::out);
}

PoseEstimator2D::~PoseEstimator2D()
{
  FreeMapDependentMemory();
}

bool PoseEstimator2D::InitScanMatcher()
{
  if (options_ == nullptr) {
    LOG(ERROR) << "Scan matching options is not set!";
    return false;
  }

  if (map_grid_ == nullptr) {
    LOG(ERROR) << "Grid map is not initalized!";
    return false;
  }

  fast_correlative_scan_matcher_ =
    std::make_unique<carto::mapping::scan_matching::FastCorrelativeScanMatcher2D>(
      *(map_grid_->grid()), options_->fast_correlative_scan_matcher_options);

  ceres_scan_matcher_ =
    std::make_unique<carto::mapping::scan_matching::CeresScanMatcher2D>(
      options_->ceres_scan_matcher_options);

  if (options_->use_online_correlative_scan_matching) {
    real_time_correlative_scan_matcher_ =
      std::make_unique<carto::mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>(
        options_->real_time_correlative_scan_matcher_options);
  }
  return true;
}

void PoseEstimator2D::ResetParticleFilter()
{
  if (particle_filter_ != nullptr) {
    pf_free(particle_filter_);
  }
#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices_.resize(0);
  for (int i = 0; i < map_->size_x; i++) {
    for (int j = 0; j < map_->size_y; j++) {
      if (map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1) {
        free_space_indices_.push_back(std::make_pair(i,j));
      }
    }
  }
#endif

  // Create the particle filter
  particle_filter_ = pf_alloc(param_->min_particles, param_->max_particles,
                param_->alpha_slow, param_->alpha_fast,
                (pf_init_model_fn_t)UniformPoseGenerator,
                (void *)map_);
  pf_set_selective_resampling(particle_filter_,
                              param_->selective_resampling);

  particle_filter_->pop_err = param_->pf_err;
  particle_filter_->pop_z = param_->pf_z;
}

void PoseEstimator2D::ResetOdomModel()
{
  delete amcl_odom_;
  amcl_odom_ = new AMCLOdom();
  assert(amcl_odom_);
  amcl_odom_->SetModel(param_->odom_model_type,
                  param_->alpha1,
                  param_->alpha2,
                  param_->alpha3,
                  param_->alpha4,
                  param_->alpha5);
  LOG(INFO) << "Done initializing Odom model.";
}

void PoseEstimator2D::ResetLaserModel()
{
  delete amcl_laser_;
  amcl_laser_ = new AMCLLaser(param_->max_beams, map_);
  assert(amcl_laser_);
  if (param_->laser_model_type == LASER_MODEL_BEAM) {
    amcl_laser_->SetModelBeam(param_->z_hit,
                         param_->z_short,
                         param_->z_max,
                         param_->z_rand,
                         param_->sigma_hit,
                         param_->lambda_short,
                         0.0);
  } else if (param_->laser_model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB) {
    LOG(INFO) << "Initializing likelihood field model; this can take some time on large maps...";
    amcl_laser_->SetModelLikelihoodFieldProb(param_->z_hit, param_->z_rand, param_->sigma_hit,
          param_->laser_likelihood_max_dist, 
          param_->do_beamskip, param_->beam_skip_distance, 
          param_->beam_skip_threshold, param_->beam_skip_error_threshold);
    LOG(INFO) << "Done initializing likelihood field model.";
  } else {
    LOG(INFO) << "Initializing likelihood field model; this can take some time on large maps...";
    amcl_laser_->SetModelLikelihoodField(param_->z_hit, param_->z_rand, param_->sigma_hit,
                                    param_->laser_likelihood_max_dist);
    LOG(INFO) << "Done initializing likelihood field model.";
  }

  delete amcl_cloud_;
  amcl_cloud_ = new AMCLCloud(param_->max_beams, map_grid_->grid());
  assert(amcl_cloud_);
}

void PoseEstimator2D::ResetPoseUpdateModel()
{
  delete amcl_pose_;
  amcl_pose_ = new AMCLPose();
  assert(amcl_pose_);
  LOG(INFO) << "Done initializing pose updater model.";
}


void PoseEstimator2D::SetInitialPose(const amcl_hyp_t& initial_pose)
{
  initial_pose_hyp_ = initial_pose;
  has_new_initial_pose_ = true;
}

void PoseEstimator2D::CancelRelocalization()
{
  has_new_initial_pose_ = false;
  loc_state_ = LocalizationState::LOCALIZATION_FAILED;
}


bool PoseEstimator2D::IsPoseValid(const carto::transform::Rigid2d& pose,
                           const carto::transform::Rigid2d& target,
                           const double dist_th,
                           const double yaw_th)
{
  double dist_diff = (pose.translation() - target.translation()).norm();
  double yaw_diff = std::abs(pose.rotation().angle() - target.rotation().angle());
  yaw_diff = normalized_angle(yaw_diff);
  LOG(INFO) << "dist diff: " << dist_diff << " yaw diff: " << yaw_diff;
  return dist_diff < dist_th && yaw_diff < yaw_th;
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to nullptr after it is used.
 */
bool PoseEstimator2D::ApplyInitialPose()
{
  if (map_ == nullptr) {
    return false;
  }

  has_new_initial_pose_ = false;

  loc_state_ = LocalizationState::RELOCALIZING;
  // 先将初始位姿初始化
  // initial_pose_hyp_.pf_pose_mean = pf_vector_zero();
  // initial_pose_hyp_.pf_pose_cov = pf_matrix_zero();

  utility::TicToc global_loc_tic;
  if (param_->use_scan_matching && fast_correlative_scan_matcher_ && curr_point_cloud_) {
    LOG(INFO) << "Apply initial pose by fast correlative scan matching.";
    float score = 0.0;
    carto::transform::Rigid2d
    global_pose_estimate = carto::transform::Rigid2d(
                              Eigen::Vector2d(initial_pose_hyp_.pf_pose_mean.v[0],
                                              initial_pose_hyp_.pf_pose_mean.v[1]),
                              Eigen::Rotation2D(initial_pose_hyp_.pf_pose_mean.v[2]));

    LOG(WARNING) << "set initial pose: " << global_pose_estimate;

    if (options_->use_global_matching) {
      fast_correlative_scan_matcher_->MatchFullSubmap(
        *curr_point_cloud_, options_->global_localization_min_score, &score, &global_pose_estimate);
      LOG(INFO) << "global localization score: " << score;

      if (score < options_->global_localization_min_score) {
        LOG(WARNING) << "global localization score is too low, use initial pose instead.";
      }
    }

    carto::transform::Rigid2d fast_correlative_pose;
    bool fast_match_success =
                  fast_correlative_scan_matcher_->Match(
                                    global_pose_estimate,
                                    *curr_point_cloud_,
                                    options_->global_localization_min_score,
                                    &score,
                                    &fast_correlative_pose);
    if (!fast_match_success) {
      LOG(ERROR) << "fast match failed, apply initial pose failed.";
      LOG(ERROR) << "maybe the global localization min score is too small, current score is: " << score;
      loc_state_ = LocalizationState::LOCALIZATION_FAILED;
      return false;
    }

    if (!IsPoseValid(global_pose_estimate, fast_correlative_pose, 1.0)) {
      LOG(ERROR) << "initial pose is not valid, apply initial pose failed.";
      loc_state_ = LocalizationState::LOCALIZATION_FAILED;
      return false;
    }

    global_pose_estimate = fast_correlative_pose;

    carto::transform::Rigid2d online_correlative_pose;
    if (options_->use_online_correlative_scan_matching) {
      const double score = real_time_correlative_scan_matcher_->Match(
        global_pose_estimate, *curr_point_cloud_,
        *map_grid_->grid(), &online_correlative_pose);

      if (score < options_->global_localization_min_score) {
        LOG(WARNING) << "online correlative scan matcher score is too low.";
        LOG(INFO) << "online correlative scan matcher score: " << score;
        loc_state_ = LocalizationState::LOCALIZATION_FAILED;
        return false;
      }

      if (!IsPoseValid(global_pose_estimate, online_correlative_pose, 1.0, 0.765)) {
        LOG(ERROR) << "initial pose is not valid, apply initial pose failed.";
        loc_state_ = LocalizationState::LOCALIZATION_FAILED;
        return false;
      }

      global_pose_estimate = online_correlative_pose;
    }

    auto pose_observation = absl::make_unique<carto::transform::Rigid2d>();
    ceres::Solver::Summary summary;
    ceres_scan_matcher_->Match(global_pose_estimate.translation(),
                               global_pose_estimate,
                               *curr_point_cloud_,
                               *map_grid_->grid(),
                               pose_observation.get(),
                               &summary);

    LOG(INFO) << "global pose estimate: "
              << global_pose_estimate.translation().x() << " "
              << global_pose_estimate.translation().y() << " "
              << global_pose_estimate.rotation().angle();
    LOG(INFO) << "global pose obersevation: "
              << pose_observation->translation().x() << " "
              << pose_observation->translation().y() << " "
              << pose_observation->rotation().angle();

    initial_pose_hyp_.pf_pose_mean.v[0] = pose_observation->translation().x();
    initial_pose_hyp_.pf_pose_mean.v[1] = pose_observation->translation().y();
    initial_pose_hyp_.pf_pose_mean.v[2] = pose_observation->rotation().angle();
    initial_pose_hyp_.pf_pose_cov = pf_matrix_zero();
    initial_pose_hyp_.pf_pose_cov.m[0][0] = param_->init_cov[0];
    initial_pose_hyp_.pf_pose_cov.m[1][1] = param_->init_cov[1];
    initial_pose_hyp_.pf_pose_cov.m[2][2] = param_->init_cov[2];

    last_pose_.time = curr_laser_data_.time;
    // Copy in the covariance, converting from 3-D to 6-D
    last_pose_.pose(0) = initial_pose_hyp_.pf_pose_mean.v[0];
    last_pose_.pose(1) = initial_pose_hyp_.pf_pose_mean.v[1];
    last_pose_.pose(2) = initial_pose_hyp_.pf_pose_mean.v[2];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        last_pose_.cov(i, j) = initial_pose_hyp_.pf_pose_cov.m[i][j];
      }
    }
  }

  assert(particle_filter_ == nullptr);
  pf_init(particle_filter_,
          initial_pose_hyp_.pf_pose_mean,
          initial_pose_hyp_.pf_pose_cov);


  // if (param_->use_ekf) {
  //   double x = initial_pose_hyp_.pf_pose_mean.v[0];
  //   double y = initial_pose_hyp_.pf_pose_mean.v[1];
  //   double theta = initial_pose_hyp_.pf_pose_mean.v[2];
  //   gtsam::Pose2 init_pose(x, y, theta);
  //   gtsam::Vector3 init_std(std::sqrt(initial_pose_hyp_.pf_pose_cov.m[0][0]),
  //                           std::sqrt(initial_pose_hyp_.pf_pose_cov.m[1][1]),
  //                           std::sqrt(initial_pose_hyp_.pf_pose_cov.m[2][2]));
  //   if (init_std.x() < 1e-6 || init_std.y() < 1e-6 || init_std.z() < 1e-6) {
  //     LOG(ERROR) << "Initialize EKF with zero covriance pose.";
  //     return false;
  //   }
  //   gtsam::Symbol x0('x', ekf_frame_cnt_);
  //   gtsam::SharedDiagonal P_initial = gtsam::noiseModel::Diagonal::Sigmas(init_std);
  //   ekf_.reset(new gtsam::ExtendedKalmanFilter<gtsam::Pose2>(x0, init_pose, P_initial));
  //   last_meas_symol_ = x0;
  //   gtsam::traits<gtsam::Pose2>::Print(init_pose, "initialze EKF");

  //   last_fusion_pose_ = init_pose;
  // }

  LOG(INFO) << "Global localization cost: " << global_loc_tic.toc() << " ms";
  loc_state_ = LocalizationState::LOCALIZED;
  return true;
}

/**
 * @brief 设置地图，并初始化粒子滤波器
 * 
 * @param[in] map 
 */
void PoseEstimator2D::SetMap(map_t* map)
{
  loc_state_ = LocalizationState::INITIALIZING;
  is_initialized_ = false;
  is_map_set_ = false;
  FreeMapDependentMemory();

  amcl_lasers_.clear();
  amcl_clouds_.clear();
  amcl_poses_.clear();
  frame_to_laser_id_.clear();
  laser_pose_.clear();

  map_ = map;
  
  std::string map_path = map_folder_ + "/" + map_name_;
  map_grid_ = CreateGridFromYaml(map_path + "/map.yaml");
  map_cloud_ = CreateCloudMap(map_path + "/map.pcd");
  highlight_map_cloud_ = CreateCloudMap(map_path + "/map_highlight.pcd");
  if (!map_grid_) {
    LOG(ERROR) << "Cannot load grid map";
    exit(-1);
  }

  if (!map_cloud_) {
    LOG(ERROR) << "Cannot load map cloud";
    exit(-1);
  }

  map_cloud_kdtree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  map_cloud_kdtree_->setInputCloud(map_cloud_);

  highlight_map_cloud_kdtree_ = std::make_unique<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  if (highlight_map_cloud_) {
    highlight_map_cloud_kdtree_->setInputCloud(highlight_map_cloud_);
  }


  ResetParticleFilter();
  ResetOdomModel();
  ResetLaserModel();
  ResetPoseUpdateModel();

  int wait_cnt = 10;
  bool init_success = false;
  for (int i = 0; i < wait_cnt; i++) {
    init_success = InitScanMatcher();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (init_success) {
      break;
    }
  }

  if (!init_success) {
    LOG(ERROR) << "Cannot init scan matcher after " << wait_cnt << " times." << std::endl;
    loc_state_ = LocalizationState::INITIALIZATION_FAILED;
    exit(-1);
  }
  LOG(INFO) << "Set map: " << map_name_ << " successfully.";

  is_map_set_ = true;
  loc_state_ = LocalizationState::INITIALIZED;
}

void PoseEstimator2D::FreeMapDependentMemory()
{
  if (map_ != nullptr) {
    map_free(map_);
    map_ = nullptr;
  }
  if (particle_filter_ != nullptr) {
    pf_free(particle_filter_);
    particle_filter_ = nullptr;
  }
  delete amcl_odom_;
  amcl_odom_ = nullptr;
  delete amcl_laser_;
  amcl_laser_ = nullptr;
  delete amcl_cloud_;
  amcl_cloud_ = nullptr;
  delete amcl_pose_;
  amcl_pose_ = nullptr;
}

std::optional<PoseWithCovariance>  PoseEstimator2D::GetCurrentPose()
{
  if (!has_new_pose_) {
    return std::nullopt;
  }
  has_new_pose_ = false;
  return current_pose_;
}

pf_sample_set_t* PoseEstimator2D::GetParticleSet() const
{
  if (particle_filter_ == nullptr) {
    return nullptr;
  }
  return particle_filter_->sets + particle_filter_->current_set;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator2D::GetMapCloud() const
{
  return map_cloud_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator2D::GetScanMatchedCloud()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (!scan_matched_cloud_) {
    return cloud_pcl;
  }

  for (const auto& p : scan_matched_cloud_->points()) {
    pcl::PointXYZ p_pcl;
    p_pcl.x = p.position.x();
    p_pcl.y = p.position.y();
    p_pcl.z = p.position.z();
    // pcl::PointXYZ p_pcl_transformed;
    // pcl::transformPoint(p_pcl, p_pcl_transformed, local_pose().cast<float>());
    cloud_pcl->push_back(p_pcl);
  }
  return cloud_pcl;
}

std::optional<AMCLOdomData> PoseEstimator2D::GetCurrOdomData(const double& time)
{
  if (odom_data_buff_.empty()) {
    return std::nullopt;
  }
  for (size_t i = odom_data_buff_.size() - 1; i > 0; i--) {
    if (odom_data_buff_[i].time <= time) {
      return odom_data_buff_[i];
    }
  }

  return std::nullopt;
}


/**
 * @brief 增加里程计数据，同时更新运动模型，判断关键帧
 * 
 * @param[in] odom_data
 */
void PoseEstimator2D::AddOdomData(const AMCLOdomData& odom_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_data_buff_.push_back(odom_data);

  while (odom_data_buff_.size() > 1000) {
    odom_data_buff_.pop_front();
  }
}

void PoseEstimator2D::AddLaserData(const AMCLLaserData& laser_data)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  curr_laser_data_ = std::move(laser_data);
  const auto odom_data = GetCurrOdomData(curr_laser_data_.time);
  if (odom_data.has_value()) {
    curr_odom_data_ = odom_data.value();
  } else {
    LOG(ERROR) << "Cannot find odom data for laser data.";
    return;
  }
  Process(curr_laser_data_);
}

void PoseEstimator2D::PreprocessLaserData(AMCLLaserData& laser_data)
{
  // Clip negative ranges
  for (int i = 0; i < laser_data.range_count; i++) {
    if (laser_data.ranges[i][0] <= param_->laser_min_range) {
      laser_data.ranges[i][0] = 0.0;
    } else if (laser_data.ranges[i][0] > param_->laser_max_range) {
      laser_data.ranges[i][0] = 0.0;
    }
  }
}

void PoseEstimator2D::AddSensorLaser(
  const std::string& sensor_frame,
  const pf_vector_t& sensor_pose)
{
  int laser_index = -1;
  if (amcl_laser_ == nullptr) {
    LOG(WARNING) << "map is not set before add sensor laser";
    return;
  }
  // ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_id_.size(), laser_scan_frame_id.c_str());
  amcl_lasers_.push_back(new AMCLLaser(*amcl_laser_));
  amcl_clouds_.push_back(new AMCLCloud(*amcl_cloud_));
  amcl_poses_.push_back(new AMCLPose(*amcl_pose_));

  laser_index = frame_to_laser_id_.size();

  amcl_lasers_[laser_index]->SetLaserPose(sensor_pose);

  frame_to_laser_id_[sensor_frame] = laser_index;
  laser_pose_[sensor_frame] = sensor_pose;
}

carto::sensor::PointCloud PoseEstimator2D::GetHighlightPointCloud(const carto::sensor::PointCloud& cloud)
{
  carto::sensor::PointCloud point_cloud;
  for (size_t i = 0; i < cloud.size(); i++) {
    if (cloud.intensities()[i] > 900) {
      point_cloud.push_back(cloud.points()[i]);
    }
  }
  return point_cloud;
}

carto::sensor::PointCloud PoseEstimator2D::ToPointCloud(const AMCLLaserData& laser_data)
{
  carto::sensor::PointCloud point_cloud;

  pf_vector_t laser_pose = laser_pose_[laser_data.frame_id];
  for (int i = 0; i < laser_data.range_count; i++) {
    double range = laser_data.ranges[i][0];
    double bearing = laser_data.ranges[i][1];
    if (range > param_->laser_max_range || range < param_->laser_min_range || range == 0.0) {
      continue;
    }
    const Eigen::AngleAxisf rotation(bearing, Eigen::Vector3f::UnitZ());
    cartographer::sensor::RangefinderPoint point{
        rotation * (range * Eigen::Vector3f::UnitX())};
    // 乘上外参
    point.position.x() = point.position.x() * cos(laser_pose.v[2]) - point.position.y() * sin(laser_pose.v[2]) + laser_pose.v[0];
    point.position.y() = point.position.x() * sin(laser_pose.v[2]) + point.position.y() * cos(laser_pose.v[2]) + laser_pose.v[1];
    point.position.z() = 0.0;
 
    point_cloud.push_back(point, laser_data.intensities[i][0]);
  }
  return point_cloud;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimator2D::ToPCLPointCloud(const AMCLLaserData& laser_data)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pf_vector_t laser_pose = laser_pose_[laser_data.frame_id];

  for (int i = 0; i < laser_data.range_count; i++) {
    double range = laser_data.ranges[i][0];
    double bearing = laser_data.ranges[i][1];
    if (range > param_->laser_max_range || range < param_->laser_min_range) {
      continue;
    }
    const Eigen::AngleAxisf rotation(bearing, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f point = rotation * (range * Eigen::Vector3f::UnitX());

    // 乘上外参
    pcl::PointXYZ p_pcl;
    p_pcl.x = point.x() * cos(laser_pose.v[2]) - point.y() * sin(laser_pose.v[2]) + laser_pose.v[0];
    p_pcl.y = point.x() * sin(laser_pose.v[2]) + point.y() * cos(laser_pose.v[2]) + laser_pose.v[1];
    p_pcl.z = 0.0;
 
    point_cloud->push_back(p_pcl);
  }
  return point_cloud;
}



static int count = 1;
void PoseEstimator2D::Process(AMCLLaserData& laser_data)
{
  if (is_map_set_ == false) {
    LOG_EVERY_N(WARNING, 100) << "Map is not set!";
    return;
  }

  if (!HasSensorLaser(laser_data.frame_id)) {
    return;
  }

  PreprocessLaserData(laser_data);

  curr_point_cloud_ = std::make_shared<carto::sensor::PointCloud>(
                        ToPointCloud(laser_data));
  highlight_point_cloud_ = std::make_shared<carto::sensor::PointCloud>(
                              GetHighlightPointCloud(*curr_point_cloud_));
  // curr_point_cloud_ = std::make_shared<carto::sensor::PointCloud>(
  //                       carto::sensor::VoxelFilter(
  //                       ToPointCloud(laser_data),
  //                       options_->voxel_filter_size));

  if (has_new_initial_pose_) {
    if (ApplyInitialPose()) {
      is_initialized_ = true;
      need_set_filter_initial_pose_ = true;
      LOG(INFO) << "Set initial pose SUCCESS.";
    } else {
      LOG(INFO) << "Set initial pose FAILED.";
      return;
    }
  }

  if (!is_initialized_) {
    return;
  }

  need_update_ = IsKeyFrame() || param_->force_update;
  if (!need_update_) {
    return;
  }

  int laser_index = frame_to_laser_id_[laser_data.frame_id];
  // 更新里程计预测
  amcl_odom_->UpdateAction(particle_filter_, (AMCLSensorData*)&curr_odom_data_);

  bool resampled = false;
  // 更新激光观测
  // If the robot has moved, update the filter
  // curr_laser_data_.sensor = amcl_lasers_[laser_index];

  // amcl_lasers_[laser_index]->UpdateSensor(particle_filter_, (AMCLSensorData*)&laser_data);


  curr_cloud_data_.cloud_ = *curr_point_cloud_;
  curr_cloud_data_.sensor = amcl_clouds_[laser_index];

  amcl_clouds_[laser_index]->UpdateSensor(particle_filter_, (AMCLSensorData*)&curr_cloud_data_);

  // Resample the particles
  if (!(++resample_count_ % param_->resample_interval)) {
    pf_update_resample(particle_filter_);
    resampled = true;
  }

  std::shared_ptr<carto::transform::Rigid2d> pose_prediction_amcl;
  // AMCL计算最后的位姿
  if (resampled) {
    // 对于每一个聚类，计算其均值和协方差
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(particle_filter_->sets[particle_filter_->current_set].cluster_count);
    for (int hyp_count = 0;
         hyp_count < particle_filter_->sets[particle_filter_->current_set].cluster_count;
         hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(particle_filter_, hyp_count, &weight, &pose_mean, &pose_cov)) {
        // ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    // 权重最大的聚类，作为当前的位姿估计
    if (max_weight > 0.0) {
      pose_prediction_amcl = std::make_shared<carto::transform::Rigid2d>(
                                Eigen::Vector2d(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                                hyps[max_weight_hyp].pf_pose_mean.v[1]),
                                Eigen::Rotation2D(hyps[max_weight_hyp].pf_pose_mean.v[2]));

      current_pose_.time = laser_data.time;
      // Copy in the covariance, converting from 3-D to 6-D
      current_pose_.pose(0) = hyps[max_weight_hyp].pf_pose_mean.v[0];
      current_pose_.pose(1) = hyps[max_weight_hyp].pf_pose_mean.v[1];
      current_pose_.pose(2) = hyps[max_weight_hyp].pf_pose_mean.v[2];
      pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          current_pose_.cov(i, j) = set->cov.m[i][j];
        }
      }
    }
  }

  if (!param_->use_scan_matching || !ceres_scan_matcher_ || !curr_point_cloud_) {
    has_new_pose_ = true;
    return;
  }

  utility::TicToc tic_total;
  double delta_dist = std::hypot(curr_odom_data_.delta.v[0], curr_odom_data_.delta.v[1]);
  carto::transform::Rigid2d odom_delta_2d(
    Eigen::Vector2d(curr_odom_data_.delta.v[0],
                    curr_odom_data_.delta.v[1]),
    Eigen::Rotation2D(curr_odom_data_.delta.v[2]));

  carto::transform::Rigid2d last_pose_2d(
    Eigen::Vector2d(last_pose_.pose(0),
                    last_pose_.pose(1)),
    Eigen::Rotation2D(last_pose_.pose(2)));

  carto::transform::Rigid2d pose_prediction_odom = carto::transform::Project2D(
                              carto::transform::Embed3D(last_pose_2d)  * delta_pose_by_odom_);
  carto::transform::Rigid2d curr_odom_2d(
    Eigen::Vector2d(curr_odom_data_.pose.v[0],
                    curr_odom_data_.pose.v[1]),
    Eigen::Rotation2D(curr_odom_data_.pose.v[2]));

  carto::transform::Rigid2d pose_prediction;
  if (pose_prediction_amcl) {
    double delta_dist = (pose_prediction_odom.translation() - pose_prediction_amcl->translation()).norm();
    if (delta_dist > 0.1) {
      LOG(INFO) << "predict pose by odom";
      pose_prediction = pose_prediction_odom;
    } else {
      pose_prediction = *pose_prediction_amcl;
    }
  } else {
    pose_prediction = pose_prediction_odom;
  }

  carto::transform::Rigid2d initial_ceres_pose = pose_prediction;
  carto::transform::Rigid2d correlative_pose = pose_prediction;
  if (options_->use_online_correlative_scan_matching) {
    const double score = real_time_correlative_scan_matcher_->Match(
      pose_prediction, *curr_point_cloud_,
      *map_grid_->grid(), &correlative_pose);
    if (score > 0.4) {
      initial_ceres_pose = correlative_pose;
    } else {
      initial_ceres_pose = pose_prediction;
      LOG(WARNING) << "Correlative scan matching score is too small, abond correlative pose";
    }
  }

  utility::TicToc tic_match;
  auto pose_observation = absl::make_unique<carto::transform::Rigid2d>();
  ceres::Solver::Summary summary;
  Eigen::Matrix3d ceres_match_cov;
  ceres_scan_matcher_->Match(initial_ceres_pose.translation(),
                             initial_ceres_pose,
                             *curr_point_cloud_,
                             *highlight_point_cloud_,
                             *map_grid_->grid(),
                             *map_cloud_kdtree_,
                             *highlight_map_cloud_kdtree_,
                             pose_observation.get(),
                             &summary,
                             ceres_match_cov,
                             true);

  std::cout << "pose prediction amcl: " << pose_prediction_amcl << std::endl;
  std::cout << "pose prediction odom: " << pose_prediction_odom << std::endl;
  std::cout << "initial ceres pose: " << initial_ceres_pose << std::endl;
  std::cout << "pose observation: " << *pose_observation << std::endl;

  if (!summary.IsSolutionUsable() || summary.final_cost == -1 ||
      std::isinf(summary.final_cost) || std::isnan(summary.final_cost))
  {
    double score;
    LOG(WARNING) << "Normal matching failed, use correlative scan matcher help localization.";    
    score = real_time_correlative_scan_matcher_->Match(
      initial_ceres_pose, *curr_point_cloud_,
      *map_grid_->grid(), &initial_ceres_pose);

    ceres_scan_matcher_->Match(initial_ceres_pose.translation(), initial_ceres_pose,
                              *curr_point_cloud_,
                              *map_grid_->grid(),
                              pose_observation.get(),
                              &summary,
                              ceres_match_cov,
                              true);

    if (!summary.IsSolutionUsable() || summary.final_cost == -1 ||
        std::isinf(summary.final_cost) || std::isnan(summary.final_cost))
    {
      LOG(ERROR) << "Localization failed !!!!!" <<std::endl;
      loc_state_ = LocalizationState::LOCALIZATION_FAILED;
      return;
    }
  }

  double matching_time = tic_match.toc();
  // std::cout << "summary: " << summary.BriefReport() << std::endl;
  current_pose_.pose(0) = pose_observation->translation().x();
  current_pose_.pose(1) = pose_observation->translation().y();
  current_pose_.pose(2) = pose_observation->rotation().angle();
  current_pose_.cov = ceres_match_cov;

  last_pose_ = current_pose_;

  ceres_match_cov = Eigen::Matrix3d::Identity() * 0.05 * 0.05;

  // std::cout << "ceres_match_cov: \n" << ceres_match_cov.matrix() << std::endl;

  curr_pose_data_.time = current_pose_.time;
  curr_pose_data_.pose.v[0] = current_pose_.pose(0);
  curr_pose_data_.pose.v[1] = current_pose_.pose(1);
  curr_pose_data_.pose.v[2] = current_pose_.pose(2);

  curr_pose_data_.cov.m[0][0] = ceres_match_cov(0, 0);
  curr_pose_data_.cov.m[0][1] = ceres_match_cov(0, 1);
  curr_pose_data_.cov.m[0][2] = ceres_match_cov(0, 2);
  curr_pose_data_.cov.m[1][0] = ceres_match_cov(1, 0);
  curr_pose_data_.cov.m[1][1] = ceres_match_cov(1, 1);
  curr_pose_data_.cov.m[1][2] = ceres_match_cov(1, 2);
  curr_pose_data_.cov.m[2][0] = ceres_match_cov(2, 0);
  curr_pose_data_.cov.m[2][1] = ceres_match_cov(2, 1);
  curr_pose_data_.cov.m[2][2] = ceres_match_cov(2, 2);

  // amcl_pose_->UpdateSensor(particle_filter_, (AMCLSensorData*)&curr_pose_data_);

  if (resampled) {
    // 对于每一个聚类，计算其均值和协方差
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(particle_filter_->sets[particle_filter_->current_set].cluster_count);
    for (int hyp_count = 0;
         hyp_count < particle_filter_->sets[particle_filter_->current_set].cluster_count;
         hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(particle_filter_, hyp_count, &weight, &pose_mean, &pose_cov)) {
        // ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    // 权重最大的聚类，作为当前的位姿估计
    if (max_weight > 0.0) {
      current_pose_.time = laser_data.time;
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = particle_filter_->sets + particle_filter_->current_set;

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          current_pose_.cov(i, j) = set->cov.m[i][j];
        }
      }

      Eigen::Vector3d pose_mean(set->mean.v[0],
                          set->mean.v[1],
                          set->mean.v[2]);
      double delta_dist = (current_pose_.pose - pose_mean).head<2>().norm();
      if (delta_dist > 0.1) {
        LOG(WARNING) << "delta_dist: " << delta_dist << std::endl;
        // 重置PF
        amcl_hyp_t reset_pose_hyp;
        reset_pose_hyp.pf_pose_mean.v[0] = current_pose_.pose.x();
        reset_pose_hyp.pf_pose_mean.v[1] = current_pose_.pose.y();
        reset_pose_hyp.pf_pose_mean.v[2] = current_pose_.pose.z();
        reset_pose_hyp.pf_pose_cov = pf_matrix_zero();
        reset_pose_hyp.pf_pose_cov.m[0][0] = param_->init_cov[0];
        reset_pose_hyp.pf_pose_cov.m[1][1] = param_->init_cov[1];
        reset_pose_hyp.pf_pose_cov.m[2][2] = param_->init_cov[2];

        assert(particle_filter_ == nullptr);
        pf_init(particle_filter_,
                reset_pose_hyp.pf_pose_mean,
                reset_pose_hyp.pf_pose_cov);
      }
    }
  }

  double total_time = tic_total.toc();
  // LOG(INFO) << "ceres scan matching cost time(ms): " << matching_time << std::endl;
  // LOG(INFO) << "total cost time(ms): " << total_time << std::endl;
  // LOG(INFO) << "prediction pose: "
  //           << pose_prediction.translation().x() << " "
  //           << pose_prediction.translation().y() << " "
  //           << pose_prediction.rotation().angle();
  // LOG(INFO) << "ceres pose: "
  //           << pose_observation->translation().x() << " "
  //           << pose_observation->translation().y() << " "
  //           << pose_observation->rotation().angle();

  // time_file_ << count++ << " " << total_time << std::endl;

// PointCloud TransformPointCloud(const PointCloud& point_cloud,
//                                const transform::Rigid3f& transform);
  auto pose3d = carto::transform::Embed3D(*pose_observation);
  scan_matched_cloud_ = std::make_shared<carto::sensor::PointCloud>(
                                        TransformPointCloud(*curr_point_cloud_, pose3d.cast<float>()));

  if (param_->output_mean_pose) {
    pose_mean_queue_.push_back(current_pose_);
    if (pose_mean_queue_.size() > 10) {
      Eigen::Vector3d pose_mean = Eigen::Vector3d::Zero();
      for (const auto & pose : pose_mean_queue_) {
        pose_mean += pose.pose;
      }
      pose_mean /= pose_mean_queue_.size();
      pose_mean_queue_.pop_front();
      LOG(INFO) << "pose mean : "
                << pose_mean.x() << " "
                << pose_mean.y() << " "
                << pose_mean.z() << std::endl;
    }
  }

  // amcl_lasers_[laser_index]->UpdateSensor(particle_filter_, (AMCLSensorData*)&laser_data);


  // if (param_->use_ekf && ekf_ != nullptr) {
  //   ekf_frame_cnt_++;
  //   gtsam::Symbol curr_meas_symbol('x', ekf_frame_cnt_);

  //   gtsam::Pose2 diff = last_odom_pose2_.inverse() * curr_odom_pose2_;
  //   // gtsam::traits<gtsam::Pose2>::Print(diff, "diff");
  //   gtsam::SharedDiagonal Q;
  //   if (std::fabs(diff.theta()) > 0.17) {
  //     Q = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.04, 0.04, 0.17), true);
  //   } else {
  //     Q = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.02, 0.02, 0.1), true);
  //   }
  //   gtsam::BetweenFactor<gtsam::Pose2> factor_predict(last_meas_symol_, curr_meas_symbol, diff, Q);
  //   gtsam::Pose2 predict = ekf_->predict(factor_predict);
  //   // gtsam::traits<gtsam::Pose2>::Print(predict, "预测");
  //   gtsam::SharedGaussian R = gtsam::noiseModel::Gaussian::Covariance(ceres_match_cov.matrix(), true);
  //   // gtsam::SharedDiagonal R = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.02, 0.02, 0.05), true);
  //   gtsam::Pose2 measurement(current_pose_.pose(0),
  //                            current_pose_.pose(1),
  //                            current_pose_.pose(2));
  //   gtsam::PriorFactor<gtsam::Pose2> factor_meas(curr_meas_symbol, measurement, R);
  //   fusion_pose_ = ekf_->update(factor_meas);
  //   // ekf_->print("当前结果");
  //   // gtsam::traits<gtsam::Pose2>::Print(fusion_pose_, "更新");

  //   last_meas_symol_ = curr_meas_symbol;

  //   current_pose_.pose(0) = fusion_pose_.x();
  //   current_pose_.pose(1) = fusion_pose_.y();
  //   current_pose_.pose(2) = fusion_pose_.theta();

  //   LOG(INFO) << "pose fusion: "
  //             << fusion_pose_.x() << " "
  //             << fusion_pose_.y() << " "
  //             << fusion_pose_.theta();
  // }

  has_new_pose_ = true;
}

/**
 * @brief 判断是否是关键帧，并计算odom的delta
 * 
 * @return true 
 * @return false 
 */
bool PoseEstimator2D::IsKeyFrame()
{
  if (last_odom_data_ == nullptr) {
    last_odom_data_ = std::make_unique<AMCLOdomData>(curr_odom_data_);
    // 第一帧
    curr_odom_data_.delta.v[0] = 0.0;
    curr_odom_data_.delta.v[1] = 0.0;
    curr_odom_data_.delta.v[2] = 0.0;
    return true;
  }

  pf_vector_t delta = pf_vector_zero();
  delta.v[0] = curr_odom_data_.pose.v[0] - last_odom_data_->pose.v[0];
  delta.v[1] = curr_odom_data_.pose.v[1] - last_odom_data_->pose.v[1];
  delta.v[2] = angle_diff(curr_odom_data_.pose.v[2], last_odom_data_->pose.v[2]);

  double delta_dist = sqrt(delta.v[0] * delta.v[0] +
                           delta.v[1] * delta.v[1]);
  double delta_time = curr_odom_data_.time - last_odom_data_->time;
  if (!(delta_dist > param_->dist_thresh ||
        fabs(delta.v[2]) > param_->angle_thresh ||
        delta_time > param_->time_thresh))
  {
    return false;
  }
  // last_odom_pose2_ = gtsam::Pose2(last_odom_data_->pose.v[0],
  //                                 last_odom_data_->pose.v[1],
  //                                 last_odom_data_->pose.v[2]);
  // curr_odom_pose2_ = gtsam::Pose2(curr_odom_data_.pose.v[0],
  //                                 curr_odom_data_.pose.v[1],
  //                                 curr_odom_data_.pose.v[2]);

  carto::transform::Rigid2d last_odom_2d(
    Eigen::Vector2d(last_odom_data_->pose.v[0],
                    last_odom_data_->pose.v[1]),
    Eigen::Rotation2D(last_odom_data_->pose.v[2]));

  carto::transform::Rigid2d curr_odom_2d(
    Eigen::Vector2d(curr_odom_data_.pose.v[0],
                    curr_odom_data_.pose.v[1]),
    Eigen::Rotation2D(curr_odom_data_.pose.v[2]));

  delta_pose_by_odom_ = carto::transform::Embed3D(last_odom_2d).inverse() * carto::transform::Embed3D(curr_odom_2d);

  curr_odom_data_.delta = delta;
  last_odom_data_ = std::make_unique<AMCLOdomData>(curr_odom_data_);
  return true;
}

bool PoseEstimator2D::NeedSetInitPoseInFilter()
{
  if (need_set_filter_initial_pose_) {
    need_set_filter_initial_pose_ = false;
    return true;
  }
  return false;
}


bool PoseEstimator2D::GlobalLocalization()
{
  if (map_ == nullptr) {
    // ROS_ERROR("Map is not set!");
    return false;
  }
  need_global_localization_ = true;
  // pf_init_model(particle_filter_,
  //               (pf_init_model_fn_t)UniformPoseGenerator,
  //               (void *)map_);
  return true;
}

pf_vector_t PoseEstimator2D::UniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices_.size();
  std::pair<int,int> free_point = free_space_indices_[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for (;;) {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if (MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1)) {
      break;
    }
  }
#endif
  return p;
}

std::unique_ptr<carto::mapping::Submap2D>
PoseEstimator2D::CreateGridFromMap(const map_t* map)
{
  int num_y_cells = map->size_x;
  int num_x_cells = map->size_y;
  // acml地图原点与grid2d的d原点不一样
  double origin_x = map->origin_x - (map->size_x / 2) * map->scale;
  double origin_y = map->origin_y - (map->size_y / 2) * map->scale;
  Eigen::Vector2d max = Eigen::Vector2d(origin_x + num_y_cells * map->scale,
                                        origin_y + num_x_cells * map->scale);
  carto::mapping::CellLimits cell_limits(num_x_cells, num_y_cells);
  carto::mapping::MapLimits map_limits(map->scale, max, cell_limits);

  Eigen::Vector2f origin(map->origin_x, map->origin_y);
  auto res_map = std::make_unique<carto::mapping::Submap2D>(
              origin,
              std::unique_ptr<carto::mapping::Grid2D>(
                static_cast<carto::mapping::Grid2D*>(
                  std::make_unique<carto::mapping::ProbabilityGrid>(
                    map_limits, &conversion_tables_).release())),
              &conversion_tables_);           
  // TODO: 地图栅格的值怎么计算
  for (int x = 0; x < num_x_cells; x++) {
    for (int y = 0; y < num_y_cells; y++) {
      int xx = num_y_cells - y - 1;
      int yy = num_x_cells - x - 1;
      uchar value = map->data[xx + yy * num_y_cells];
      if (value == 49|| value == 50 || value == 51 || value == 255) {
        res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
          res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
            carto::mapping::kUnknownCorrespondenceValue;
      } else {
        double occ_prob = (100 - value) / 100.0;
        if (occ_prob < 0.3) {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) = 1;
        } else if (occ_prob < 0.7) {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
              static_cast<uint16_t>(occ_prob * carto::mapping::kFreeCorrespondenceValue);
        } else {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
              carto::mapping::kFreeCorrespondenceValue;
        }
      }
    }
  }
  res_map->mutable_grid()->mutable_known_cells_box()->extend(
    Eigen::Vector2i(
      num_x_cells,
      num_y_cells).matrix());
  res_map->mutable_grid()->mutable_known_cells_box()->extend(
    Eigen::Array2i(0, 0).matrix());
  res_map->set_insertion_finished(true);
  // res_map->grid()->DrawAsImage("/home/ubt/data/", "loc_map_2.png");
  return res_map;
}

std::unique_ptr<carto::mapping::Submap2D>
PoseEstimator2D::CreateGridFromYaml(const std::string& yaml_filename)
{
  double resolution;
  bool negate;
  double occ_th, free_th;
  double origin[3];
  std::string mapfname = "";
  // 读取yaml文件
  std::ifstream fin(yaml_filename.c_str());
  if (fin.fail()) {
    LOG(ERROR) << "Failed to open file: " << yaml_filename;
    return nullptr;
  }

  YAML::Node doc = YAML::Load(fin);
  try {
    resolution = doc["resolution"].as<double>();
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }
  try {
    negate = static_cast<bool>(doc["negate"].as<int>());
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }
  try {
    occ_th = doc["occupied_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }
  try {
    free_th = doc["free_thresh"].as<double>();
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }
  try {
    origin[0] = doc["origin"][0].as<double>();
    origin[1] = doc["origin"][1].as<double>();
    origin[2] = doc["origin"][2].as<double>();
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }

  try {
    mapfname = doc["image"].as<std::string>();
    // TODO: make this path-handling more robust
    if(mapfname.size() == 0)
    {
      LOG(ERROR) << "The image tag cannot be an empty string.";
      return nullptr;
    }

    boost::filesystem::path mapfpath(mapfname);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(yaml_filename);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      mapfname = mapfpath.string();
    }
  } catch (YAML::InvalidScalar &) {
    return nullptr;
  }

  cv::Mat img = cv::imread(mapfname, cv::IMREAD_GRAYSCALE);
  std::vector<uint8_t> data(img.begin<uint8_t>(), img.end<uint8_t>());
  
  int num_x_cells = img.rows; 
  int num_y_cells = img.cols; 
  double origin_x = origin[0];
  double origin_y = origin[1];
  double scale = resolution;

  Eigen::Vector2d max = Eigen::Vector2d(origin_x + num_y_cells * scale,
                                        origin_y + num_x_cells * scale);
  carto::mapping::CellLimits cell_limits(num_x_cells, num_y_cells);
  carto::mapping::MapLimits map_limits(scale, max, cell_limits);

  Eigen::Vector2f origin_eigen(origin_x, origin_y);
  auto res_map = std::make_unique<carto::mapping::Submap2D>(
              origin_eigen,
              std::unique_ptr<carto::mapping::Grid2D>(
                static_cast<carto::mapping::Grid2D*>(
                  std::make_unique<carto::mapping::ProbabilityGrid>(
                    map_limits, &conversion_tables_).release())),
              &conversion_tables_);
  // TODO: 地图栅格的值怎么计算

  for (int x = 0; x < num_x_cells; x++) {
    for (int y = 0; y < num_y_cells; y++) {
      int xx = num_y_cells - y - 1;
      int yy = x;

      // 图片的像素值和栅格地图的值是相反的
      // 图片 0: 黑色， 255: 白色
      // 然后 kFreeCorrespondenceValue: 与占用概率prob也是相反的
      uchar value = data[xx + yy * num_y_cells];
      if (value == 126|| value == 127 || value == 128) {
        res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
          res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
            carto::mapping::kUnknownCorrespondenceValue;
      } else {
        double occ_prob = value / 255.0;
        if (occ_prob < 0.3) {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) = 1;
        } else if (occ_prob < 0.7) {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
              static_cast<uint16_t>(occ_prob * carto::mapping::kFreeCorrespondenceValue);
        } else {
          res_map->mutable_grid()->mutable_correspondence_cost_cells()->at(
            res_map->mutable_grid()->ToFlatIndex(Eigen::Array2i{x, y})) =
              carto::mapping::kFreeCorrespondenceValue;
        }
      }
    }
  }
  res_map->mutable_grid()->mutable_known_cells_box()->extend(
    Eigen::Vector2i(
      num_x_cells,
      num_y_cells).matrix());
  res_map->mutable_grid()->mutable_known_cells_box()->extend(
    Eigen::Array2i(0, 0).matrix());
  res_map->set_insertion_finished(true);
  // res_map->grid()->DrawAsImage("/home/ubt/data/", "loc_map.png");
  return res_map;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
PoseEstimator2D::CreateCloudMap(const std::string& cloud_filename)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(cloud_filename, *cloud_map);
  for (auto& point : cloud_map->points) {
    point.z = 0;
  }

  return cloud_map;
}

} // namespace amcl
