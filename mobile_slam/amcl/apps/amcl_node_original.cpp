#include <iostream>
#include <string>
#include <memory>
#include <functional>

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "dcm_srvs/srv/get_map_info.hpp"

#include "glog/logging.h"

#include "amcl_node.hpp"

namespace amcl
{
AmclNode::AmclNode()
  : rclcpp::Node("amcl_node"),
    sent_first_transform_(false),
    latest_tf_valid_(false),
    amcl_param_(std::make_shared<AmclParam>()),
    pose_estimator_2d_(std::make_unique<PoseEstimator2D>(amcl_param_))
{
  InitializeNode();
}

AmclNode::~AmclNode()
{
  pose_publish_timer_.reset();
  check_laser_timer_.reset();
  publish_particle_timer_.reset();
  
  if (laser_scan_sub_) {
    laser_scan_sub_.reset();
  }
  
  tfl_.reset();
  tf_.reset();
  tfb_.reset();
}

void AmclNode::InitializeNode()
{
  InitAMCLParams();
  InitNodeParams();

  tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfl_ = std::make_unique<tf2_ros::TransformListener>(*tf_);

  // Create publishers
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("loc_pose", 2);
  map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 2);
  matched_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_matched_points2", 2);
  particlecloud_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particle_cloud", 2);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("amcl_path", 2);
  amcl_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("amcl_odom", 2);
  loc_state_pub_ = this->create_publisher<dcm_msgs::msg::LocalizationState>("localization_state", 2);

  // Create services
  global_loc_srv_ = this->create_service<dcm_srvs::srv::GetMapInfo>(
      "global_localization",
      std::bind(&AmclNode::GlobalLocalizationCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
  
  nomotion_update_srv_ = this->create_service<dcm_srvs::srv::GetMapInfo>(
      "request_nomotion_update",
      std::bind(&AmclNode::NoMotionUpdateCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
  
  set_map_changed_srv_ = this->create_service<dcm_srvs::srv::GetMapInfo>(
      "set_map_changed",
      std::bind(&AmclNode::SetMapChangedCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
  
  relocalization_srv_ = this->create_service<dcm_srvs::srv::GetMapInfo>(
      "relocalization",
      std::bind(&AmclNode::RelocaliztoinCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
  
  cancel_relocalization_srv_ = this->create_service<dcm_srvs::srv::GetMapInfo>(
      "cancel_relocalization",
      std::bind(&AmclNode::CancelRelocaliztoinCallback, this, 
                std::placeholders::_1, std::placeholders::_2));

  // Create subscribers  
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 100,
      std::bind(&AmclNode::HandleLaserScanMessage, this, std::placeholders::_1));
  
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 2,
      std::bind(&AmclNode::InitialPoseReceived, this, std::placeholders::_1));
  
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 2,
      std::bind(&AmclNode::HandleOdometryMessage, this, std::placeholders::_1));

  // Create timers
  pose_publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / 50.0)),  // 50Hz default
      std::bind(&AmclNode::PublishThread, this));
  
  check_laser_timer_ = this->create_wall_timer(
      std::chrono::seconds(15),
      std::bind(&AmclNode::CheckLaserReceived, this));
  
  publish_particle_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),  // 0.5s interval
      std::bind(&AmclNode::PublishParticleCloud, this));

  // Initialize diagnostic updater
  diagnosic_updater_.setHardwareID("None");
  diagnosic_updater_.add("Standard deviation", this, &AmclNode::StandardDeviationDiagnostics);

  // Initialize pose saving if enabled
  is_save_pose_ = this->declare_parameter("is_save_pose", false);
  
  if (is_save_pose_) {
    std::string data_directory = this->declare_parameter("data_directory", std::string(""));
    dcm::utility::FileManager::CreateDirectory(data_directory, "pose data save");
    
    std::string pose_filename = this->declare_parameter("pose_filename", std::string("amcl_poses.txt"));
    std::string pose_format = this->declare_parameter("pose_format", std::string("tum"));

    std::string pose_save_path = data_directory + pose_filename;
    amcl_pose_tool_ = std::make_unique<dcm::utility::PoseSaveTool>(pose_save_path, pose_format);
  }

  if (!pose_estimator_2d_->IsMapSet()) {
    RCLCPP_WARN(this->get_logger(), "Request map failed in initialization, please set map in service");
  }
}

void AmclNode::InitNodeParams()
{
  first_map_only_ = this->declare_parameter("first_map_only", false);
  map_topic_ = this->declare_parameter("map_topic", std::string("map"));
  scan_topic_ = this->declare_parameter("scan_topic", std::string("scan"));

  double pose_publish_rate = this->declare_parameter("pose_publish_rate", 50.0);
  pose_pub_interval_ = std::chrono::duration<double>(1.0 / pose_publish_rate);

  odom_frame_id_ = this->declare_parameter("odom_frame_id", std::string("odom"));
  base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_link"));
  global_frame_id_ = this->declare_parameter("global_frame_id", std::string("map"));
  
  odom_frame_id_ = StripSlash(odom_frame_id_);
  base_frame_id_ = StripSlash(base_frame_id_);
  global_frame_id_ = StripSlash(global_frame_id_);

  double tmp_tol = this->declare_parameter("transform_tolerance", 0.1);

  tf_broadcast_ = this->declare_parameter("tf_broadcast", true);
  force_update_after_initialpose_ = this->declare_parameter("force_update_after_initialpose", false);
  force_update_after_set_map_ = this->declare_parameter("force_update_after_set_map", false);

  // For diagnostics
  std_warn_level_x_ = this->declare_parameter("std_warn_level_x", 0.2);
  std_warn_level_y_ = this->declare_parameter("std_warn_level_y", 0.2);
  std_warn_level_yaw_ = this->declare_parameter("std_warn_level_yaw", 0.1);
}

void AmclNode::InitAMCLParams()
{
  amcl_param_->dist_thresh = this->declare_parameter("update_min_d", 0.2);
  amcl_param_->angle_thresh = this->declare_parameter("update_min_a", M_PI / 6.0);
  amcl_param_->time_thresh = this->declare_parameter("update_min_t", 0.5);

  amcl_param_->laser_min_range = this->declare_parameter("laser_min_range", 0.2);
  amcl_param_->laser_max_range = this->declare_parameter("laser_max_range", 15.0);
  amcl_param_->max_beams = this->declare_parameter("laser_max_beams", 30);
  amcl_param_->min_particles = this->declare_parameter("min_particles", 100);
  amcl_param_->max_particles = this->declare_parameter("max_particles", 5000);
  amcl_param_->pf_err = this->declare_parameter("kld_err", 0.01);
  amcl_param_->pf_z = this->declare_parameter("kld_z", 0.99);
  amcl_param_->alpha_slow = this->declare_parameter("recovery_alpha_slow", 0.001);
  amcl_param_->alpha_fast = this->declare_parameter("recovery_alpha_fast", 0.1);
  amcl_param_->alpha1 = this->declare_parameter("odom_alpha1", 0.2);
  amcl_param_->alpha2 = this->declare_parameter("odom_alpha2", 0.2);
  amcl_param_->alpha3 = this->declare_parameter("odom_alpha3", 0.2);
  amcl_param_->alpha4 = this->declare_parameter("odom_alpha4", 0.2);
  amcl_param_->alpha5 = this->declare_parameter("odom_alpha5", 0.2);

  amcl_param_->resample_interval = this->declare_parameter("resample_interval", 2);
  amcl_param_->selective_resampling = this->declare_parameter("selective_resampling", false);

  amcl_param_->do_beamskip = this->declare_parameter("do_beamskip", false);
  amcl_param_->beam_skip_distance = this->declare_parameter("beam_skip_distance", 0.5);
  amcl_param_->beam_skip_threshold = this->declare_parameter("beam_skip_threshold", 0.3);
  amcl_param_->beam_skip_error_threshold = this->declare_parameter("beam_skip_error_threshold", 0.9);

  // laser model
  amcl_param_->z_hit = this->declare_parameter("laser_z_hit", 0.95);
  amcl_param_->z_short = this->declare_parameter("laser_z_short", 0.1);
  amcl_param_->z_max = this->declare_parameter("laser_z_max", 0.05);
  amcl_param_->z_rand = this->declare_parameter("laser_z_rand", 0.05);
  amcl_param_->sigma_hit = this->declare_parameter("laser_sigma_hit", 0.2);
  amcl_param_->lambda_short = this->declare_parameter("laser_lambda_short", 0.1);
  amcl_param_->laser_likelihood_max_dist = this->declare_parameter("laser_likelihood_max_dist", 2.0);

  std::string tmp_model_type = this->declare_parameter("laser_model_type", std::string("likelihood_field"));
  if (tmp_model_type == "beam") {
    amcl_param_->laser_model_type = LASER_MODEL_BEAM;
  } else if(tmp_model_type == "likelihood_field") {
    amcl_param_->laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  } else if(tmp_model_type == "likelihood_field_prob") {
    amcl_param_->laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown laser model type \"%s\"; defaulting to likelihood_field model",
                tmp_model_type.c_str());
    amcl_param_->laser_model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  // odom model
  tmp_model_type = this->declare_parameter("odom_model_type", std::string("diff"));
  if (tmp_model_type == "diff") {
    amcl_param_->odom_model_type = ODOM_MODEL_DIFF;
  } else if(tmp_model_type == "omni") {
    amcl_param_->odom_model_type = ODOM_MODEL_OMNI;
  } else if(tmp_model_type == "diff-corrected") {
    amcl_param_->odom_model_type = ODOM_MODEL_DIFF_CORRECTED;
  } else if(tmp_model_type == "omni-corrected") {
    amcl_param_->odom_model_type = ODOM_MODEL_OMNI_CORRECTED;
  } else {
    RCLCPP_WARN(this->get_logger(), "Unknown odom model type \"%s\"; defaulting to diff model",
                tmp_model_type.c_str());
    amcl_param_->odom_model_type = ODOM_MODEL_DIFF;
  }

  // debug param
  amcl_param_->output_mean_pose = this->declare_parameter("output_mean_pose", false);

  amcl_param_->use_scan_matching = this->declare_parameter("use_scan_matching", true);
  amcl_param_->use_ekf = this->declare_parameter("use_ekf", false);
  amcl_param_->use_dead_reckoning = this->declare_parameter("use_DR", false);
}

void AmclNode::UpdatePoseFromServer()
{
  double init_pose[3];
  double init_cov[3];
  init_pose[0] = 0.0;
  init_pose[1] = 0.0;
  init_pose[2] = 0.0;
  init_cov[0] = 0.5 * 0.5;
  init_cov[1] = 0.5 * 0.5;
  init_cov[2] = (M_PI / 12.0) * (M_PI / 12.0);

  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, amcl_param_->init_pose[0]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_pose[0] = tmp_pos;
  else 
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, amcl_param_->init_pose[1]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_pose[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, amcl_param_->init_pose[2]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_pose[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");

  private_nh_.param("initial_cov_xx", tmp_pos, amcl_param_->init_cov[0]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_cov[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, amcl_param_->init_cov[1]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_cov[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, amcl_param_->init_cov[2]);
  if(!std::isnan(tmp_pos))
    amcl_param_->init_cov[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");
  LOG(INFO) << "Initial Pose: " << amcl_param_->init_pose[0] << " " << amcl_param_->init_pose[1] << " " << amcl_param_->init_pose[2];
  amcl_hyp_t initial_pose_hyp;
  initial_pose_hyp.pf_pose_mean.v[0] = amcl_param_->init_pose[0];
  initial_pose_hyp.pf_pose_mean.v[1] = amcl_param_->init_pose[1];
  initial_pose_hyp.pf_pose_mean.v[2] = amcl_param_->init_pose[2];
  initial_pose_hyp.pf_pose_cov.m[0][0] = amcl_param_->init_cov[0];
  initial_pose_hyp.pf_pose_cov.m[1][1] = amcl_param_->init_cov[1];
  initial_pose_hyp.pf_pose_cov.m[2][2] = amcl_param_->init_cov[2];

  pose_estimator_2d_->SetInitialPose(initial_pose_hyp);
}

void AmclNode::SavePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf2::Transform odom_pose_tf2;
  tf2::convert(latest_odom_pose_.pose, odom_pose_tf2);
  tf2::Transform map_pose = latest_tf_.inverse() * odom_pose_tf2;

  double yaw = tf2::getYaw(map_pose.getRotation());

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx", 
                       last_published_pose_.pose.covariance[6*0+0]);
  private_nh_.setParam("initial_cov_yy", 
                       last_published_pose_.pose.covariance[6*1+1]);
  private_nh_.setParam("initial_cov_aa", 
                       last_published_pose_.pose.covariance[6*5+5]);
}

void AmclNode::PublishThread()
{
  dcm_msgs::LocalizationState localization_state;
  const auto loc_state = pose_estimator_2d_->GetLocalizationState();
  localization_state = ToMsg(loc_state);
  loc_state_pub_.publish(localization_state);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map = pose_estimator_2d_->GetMapCloud();
  if (cloud_map != nullptr && map_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 cloud_map_msg;
    pcl::toROSMsg(*cloud_map, cloud_map_msg);
    cloud_map_msg.header.stamp = ros::Time::now();
    cloud_map_msg.header.frame_id = global_frame_id_;
    map_cloud_pub_.publish(cloud_map_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr matched_cloud = pose_estimator_2d_->GetScanMatchedCloud();
  if (matched_cloud != nullptr && matched_cloud_pub_.getNumSubscribers() > 0) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*matched_cloud, cloud_msg);
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    matched_cloud_pub_.publish(cloud_msg);
  }

  std::optional<PoseWithCovariance> amcl_pose = pose_estimator_2d_->GetCurrentPose();
  if (!amcl_pose.has_value()) {
    if (!latest_tf_valid_) {
      return;
    }
    if (tf_broadcast_ && !amcl_param_->use_dead_reckoning) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      ros::Time transform_expiration = (ros::Time::now() +
                                        transform_tolerance_);
      geometry_msgs::TransformStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = global_frame_id_;
      tmp_tf_stamped.header.stamp = transform_expiration;
      tmp_tf_stamped.child_frame_id = odom_frame_id_;
      tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
      this->tfb_->sendTransform(tmp_tf_stamped);
    }
    diagnosic_updater_.update();

    return;
  }

  if (amcl_param_->use_dead_reckoning) {
    tmp_P_.x() = amcl_pose.value().pose(0);
    tmp_P_.y() = amcl_pose.value().pose(1);
    tmp_Q_ = Eigen::Quaterniond(Eigen::AngleAxisd(amcl_pose.value().pose(2), Eigen::Vector3d::UnitZ()));

    Eigen::Isometry3d tmp_pose = Eigen::Isometry3d::Identity();
    tmp_pose.translate(tmp_P_);
    tmp_pose.rotate(tmp_Q_);
    dead_reckoning_.AddPose(tmp_pose, amcl_pose.value().time);
    std::cout << "refined : \n";
    std::cout << tmp_P_.x() << " " << tmp_P_.y() << std::endl;
    std::cout << tmp_Q_.x() << " " << tmp_Q_.y() << " " << tmp_Q_.z() << " " << tmp_Q_.w() << std::endl;
  }

  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.frame_id = global_frame_id_;
  p.header.stamp = ros::Time(amcl_pose.value().time);
  p.pose.pose.position.x = amcl_pose.value().pose(0);
  p.pose.pose.position.y = amcl_pose.value().pose(1);
  tf2::Quaternion q;
  q.setRPY(0, 0, amcl_pose.value().pose(2));
  tf2::convert(q, p.pose.pose.orientation);
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      p.pose.covariance[6 * i + j] = amcl_pose.value().cov(i, j);
    }
  }
  p.pose.covariance[6 * 5 + 5] = amcl_pose.value().cov(2, 2);
  pose_pub_.publish(p);
  last_published_pose_ = p;

  if (is_save_pose_) {
    amcl_pose_tool_->AddPoseData(amcl_pose.value().pose, amcl_pose.value().time);
    LOG(INFO) << "save pose: " << amcl_pose.value().pose.transpose() << " at time: " << amcl_pose.value().time;
  }

  // 发布路径
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = global_frame_id_;
  pose_stamped.header.stamp = ros::Time(amcl_pose.value().time);
  pose_stamped.pose = p.pose.pose;

  // path_msg_.header = pose_stamped.header;
  // path_msg_.poses.push_back(pose_stamped);
  // path_pub_.publish(path_msg_);

  nav_msgs::Odometry amcl_odom;
  amcl_odom.header.frame_id = global_frame_id_;
  amcl_odom.header.stamp = ros::Time(amcl_pose.value().time);
  amcl_odom.pose.pose = p.pose.pose;
  amcl_odom.pose.covariance = p.pose.covariance;
  amcl_odom.child_frame_id = base_frame_id_;

  amcl_odom_pub_.publish(amcl_odom);

  geometry_msgs::PoseStamped odom_to_map;
  try {
    tf2::Quaternion q;
    q.setRPY(0, 0, amcl_pose.value().pose(2));
    tf2::Transform tmp_tf(q, tf2::Vector3(amcl_pose.value().pose(0),
                                          amcl_pose.value().pose(1),
                                          0.0));

    geometry_msgs::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = ros::Time(amcl_pose.value().time);
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (const tf2::TransformException&) {
    ROS_DEBUG("Failed to subtract base to odom transform");
    return;
  }

  tf2::convert(odom_to_map.pose, latest_tf_);
  latest_tf_valid_ = true;

  if (tf_broadcast_ == true) {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (ros::Time(amcl_pose.value().time) +
                                      transform_tolerance_);
    geometry_msgs::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = global_frame_id_;
    tmp_tf_stamped.header.stamp = transform_expiration;
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

    this->tfb_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
  }
}

dcm_msgs::LocalizationState AmclNode::ToMsg(LocalizationState state)
{
  dcm_msgs::LocalizationState msg;    
  switch (state) {
    case LocalizationState::LOCALIZATION_FAILED:
      msg.status = dcm_msgs::LocalizationState::LOC_FAILED;
      break;
    case LocalizationState::INITIALIZATION_FAILED:
      msg.status = dcm_msgs::LocalizationState::LOC_INIT_FAILED;
      break;
    case LocalizationState::NOT_INITAILIED:
      msg.status = dcm_msgs::LocalizationState::LOC_NOT_INIT;
      break;
    case LocalizationState::INITIALIZING:
      msg.status = dcm_msgs::LocalizationState::LOC_INITILIZATING;
      break;
    case LocalizationState::INITIALIZED:
      msg.status = dcm_msgs::LocalizationState::LOC_INITED;
      break;
    case LocalizationState::RELOCALIZING:
      msg.status = dcm_msgs::LocalizationState::LOC_RELOCALIZING;
      break;
    case LocalizationState::LOCALIZED:
      msg.status = dcm_msgs::LocalizationState::LOC_SUCCESS;
      break;
    default:
      break;
  }
  return msg;
}

void AmclNode::PublishParticleCloud()
{
  pf_sample_set_t* set = pose_estimator_2d_->GetParticleSet();

  if (set == nullptr) {
    return;
  }

  geometry_msgs::PoseArray cloud_msg;
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = global_frame_id_;
  cloud_msg.poses.resize(set->sample_count);
  for (int i = 0; i < set->sample_count; i++) {
    cloud_msg.poses[i].position.x = set->samples[i].pose.v[0];
    cloud_msg.poses[i].position.y = set->samples[i].pose.v[1];
    cloud_msg.poses[i].position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, set->samples[i].pose.v[2]);
    tf2::convert(q, cloud_msg.poses[i].orientation);
  }
  particlecloud_pub_.publish(cloud_msg);
}

void AmclNode::HandleLaserScanMessage(
  const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  if (!pose_estimator_2d_->IsMapSet()) {
    return;
  }

  std::string laser_scan_frame_id = StripSlash(laser_scan->header.frame_id);
  last_laser_received_ts_ = ros::Time::now();
  std::optional<AMCLLaserData> ldata = ToLaserData(*laser_scan);
  if (!ldata.has_value()) {
    return;
  }
  // 设置雷达外参
  if (!pose_estimator_2d_->HasSensorLaser(laser_scan_frame_id)) {
    geometry_msgs::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = ros::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    geometry_msgs::PoseStamped laser_pose;
    try {
      this->tf_->transform(ident, laser_pose, base_frame_id_);
    } catch (const tf2::TransformException& e) {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan_frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }
    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    
    ROS_DEBUG("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    pose_estimator_2d_->AddSensorLaser(laser_scan_frame_id, laser_pose_v);
  }

  // pf_vector_t pose;
  // if (!GetOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
  //                  laser_scan->header.stamp, base_frame_id_))
  // {
  //   ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
  //   return;
  // }

  // AMCLOdomData odom_data;
  // odom_data.pose = pose;
  // odom_data.time = laser_scan->header.stamp.toSec();
  // pose_estimator_2d_->AddOdomData(odom_data);
  pose_estimator_2d_->AddLaserData(ldata.value());

  if (pose_estimator_2d_->NeedSetInitPoseInFilter()) {
    SetInitialPoseInFilter();
  }
}

void AmclNode::HandleOdometryMessage(
  const nav_msgs::OdometryConstPtr& odom)
{
  if (!pose_estimator_2d_->IsMapSet()) {
    return;
  }

  std::optional<AMCLOdomData> odom_data = ToOdomData(*odom);
  pose_estimator_2d_->AddOdomData(odom_data.value());

  if (!amcl_param_->use_dead_reckoning) {
    return;
  }
  PredictOdom(*odom);

  dead_reckoning_.AddOdomData(*odom);
  Eigen::Isometry3d predict_odom = dead_reckoning_.GetCurrentPose();

  geometry_msgs::PoseStamped odom_to_map;
  try {
    tmp_Q_ = predict_odom.rotation();
    tmp_P_ = predict_odom.translation();
    tf2::Quaternion q(tmp_Q_.x(), tmp_Q_.y(), tmp_Q_.z(), tmp_Q_.w());
    tf2::Transform tmp_tf(q, tf2::Vector3(tmp_P_.x(),
                                          tmp_P_.y(),
                                          0.0));

    geometry_msgs::PoseStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = base_frame_id_;
    tmp_tf_stamped.header.stamp = odom->header.stamp;
    tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

    this->tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
  } catch (const tf2::TransformException&) {
    ROS_DEBUG("Failed to subtract base to odom transform");
    return;
  }

  tf2::convert(odom_to_map.pose, latest_tf_);

  if (tf_broadcast_ == true) {
    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used
    ros::Time transform_expiration = (odom->header.stamp +
                                      transform_tolerance_);
    geometry_msgs::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = global_frame_id_;
    tmp_tf_stamped.header.stamp = transform_expiration;
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);

    this->tfb_->sendTransform(tmp_tf_stamped);
    sent_first_transform_ = true;
    std::cout << "Sent transform" << std::endl;
  }
}

double last_time;
bool is_first_frame = true;
Eigen::Isometry3d last_odom_ = Eigen::Isometry3d::Identity();
void AmclNode::PredictOdom(const nav_msgs::Odometry& odom_msg)
{
  double curr_time = odom_msg.header.stamp.toSec();

  Eigen::Vector3d vel_0, gyr_0;
  if (is_first_frame) {
    last_time = curr_time;
    is_first_frame = false;

    last_odom_.translate(Eigen::Vector3d(odom_msg.pose.pose.position.x,
                                         odom_msg.pose.pose.position.y,
                                         0.0));
    Eigen::Quaterniond q(odom_msg.pose.pose.orientation.w,
                         odom_msg.pose.pose.orientation.x,
                         odom_msg.pose.pose.orientation.y,
                         odom_msg.pose.pose.orientation.z);
    last_odom_.rotate(q);
    return;
  }
  Eigen::Isometry3d curr_odom = Eigen::Isometry3d::Identity();
  curr_odom.translate(Eigen::Vector3d(odom_msg.pose.pose.position.x,
                                      odom_msg.pose.pose.position.y,
                                      odom_msg.pose.pose.position.z));
  Eigen::Quaterniond q(odom_msg.pose.pose.orientation.w,
                        odom_msg.pose.pose.orientation.x,
                        odom_msg.pose.pose.orientation.y,
                        odom_msg.pose.pose.orientation.z);
  curr_odom.rotate(q);

  Eigen::Isometry3d delta_odom = last_odom_.inverse() * curr_odom;

  last_odom_ = curr_odom;

  Eigen::Isometry3d curr_pose  = Eigen::Isometry3d::Identity();
  curr_pose.translate(tmp_P_);
  curr_pose.rotate(tmp_Q_);

  curr_pose = curr_pose * delta_odom;
  tmp_P_ = curr_pose.translation();
  tmp_Q_ = curr_pose.rotation();

  std::cout << "predict : \n";
  std::cout << tmp_P_.x() << " " << tmp_P_.y() << std::endl;
  std::cout << tmp_Q_.x() << " " << tmp_Q_.y() << " " << tmp_Q_.z() << " " << tmp_Q_.w() << std::endl;
}

void AmclNode::CheckLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if (d > laser_check_interval_) {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

bool AmclNode::GlobalLocalizationCallback(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  if (map_ == nullptr) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  if (pose_estimator_2d_->GlobalLocalization()) {
    ROS_INFO("Global initialisation done!");
  } else {
    ROS_ERROR("Global initialisation failed!");
  }

  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool AmclNode::NoMotionUpdateCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& res)
{
  amcl_param_->force_update = true;
  ROS_INFO("Requesting no-motion update");
  return true;
}

bool AmclNode::RequestMap()
{
  std::string map_name, map_folder;

  ros::param::get("/map_manager/map_name", map_name);
  ros::param::get("/map_manager/map_folder", map_folder);

  if (map_name.empty() || map_folder.empty()) {
    ROS_ERROR("map_name or map_folder is empty");
    return false;
  }

  pose_estimator_2d_->SetMapName(map_name);
  pose_estimator_2d_->SetMapFolder(map_folder);

  ros::Subscriber map_sub;
  std::string map_topic = "/map";
  bool map_received = false;
  auto incomingMap = [&] (const nav_msgs::OccupancyGridConstPtr& map_msg) {
    map_received = true;
    
    map_ = ConvertMap(*map_msg);
    pose_estimator_2d_->SetMap(map_);
  };
  
  map_sub = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, incomingMap);

  int i = 0;
  ros::Rate r(10);
  while (!map_received && nh_.ok() && i < 20) {
    ros::spinOnce();
    r.sleep();
    i++;
  }
  if (!map_received) {
    ROS_ERROR_NAMED("amcl", "No map received");
    return false;
  }

  return true;
}

bool AmclNode::RelocaliztoinCallback(
                      dcm_srvs::SetPose::Request& req,
                      dcm_srvs::SetPose::Response& res)
{
  HandleInitialPoseMessage(req.pose);
  if (force_update_after_initialpose_) {
    amcl_param_->force_update = true;
  }
  res.success = true;
  return res.success;
}

bool AmclNode::CancelRelocaliztoinCallback(
                        std_srvs::Empty::Request& req,
                        std_srvs::Empty::Response& res)
{
  pose_estimator_2d_->CancelRelocalization();
  return true;
}

void AmclNode::InitialPoseReceived(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  HandleInitialPoseMessage(*msg);
  if (force_update_after_initialpose_) {
    amcl_param_->force_update = true;
  }
}

void AmclNode::HandleInitialPoseMessage(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if (msg.header.frame_id == "") {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  using default frame_id = map.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (StripSlash(msg.header.frame_id) != global_frame_id_) {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             StripSlash(msg.header.frame_id).c_str(),
             global_frame_id_.c_str());
    return;
  }


  tf2::Transform pose_new;
  tf2::convert(msg.pose.pose, pose_new);

  if (std::isnan(tf2::getYaw(pose_new.getRotation()))) {
    ROS_WARN("Invalid quaternion in initial pose message, ignoring message");
    pose_new.setRotation(tf2::Quaternion::getIdentity()); 
  }

  // Transform into the global frame
  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           tf2::getYaw(pose_new.getRotation()));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

  amcl_hyp_t initial_pose_hyp;
  initial_pose_hyp.pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp.pf_pose_cov = pf_init_pose_cov;
  pose_estimator_2d_->SetInitialPose(initial_pose_hyp);
}

map_t* AmclNode::ConvertMap(const nav_msgs::OccupancyGrid& map_msg)
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t) * map->size_x * map->size_y);
  map->data = (uint8_t*)malloc(sizeof(uint8_t) * map->size_x * map->size_y);
  ROS_ASSERT(map->cells);
  ROS_ASSERT(map->data);
  for (int i = 0; i < map->size_x * map->size_y; i++) {
    map->data[i] = map_msg.data[i];
    if(map_msg.data[i] == 0) {
      map->cells[i].occ_state = -1;
    } else if(map_msg.data[i] == 100) {
      map->cells[i].occ_state = +1;
    } else {
      map->cells[i].occ_state = 0;
    }
  }

  return map;
}

std::optional<AMCLLaserData> AmclNode::ToLaserData(
  const sensor_msgs::LaserScan& scan_msg)
{
  AMCLLaserData laser_data;
  std::string laser_scan_frame_id = StripSlash(scan_msg.header.frame_id);
  laser_data.frame_id = laser_scan_frame_id;
  laser_data.time = scan_msg.header.stamp.toSec();
  laser_data.range_count = scan_msg.ranges.size();

  if (laser_max_range_ > 0.0) {
    laser_data.range_max = std::min(scan_msg.range_max, (float)amcl_param_->laser_max_range);
  } else {
    laser_data.range_max = scan_msg.range_max;
  }

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, scan_msg.angle_min);
  geometry_msgs::QuaternionStamped min_q, inc_q;
  min_q.header.stamp = scan_msg.header.stamp;
  min_q.header.frame_id = laser_scan_frame_id;
  tf2::convert(q, min_q.quaternion);

  q.setRPY(0.0, 0.0, scan_msg.angle_min + scan_msg.angle_increment);
  inc_q.header = min_q.header;
  tf2::convert(q, inc_q.quaternion);
  try { // 将激光雷达的最小角度和角度增量转换到base坐标系中
    tf_->transform(min_q, min_q, base_frame_id_);
    tf_->transform(inc_q, inc_q, base_frame_id_);
  } catch (const tf2::TransformException& e) {
    ROS_WARN("Unable to transform min/max laser angles into base frame: %s",
              e.what());
    return std::nullopt;
  }
  double angle_min = tf2::getYaw(min_q.quaternion);
  double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

  // ROS_DEBUG("Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min, angle_increment);


  // The AMCLLaserData destructor will free this memory
  laser_data.ranges = new double[laser_data.range_count][2];
  laser_data.intensities = new double[laser_data.range_count][2];

  ROS_ASSERT(laser_data.ranges);
  ROS_ASSERT(laser_data.intensities);

  for (int i = 0;i < laser_data.range_count; i++) {
    laser_data.ranges[i][1] = angle_min + (i * angle_increment);
    laser_data.intensities[i][1] = angle_min + (i * angle_increment);

    if (std::isnan(scan_msg.ranges[i]) ||
        std::isinf(scan_msg.ranges[i]))
    {
      laser_data.ranges[i][0] = 0.0;
      laser_data.intensities[i][0] = 0.0;
      continue;
    }
    laser_data.ranges[i][0] = scan_msg.ranges[i];
    laser_data.intensities[i][0] = scan_msg.intensities[i];
  }

  return std::optional<AMCLLaserData>(std::move(laser_data));
}

std::optional<AMCLOdomData> AmclNode::ToOdomData(const nav_msgs::Odometry& odom_msg)
{
  AMCLOdomData odom_data;
  odom_data.time = odom_msg.header.stamp.toSec();
  odom_data.pose.v[0] = odom_msg.pose.pose.position.x;
  odom_data.pose.v[1] = odom_msg.pose.pose.position.y;
  odom_data.pose.v[2] = tf2::getYaw(odom_msg.pose.pose.orientation);
  return odom_data;
}

bool AmclNode::GetOdomPose(
  geometry_msgs::PoseStamped& odom_pose,
  double& x, double& y, double& yaw,
  const ros::Time& t, const std::string& base_frame)
{
  // Get the robot's pose
  geometry_msgs::PoseStamped ident;
  ident.header.frame_id = StripSlash(base_frame);
  ident.header.stamp = t;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
  try {
    this->tf_->transform(ident, odom_pose, odom_frame_id_);
  } catch(const tf2::TransformException& e) {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.pose.position.x;
  y = odom_pose.pose.position.y;
  yaw = tf2::getYaw(odom_pose.pose.orientation);

  return true;
}

void AmclNode::StandardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status)
{
  double std_x = sqrt(last_published_pose_.pose.covariance[6 * 0 + 0]);
  double std_y = sqrt(last_published_pose_.pose.covariance[6 * 1 + 1]);
  double std_yaw = sqrt(last_published_pose_.pose.covariance[6 * 5 + 5]);

  diagnostic_status.add("std_x", std_x);
  diagnostic_status.add("std_y", std_y);
  diagnostic_status.add("std_yaw", std_yaw);
  diagnostic_status.add("std_warn_level_x", std_warn_level_x_);
  diagnostic_status.add("std_warn_level_y", std_warn_level_y_);
  diagnostic_status.add("std_warn_level_yaw", std_warn_level_yaw_);

  if (std_x > std_warn_level_x_ || std_y > std_warn_level_y_ || std_yaw > std_warn_level_yaw_) {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Too large");
  } else {
    diagnostic_status.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
  }
}

bool AmclNode::SetInitialPoseInFilter()
{
  ros::ServiceClient reset_filter_client =
    nh_.serviceClient<std_srvs::Trigger>("/robot_localization/reset_filter");
  if (!reset_filter_client.waitForExistence(ros::Duration(3.0)) ||
      !reset_filter_client.exists())
  {
    ROS_ERROR_NAMED("amcl", "Timed out waiting for reset filter service");
    return false;
  }

  std_srvs::Trigger trigger_cmd;
  if (reset_filter_client.isValid() &&
      !reset_filter_client.call(trigger_cmd))
  {
    ROS_ERROR("Failed to call service reset_filter");
    return false;
  }

  ros::ServiceClient set_pose_client =
    nh_.serviceClient<dcm_srvs::SetPose>("/robot_localization/set_pose");
  if (!set_pose_client.waitForExistence(ros::Duration(3.0)) ||
      !set_pose_client.exists())
  {
    ROS_ERROR_NAMED("amcl", "Timed out waiting for SetPose service");
    return false;
  }

  std::optional<PoseWithCovariance> amcl_pose = pose_estimator_2d_->GetCurrentPose();
  if (!amcl_pose.has_value()) {
    ROS_ERROR_NAMED("amcl_node", "Failed to get current pose from AMCL");
    return false;
  }

  dcm_srvs::SetPose pose_cmd;
  pose_cmd.request.pose.pose.pose.position.x = amcl_pose->pose.x();
  pose_cmd.request.pose.pose.pose.position.y = amcl_pose->pose.y();
  pose_cmd.request.pose.pose.pose.position.z = 0.0;
  pose_cmd.request.pose.pose.pose.orientation =
    tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), amcl_pose->pose.z()));

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      pose_cmd.request.pose.pose.covariance[6 * i + j] = amcl_pose.value().cov(i, j);
    }
  }
  pose_cmd.request.pose.pose.covariance[6 * 5 + 5] = amcl_pose.value().cov(2, 2);

  if (set_pose_client.isValid() &&
      !set_pose_client.call(trigger_cmd))
  {
    ROS_ERROR("Failed to call service set_pose");
    return false;
  }
          
  ROS_WARN_NAMED("amcl", "SetPose service called");
  return true;
}

bool AmclNode::SetMapChangedCallback(
                      std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res)
{
  res.success = RequestMap();
  return res.success;
}

} // namespace amcl
