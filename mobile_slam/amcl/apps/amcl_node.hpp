#ifndef AMCL_NODE_HPP_
#define AMCL_NODE_HPP_

#include <iostream>
#include <string>
#include <memory>
#include <optional>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/srv/set_map.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"

// ROS2 parameter system
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_client.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "amcl/map/map.h"
#include "amcl/pf/pf.h"
#include "amcl/sensors/amcl_odom.h"
#include "amcl/sensors/amcl_laser.h"
#include "amcl/utils/portable_utils.hpp"
#include "amcl/core/pose_estimator_2d.hpp"
#include "amcl/core/amcl_param.hpp"
// #include "amcl/AMCLConfig.h"  // Dynamic reconfigure not used in ROS2
#include "file_manager.hpp"
#include "pose_save_tool.hpp"
#include "nav_interfaces/msg/localization_state.hpp"
#include "nav_interfaces/srv/set_pose.hpp"

namespace amcl
{

inline std::string StripSlash(const std::string& in)
{
  std::string out = in;
  if (( !in.empty() ) && (in[0] == '/')) {
    out.erase(0,1);
	}
  return out;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
  typedef typename Derived::Scalar Scalar_t;

  Eigen::Quaternion<Scalar_t> dq;
  Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
  half_theta /= static_cast<Scalar_t>(2.0);
  dq.w() = static_cast<Scalar_t>(1.0);
  dq.x() = half_theta.x();
  dq.y() = half_theta.y();
  dq.z() = half_theta.z();
  return dq;
}

class AmclNode : public rclcpp::Node
{
public:
  AmclNode();
  ~AmclNode();

private:
  void InitializeNode();
  void InitAMCLParams();
  void InitNodeParams();

  void UpdatePoseFromServer();
  void SavePoseToServer();

  void PublishThread();
  void PublishParticleCloud();

  void GlobalLocalizationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void NoMotionUpdateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void SetMapCallback(const std::shared_ptr<nav_msgs::srv::SetMap::Request> req,
                      std::shared_ptr<nav_msgs::srv::SetMap::Response> res);
  void RelocaliztoinCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void CancelRelocaliztoinCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  void SetMapChangedCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  void InitialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void HandleInitialPoseMessage(const geometry_msgs::msg::PoseWithCovarianceStamped& msg);

  bool RequestMap();
  map_t* ConvertMap(const nav_msgs::msg::OccupancyGrid& map_msg);

  void CheckLaserReceived();

  void HandleLaserScanMessage(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);
  void HandleOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr odom);

  std::optional<AMCLLaserData> ToLaserData(const sensor_msgs::msg::LaserScan& scan_msg);
  std::optional<AMCLOdomData> ToOdomData(const nav_msgs::msg::Odometry& odom_msg);

  bool GetOdomPose(geometry_msgs::msg::PoseStamped& odom_pose,
                   double& x, double& y, double& yaw,
                   const rclcpp::Time& t, const std::string& base_frame);

  void PredictOdom(const nav_msgs::msg::Odometry& odom_msg);

  // 设置 robot localization 中的初始位姿
  bool SetInitialPoseInFilter();

  nav_interfaces::msg::LocalizationState ToMsg(LocalizationState state);
private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_loc_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nomotion_update_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_relocalization_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_map_changed_srv_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_old_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  bool tf_broadcast_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_;

  //time for tolerance on the published transform,
  //basically defines how long a map->odom transform is good for
  rclcpp::Duration transform_tolerance_;

  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string global_frame_id_;

  bool use_map_topic_;
  bool first_map_only_;
  std::string map_topic_;
  std::string scan_topic_;

  rclcpp::Duration gui_publish_period;
  rclcpp::Time save_pose_last_time;
  rclcpp::Duration save_pose_period;

  rclcpp::Duration cloud_pub_interval;
  rclcpp::Time last_cloud_pub_time;
  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan>* laser_scan_sub_;
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>* laser_scan_filter_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  diagnostic_updater::Updater diagnosic_updater_;
  void StandardDeviationDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& diagnostic_status);
  double std_warn_level_x_;
  double std_warn_level_y_;
  double std_warn_level_yaw_;

  bool first_map_received_;
  bool first_reconfigure_call_;
  bool force_update_after_initialpose_;
  bool force_update_after_set_map_;

  bool sent_first_transform_;

  tf2::Transform latest_tf_;
  bool latest_tf_valid_;

  std::mutex configuration_mutex_;
  // Dynamic reconfigure not used in ROS2

  rclcpp::TimerBase::SharedPtr check_laser_timer_;
  rclcpp::Time last_laser_received_ts_;
  rclcpp::Duration laser_check_interval_;

  rclcpp::Duration cloud_pub_interval_;
  rclcpp::TimerBase::SharedPtr publish_particle_timer_;

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  rclcpp::Duration pose_pub_interval_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr amcl_odom_pub_;
  rclcpp::Publisher<nav_interfaces::msg::LocalizationState>::SharedPtr loc_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr matched_cloud_pub_;

  // TODO: 增加自己的代码
  bool is_save_pose_ = false;
  std::shared_ptr<dcm::utility::PoseSaveTool> amcl_pose_tool_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::Path path_msg_;

  std::shared_ptr<AmclParam> amcl_param_;
  std::unique_ptr<PoseEstimator2D> pose_estimator_2d_;
  map_t* map_;

  geometry_msgs::msg::PoseStamped latest_odom_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped last_published_pose_;

  double laser_min_range_;
  double laser_max_range_;

  Eigen::Vector3d tmp_P_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond tmp_Q_ = Eigen::Quaterniond::Identity();
};

} // namespace amcl

#endif // AMCL_NODE_HPP_laser_data