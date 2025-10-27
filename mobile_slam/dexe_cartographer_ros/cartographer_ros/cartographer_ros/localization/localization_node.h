#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

// ROS2 messages
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "dcm_msgs/msg/localization_state.hpp"

// ROS2 services
#include "std_srvs/srv/trigger.hpp"
#include "dcm_srvs/srv/set_pose.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"

// Cartographer
#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/localization/localization_updater.h"

// Bridge
#include "cartographer_ros/localization/localization_updater_bridge.h"

// Metrics/Constants/Options
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/common/node_constants.h"
#include "cartographer_ros/options/mapping_node_options.h"
#include "cartographer_ros/options/localization_node_options.h"
#include "cartographer_ros/options/trajectory_options.h"

#include "file_manager.hpp"
#include "pose_save_tool.hpp"

namespace cartographer_ros
{
namespace localization
{

inline std::string StripSlash(const std::string& in)
{
  std::string out = in;
  if ((!in.empty()) && (in[0] == '/')) {
    out.erase(0, 1);
  }
  return out;
}

class LocalizationNode
{
public:
  LocalizationNode(const LocalizationNodeOptions& node_options,
                   std::unique_ptr<cartographer::localization::LocalizationUpdater> localization_updater,
                   std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                   rclcpp::Node::SharedPtr node);
  ~LocalizationNode() = default;

  LocalizationNode(const LocalizationNode&) = delete;
  LocalizationNode& operator=(const LocalizationNode&) = delete;

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Public controls
  void FinishAllTrajectories();

  // Serialization
  void SerializeState(const std::string& filename, const bool include_unfinished_submaps);
  void LoadState(const std::string& state_filename, bool load_frozen_state);
  void LoadMapData(const std::string& map_filename);

  bool SaveMap(const std::string& map_filename, const double resolution);

private:
  struct Subscriber {
    rclcpp::SubscriptionBase::SharedPtr sub;
    std::string topic;
  };

  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
  void HandleLandmarkMessage(int trajectory_id, const std::string& sensor_id,
                             const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void HandleInitialPoseMessage(int trajectory_id, const std::string& sensor_id,
                                const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

  // Service handlers (ROS2 signatures)
  void HandleSubmapQuery(
      const std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Response> response);

  void HandleTrajectoryQuery(
      const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response);

  void HandleStartTrajectory(
      const std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Response> response);

  void HandleFinishTrajectory(
      const std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Response> response);

  void HandleWriteState(
      const std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Response> response);

  void HandleGetTrajectoryStates(
      const std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Request> request,
      std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Response> response);

  void HandleRelocalization(
      const std::shared_ptr<dcm_srvs::srv::SetPose::Request> request,
      std::shared_ptr<dcm_srvs::srv::SetPose::Response> response);

  void HandleSetMapChanged(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Helpers
  bool SetInitialPoseInFilter(const geometry_msgs::msg::PoseWithCovarianceStamped& pose_cov_msg);

  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;

  // Timed publishers
  void PublishSubmapList();
  void PublishLocalTrajectoryData();
  void PublishTrajectoryNodeList();
  void PublishLandmarkPosesList();
  void PublishConstraintList();
  void PublishLocalizationState();

  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);

  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::msg::StatusResponse FinishTrajectoryUnderLock(
    int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(localization_updater_mutex_);
  void MaybeWarnAboutTopicMismatch();

  cartographer_ros_msgs::msg::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
    const std::set<
      cartographer::mapping::PoseGraphInterface::TrajectoryState>&
      valid_states);

  // const LocalizationNodeOptions node_options_;
  const LocalizationNodeOptions node_options_;

  // ROS2 Node handle
  rclcpp::Node::SharedPtr node_;

  // TF broadcaster (ROS2)
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Thread-safety: 拆分为两个独立的 mutex 以减少锁竞争
  absl::Mutex localization_updater_mutex_;  // 保护 localization_updater_bridge_ 的访问
  absl::Mutex sensor_samplers_mutex_;       // 保护 sensor_samplers_ 的访问

  // Metrics registry
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;

  // Bridge（非 Node，ROS2 化）
  LocalizationUpdaterBridge localization_updater_bridge_ GUARDED_BY(localization_updater_mutex_);

  // Publishers
  rclcpp::Publisher<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_poses_list_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr loc_pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tracked_path_publisher_;
  nav_msgs::msg::Path tracked_path_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_feature_map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_cloud_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_feature_map_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr matched_feature_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  rclcpp::Publisher<dcm_msgs::msg::LocalizationState>::SharedPtr localization_state_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_poses_publisher_;

  // Services (kept alive as members)
  rclcpp::Service<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr submap_query_service_;
  rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_service_;
  rclcpp::Service<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_service_;
  rclcpp::Service<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_service_;
  rclcpp::Service<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_service_;
  rclcpp::Service<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_trajectory_states_service_;
  rclcpp::Service<dcm_srvs::srv::SetPose>::SharedPtr relocalization_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_map_changed_service_;

  // Subscribers per trajectory
  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  std::map<int, rclcpp::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_ GUARDED_BY(sensor_samplers_mutex_);

  // Subscribers grouped by trajectory
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_submap_list_timer_;
  rclcpp::TimerBase::SharedPtr publish_trajectory_node_list_timer_;
  rclcpp::TimerBase::SharedPtr publish_landmark_poses_list_timer_;
  rclcpp::TimerBase::SharedPtr publish_constraint_list_timer_;
  rclcpp::TimerBase::SharedPtr maybe_warn_about_topic_mismatch_timer_;

  // Regular timer for local trajectory data (与 ROS1 注释保持一致)
  rclcpp::TimerBase::SharedPtr publish_local_trajectory_data_timer_;

  // Pose save tool
  std::shared_ptr<dcm::utility::PoseSaveTool> carto_pose_tool_;

  // Flags
  bool need_set_pose_in_filter_ = false;
};

}  // namespace localization
}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_
