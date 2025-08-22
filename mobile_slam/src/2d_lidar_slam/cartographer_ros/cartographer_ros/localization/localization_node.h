#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"
#include "std_srvs/Trigger.h"
#include "dcm_srvs/SetPose.h"

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/localization/localization_updater.h"
#include "cartographer_ros/localization/localization_updater_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/common/node_constants.h"

#include "cartographer_ros/options/mapping_node_options.h"

#include "cartographer_ros/options/localization_node_options.h"
#include "cartographer_ros/options/trajectory_options.h"
#include "cartographer_ros_msgs/FinishTrajectory.h"
#include "cartographer_ros_msgs/GetTrajectoryStates.h"
#include "cartographer_ros_msgs/ReadMetrics.h"
#include "cartographer_ros_msgs/StartTrajectory.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros_msgs/SubmapEntry.h"  
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/WriteState.h"

#include "file_manager.hpp"
#include "pose_save_tool.hpp"

namespace cartographer_ros
{
namespace localization
{

inline std::string StripSlash(const std::string& in)
{
  std::string out = in;
  if (( !in.empty() ) && (in[0] == '/')) {
    out.erase(0,1);
	}
  return out;
}

class LocalizationNode
{
public:
  LocalizationNode(const LocalizationNodeOptions& node_options,
    std::unique_ptr<cartographer::localization::LocalizationUpdater> localization_updater,
    tf2_ros::Buffer* tf_buffer);
  ~LocalizationNode() = default;

  LocalizationNode(const LocalizationNode&) = delete;
  LocalizationNode& operator=(const LocalizationNode&) = delete;

  void FinishAllTrajectories();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
               const nav_msgs::Odometry::ConstPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                const sensor_msgs::NavSatFix::ConstPtr& msg);
  void HandleLandmarkMessage(
    int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
            const sensor_msgs::Imu::ConstPtr& msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                const sensor_msgs::LaserScan::ConstPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                const sensor_msgs::PointCloud2::ConstPtr& msg);
  void HandleInitialPoseMessage(
    int trajectory_id, const std::string& sensor_id,
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  // Serializes the complete Node state.
  void SerializeState(const std::string& filename,
            const bool include_unfinished_submaps);

  // Loads a serialized kSLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  void LoadMapData(const std::string& map_filename);

  ::ros::NodeHandle* node_handle();
  bool SaveMap(const std::string & map_filename, const double resolution);
private:
  struct Subscriber
  {
  ::ros::Subscriber subscriber;
  std::string topic;
  };

  bool HandleSubmapQuery(
    cartographer_ros_msgs::SubmapQuery::Request& request,
    cartographer_ros_msgs::SubmapQuery::Response& response);
  bool HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response);
  bool HandleStartTrajectory(
    cartographer_ros_msgs::StartTrajectory::Request& request,
    cartographer_ros_msgs::StartTrajectory::Response& response);
  bool HandleFinishTrajectory(
    cartographer_ros_msgs::FinishTrajectory::Request& request,
    cartographer_ros_msgs::FinishTrajectory::Response& response);
  bool HandleWriteState(cartographer_ros_msgs::WriteState::Request& request,
            cartographer_ros_msgs::WriteState::Response& response);
  bool HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response);
  bool HandleRelocalization(
    dcm_srvs::SetPose::Request& request,
    dcm_srvs::SetPose::Response& response);
  bool HandleSetMapChanged(
    std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response);

  bool SetInitialPoseInFilter(const ::geometry_msgs::PoseWithCovarianceStamped& pose_cov_msg);

  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void PublishSubmapList(const ::ros::WallTimerEvent& timer_event);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event);
  void PublishTrajectoryNodeList(const ::ros::WallTimerEvent& timer_event);
  void PublishLandmarkPosesList(const ::ros::WallTimerEvent& timer_event);
  void PublishConstraintList(const ::ros::WallTimerEvent& timer_event);
  void PublishLocalizationState(const ::ros::WallTimerEvent& timer_event);
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::StatusResponse FinishTrajectoryUnderLock(
    int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(const ::ros::WallTimerEvent&);

  cartographer_ros_msgs::StatusResponse TrajectoryStateToStatus(
    int trajectory_id,
    const std::set<
      cartographer::mapping::PoseGraphInterface::TrajectoryState>&
      valid_states);

  // const LocalizationNodeOptions node_options_;
  const LocalizationNodeOptions node_options_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  LocalizationUpdaterBridge localization_updater_bridge_ GUARDED_BY(mutex_);

  ::ros::NodeHandle node_handle_;
  ::ros::Publisher submap_list_publisher_;
  ::ros::Publisher trajectory_node_list_publisher_;
  ::ros::Publisher landmark_poses_list_publisher_;
  ::ros::Publisher constraint_list_publisher_;
  ::ros::Publisher loc_pose_publisher_;
  ::ros::Publisher tracked_path_publisher_;
  ::nav_msgs::Path tracked_path_msg_;
  // These ros::ServiceServers need to live for the lifetime of the node.
  std::vector<::ros::ServiceServer> service_servers_;
  ::ros::Publisher global_map_cloud_publisher_;
  ::ros::Publisher global_feature_map_publisher_;
  ::ros::Publisher local_map_cloud_publisher_;
  ::ros::Publisher local_feature_map_publisher_;
  ::ros::Publisher matched_feature_publisher_;
  ::ros::Publisher scan_matched_point_cloud_publisher_;
  ::ros::Publisher localization_state_publisher_;
  ::ros::Publisher particle_poses_publisher_;

  struct TrajectorySensorSamplers
  {
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

  std::map<int, ::ros::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::ros::WallTimer> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::ros::Timer publish_local_trajectory_data_timer_;

  std::shared_ptr<dcm::utility::PoseSaveTool> carto_pose_tool_;

  bool need_set_pose_in_filter_ = false;
};

}

}

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_LOCALIZATION_LOCALIZATION_NODE_H_
