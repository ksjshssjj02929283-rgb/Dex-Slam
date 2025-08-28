/**
 * @file node.h
 * @author ZheHe
 * @brief 订阅ROS话题传入SLAM算法，并调用 消息格式转换/传感器数据处理/局部SLAM（TrajectoryBuilder）和全局SLAM（MapBuilder） 的ROS接口，进行模块封装
 * @version 0.1
 * @date 2023-05-22
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// added by hz
#include "cartographer/transform/rigid_transform.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// addded by hz end

namespace cartographer_ros {

// Wires up ROS topics to SLAM.
class Node {
 public:
  Node(const NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       std::shared_ptr<tf2_ros::Buffer> tf_buffer,
       rclcpp::Node::SharedPtr node,
       bool collect_metrics);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // added by hz
  void StartLocalizationWithInitialPose(cartographer::transform::Rigid3d initial_pose);    
  // end

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options, 
                                        bool flag_localization, 
                                        const std::string& initial_poses_filename, 
                                        const std::string& apriltag_poses_filename,
                                        const std::string& location_name); // added by hz

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

 private:
  struct Subscriber {
    rclcpp::SubscriptionBase::SharedPtr subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };

  bool handleSubmapQuery(
      const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
      cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response);
  bool handleTrajectoryQuery(
      const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
      cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response);
  bool handleStartTrajectory(
      const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response);
  bool handleFinishTrajectory(
      const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response);
  bool handleWriteState(
      const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
      cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response);
  bool handleGetTrajectoryStates(
      const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr,
      cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response);
  bool handleReadMetrics(const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
      cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response);

  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void PublishSubmapList();
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData();
  void PublishTrajectoryNodeList();
  void PublishLandmarkPosesList();
  void PublishConstraintList();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  cartographer_ros_msgs::msg::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch();

  // Helper function for service handlers that need to check trajectory states.
  cartographer_ros_msgs::msg::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  absl::Mutex mutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  // 在访问 map_builder_bridge_ 之前，需要先获得 mutex_ 锁，并在访问结束后释放锁,确保多个线程在同时访问 map_builder_bridge_ 时不会产生数据竞争问题
  std::shared_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

  rclcpp::Node::SharedPtr node_;
  ::rclcpp::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr landmark_poses_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;
  ::rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_publisher_;
  ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  // new added by hz:
  ::rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_publisher_;
  ::rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_subscriber_;
  ::rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr apriltag_pose_subscriber_;
  void HandleInitPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void HandleApriltagPoseMsg(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  cartographer::transform::Rigid3d init_pose;
  bool flag_init_pose_received = false;
  TrajectoryOptions trajectory_options;
  cartographer::transform::Rigid3d apriltag_pose;
  bool flag_apriltag_pose_received = false;
  cartographer::transform::Rigid3d recorded_initial_pose;
  bool flag_recorded_initial_pose_received = false;
  cartographer::transform::Rigid3d recorded_apriltag_pose;
  bool flag_recorded_apriltag_pose_received = false;
  int miss_apriltag_detection_count = 0;
  bool flag_finish_initial_localization = false; 
  // new added by hz end
  // These ros service servers need to live for the lifetime of the node.
  ::rclcpp::Service<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr submap_query_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_server;
  ::rclcpp::Service<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_trajectory_states_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::ReadMetrics>::SharedPtr read_metrics_server_;


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

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, builtin_interfaces::msg::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::rclcpp::TimerBase::SharedPtr submap_list_timer_;
  ::rclcpp::TimerBase::SharedPtr local_trajectory_data_timer_;
  ::rclcpp::TimerBase::SharedPtr trajectory_node_list_timer_;
  ::rclcpp::TimerBase::SharedPtr landmark_pose_list_timer_;
  ::rclcpp::TimerBase::SharedPtr constrain_list_timer_;
  ::rclcpp::TimerBase::SharedPtr maybe_warn_about_topic_mismatch_timer_;

  // added by hz
  std::map<int, geometry_msgs::msg::TransformStamped> map_to_local_buffer;
  std::map<int, geometry_msgs::msg::TransformStamped> local_to_published_buffer;
  int transform_id;
  // end
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
