#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "visualization_msgs/MarkerArray.h"

#include "glog/logging.h"
#include "nav_msgs/Odometry.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/mapping/sensor_bridge.h"
#include "cartographer_ros/mapping/tf_bridge.h"
#include "cartographer_ros/conversion/time_conversion.h"
#include "cartographer_ros_msgs/StatusCode.h"
#include "cartographer_ros_msgs/StatusResponse.h"
#include "cartographer_ros/localization/localization_node.h"

#include "dcm_msgs/LocalizationState.h"

namespace cartographer_ros
{
namespace localization
{

namespace carto = ::cartographer;
using carto::transform::Rigid3d;
using TrajectoryState =
  ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace
{

template <typename MessageType>
::ros::Subscriber SubscribeWithHandler(
    void (LocalizationNode::*handler)(int, const std::string&,
                          const typename MessageType::ConstPtr&),
    const int trajectory_id, const std::string& topic,
    ::ros::NodeHandle* const node_handle, LocalizationNode* const node)
{
  return node_handle->subscribe<MessageType>(
      topic, kInfiniteSubscriberQueueSize,
      boost::function<void(const typename MessageType::ConstPtr&)>(
          [node, handler, trajectory_id,
           topic](const typename MessageType::ConstPtr& msg) {
            (node->*handler)(trajectory_id, topic, msg);
          }));
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

} // namespace

LocalizationNode::LocalizationNode(
  const LocalizationNodeOptions& node_options,
  std::unique_ptr<cartographer::localization::LocalizationUpdater> localization_updater,
  tf2_ros::Buffer* const tf_buffer)
: node_options_(node_options),
  localization_updater_bridge_(node_options_, std::move(localization_updater), tf_buffer)
{
  absl::MutexLock lock(&mutex_);

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  particle_poses_publisher_ =
    node_handle_.advertise<::geometry_msgs::PoseArray>(
        kParticlePosesTopic, kLatestOnlyPublisherQueueSize);

  // 发布跟踪的位姿
  if (node_options_.publish_tracked_pose) {
    loc_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseWithCovarianceStamped>(
            kLocaliztionPoseTopic, kLatestOnlyPublisherQueueSize);
    tracked_path_publisher_ =
        node_handle_.advertise<::nav_msgs::Path>(
            kTrackedPathTopic, kLatestOnlyPublisherQueueSize);
  }
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &LocalizationNode::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &LocalizationNode::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &LocalizationNode::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &LocalizationNode::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &LocalizationNode::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &LocalizationNode::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kHandleRelocalizationServiceName, &LocalizationNode::HandleRelocalization, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kHandleSetMapChangedServiceName, &LocalizationNode::HandleSetMapChanged, this));

  global_map_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kGlobalPointCloudTopic, kLatestOnlyPublisherQueueSize, true);
  global_feature_map_publisher_ =
      node_handle_.advertise<visualization_msgs::MarkerArray>(
          kGlobalFeatureMapTopic, kLatestOnlyPublisherQueueSize, true);
  local_map_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kLocalPointCloudTopic, kLatestOnlyPublisherQueueSize);
  local_feature_map_publisher_ =
      node_handle_.advertise<visualization_msgs::MarkerArray>(
          kLocalFeatureMapTopic, kLatestOnlyPublisherQueueSize);
  matched_feature_publisher_ =
      node_handle_.advertise<visualization_msgs::MarkerArray>(
          kMatchedFeatureTopic, kLatestOnlyPublisherQueueSize);
  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  localization_state_publisher_ = 
      node_handle_.advertise<dcm_msgs::LocalizationState>(
          kLocalizationStateTopic, kLatestOnlyPublisherQueueSize);

  // submap发布间隔
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &LocalizationNode::PublishSubmapList, this));
  // 发布局部轨迹
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &LocalizationNode::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &LocalizationNode::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &LocalizationNode::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &LocalizationNode::PublishConstraintList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(0.1),
      &LocalizationNode::PublishLocalizationState, this));

  // 定位位姿的保存
  if(node_options_.save_tracked_pose) {
    if (!boost::filesystem::is_directory(node_options_.tracked_pose_directory)) {
      dcm::utility::FileManager::InitDirectory(node_options_.tracked_pose_directory, "tracked_pose of cartographer");
    }
    std::string file_path = node_options_.tracked_pose_directory + node_options_.tracked_pose_filename;
    dcm::utility::FileManager::RemoveFile(file_path);
    carto_pose_tool_ = std::make_shared<dcm::utility::PoseSaveTool>(file_path, node_options_.tracked_pose_format);
  }
}

::ros::NodeHandle* LocalizationNode::node_handle() { return &node_handle_; }

// TODO: 需要更改，改成显示local map的submap
bool LocalizationNode::HandleSubmapQuery(
    ::cartographer_ros_msgs::SubmapQuery::Request& request,
    ::cartographer_ros_msgs::SubmapQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  localization_updater_bridge_.HandleSubmapQuery(request, response);
  return true;
}

// TODO: 确定是发布轨迹的逻辑
bool LocalizationNode::HandleTrajectoryQuery(
    ::cartographer_ros_msgs::TrajectoryQuery::Request& request,
    ::cartographer_ros_msgs::TrajectoryQuery::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = TrajectoryStateToStatus(
      request.trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response.status.message;
    return true;
  }
  localization_updater_bridge_.HandleTrajectoryQuery(request, response);

  return true;
}

void LocalizationNode::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  // 根据不同传感器的采样率，创建采样器
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void LocalizationNode::PublishLocalizationState(const ::ros::WallTimerEvent& timer_event)
{
  const auto loc_state = localization_updater_bridge_.GetLocalizationState();
  dcm_msgs::LocalizationState loc_state_msg;
  if (loc_state == carto::localization::LocalizationState::LOCALIZATION_FAILED) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_FAILED;
  } else if (loc_state == carto::localization::LocalizationState::INITIALIZATION_FAILED) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_INIT_FAILED;
  } else if (loc_state == carto::localization::LocalizationState::NOT_INITAILIED) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_NOT_INIT;
  } else if (loc_state == carto::localization::LocalizationState::INITIALIZATING) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_INITILIZATING;
  } else if (loc_state == carto::localization::LocalizationState::INITIALIZED) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_INITED;
  } else if (loc_state == carto::localization::LocalizationState::RELOCALIZING) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_RELOCALIZING;
  } else if (loc_state == carto::localization::LocalizationState::LOCALIZED) {
    loc_state_msg.status = dcm_msgs::LocalizationState::LOC_SUCCESS;
  }
  localization_state_publisher_.publish(loc_state_msg);
}

/**/
void LocalizationNode::PublishLocalTrajectoryData(const ::ros::TimerEvent& timer_event) {
  absl::MutexLock lock(&mutex_);
  // 获得不同轨迹的数据，并进行插值
  for (const auto& entry : localization_updater_bridge_.GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (scan_matched_point_cloud_publisher_.getNumSubscribers() > 0) {
      // TODO(gaschler): Consider using other message without time
      // information.
      carto::sensor::TimedPointCloud point_cloud;
      point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                              .returns.size());
      for (const cartographer::sensor::RangefinderPoint point :
            trajectory_data.local_slam_data->range_data_in_local.returns) {
        point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
            point, 0.f /* time */));
      }
      scan_matched_point_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_data.local_slam_data->time),
          node_options_.map_frame,
          carto::sensor::TransformTimedPointCloud(
              point_cloud, carto::transform::Rigid3f::Identity())));
    }

    if (matched_feature_publisher_.getNumSubscribers() > 0) {
      visualization_msgs::MarkerArray local_line_msg;
      const auto& line_data = trajectory_data.local_slam_data->line_data_in_local;
      for (const auto& line_element : line_data) {
        for (const auto& line : line_element.second) {
          std::pair<Eigen::Vector2d, Eigen::Vector2d> line_eigen;
          line_eigen.first = line.begin();
          line_eigen.second = line.end();
          local_line_msg.markers.push_back(CreateLineMsg(
                                            line_element.first,
                                            node_options_.map_frame,
                                            line_eigen,
                                            COLOR::green));
        }
      }
      matched_feature_publisher_.publish(local_line_msg);
    }

    if (local_feature_map_publisher_.getNumSubscribers() > 0) {
      visualization_msgs::MarkerArray map_line_msg;
      const auto& line_map_2d_list = trajectory_data.local_slam_data->line_map_2d_list->GetLineMapData2DList();
      for (const auto& line_map : line_map_2d_list) {
        std::pair<Eigen::Vector2d, Eigen::Vector2d> line_eigen;
        line_eigen.first = line_map.second.line_data().begin();
        line_eigen.second = line_map.second.line_data().end();
        map_line_msg.markers.push_back(CreateLineMsg(
                                          line_map.first,
                                          node_options_.map_frame,
                                          line_eigen,
                                          COLOR::red));
      }
      local_feature_map_publisher_.publish(map_line_msg);
    }

    if (local_map_cloud_publisher_.getNumSubscribers() > 0) {
      carto::sensor::TimedPointCloud local_map_cloud;
      for (const auto& point : trajectory_data.local_slam_data->local_map->points()) {
        local_map_cloud.push_back(carto::sensor::ToTimedRangefinderPoint(
            point, 0.f /* time */));
      }
      local_map_cloud_publisher_.publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_data.local_slam_data->time),
          node_options_.map_frame,
          carto::sensor::TransformTimedPointCloud(
              local_map_cloud, carto::transform::Rigid3f::Identity())));
    }

    static bool first_publish_map = true;
    if (first_publish_map) {
      global_map_cloud_publisher_.publish(localization_updater_bridge_.GetCloudMap());

      visualization_msgs::MarkerArray feature_map_msg;
      int marker_id = 0;
      const auto line_map_msg = localization_updater_bridge_.GetLineMap(marker_id);
      feature_map_msg.markers.insert(feature_map_msg.markers.end(), line_map_msg.begin(), line_map_msg.end());
      global_feature_map_publisher_.publish(feature_map_msg);
      
      first_publish_map = false;
    }

    geometry_msgs::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = FromRos(ros::Time::now());
    stamped_transform.header.stamp = ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

    // 获取当前机器人在local坐标系下的位姿
    const Rigid3d tracking_to_local_3d = trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      // 将位姿映射到2D平面
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();
    // 当前跟踪的机器人位姿 = 局部到地图的位姿 * 跟踪到局部的位姿
    // T_map_base = T_map_local * T_local_base
    // const Rigid3d tracking_to_map =
    //     trajectory_data.local_to_map * tracking_to_local;

    const Rigid3d tracking_to_map = tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {
      if (node_options_.publish_to_tf) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          // 发布
          std::vector<geometry_msgs::TransformStamped> stamped_transforms;
          // odom to map
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.transform =
              ToGeometryMsgTransform(carto::transform::Rigid3d::Identity());
          stamped_transforms.push_back(stamped_transform);

          // published_frame to odom(什么是 published_frame?)
          // 来自官方的解释
          // 发布姿态时要使用的ROS坐标系ID作为子坐标系。例如，如果系统的其他部分
          // 提供了一个“odom”坐标系，则可以将其设置为“odom”。在这种情况下，
          // 将发布“odom”在“map_frame”中的姿态。否则，将其设置为“base_link”可能是合适的。
          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          // T_odom_published = T_odom_base * T_base_published
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);

          tf_broadcaster_.sendTransform(stamped_transforms);
        } else {
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_.sendTransform(stamped_transform);
        }
      }
      // 发布跟踪的位姿
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header.frame_id = node_options_.map_frame;
        pose_cov_msg.header.stamp = stamped_transform.header.stamp;
        pose_cov_msg.pose.pose = ToGeometryMsgPose(tracking_to_map);
        // for (int i = 0; i < 2; i++) {
        //   for (int j = 0; j < 2; j++) {
        //     pose_cov_msg.pose.covariance[6 * i + j] = 0.1 * 0.1;
        //   }
        // }
        pose_cov_msg.pose.covariance[6 * 0 + 0] = 0.05 * 0.05;
        pose_cov_msg.pose.covariance[6 * 1 + 1] = 0.05 * 0.05;
        pose_cov_msg.pose.covariance[6 * 5 + 5] = 0.05 * 0.05;

        loc_pose_publisher_.publish(pose_cov_msg);
        if (need_set_pose_in_filter_) {
          SetInitialPoseInFilter(pose_cov_msg);
        }
        // ::geometry_msgs::PoseStamped pose_stamped_msg;
        // pose_stamped_msg.header = pose_cov_msg.header;
        // pose_stamped_msg.pose = pose_cov_msg.pose.pose;
        // tracked_path_msg_.header.stamp = stamped_transform.header.stamp;
        // tracked_path_msg_.header.frame_id = node_options_.map_frame;
        // tracked_path_msg_.poses.push_back(pose_stamped_msg);
        // tracked_path_publisher_.publish(tracked_path_msg_);
        if (particle_poses_publisher_.getNumSubscribers() > 0) {
          std::vector<carto::transform::Rigid2d> pf_poses = trajectory_data.local_slam_data->particle_poses;
          geometry_msgs::PoseArray pose_array_msg;
          pose_array_msg.header.stamp = ros::Time::now();
          pose_array_msg.header.frame_id = node_options_.map_frame;
          pose_array_msg.poses.resize(pf_poses.size());
          for (size_t i = 0; i < pf_poses.size(); i++) {
            pose_array_msg.poses[i].position.x = pf_poses[i].translation().x();
            pose_array_msg.poses[i].position.y = pf_poses[i].translation().y(); 
            pose_array_msg.poses[i].position.z = 0;
            tf2::Quaternion q;
            q.setRPY(0, 0, pf_poses[i].rotation().angle());
            tf2::convert(q, pose_array_msg.poses[i].orientation);
          }
          particle_poses_publisher_.publish(pose_array_msg);
        }
      }

      if (node_options_.save_tracked_pose &&
        trajectory_data.trajectory_options.trajectory_builder_options.has_pure_localization_trimmer())
      {
        Eigen::Vector3d t = tracking_to_map.translation();
        Eigen::Quaterniond q = tracking_to_map.rotation();
        double time = stamped_transform.header.stamp.toSec();
        if (!carto_pose_tool_) {
          LOG(INFO) << "carto_pose_tool_ is nullptr\n";
        } else {
          carto_pose_tool_->AddPoseData(t, q, time);
        }
      }
    }
  }
}

bool LocalizationNode::SaveMap(
  const std::string & map_filename, const double resolution)
{
  return false;
  // return localization_updater_bridge_.SaveMap(map_filename, resolution);
}


void LocalizationNode::PublishSubmapList(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_.publish(localization_updater_bridge_.GetSubmapList());
}

void LocalizationNode::PublishTrajectoryNodeList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (trajectory_node_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_.publish(
        localization_updater_bridge_.GetTrajectoryNodeList());
  }
}

void LocalizationNode::PublishLandmarkPosesList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (landmark_poses_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_.publish(
        localization_updater_bridge_.GetLandmarkPosesList());
  }
}

void LocalizationNode::PublishConstraintList(
    const ::ros::WallTimerEvent& unused_timer_event) {
  if (constraint_list_publisher_.getNumSubscribers() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_.publish(localization_updater_bridge_.GetConstraintList());
  }
}


std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
LocalizationNode ::ComputeExpectedSensorIds(const TrajectoryOptions& options) const
{
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data()) {
    ROS_WARN("subscribe to imu topic.");
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int LocalizationNode::AddTrajectory(const TrajectoryOptions& options) {
  // 1. 根据配置，检查具有的传感器以及期望订阅的topic
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  // 2. 根据配置，向localization_updater中增加轨迹优化器
  const int trajectory_id =
      localization_updater_bridge_.AddTrajectory(expected_sensor_ids, options);
  // 4. 增加传感器采样器(用于计算和控制传感器采样输入)
  AddSensorSamplers(trajectory_id, options);
  // 5. 订阅传感器数据
  LaunchSubscribers(options, trajectory_id);
  // 6. 每kTopicMismatchCheckDelaySec秒检查一次topic是否匹配
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kTopicMismatchCheckDelaySec),
      &LocalizationNode::MaybeWarnAboutTopicMismatch, this, /*oneshot=*/true));
  // 7. 订阅的topic以及与之对应的传感器id
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void LocalizationNode::LaunchSubscribers(
  const TrajectoryOptions& options, const int trajectory_id) 
{
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::LaserScan>(
             &LocalizationNode::HandleLaserScanMessage, trajectory_id, topic, &node_handle_,
             this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::PointCloud2>(
             &LocalizationNode::HandlePointCloud2Message, trajectory_id, topic,
             &node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  // TODO: 这里需要修改一下trajectory_builder_2d_options，应该改成pose estimator options
  if (options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data()) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::Imu>(&LocalizationNode::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                &node_handle_, this),
         kImuTopic});
  }

  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::Odometry>(&LocalizationNode::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  &node_handle_, this),
         kOdometryTopic});
  }
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::NavSatFix>(
             &LocalizationNode::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             &node_handle_, this),
         kNavSatFixTopic});
  }
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::LandmarkList>(
             &LocalizationNode::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             &node_handle_, this),
         kLandmarkTopic});
  }

  subscribers_[trajectory_id].push_back(
      {SubscribeWithHandler<geometry_msgs::PoseWithCovarianceStamped>(
            &LocalizationNode::HandleInitialPoseMessage, trajectory_id, kSetInitialPoseTopic,
            &node_handle_, this),
        kSetInitialPoseTopic});
}

bool LocalizationNode::ValidateTrajectoryOptions(const TrajectoryOptions& options)
{
  return true;
}

cartographer_ros_msgs::StatusResponse LocalizationNode::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = localization_updater_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
    status_response.code = cartographer_ros_msgs::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message =
      absl::StrCat("Trajectory ", trajectory_id, " is in '",
                   TrajectoryStateToString(it->second), "' state.");
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::StatusCode::OK
          : cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

bool LocalizationNode::ValidateTopicNames(const TrajectoryOptions& options)
{
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::StatusResponse LocalizationNode::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    status_response.code = cartographer_ros_msgs::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.shutdown();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  localization_updater_bridge_.FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::StatusCode::OK;
  return status_response;
}

bool LocalizationNode::HandleStartTrajectory(
    ::cartographer_ros_msgs::StartTrajectory::Request& request,
    ::cartographer_ros_msgs::StartTrajectory::Response& response) {
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadMappingOptions(
      request.configuration_directory, request.configuration_basename);

  if (request.use_initial_pose) {
    const auto pose = ToRigid3d(request.initial_pose);
    if (!pose.IsValid()) {
      response.status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response.status.message;
      response.status.code =
          cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response.status = TrajectoryStateToStatus(
        request.relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response.status.code != cartographer_ros_msgs::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response.status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request.relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(ros::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response.status.message = "Invalid trajectory options.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response.status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response.status.message;
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
  } else {
    response.status.message = "Success.";
    response.trajectory_id = AddTrajectory(trajectory_options);
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
  }
  return true;
}

void LocalizationNode::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options)
{
  absl::MutexLock lock(&mutex_);
  // 检查是否有是否有轨迹优化器参数
  CHECK(ValidateTrajectoryOptions(options));
  // 创建轨迹优化器
  AddTrajectory(options);
}

void LocalizationNode::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : localization_updater_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::StatusCode::OK);
    }
  }
}


/* --------------------------------- handle service ---------------------------------------- */
bool LocalizationNode::HandleGetTrajectoryStates(
    ::cartographer_ros_msgs::GetTrajectoryStates::Request& request,
    ::cartographer_ros_msgs::GetTrajectoryStates::Response& response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response.status.code = ::cartographer_ros_msgs::StatusCode::OK;
  response.trajectory_states.header.stamp = ros::Time::now();
  for (const auto& entry : localization_updater_bridge_.GetTrajectoryStates()) {
    response.trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response.trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

bool LocalizationNode::HandleFinishTrajectory(
    ::cartographer_ros_msgs::FinishTrajectory::Request& request,
    ::cartographer_ros_msgs::FinishTrajectory::Response& response) {
  absl::MutexLock lock(&mutex_);
  response.status = FinishTrajectoryUnderLock(request.trajectory_id);
  return true;
}

bool LocalizationNode::HandleWriteState(
    ::cartographer_ros_msgs::WriteState::Request& request,
    ::cartographer_ros_msgs::WriteState::Response& response) {
  absl::MutexLock lock(&mutex_);
  if (localization_updater_bridge_.SerializeState(request.filename,
                                         request.include_unfinished_submaps)) {
    response.status.code = cartographer_ros_msgs::StatusCode::OK;
    response.status.message =
        absl::StrCat("State written to '", request.filename, "'.");
  } else {
    response.status.code = cartographer_ros_msgs::StatusCode::INVALID_ARGUMENT;
    response.status.message =
        absl::StrCat("Failed to write '", request.filename, "'.");
  }
  return true;
}

bool LocalizationNode::HandleRelocalization(
            dcm_srvs::SetPose::Request& request,
            dcm_srvs::SetPose::Response& response)
{
  if (request.pose.header.frame_id == "") {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  using default frame_id = map.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (StripSlash(request.pose.header.frame_id) != node_options_.map_frame) {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             StripSlash(request.pose.header.frame_id).c_str(),
             node_options_.map_frame.c_str());
    return false;
  }

  const carto::transform::Rigid3d initial_pose = ToRigid3d(request.pose.pose.pose);

  need_set_pose_in_filter_ = true;
  auto initial_pose_2d = carto::transform::Project2D(initial_pose);
  ROS_WARN("Set initial pose: %f, %f, %f",
        initial_pose_2d.translation().x(),
        initial_pose_2d.translation().y(),
        initial_pose_2d.rotation().angle());
  response.success = localization_updater_bridge_.SetInitialPose(initial_pose);
  return response.success;
}

bool LocalizationNode::HandleSetMapChanged(
            std_srvs::Trigger::Request& request,
            std_srvs::Trigger::Response& response)
{
  response.success = true;
  return true;
}

/* --------------------------------- service client ---------------------------------------- */
bool LocalizationNode::SetInitialPoseInFilter(const ::geometry_msgs::PoseWithCovarianceStamped& pose_cov_msg)
{
  ROS_INFO_NAMED("LocalizationNode", "Seting initial pose in filter");
  ros::ServiceClient set_pose_client =
    node_handle_.serviceClient<dcm_srvs::SetPose>("/robot_localization/set_pose");
  if (!set_pose_client.exists()) {
    if (!set_pose_client.waitForExistence(ros::Duration(3.0))) {
      ROS_ERROR_NAMED("LocalizationNode", "Timed out waiting for SetPose service");
      need_set_pose_in_filter_ = false;
      return false;
    }
  }

  dcm_srvs::SetPose pose_cmd;
  pose_cmd.request.pose = pose_cov_msg;

  if (!set_pose_client.call(pose_cmd)) {
    ROS_ERROR("Failed to call service set_pose");
    return false;
  }

  need_set_pose_in_filter_ = false;
  ROS_WARN_NAMED("LocalizationNode", "SetPose service called");
  return true;
}

/* --------------------------------- handle message ---------------------------------------- */
void LocalizationNode::HandleInitialPoseMessage(
  int trajectory_id, const std::string& sensor_id,
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  if (msg->header.frame_id == "") {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  using default frame_id = map.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (StripSlash(msg->header.frame_id) != node_options_.map_frame) {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             StripSlash(msg->header.frame_id).c_str(),
             node_options_.map_frame.c_str());
    return;
  }

  carto::transform::Rigid3d initial_pose = ToRigid3d(msg->pose.pose);

  if (!localization_updater_bridge_.SetInitialPose(initial_pose)) {
    return;
  }
  auto initial_pose_2d = carto::transform::Project2D(initial_pose);
  ROS_WARN("Set initial pose: %f, %f, %f",
            initial_pose_2d.translation().x(),
            initial_pose_2d.translation().y(),
            initial_pose_2d.rotation().angle());

  if (!node_options_.publish_to_tf) {
    need_set_pose_in_filter_ = true;
  }
}

void LocalizationNode::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::Odometry::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = localization_updater_bridge_.sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);

  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void LocalizationNode::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::NavSatFix::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void LocalizationNode::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::LandmarkList::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void LocalizationNode::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::Imu::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = localization_updater_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);

  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void LocalizationNode::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}


void LocalizationNode::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void LocalizationNode::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      localization_updater_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}


void LocalizationNode::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  localization_updater_bridge_.LoadState(state_filename, load_frozen_state);
}

void LocalizationNode::LoadMapData(const std::string& map_filename)
{
  absl::MutexLock lock(&mutex_);
  localization_updater_bridge_.LoadMapData(map_filename);
}

void LocalizationNode::MaybeWarnAboutTopicMismatch(
    const ::ros::WallTimerEvent& unused_timer_event) {
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
  if (print_topics) {
    LOG(WARNING) << "Currently available topics are: "
                 << published_topics_string.str();
  }
}

} // namespace localization

} // namespace cartographer_ros
