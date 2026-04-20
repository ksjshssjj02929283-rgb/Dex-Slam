#include <chrono>
#include <string>
#include <vector>
#include <sstream>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2/utils.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

// ROS2 messages  
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// Services/messages for cartographer_ros_msgs (ROS2)  
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/trajectory_query.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/msg/landmark_list.hpp"

// 自定义消息/服务
#include "std_srvs/srv/trigger.hpp"
#include "nav_interfaces/srv/set_pose.hpp"
#include "nav_interfaces/msg/localization_state.hpp"

// Cartographer  
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

// ROS2 版 cartographer_ros 工具/桥  
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/mapping/sensor_bridge.h"
#include "cartographer_ros/mapping/tf_bridge.h"
#include "cartographer_ros/conversion/time_conversion.h"

#include "cartographer_ros/localization/localization_node.h"

#include "cartographer/utility/include/file_manager.hpp"
#include "cartographer/utility/include/pose_save_tool.hpp"

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
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (LocalizationNode::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, LocalizationNode* const node,
    const rclcpp::QoS& qos = rclcpp::SystemDefaultsQoS()) {

  return node_handle->create_subscription<MessageType>(
      topic, qos,
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
          });
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


// -------------------- 构造与初始化 --------------------  
LocalizationNode::LocalizationNode(
    const LocalizationNodeOptions& node_options,
    std::unique_ptr<cartographer::localization::LocalizationUpdater> localization_updater,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node)  
  : node_options_(node_options),
    node_(node),
    localization_updater_bridge_(node_options_, std::move(localization_updater), tf_buffer.get())
{
  absl::MutexLock lock(&localization_updater_mutex_);

  // TF 广播器  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  // QoS  
  auto qos_latched = rclcpp::QoS(rclcpp::KeepLast(kLatestOnlyPublisherQueueSize)).transient_local().reliable();

  // 发布者  
  submap_list_publisher_ = node_->create_publisher<cartographer_ros_msgs::msg::SubmapList>(
      kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kConstraintListTopic, kLatestOnlyPublisherQueueSize);

  // 发布跟踪的位姿
  if (node_options_.publish_tracked_pose) {
    loc_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        kLocaliztionPoseTopic, rclcpp::SystemDefaultsQoS());
    tracked_path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
        kTrackedPathTopic, kLatestOnlyPublisherQueueSize);
  }

  global_map_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      kGlobalPointCloudTopic, qos_latched);
  global_feature_map_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kGlobalFeatureMapTopic, kLatestOnlyPublisherQueueSize);
  local_map_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      kLocalPointCloudTopic, kLatestOnlyPublisherQueueSize);
  local_feature_map_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kLocalFeatureMapTopic, kLatestOnlyPublisherQueueSize);
  matched_feature_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      kMatchedFeatureTopic, kLatestOnlyPublisherQueueSize);
  scan_matched_point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  localization_state_publisher_ = node_->create_publisher<nav_interfaces::msg::LocalizationState>(
      kLocalizationStateTopic, kLatestOnlyPublisherQueueSize);

  // 服务  
  submap_query_service_ = node_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName,
      std::bind(&LocalizationNode::HandleSubmapQuery, this, 
                std::placeholders::_1, std::placeholders::_2));

  trajectory_query_service_ = node_->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
      kTrajectoryQueryServiceName,
      std::bind(&LocalizationNode::HandleTrajectoryQuery, this,
                std::placeholders::_1, std::placeholders::_2));

  start_trajectory_service_ = node_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName,
      std::bind(&LocalizationNode::HandleStartTrajectory, this,
                std::placeholders::_1, std::placeholders::_2));

  finish_trajectory_service_ = node_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName,
      std::bind(&LocalizationNode::HandleFinishTrajectory, this,
                std::placeholders::_1, std::placeholders::_2));

  write_state_service_ = node_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName,
      std::bind(&LocalizationNode::HandleWriteState, this,
                std::placeholders::_1, std::placeholders::_2));

  get_trajectory_states_service_ = node_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      kGetTrajectoryStatesServiceName,
      std::bind(&LocalizationNode::HandleGetTrajectoryStates, this,
                std::placeholders::_1, std::placeholders::_2));

  relocalization_service_ = node_->create_service<nav_interfaces::srv::SetPose>(
      kHandleRelocalizationServiceName,
      std::bind(&LocalizationNode::HandleRelocalization, this,
                std::placeholders::_1, std::placeholders::_2));

  set_map_changed_service_ = node_->create_service<std_srvs::srv::Trigger>(
      kHandleSetMapChangedServiceName,
      std::bind(&LocalizationNode::HandleSetMapChanged, this,
                std::placeholders::_1, std::placeholders::_2));

  // 定时器（Wall timers 与 regular timer）
  using namespace std::chrono_literals;
  publish_submap_list_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(node_options_.submap_publish_period_sec),
      std::bind(&LocalizationNode::PublishSubmapList, this));

  if (node_options_.pose_publish_period_sec > 0.) {
    std::cout << "Creating pose publish timer with period: "
              << node_options_.pose_publish_period_sec << " seconds." << std::endl;
    publish_local_trajectory_data_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(node_options_.pose_publish_period_sec),
        std::bind(&LocalizationNode::PublishLocalTrajectoryData, this));
  }

  publish_trajectory_node_list_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(node_options_.trajectory_publish_period_sec),
      std::bind(&LocalizationNode::PublishTrajectoryNodeList, this));

  publish_landmark_poses_list_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(node_options_.trajectory_publish_period_sec),
      std::bind(&LocalizationNode::PublishLandmarkPosesList, this));

  publish_constraint_list_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(kConstraintPublishPeriodSec),
      std::bind(&LocalizationNode::PublishConstraintList, this));

  maybe_warn_about_topic_mismatch_timer_ = node_->create_wall_timer(
      100ms, std::bind(&LocalizationNode::PublishLocalizationState, this));

  // TODO: linfeng.guo 按需保存跟踪位姿
  // if (node_options_.save_tracked_pose) {
  //   if (!boost::filesystem::is_directory(node_options_.tracked_pose_directory)) {
  //     cartographer::utility::FileManager::InitDirectory(
  //         node_options_.tracked_pose_directory, "tracked_pose of cartographer");
  //   }
  //   std::string file_path =
  //       node_options_.tracked_pose_directory + node_options_.tracked_pose_filename;
  //   cartographer::utility::FileManager::RemoveFile(file_path);
  //   carto_pose_tool_ = std::make_shared<cartographer::utility::PoseSaveTool>(
  //       file_path, node_options_.tracked_pose_format);
  // }
}

// 这里后续放service

// 放pub

// 这里是启动先关

void LocalizationNode::HandleSubmapQuery(
    const std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::SubmapQuery::Response> response)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  
  // 调用bridge处理请求
  localization_updater_bridge_.HandleSubmapQuery(*request, *response);
  RCLCPP_DEBUG(node_->get_logger(), "Handled submap query for submap_index: %d, trajectory_id: %d",
               request->submap_index, request->trajectory_id);
}

// TODO: 确定是发布轨迹的逻辑  
void LocalizationNode::HandleTrajectoryQuery(
    const std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::TrajectoryQuery::Response> response)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED, TrajectoryState::FROZEN});
  
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    RCLCPP_ERROR(node_->get_logger(), "Can't query trajectory from pose graph: %s",
                 response->status.message.c_str());
    return;
  }
  localization_updater_bridge_.HandleTrajectoryQuery(*request, *response);
  RCLCPP_DEBUG(node_->get_logger(), "Handled trajectory query for trajectory_id: %d",
               request->trajectory_id);
}

void LocalizationNode::AddSensorSamplers(const int trajectory_id,
                                         const TrajectoryOptions& options)
{
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  // 根据不同传感器的采样率，创建采样器
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void LocalizationNode::PublishLocalizationState()
{
  const auto loc_state = localization_updater_bridge_.GetLocalizationState();
  nav_interfaces::msg::LocalizationState loc_state_msg;

  if (!loc_state.has_value()) {
    // 未初始化或不可用时可选择填充一个默认状态
    loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_NOT_INIT;
  } else {
    using S = carto::localization::LocalizationState;
    // 90%的可能性是定位成功的
    if (*loc_state == S::LOCALIZED) {
      loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_SUCCESS;
    } else {
      switch (*loc_state) {
        case S::LOCALIZATION_FAILED:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_FAILED;
          break;
        case S::INITIALIZATION_FAILED:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_INIT_FAILED;
          break;
        case S::NOT_INITIALIZED:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_NOT_INIT;
          break;
        case S::INITIALIZING:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_INITIALIZING;
          break;
        case S::INITIALIZED:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_INITIALIZED;
          break;
        case S::RELOCALIZING:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_RELOCALIZING;
          break;
        case S::LOCALIZED:
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_SUCCESS;
          break;
        default:
          // 兜底处理，防止未来新增状态导致未设置
          loc_state_msg.status = nav_interfaces::msg::LocalizationState::LOC_FAILED;
          RCLCPP_WARN(node_->get_logger(),
                      "Unknown localization state value: %d",
                      static_cast<int>(*loc_state));
          break;
      }
    }
  }

  // 检测定位失败状态并调用 stop filter + 触发全局重定位
  if (loc_state_msg.status == nav_interfaces::msg::LocalizationState::LOC_FAILED || 
      loc_state_msg.status == nav_interfaces::msg::LocalizationState::LOC_INIT_FAILED) {
    
    // 只在状态改变时调用，避免重复调用
    if (last_localization_status_ != loc_state_msg.status && !filter_stopped_) {
      RCLCPP_ERROR(node_->get_logger(), 
                   "Localization failed with status %d, stopping filter and triggering global relocalization",
                   loc_state_msg.status);
      StopFilter();
      localization_updater_bridge_.TriggerGlobalRelocalization();
    }
  } else {
    // 定位恢复成功，重新启动 filter
    if (filter_stopped_ && 
        (loc_state_msg.status == nav_interfaces::msg::LocalizationState::LOC_SUCCESS ||
         loc_state_msg.status == nav_interfaces::msg::LocalizationState::LOC_INITIALIZED)) {
      RCLCPP_INFO(node_->get_logger(), 
                  "Localization status recovered to %d, restarting filter",
                  loc_state_msg.status);
      if (StartFilter()) {
        need_set_pose_in_filter_ = true;
      }
      filter_stopped_ = false;
    }
  }
  
  last_localization_status_ = loc_state_msg.status;
  
  localization_state_publisher_->publish(loc_state_msg);
}


void LocalizationNode::PublishLocalTrajectoryData()
{
  absl::MutexLock lock(&localization_updater_mutex_);

      // 首次发布全局地图及全局特征  
    static bool first_publish_map = true;
    if (first_publish_map) {
      std::cout << "First publish global map and feature map." << std::endl;
      global_map_cloud_publisher_->publish(localization_updater_bridge_.GetCloudMap());

      visualization_msgs::msg::MarkerArray feature_map_msg;
      int marker_id = 0;
      const auto line_map_msg = localization_updater_bridge_.GetLineMap(marker_id);
      feature_map_msg.markers.insert(feature_map_msg.markers.end(),
                                     line_map_msg.begin(), line_map_msg.end());
      global_feature_map_publisher_->publish(feature_map_msg);

      first_publish_map = false;
    }

  // 获得不同轨迹的数据，并进行插值
  for (const auto& entry : localization_updater_bridge_.GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) {
      carto::sensor::TimedPointCloud point_cloud;
      point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                              .returns.size());
      for (const cartographer::sensor::RangefinderPoint & point :
            trajectory_data.local_slam_data->range_data_in_local.returns) {
        point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
            point, 0.f /* time */));
      }
      scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_data.local_slam_data->time),
          node_options_.map_frame,
          carto::sensor::TransformTimedPointCloud(
              point_cloud, carto::transform::Rigid3f::Identity())));
    }

    // Matched features (line segments)  
    if (matched_feature_publisher_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray local_line_msg;
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
      matched_feature_publisher_->publish(local_line_msg);
    }

    // Local feature map  
    if (local_feature_map_publisher_->get_subscription_count() > 0) {
      visualization_msgs::msg::MarkerArray map_line_msg;
      const auto& line_map_2d_list =
          trajectory_data.local_slam_data->line_map_2d_list->GetLineMapData2DList();
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
      local_feature_map_publisher_->publish(map_line_msg);
    }

    // Local map cloud (from local_map points)  
    if (local_map_cloud_publisher_->get_subscription_count() > 0) {
      carto::sensor::TimedPointCloud local_map_cloud;
      for (const auto& point : trajectory_data.local_slam_data->local_map->points()) {
        local_map_cloud.push_back(carto::sensor::ToTimedRangefinderPoint(
            point, 0.f /* time */));
      }
      local_map_cloud_publisher_->publish(ToPointCloud2Message(
          carto::common::ToUniversal(trajectory_data.local_slam_data->time),
          node_options_.map_frame,
          carto::sensor::TransformTimedPointCloud(
              local_map_cloud, carto::transform::Rigid3f::Identity())));
    }



    // TF 与追踪位姿
    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = FromRos(node_->get_clock()->now());
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
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;
          // odom to map
          geometry_msgs::msg::TransformStamped tf_map_odom;
          tf_map_odom.header.stamp = stamped_transform.header.stamp;
          tf_map_odom.header.frame_id = node_options_.map_frame;
          tf_map_odom.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          tf_map_odom.transform =
              ToGeometryMsgTransform(carto::transform::Rigid3d::Identity());
          stamped_transforms.push_back(tf_map_odom);

          // published_frame to odom
          geometry_msgs::msg::TransformStamped tf_odom_pub;
          tf_odom_pub.header.stamp = stamped_transform.header.stamp;
          tf_odom_pub.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          tf_odom_pub.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          // T_odom_published = T_odom_base * T_base_published
          tf_odom_pub.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(tf_odom_pub);

          tf_broadcaster_->sendTransform(stamped_transforms);
        } else {
          // published_frame to map
          geometry_msgs::msg::TransformStamped tf_map_pub;
          tf_map_pub.header.stamp = stamped_transform.header.stamp;
          tf_map_pub.header.frame_id = node_options_.map_frame;
          tf_map_pub.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          tf_map_pub.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_->sendTransform(tf_map_pub);
        }
      }

      // 发布跟踪位姿（PoseWithCovarianceStamped）
      if (node_options_.publish_tracked_pose && loc_pose_publisher_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header.frame_id = node_options_.map_frame;
        pose_cov_msg.header.stamp = stamped_transform.header.stamp;
        pose_cov_msg.pose.pose = ToGeometryMsgPose(tracking_to_map);

        // 简单的协方差示例
        pose_cov_msg.pose.covariance[6 * 0 + 0] = 0.05 * 0.05;
        pose_cov_msg.pose.covariance[6 * 1 + 1] = 0.05 * 0.05;
        pose_cov_msg.pose.covariance[6 * 5 + 5] = 0.05 * 0.05;

        loc_pose_publisher_->publish(pose_cov_msg);

        // 如果需要将初始位姿推送给外部滤波器（如 robot_localization）
        if (need_set_pose_in_filter_) {
          SetInitialPoseInFilter(pose_cov_msg);
        }
        // ::geometry_msgs::PoseStamped pose_stamped_msg;
        // pose_stamped_msg.header = pose_cov_msg.header;
        // pose_stamped_msg.pose = pose_cov_msg.pose.pose;
        // tracked_path_msg_.header.stamp = stamped_transform.header.stamp;
        // tracked_path_msg_.header.frame_id = node_options_.map_frame;
        // tracked_path_msg_.poses.push_back(pose_stamped_msg);
        // tracked_path_publisher_->publish(tracked_path_msg_);
      }

      // 保存位姿（可选）
      if (node_options_.save_tracked_pose &&
          trajectory_data.trajectory_options.trajectory_builder_options.has_pure_localization_trimmer())
      {
        Eigen::Vector3d t = tracking_to_map.translation();
        Eigen::Quaterniond q = tracking_to_map.rotation();
        const double time_sec = rclcpp::Time(stamped_transform.header.stamp).seconds();
        if (!carto_pose_tool_) {
          LOG(INFO) << "carto_pose_tool_ is nullptr";
        } else {
          carto_pose_tool_->AddPoseData(t, q, time_sec);
        }
      }
    }
  }
}

bool LocalizationNode::SaveMap(
  const std::string & map_filename, const double resolution)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  return localization_updater_bridge_.SaveMap(
      map_filename,
      static_cast<float>(node_options_.mapping_occ_thresh),
      static_cast<float>(node_options_.mapping_free_thresh));
}


void LocalizationNode::PublishSubmapList()
{
  absl::MutexLock lock(&localization_updater_mutex_);
  if (submap_list_publisher_) {
    submap_list_publisher_->publish(localization_updater_bridge_.GetSubmapList());
  }
}

void LocalizationNode::PublishTrajectoryNodeList()
{
  if (trajectory_node_list_publisher_ &&
      trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&localization_updater_mutex_);
    trajectory_node_list_publisher_->publish(
        localization_updater_bridge_.GetTrajectoryNodeList());
  }
}

void LocalizationNode::PublishLandmarkPosesList()
{
  if (landmark_poses_list_publisher_ &&
      landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&localization_updater_mutex_);
    landmark_poses_list_publisher_->publish(
        localization_updater_bridge_.GetLandmarkPosesList());
  }
}

void LocalizationNode::PublishConstraintList()
{
  if (constraint_list_publisher_ &&
      constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&localization_updater_mutex_);
    constraint_list_publisher_->publish(localization_updater_bridge_.GetConstraintList());
  }
}


std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
LocalizationNode::ComputeExpectedSensorIds(const TrajectoryOptions& options) const
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
  if (options.trajectory_builder_options.
                trajectory_builder_2d_options().use_imu_data()) {
    RCLCPP_WARN(node_->get_logger(), "Subscribe to IMU topic.");
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

int LocalizationNode::AddTrajectory(const TrajectoryOptions& options)
{
  // 1. 根据配置，检查具有的传感器以及期望订阅的topic
  const auto expected_sensor_ids = ComputeExpectedSensorIds(options);
  // 2. 根据配置，向localization_updater中增加轨迹优化器
  const int trajectory_id =
      localization_updater_bridge_.AddTrajectory(expected_sensor_ids, options);
  // 4. 增加传感器采样器(用于计算和控制传感器采样输入)
  AddSensorSamplers(trajectory_id, options);
  // 5. 订阅传感器数据
  LaunchSubscribers(options, trajectory_id);
  // 6. 每kTopicMismatchCheckDelaySec秒检查一次topic是否匹配
  // 一次性延迟检查 Topic 匹配
  using namespace std::chrono_literals;
  node_->create_wall_timer(
      std::chrono::duration<double>(kTopicMismatchCheckDelaySec),
      std::bind(&LocalizationNode::MaybeWarnAboutTopicMismatch, this));
  // 7. 订阅的topic以及与之对应的传感器id
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void LocalizationNode::LaunchSubscribers(const TrajectoryOptions& options,  
                                         const int trajectory_id) {  
  // 使用传入的node_成员变量  
  auto node_handle = node_;  

  // 🔧 Range sensors (LaserScan & PointCloud2): BEST_EFFORT QoS
  // 激光扫描和点云频率较低，使用 BEST_EFFORT 减少延迟
  auto range_sensor_qos = rclcpp::QoS(rclcpp::KeepLast(50))
                              .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                              .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  
  // 🔧 High-frequency sensors (IMU & Odometry): RELIABLE QoS with large buffer
  // IMU 和 Odometry 频率高，需要大缓冲和可靠传输
  // 启动时需要足够的历史数据用于点云去畸变
  auto high_freq_sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000))
                                  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // LaserScan
  for (const std::string& topic :  
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {  
    auto sub = SubscribeWithHandler<sensor_msgs::msg::LaserScan>(  
        &LocalizationNode::HandleLaserScanMessage,  
        trajectory_id,  
        topic,  
        node_handle,  
        this,
        range_sensor_qos);  
    subscribers_[trajectory_id].push_back({sub, topic});
  }  

  // PointCloud2
  for (const std::string& topic :  
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {  
    auto sub = SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(  
        &LocalizationNode::HandlePointCloud2Message,  
        trajectory_id,  
        topic,  
        node_handle,  
        this,
        range_sensor_qos);  
    subscribers_[trajectory_id].push_back({sub, topic});
  }  

  // IMU
  if (options.trajectory_builder_options.trajectory_builder_2d_options().use_imu_data()) {
    auto sub = SubscribeWithHandler<sensor_msgs::msg::Imu>(  
        &LocalizationNode::HandleImuMessage,  
        trajectory_id,  
        kImuTopic,  
        node_handle,  
        this,
        high_freq_sensor_qos);  
    subscribers_[trajectory_id].push_back({sub, kImuTopic});
  }  

  // Odometry
  if (options.use_odometry) {
    auto sub = SubscribeWithHandler<nav_msgs::msg::Odometry>(  
        &LocalizationNode::HandleOdometryMessage,  
        trajectory_id,  
        kOdometryTopic,  
        node_handle,  
        this,
        high_freq_sensor_qos);  
    subscribers_[trajectory_id].push_back({sub, kOdometryTopic});
  }  

  // NavSatFix  
  if (options.use_nav_sat) {  
    auto sub = SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(  
        &LocalizationNode::HandleNavSatFixMessage,  
        trajectory_id,  
        kNavSatFixTopic,  
        node_handle,  
        this);  
    subscribers_[trajectory_id].push_back({sub, kNavSatFixTopic});  
  }  

  // Landmarks  
  if (options.use_landmarks) {  
    auto sub = SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(  
        &LocalizationNode::HandleLandmarkMessage,  
        trajectory_id,  
        kLandmarkTopic,  
        node_handle,  
        this);  
    subscribers_[trajectory_id].push_back({sub, kLandmarkTopic});  
  }  

  // Initial pose  
  {  
    auto sub = SubscribeWithHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(  
        &LocalizationNode::HandleInitialPoseMessage,  
        trajectory_id,  
        kSetInitialPoseTopic,  
        node_handle,  
        this);  
    subscribers_[trajectory_id].push_back({sub, kSetInitialPoseTopic});  
  }  
}  

bool LocalizationNode::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  // TODO: linfeng.guo 补充更多的检查
  return true;
}

cartographer_ros_msgs::msg::StatusResponse LocalizationNode::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states)
{
  const auto trajectory_states = localization_updater_bridge_.GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message =
        absl::StrCat("Trajectory ", trajectory_id, " doesn't exist.");
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message =
      absl::StrCat("Trajectory ", trajectory_id, " is in '",
                   TrajectoryStateToString(it->second), "' state.");
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
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

cartographer_ros_msgs::msg::StatusResponse
LocalizationNode::FinishTrajectoryUnderLock(const int trajectory_id)
{
  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.sub.reset();  // SharedPtr 置空，解除订阅
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }

  localization_updater_bridge_.FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);

  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return status_response;
}

void LocalizationNode::HandleStartTrajectory(
    const std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory::Response> response)
{
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadMappingOptions(
      request->configuration_directory, request->configuration_basename);

  if (request->use_initial_pose) {
    const auto pose = ToRigid3d(request->initial_pose);
    if (!pose.IsValid()) {
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      RCLCPP_ERROR(node_->get_logger(), "%s", response->status.message.c_str());
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Can't start a trajectory with initial pose: %s",
                   response->status.message.c_str());
      return;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request->relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(rclcpp::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response->status.message = "Invalid trajectory options.";
    RCLCPP_ERROR(node_->get_logger(), "%s", response->status.message.c_str());
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response->status.message = "Topics are already used by another trajectory.";
    RCLCPP_ERROR(node_->get_logger(), "%s", response->status.message.c_str());
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else {
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
}

void LocalizationNode::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options)
{
  // 检查是否有是否有轨迹优化器参数
  CHECK(ValidateTrajectoryOptions(options));
  // 创建轨迹优化器（内部使用对应的锁）
  AddTrajectory(options);
}

void LocalizationNode::FinishAllTrajectories() {
  absl::MutexLock lock(&localization_updater_mutex_);
  for (const auto& entry : localization_updater_bridge_.GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}


// /* --------------------------------- handle service ---------------------------------------- */
void LocalizationNode::HandleGetTrajectoryStates(
    const std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::GetTrajectoryStates::Response> response)
{
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&localization_updater_mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = node_->get_clock()->now();
  for (const auto& entry : localization_updater_bridge_.GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
}

void LocalizationNode::HandleFinishTrajectory(
    const std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::FinishTrajectory::Response> response)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
}

void LocalizationNode::HandleWriteState(
    const std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Request> request,
    std::shared_ptr<cartographer_ros_msgs::srv::WriteState::Response> response)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  if (localization_updater_bridge_.SerializeState(request->filename,
                                                 request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        absl::StrCat("State written to '", request->filename, "'.");
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        absl::StrCat("Failed to write '", request->filename, "'.");
  }
}

void LocalizationNode::HandleRelocalization(
    const std::shared_ptr<nav_interfaces::srv::SetPose::Request> request,
    std::shared_ptr<nav_interfaces::srv::SetPose::Response> response)
{
  if (request->pose.header.frame_id == "") {
    // This should be removed at some point
    RCLCPP_WARN(node_->get_logger(),
                "Received initial pose with empty frame_id. Using default frame_id = map.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if (StripSlash(request->pose.header.frame_id) != node_options_.map_frame) {
    RCLCPP_WARN(node_->get_logger(),
                "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
                StripSlash(request->pose.header.frame_id).c_str(),
                node_options_.map_frame.c_str());
    response->success = false;
    return;
  }

  const carto::transform::Rigid3d initial_pose = ToRigid3d(request->pose.pose.pose);

  need_set_pose_in_filter_ = true;
  auto initial_pose_2d = carto::transform::Project2D(initial_pose);
  RCLCPP_WARN(node_->get_logger(),
              "Set initial pose: %f, %f, %f",
              initial_pose_2d.translation().x(),
              initial_pose_2d.translation().y(),
              initial_pose_2d.rotation().angle());
  response->success = localization_updater_bridge_.SetInitialPose(initial_pose);
}

void LocalizationNode::HandleSetMapChanged(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  response->success = true;
}

/* --------------------------------- service client ---------------------------------------- */
bool LocalizationNode::SetInitialPoseInFilter(
    const geometry_msgs::msg::PoseWithCovarianceStamped& pose_cov_msg)
{
  RCLCPP_INFO(node_->get_logger(), "Setting initial pose in filter");
  // Create service client (should be created in constructor for better practice)
  auto set_pose_client = node_->create_client<nav_interfaces::srv::SetPose>("/robot_localization/set_pose");
  if (!set_pose_client) {
    RCLCPP_ERROR(node_->get_logger(), "SetPose client not initialized");
    return false;
  }
  // Wait for service to be available
  if (!set_pose_client->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for SetPose service");
    need_set_pose_in_filter_ = false;
    return false;
  }

  auto pose_cmd = std::make_shared<nav_interfaces::srv::SetPose::Request>();
  pose_cmd->pose = pose_cov_msg;

  // Call service asynchronously with timeout
  auto future = set_pose_client->async_send_request(pose_cmd);
  
  // Wait for response with timeout
  auto status = future.wait_for(std::chrono::seconds(5));
  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_pose - timeout");
    return false;
  }

  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_pose - no response");
    return false;
  }

  need_set_pose_in_filter_ = false;
  RCLCPP_WARN(node_->get_logger(), "SetPose service called successfully");
  return result->success;
}

bool LocalizationNode::StopFilter()
{
  RCLCPP_INFO(node_->get_logger(), "Stopping robot localization filter");
  
  try {
    // 创建临时节点进行服务调用
    auto temp_node = std::make_shared<rclcpp::Node>(
        "temp_stop_filter_client",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
    
    // 创建临时客户端
    auto temp_client = temp_node->create_client<std_srvs::srv::Trigger>(
        "/robot_localization/stop_filter");
    
    if (!temp_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(node_->get_logger(), "Timed out waiting for stop_filter service");
      return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = temp_client->async_send_request(request);
    
    // 使用临时 executor 来处理响应
    rclcpp::executors::SingleThreadedExecutor temp_executor;
    temp_executor.add_node(temp_node);
    
    auto timeout = std::chrono::seconds(5);
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok() && 
           future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      temp_executor.spin_some();
      
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service stop_filter - timeout");
        return false;
      }
    }
    
    auto result = future.get();
    if (!result) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service stop_filter - no response");
      return false;
    }
    
    if (result->success) {
      RCLCPP_WARN(node_->get_logger(), "StopFilter service called successfully");
      filter_stopped_ = true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "StopFilter service returned failure: %s", 
                   result->message.c_str());
    }
    
    return result->success;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception while calling stop_filter: %s", e.what());
    return false;
  }
}

bool LocalizationNode::StartFilter()
{
  RCLCPP_INFO(node_->get_logger(), "Starting robot localization filter");
  
  try {
    auto temp_node = std::make_shared<rclcpp::Node>(
        "temp_start_filter_client",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(false));
    
    auto temp_client = temp_node->create_client<std_srvs::srv::Trigger>(
        "/robot_localization/start_filter");
    
    if (!temp_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_WARN(node_->get_logger(), "Timed out waiting for start_filter service");
      return false;
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = temp_client->async_send_request(request);
    
    rclcpp::executors::SingleThreadedExecutor temp_executor;
    temp_executor.add_node(temp_node);
    
    auto timeout = std::chrono::seconds(5);
    auto start_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok() && 
           future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      temp_executor.spin_some();
      
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service start_filter - timeout");
        return false;
      }
    }
    
    auto result = future.get();
    if (!result) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to call service start_filter - no response");
      return false;
    }
    
    if (result->success) {
      RCLCPP_INFO(node_->get_logger(), "StartFilter service called successfully");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "StartFilter service returned failure: %s", 
                   result->message.c_str());
    }
    
    return result->success;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception while calling start_filter: %s", e.what());
    return false;
  }
}

// TODO: linfeng.guo 后面把这个client写到构造函数中
// Alternative synchronous version (simpler but blocking)
// bool LocalizationNode::SetInitialPoseInFilterSync(
//     const geometry_msgs::msg::PoseWithCovarianceStamped& pose_cov_msg) {
  
//   RCLCPP_INFO(node_->get_logger(), "Setting initial pose in filter");
  
//   if (!set_pose_client_) {
//     RCLCPP_ERROR(node_->get_logger(), "SetPose client not initialized");
//     return false;
//   }
  
//   if (!set_pose_client_->wait_for_service(std::chrono::seconds(3))) {
//     RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for SetPose service");
//     need_set_pose_in_filter_ = false;
//     return false;
//   }

//   auto pose_cmd = std::make_shared<dcm_srvs::srv::SetPose::Request>();
//   pose_cmd->pose = pose_cov_msg;

//   try {
//     auto result = set_pose_client_->async_send_request(pose_cmd).get();
//     need_set_pose_in_filter_ = false;
//     RCLCPP_WARN(node_->get_logger(), "SetPose service called successfully");
//     return result->success;
//   } catch (const std::exception& e) {
//     RCLCPP_ERROR(node_->get_logger(), "Failed to call service set_pose: %s", e.what());
//     return false;
//   }
// }

// // Constructor addition - initialize service client (recommended practice)
// // Add this to your LocalizationNode constructor:
// /*
// set_pose_client_ = this->create_client<dcm_srvs::srv::SetPose>("/robot_localization/set_pose");
// */

// // Header file addition - add this to your class definition:
// /*
// private:
//   rclcpp::Client<dcm_srvs::srv::SetPose>::SharedPtr set_pose_client_;
// */

/* --------------------------------- handle message ---------------------------------------- */
void LocalizationNode::HandleInitialPoseMessage(
  int trajectory_id, const std::string& sensor_id,
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  if (msg->header.frame_id == "") {
    RCLCPP_WARN(node_->get_logger(),
                "Received initial pose with empty frame_id. Using default: map.");
  } else if (StripSlash(msg->header.frame_id) != node_options_.map_frame) {
    RCLCPP_WARN(node_->get_logger(),
                "Ignoring initial pose in frame '%s'; must be in global frame '%s'",
                StripSlash(msg->header.frame_id).c_str(), node_options_.map_frame.c_str());
    return;
  }

  carto::transform::Rigid3d initial_pose = ToRigid3d(msg->pose.pose);

  if (!localization_updater_bridge_.SetInitialPose(initial_pose)) {
    return;
  }
  auto initial_pose_2d = carto::transform::Project2D(initial_pose);
  RCLCPP_WARN(node_->get_logger(),
              "Set initial pose: %.3f, %.3f, %.3f",
              initial_pose_2d.translation().x(),
              initial_pose_2d.translation().y(),
              initial_pose_2d.rotation().angle());

  if (!node_options_.publish_to_tf) {
    need_set_pose_in_filter_ = true;
  }
}


void LocalizationNode::HandleOdometryMessage(
  const int trajectory_id, const std::string& sensor_id,
  const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
      return;
    }
  }
  auto sensor_bridge_ptr = localization_updater_bridge_.sensor_bridge(trajectory_id);
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}


void LocalizationNode::HandleNavSatFixMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
      return;
    }
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}


void LocalizationNode::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
      return;
    }
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}


void LocalizationNode::HandleImuMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
      return;
    }
  }
  auto sensor_bridge_ptr = localization_updater_bridge_.sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  // if (imu_data_ptr != nullptr &&
  //     !sensor_bridge_ptr->IgnoreMessage(sensor_id, imu_data_ptr->time)) {
  //   extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  // }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}


void LocalizationNode::HandleLaserScanMessage(
  const int trajectory_id, const std::string& sensor_id,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
      return;
    }
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
    ->HandleLaserScanMessage(sensor_id, msg);
}


void LocalizationNode::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  {
    absl::MutexLock lock(&sensor_samplers_mutex_);
    if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
      return;
    }
  }
  localization_updater_bridge_.sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void LocalizationNode::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  CHECK(
      localization_updater_bridge_.SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}


void LocalizationNode::LoadState(const std::string& state_filename,
                                 const bool load_frozen_state)
{
  absl::MutexLock lock(&localization_updater_mutex_);
  localization_updater_bridge_.LoadState(state_filename, load_frozen_state);
}


void LocalizationNode::MaybeWarnAboutTopicMismatch()
{
  // 获取当前可用 topic 名称及类型
  const auto topics_and_types = node_->get_topic_names_and_types();
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& kv : topics_and_types) {
    published_topics.insert(kv.first);
    published_topics_string << kv.first << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    const int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      // ROS2 中名称已是全名
      const std::string& resolved_topic = subscriber.topic;
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
