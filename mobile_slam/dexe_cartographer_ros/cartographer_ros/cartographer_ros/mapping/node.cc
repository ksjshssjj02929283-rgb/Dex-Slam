/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/mapping/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
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
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
//#include "ros/serialization.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <yaml-cpp/yaml.h>

namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node,
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

}  // namespace

/**
 * @brief Construct a new Node:: Node object
 * 声明ROS topic发布器，注册服务，绑定定时器与topic发布器
 * @param[in] node_options 
 * @param[in] map_builder 
 * @param[in] tf_buffer 
 * @param[in] node 
 * @param[in] collect_metrics 
 */
Node::Node(
    const MappingNodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Node::SharedPtr node,
    const bool collect_metrics)
    : node_options_(node_options)
{
  node_ = node;
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_) ;
  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer.get()));

  absl::MutexLock lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }
  // // added by hz
  // // step: 0 注册初始位姿订阅器
  // if (node_options_.sub_initial_pose) {
  //   init_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //       "/initialpose",
  //       1, 
  //       [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  //       {
  //         HandleInitPoseMsg(msg);
  //       });
  // }

  // if (node_options_.sub_apriltag_pose) {
  //   apriltag_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
  //       "/apriltag_poses",
  //       1, 
  //       [this](const geometry_msgs::msg::PoseArray::SharedPtr msg)
  //       {
  //         HandleApriltagPoseMsg(msg);
  //       });
  // }
  // // added by hz end

  // Step: 1 注册发布器
  submap_list_publisher_ =
      node_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          std::string("~/") + kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          std::string("~/") + kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          std::string("~/") + kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          std::string("~/") + kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  // 发布跟踪的位姿
  if (node_options_.publish_tracked_pose) { // node_options里面默认是false，如果想要改成true，需要在lua里设置
    tracked_pose_publisher_ =
        node_->create_publisher<::geometry_msgs::msg::PoseStamped>(
            std::string("~/") + kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  // tracked_path_publisher_ =
  //     node_->create_publisher<::visualization_msgs::msg::MarkerArray>(
  //         kTrackedPathTopic, kLatestOnlyPublisherQueueSize);

  // // new added by hz: 
  // if (node_options_.publish_local_pose) {
  //   local_pose_publisher_ =
  //       node_->create_publisher<::geometry_msgs::msg::PoseStamped>(
  //           node_options_.localPoseTopic, kLatestOnlyPublisherQueueSize);
  // }

  scan_matched_point_cloud_publisher_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        std::string("~/") + kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  local_map_cloud_publisher_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          std::string("~/") + kLocalPointCloudTopic, kLatestOnlyPublisherQueueSize);
  matched_feature_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          std::string("~/") + kMatchedFeatureTopic, kLatestOnlyPublisherQueueSize);
  local_feature_map_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          std::string("~/") + kLocalFeatureMapTopic, kLatestOnlyPublisherQueueSize);

  // Step: 2 注册服务
  submap_query_server_ = node_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      std::string("~/") + kSubmapQueryServiceName,
      std::bind(
          &Node::HandleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2));
  trajectory_query_server = node_->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
      std::string("~/") + kTrajectoryQueryServiceName,
      std::bind(
          &Node::HandleTrajectoryQuery, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      std::string("~/") + kStartTrajectoryServiceName,
      std::bind(
          &Node::HandleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  finish_trajectory_server_ = node_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      std::string("~/") + kFinishTrajectoryServiceName,
      std::bind(
          &Node::HandleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  write_state_server_ = node_->create_service<cartographer_ros_msgs::srv::WriteState>(
      std::string("~/") + kWriteStateServiceName,
      std::bind(
          &Node::HandleWriteState, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_states_server_ = node_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      std::string("~/") + kGetTrajectoryStatesServiceName,
      std::bind(
          &Node::HandleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2));
  read_metrics_server_ = node_->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
      std::string("~/") + kReadMetricsServiceName,
      std::bind(
          &Node::HandleReadMetrics, this, std::placeholders::_1, std::placeholders::_2));

  //Step: 3 绑定定时器和topic的发布器
  submap_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    [this]() {
      PublishSubmapList();
    });
  if (node_options_.pose_publish_period_sec > 0) {
    local_trajectory_data_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
      [this]() {
        PublishLocalTrajectoryData();
      });
  }
  trajectory_node_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryNodeList();
    });
  landmark_pose_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishLandmarkPosesList();
    });
  constrain_list_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
    [this]() {
      PublishConstraintList();
    });
  // 增加定位位姿的保存 by linfeng.guo
  if(node_options_.save_tracked_pose) {
    if (!boost::filesystem::is_directory(node_options_.tracked_pose_directory)) {
      cartographer::utility::FileManager::InitDirectory(node_options_.tracked_pose_directory, "tracked_pose of cartographer");
    }
    std::string file_path = node_options_.tracked_pose_directory + node_options_.tracked_pose_filename;
    cartographer::utility::FileManager::RemoveFile(file_path);
    carto_pose_tool_ = std::make_shared<cartographer::utility::PoseSaveTool>(file_path, node_options_.tracked_pose_format);
  }
}

Node::~Node() { FinishAllTrajectories(); }

// TODO: linfeng.guo: 根据需要，判断这些是否需要删除掉
// // added by hz
// // step: 0 注册初始位姿订阅器
// void Node::HandleInitPoseMsg(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
// {
//     auto pose = ToRigid3d(msg->pose.pose);
//     if (!pose.IsValid()) 
//     {
//       std::cout << "Invalid estimated pose." << std::endl;
//       return;
//     }
//     else
//     {
//       init_pose = pose;
//       flag_init_pose_received = true;
//       std::cout << "Init pose is set." << std::endl;
//       std::cout << "Init pose x : " << msg->pose.pose.position.x << std::endl;
//       std::cout << "Init pose y : " << msg->pose.pose.position.y << std::endl;
//       std::cout << "Init pose z : " << msg->pose.pose.position.z << std::endl;
//       std::cout << "Init pose qx : " << msg->pose.pose.orientation.x << std::endl;
//       std::cout << "Init pose qy : " << msg->pose.pose.orientation.y << std::endl;
//       std::cout << "Init pose qz : " << msg->pose.pose.orientation.z << std::endl;
//       std::cout << "Init pose qw : " << msg->pose.pose.orientation.w << std::endl;
//       // tag:增加手动设置初始位姿，需要循环等待设置完初始位姿之后再新建路径
//       // sub_initial_pose默认是false
//       if (node_options_.sub_initial_pose)
//       {
//         if (flag_init_pose_received) {
//           StartLocalizationWithInitialPose(init_pose);
//           CHECK(ValidateTrajectoryOptions(trajectory_options));
//           AddTrajectory(trajectory_options);

//           flag_finish_initial_localization = true;
//         }
//       }
//     }
// }
// // 注册apriltag位姿订阅器
// void Node::HandleApriltagPoseMsg(const geometry_msgs::msg::PoseArray::SharedPtr msg)
// {
//   if (!flag_finish_initial_localization)
//   {
//     // 如果没检测到apriltag，则返回
//     if (!msg->poses.size())
//     {
//       std::cout << "Apriltag is not detected." << std::endl;
//       miss_apriltag_detection_count++;
//       // 如果连续10次都没有检测到apriltag，则使用记录的初始位姿进行定位
//       if (miss_apriltag_detection_count > 10 && flag_recorded_initial_pose_received)
//       {
//         std::cout << "Miss apriltag detection for 10 times, start localization with recorded initial pose." << std::endl;
//         StartLocalizationWithInitialPose(recorded_initial_pose);
//         CHECK(ValidateTrajectoryOptions(trajectory_options));
//         AddTrajectory(trajectory_options);

//         flag_finish_initial_localization = true;
//         miss_apriltag_detection_count = 0;
//         flag_recorded_initial_pose_received = false;
//       }
//       return;
//     }
//     auto pose = ToRigid3d(msg->poses[0]);
//     // 如果pose无效，则返回
//     if (!pose.IsValid()) 
//     {
//       std::cout << "Invalid estimated pose." << std::endl;
//       return;
//     }
//     else
//     {
//       miss_apriltag_detection_count = 0;
//       if (!flag_apriltag_pose_received)
//       {
//         apriltag_pose = pose;
//         flag_apriltag_pose_received = true;
//         std::cout << "Apriltag pose is set." << std::endl;
//         std::cout << "Apriltag pose x : " << msg->poses[0].position.x << std::endl;
//         std::cout << "Apriltag pose y : " << msg->poses[0].position.y << std::endl;
//         std::cout << "Apriltag pose z : " << msg->poses[0].position.z << std::endl;
//         std::cout << "Apriltag pose qx : " << msg->poses[0].orientation.x << std::endl;
//         std::cout << "Apriltag pose qy : " << msg->poses[0].orientation.y << std::endl;
//         std::cout << "Apriltag pose qz : " << msg->poses[0].orientation.z << std::endl;
//         std::cout << "Apriltag pose qw : " << msg->poses[0].orientation.w << "\n" << std::endl;
//       }
//       else
//       {
//         std::cout << "Apriltag pose has been updated." << std::endl;
//       }
//       // 
//       if (flag_recorded_initial_pose_received) 
//       {
//         auto tmp_init_pose = recorded_initial_pose * recorded_apriltag_pose * apriltag_pose.inverse();
//         // TODO: 这里需要只取x,y,yaw
//         std::cout << "Current initial pose using apriltag pose: " << std::endl;
//         std::cout << "Position: (" << tmp_init_pose.translation().x() << ", "
//                   << tmp_init_pose.translation().y() << ", "
//                   << tmp_init_pose.translation().z() << ")" << std::endl;
//         std::cout << "Orientation: (" << tmp_init_pose.rotation().x() << ", "
//                   << tmp_init_pose.rotation().y() << ", "
//                   << tmp_init_pose.rotation().z() << ", "
//                   << tmp_init_pose.rotation().w() << ") \n"
//                   << std::endl;

//         // 初始位姿z轴置零
//         cartographer::transform::Rigid3d z_offset_transform(
//             Eigen::Vector3d(0.0, 0.0, -tmp_init_pose.translation().z()),
//             Eigen::Quaterniond::Identity()
//         );
//         auto adjusted_pose = tmp_init_pose * z_offset_transform;

//         // 打印调整后的初始位姿
//         std::cout << "Adjusted initial pose: " << std::endl;
//         std::cout << "Position: (" << adjusted_pose.translation().x() << ", " 
//                   << adjusted_pose.translation().y() << ", " 
//                   << adjusted_pose.translation().z() << ")" << std::endl;
//         std::cout << "Orientation: (" << adjusted_pose.rotation().x() << ", " 
//                   << adjusted_pose.rotation().y() << ", " 
//                   << adjusted_pose.rotation().z() << ", " 
//                   << adjusted_pose.rotation().w() << ") \n" << std::endl;

//         // TODO: 如果想计算初始定位的误差，需要跟真实初始位姿进行比较
//         // 计算平移误差
//         Eigen::Vector3d translation_error = adjusted_pose.translation() - recorded_initial_pose.translation();
//         std::cout << "Translation error: (" << translation_error.x() << ", " 
//                   << translation_error.y() << ", " 
//                   << translation_error.z() << ")" << std::endl;

//         // 计算角度误差
//         Eigen::Quaterniond rotation_error = adjusted_pose.rotation() * recorded_initial_pose.rotation().inverse();
//         Eigen::AngleAxisd angle_axis(rotation_error);
//         double angle_error = angle_axis.angle();
//         std::cout << "Angle error (in radians): " << angle_error << std::endl;
//         std::cout << "Angle error (in degrees): " << angle_error * 180.0 / M_PI << "\n" << std::endl;

//         StartLocalizationWithInitialPose(adjusted_pose);

//         flag_recorded_initial_pose_received = false;
//       }
//       else
//       {
//         std::cout << "No initial pose received, using default initial pose" << std::endl;
//       }
//       CHECK(ValidateTrajectoryOptions(trajectory_options));
//       AddTrajectory(trajectory_options);
//       flag_finish_initial_localization = true;
//     }
//   }
// }
// // added by hz end

bool Node::HandleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
  return true;
}

bool Node::HandleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response->status.message;
    return true;
  }
  map_builder_bridge_->HandleTrajectoryQuery(request, response);

  return true;
}

void Node::PublishSubmapList() {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList(node_->now()));
}

/**
 * @brief 节点中创建位姿的插值器
 * 
 * @param trajectory_id 
 * @param options 
 */
void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  // 插值器的时间间隔
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  // 检查是否已经为对应trajectory_id的轨迹添加了位姿外推器
  CHECK(extrapolators_.count(trajectory_id) == 0);
  // 重力时间常数
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  // 构建插值器
  // 往extrapolators_里插入 轨迹id 和 外插器（用外插器估计时间步长和重力时间常数初始化），其中gravity_time_constant是重力加速度的权重，用来平衡加速度计测量和重力的影响
  extrapolators_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

/**
 * @brief 创建传感器采样器
 * 
 * @param trajectory_id 
 * @param options 
 */
void Node::AddSensorSamplers(const int trajectory_id,
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

void Node::PublishLocalTrajectoryData() {
  absl::MutexLock lock(&mutex_);
  // 获得不同轨迹的数据，并进行插值
  // 在这里会获取published_to_tracking
  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) { 
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
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
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
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
                local_map_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      // 获取local到map的变换矩阵
      const Rigid3d& local_to_map = trajectory_data.local_to_map;
      
      if (matched_feature_publisher_->get_subscription_count() > 0) {
        visualization_msgs::msg::MarkerArray local_line_msg;
        const auto& line_data = trajectory_data.local_slam_data->line_data_in_local;
        
        for (const auto& line_element : line_data) {
          for (const auto& line : line_element.second) {
            // 使用全局函数风格的坐标转换，与PointCloud保持一致
            auto transformed_line = cartographer::sensor::TransformLineData(line, local_to_map);
            std::pair<Eigen::Vector2d, Eigen::Vector2d> line_eigen = {transformed_line.begin(), transformed_line.end()};
            local_line_msg.markers.push_back(CreateLineMsg(
                                              line_element.first,
                                              node_options_.map_frame,
                                              line_eigen,
                                              COLOR::green));
          }
        }
        matched_feature_publisher_->publish(local_line_msg);
      }
      
      if (local_feature_map_publisher_->get_subscription_count() > 0) {
        visualization_msgs::msg::MarkerArray map_line_msg;
        const auto& line_map_2d_list = trajectory_data.local_slam_data->line_map_2d_list->GetLineMapData2DList();
        
        for (const auto& line_map : line_map_2d_list) {
          // 使用全局函数风格的坐标转换，与PointCloud保持一致
          auto transformed_line = cartographer::sensor::TransformLineData(line_map.second.line_data(), local_to_map);
          std::pair<Eigen::Vector2d, Eigen::Vector2d> line_eigen = {transformed_line.begin(), transformed_line.end()};
          map_line_msg.markers.push_back(CreateLineMsg(
                                            line_map.first,
                                            node_options_.map_frame,
                                            line_eigen,
                                            COLOR::red));
        }
        local_feature_map_publisher_->publish(map_line_msg);
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }
 
    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        FromRos(node_->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? ToRos(now)
            : ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

    // 获取当前机器人在local坐标系下的位姿
    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    // 如果需要投影到2d，做一个投影的操作，如果不需要投影到2d，直接发布3d位姿，可以在lua中做配置
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
    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {
      if (node_options_.publish_to_tf) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          // 发布
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;
          // noted by hz:发布map->odom_base，这些坐标系在launch中赋值
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform);

          // published_frame to odom(什么是 published_frame?)
          // 来自官方的解释
          // 发布姿态时要使用的ROS坐标系ID作为子坐标系。例如，如果系统的其他部分
          // 提供了一个“odom”坐标系，则可以将其设置为“odom”。在这种情况下，
          // 将发布“odom”在“map_frame”中的姿态。否则，将其设置为“base_link”可能是合适的。

          // noted by hz:发布odom_base->footprint
          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          // T_odom_published = T_odom_base * T_base_published
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);

          tf_broadcaster_->sendTransform(stamped_transforms);
        } else {
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          // 发布map->published_frame
          tf_broadcaster_->sendTransform(stamped_transform);
        }
      }
      // 发布跟踪的位姿
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
        // 发布map坐标系下的tracking frame位姿
        tracked_pose_publisher_->publish(pose_msg);

        // TODO: linfeng.guo: 根据需要，判断是否发布path
        // tracked_path_msg_.header.stamp = stamped_transform.header.stamp;
        // tracked_path_msg_.header.frame_id = node_options_.map_frame;
        // tracked_path_msg_.poses.push_back(pose_msg);
        // tracked_path_publisher_->publish(tracked_path_msg_);
      }

        // TODO: linfeng.guo: 根据需要，判断是否需要保存位姿
      if (node_options_.save_tracked_pose &&
        trajectory_data.trajectory_options.trajectory_builder_options.has_pure_localization_trimmer())
      {
        Eigen::Vector3d t = tracking_to_map.translation();
        Eigen::Quaterniond q = tracking_to_map.rotation();
        double time = cartographer_ros::ToRos(trajectory_data.local_slam_data->time).seconds();
        if (!carto_pose_tool_) {
          LOG(INFO) << "carto_pose_tool_ is nullptr\n";
        } else {
          carto_pose_tool_->AddPoseData(t, q, time);
        }
      }
    }
  }
}

void Node::PublishTrajectoryNodeList()
{
  if (trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList(node_->now()));
  }
}

void Node::PublishLandmarkPosesList() {
  if (landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_->GetLandmarkPosesList(node_->now()));
  }
}

void Node::PublishConstraintList() {
  if (constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(node_->now()));
  }
}

/**
 * @brief 根据配置，检查系统具有的传感器id
 * 
 * @param options 
 * @return std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
 *         set形式的传感器列表
 */
std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  // 如果laser数量大于1,将kLaserScanTopic作为topic，后面加上 '_num' 后缀
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
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    RCLCPP_WARN(node_->get_logger(), "subscribing to IMU topic");
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
/**
 * @brief 创建轨迹优化器，配置节点初始参数
 *        添加轨迹；添加并初始化 外推器，采样器，订阅器
 * 
 * @param options 优化器配置参数
 * @return int trajectory_id
 */
int Node::AddTrajectory(const TrajectoryOptions& options) {
  // 1. 根据配置，检查具有的传感器以及期望订阅的topic
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  // 2. 根据配置，向map_builder中增加轨迹优化器，返回轨迹id
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  // 3. 增加轨迹插值器
  AddExtrapolator(trajectory_id, options);
  // 4. 增加传感器采样器(用于计算和控制传感器采样输入)
  AddSensorSamplers(trajectory_id, options);
  // 5. 启动订阅器，根据参数配置，为对应trajectory_id添加各传感器的订阅器（当然包含回调函数）
  LaunchSubscribers(options, trajectory_id);
  // 6. 每kTopicMismatchCheckDelaySec秒检查一次topic是否匹配
  maybe_warn_about_topic_mismatch_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
    [this]() {
      MaybeWarnAboutTopicMismatch(); // 实际上没有起作用，内容是空的
    });
  // 7. 订阅的topic以及与之对应的传感器id
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}


/**
 * @brief 根据配置，订阅传感器数据
 * 
 * @param options 
 * @param trajectory_id 
 */
void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  auto node_handle = node_;

  // 激光扫描和点云频率较低，使用 BEST_EFFORT 减少延迟
  auto range_sensor_qos = rclcpp::QoS(rclcpp::KeepLast(50))
                              .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                              .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  
  // IMU 和 Odometry 频率高，需要大缓冲和可靠传输
  auto high_freq_sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000))
                                  .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                  .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // LaserScan
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_handle, this,
             range_sensor_qos),
         topic});
  }

  // MultiEchoLaserScan
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic, node_handle, this,
             range_sensor_qos),
         topic});
  }

  // PointCloud2
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic, node_handle, this,
             range_sensor_qos),
         topic});
  }

  // IMU
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                node_handle, this,
                                                high_freq_sensor_qos),
         kImuTopic});
  }

  // Odometry
  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  node_handle, this,
                                                  high_freq_sensor_qos),
         kOdometryTopic});
  }

  // NavSatFix
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             node_handle, this),
         kNavSatFixTopic});
  }

  // Landmarks
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             node_handle, this),
         kLandmarkTopic});
  }
}

/**
 * @brief 检查是否有2d/3d建图时所对应的轨迹优化器参数
 * 
 * @param options 配置参数
 * @return true 
 * @return false 
 */
bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message = "Trajectory " + std::to_string(trajectory_id) + " is in '" +
    TrajectoryStateToString(it->second) + "' state.";
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message = absl::StrCat("Trajectory ", trajectory_id,
                                           " already pending to finish.");
    // status_response.message =
    //     "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
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
      entry.subscriber.reset();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      absl::StrCat("Finished trajectory ", trajectory_id, ".");
      // "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return status_response;
}
// tag: service 开始一条新的轨迹
bool Node::HandleStartTrajectory(
    const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response) {
  TrajectoryOptions trajectory_options;
  // LoadOptions返回 NodeOptions 和 TrajectoryOptions
  // 这里我们不关心 NodeOptions，使用 std::tie 解构 LoadOptions（）的返回值，只将第二个元素赋值给 trajectory_options
  // 说人话就是，从request给的路径中，提取 TrajectoryOptions 赋值给 trajectory_options
  std::tie(std::ignore, trajectory_options) = LoadMappingOptions(
      request->configuration_directory, request->configuration_basename);

  if (request->use_initial_pose) {
    const auto pose = ToRigid3d(request->initial_pose);
    if (!pose.IsValid()) {
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response->status.message;
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response->status.message;
      return true;
    }
    // tag: 用于设置 trajectory_options 的 initial_trajectory_pose 字段
    // TODO：也就是说可以在lua文件中进行设置？
    // DONE：应该是在 ::cartographer::mapping::proto::TrajectoryBuilderOptions 中进行设置
    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    // 设置 initial_trajectory_pose 的 to_trajectory_id 字段，一般设置为想对于轨迹0,即建图的轨迹
    initial_trajectory_pose.set_to_trajectory_id(
        request->relative_to_trajectory_id);
    // 将 pose 转换为 protobuf 格式的消息（对于消息字段，用mutable），设置为 relative_pose 字段，表示想对于轨迹0的初始位姿
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    // 设置 timestamp 字段
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(rclcpp::Time(0))));
    // 将 initial_trajectory_pose 赋值 trajectory_options.trajectory_builder_options 的initial_trajectory_pose字段
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }
  // 检查确认是否设置了use_trajectory_builder_2d()/use_trajectory_builder_3d()
  // 检查topic是否已经被其他轨迹使用，如果topic正在被其他轨迹使用，会报错，所以要先结束之前的轨迹，释放订阅器和topics，才能开始新的轨迹
  // 检查都OK，则添加新的轨迹
  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response->status.message = "Invalid trajectory options.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response->status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else {
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
  return true;
}

// TODO: linfeng.guo: 判断是否需要删除
// void Node::StartLocalizationWithInitialPose(cartographer::transform::Rigid3d initial_pose)
// {
//   // 将初始位姿转换为cartographer的proto格式
//   ::cartographer::mapping::proto::InitialTrajectoryPose initial_trajectory_pose;
//   // 设置 initial_trajectory_pose 的 to_trajectory_id
//   // 字段，一般设置为想对于轨迹0,即建图的轨迹
//   initial_trajectory_pose.set_to_trajectory_id(0);
//   // 将 pose 转换为 protobuf 格式的消息（对于消息字段，用mutable），设置为
//   // relative_pose 字段，表示想对于轨迹0的初始位姿
//   *initial_trajectory_pose.mutable_relative_pose() =
//       cartographer::transform::ToProto(initial_pose);
//   // 设置 timestamp 字段
//   initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
//       ::cartographer_ros::FromRos(rclcpp::Time(0))));
//   // 将 initial_trajectory_pose 赋值
//   // trajectory_options.trajectory_builder_options
//   // 的initial_trajectory_pose字段
//   *trajectory_options.trajectory_builder_options
//        .mutable_initial_trajectory_pose() = initial_trajectory_pose;
//   // end
// }
// // end
/**
 * @brief 创建轨迹优化器，订阅ros消息
 * 
 * @param options 
 */
void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  // 检查是否有是否有轨迹优化器参数
  CHECK(ValidateTrajectoryOptions(options));
  // 创建轨迹优化器
  AddTrajectory(options);
}

// 传入轨迹配置项，检查配置项有效，然后添加一个新的轨迹
// void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options, 
//                                             bool flag_localization, 
//                                             const std::string& initial_poses_filename, 
//                                             const std::string& apriltag_poses_filename,
//                                             const std::string& location_name) {
//   absl::MutexLock lock(&mutex_); // 创建一个互斥锁对象，此时共享资源不会被其他线程访问
//   // added by hz
//   std::cout << "The current mode is localization mode? " << flag_localization << "\n" << std::endl;
//   // 定位模式
//   if(flag_localization)
//   {
//     trajectory_options = options; // 将传入的轨迹配置项赋值给成员变量
//     // 如果不通过rviz手动设置初值，则从文件中读取
//     // 如果通过rviz手动设置初值，则等待给初值
//     if (!node_options_.sub_initial_pose)
//     {
//       geometry_msgs::msg::Pose msg;
//       YAML::Node config;
//       if (trajectory_options.use_predefined_init_pose)
//       {
//         // 解析 YAML 文件，从yaml文件获取记录的初始位姿
//         config = YAML::LoadFile(initial_poses_filename);
//         if (config.IsNull())
//         {
//           std::cerr << "Failed to load initial YAML file: " << initial_poses_filename << "\n" << std::endl;
//         }
//         else
//         {
//           // 获取位姿列表
//           const YAML::Node& poses = config["poses"];

//           for (const auto& pose : poses) 
//           {
//             std::string location = pose["location"].as<std::string>();
//             std::string frame_id = pose["frame_id"].as<std::string>();
//             // 根据测试地区名字，获取对应的初始位姿信息
//             if (location == location_name)
//             {
//               msg.position.x = pose["position"]["x"].as<double>();
//               msg.position.y = pose["position"]["y"].as<double>();
//               msg.position.z = pose["position"]["z"].as<double>();
//               msg.orientation.x = pose["orientation"]["x"].as<double>();
//               msg.orientation.y = pose["orientation"]["y"].as<double>();
//               msg.orientation.z = pose["orientation"]["z"].as<double>();
//               msg.orientation.w = pose["orientation"]["w"].as<double>();

//               // 打印位姿信息
//               std::cout << "Location: " << location << std::endl;
//               std::cout << "Frame ID: " << frame_id << std::endl;
//               std::cout << "Position: (" << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << ")" << std::endl;
//               std::cout << "Orientation: (" << msg.orientation.x << ", " << msg.orientation.y << ", " << msg.orientation.z << ", " << msg.orientation.w << ") \n" << std::endl;
              
//               recorded_initial_pose = ToRigid3d(msg);
//               flag_recorded_initial_pose_received = true;
//             }
//           }
//           if (!flag_recorded_initial_pose_received)
//           {
//             std::cerr << "No recorded initial pose found for location: " << location_name << std::endl;
//           }
//         }
//       }
//       // 如果订阅了apriltag，读取记录的apriltag对应初值，在回调计算实际初始定位
//       if (node_options_.sub_apriltag_pose)
//       {
//         // 解析 YAML 文件，从yaml文件获取记录的初始apriltag位姿
//         config = YAML::LoadFile(apriltag_poses_filename);
//         if (config.IsNull())
//         {
//           std::cerr << "Failed to load apriltag YAML file: " << apriltag_poses_filename << std::endl;
//         }
//         else
//         {
//           for (const auto& pose : config["poses"]) 
//           {
//             std::string location = pose["location"].as<std::string>();
//             std::string frame_id = pose["frame_id"].as<std::string>();
//             if (location == location_name)
//             {
//               msg.position.x = pose["position"]["x"].as<double>();
//               msg.position.y = pose["position"]["y"].as<double>();
//               msg.position.z = pose["position"]["z"].as<double>();
//               msg.orientation.x = pose["orientation"]["x"].as<double>();
//               msg.orientation.y = pose["orientation"]["y"].as<double>();
//               msg.orientation.z = pose["orientation"]["z"].as<double>();
//               msg.orientation.w = pose["orientation"]["w"].as<double>();

//               std::cout << "Location: " << location << std::endl;
//               std::cout << "Frame ID: " << frame_id << std::endl;
//               std::cout << "Position: (" << msg.position.x << ", " << msg.position.y << ", " << msg.position.z << ")" << std::endl;
//               std::cout << "Orientation: (" << msg.orientation.x << ", " << msg.orientation.y << ", " << msg.orientation.z << ", " << msg.orientation.w << ") \n" << std::endl;

//               recorded_apriltag_pose = ToRigid3d(msg);
//               flag_recorded_apriltag_pose_received = true;
//             }
//           }
//           if (!flag_recorded_apriltag_pose_received)
//           {
//             std::cerr << "No recorded apriltag pose found for location " << location_name << std::endl;
//           }
//         }
//       }
//       // 如果没有订阅apriltag，则使用记录的初始位姿
//       else
//       {
//         // 如果读取到了记录的初始位姿，则使用它
//         if (flag_recorded_initial_pose_received)
//         {
//           std::cout << "Using recorded initial pose" << std::endl;
//           StartLocalizationWithInitialPose(recorded_initial_pose);
//         }
//         else
//         {
//           std::cout << "There is not any recorded initial pose，start localization with default initial pose" << std::endl;
//         }
//         CHECK(ValidateTrajectoryOptions(trajectory_options));
//         AddTrajectory(trajectory_options);

//         flag_finish_initial_localization = true;
//       }
//     }
//     else
//     {
//       std::cout << "Waiting for initial pose message" << std::endl;
//     }
//   }
//   // 建图模式
//   else{
//     CHECK(ValidateTrajectoryOptions(options));
//     AddTrajectory(options);
//   }
//   // added by hz end
//   // commented by hz
//   // source code
//   // CHECK(ValidateTrajectoryOptions(options));
//   // AddTrajectory(options);
//   // end
// }

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::HandleGetTrajectoryStates(
    const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr ,
    cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response) {

  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = node_->now();
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
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
  return true;
}

bool Node::HandleFinishTrajectory(
    const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
  return true;
}

bool Node::HandleWriteState(
    const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
    cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename,
                                         request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        "Failed to write '" + request->filename + "'.";
  }
  return true;
}

bool Node::HandleReadMetrics(
    const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
    cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response)
{
  absl::MutexLock lock(&mutex_);
  response->timestamp = node_->now();
  if (!metrics_registry_) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(response);
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = "Successfully read metrics.";
  return true;
}

void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

/**
 * @brief 保存指定id轨迹优化后的位姿
 * 
 * @param trajectory_id 轨迹ID
 */
void Node::SaveOptimizedPoses(const int & trajectory_id)
{
  LOG(INFO) << "Saving Optimized Poses\n";
  const auto node_poses = map_builder_bridge_->GetTrajectoryPoses();
  for (const auto & node_id_data :
      node_poses.trajectory(trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }

    double time = ToRos(node_id_data.data.constant_pose_data.value().time).seconds();
    Eigen::Vector3d t = node_id_data.data.global_pose.translation();
    Eigen::Quaterniond q = node_id_data.data.global_pose.rotation();

    if (!carto_pose_tool_) {
      LOG(INFO) << "carto_pose_tool_ is nullptr\n";
    } else {
      carto_pose_tool_->AddPoseData(t, q, time);
    }
  }
}

bool Node::SaveMap(
  const std::string & map_filename, const double resolution)
{
  return map_builder_bridge_->SaveMap(map_filename, resolution);
}

void Node::ApplyMapOriginTransform(const Rigid3d& T_old_to_new) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->ApplyMapOriginTransform(T_old_to_new);
}


void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  // sensor_bridge（返回SensorBridge类）是map_builder_bridge_（MapBuilderBridge类）的一个成员函数，用来
  map_builder_bridge_->sensor_bridge(trajectory_id) 
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_->SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

/**
 * @brief 读取pstream文件（子图、约束、轨迹等信息）
 * 
 * @param state_filename 
 * @param load_frozen_state 
 */
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}

// TODO: find ROS equivalent to ros::master::getTopics
void Node::MaybeWarnAboutTopicMismatch() {
  // const auto topics_and_types = node_->get_topic_names_and_types();

  // // 已是绝对全名
  // std::set<std::string> published_topics;
  // std::stringstream published_topics_ss;
  // for (const auto& kv : topics_and_types) {
  //   const std::string& abs_name = kv.first;
  //   if (published_topics.insert(abs_name).second) {
  //     published_topics_ss << abs_name << ",";
  //   }
  // }

  // bool print_topics = false;
  // for (const auto& entry : subscribers_) {
  //   int trajectory_id = entry.first;
  //   for (const auto& subscriber : entry.second) {
  //     const std::string resolved_topic = node_->resolve_name(subscriber.topic);
  //     if (published_topics.count(resolved_topic) == 0) {
  //       RCLCPP_WARN(node_->get_logger(),
  //         "Expected topic \"%s\" (trajectory %d) (resolved \"%s\") but no publisher is currently active.",
  //         subscriber.topic.c_str(), trajectory_id, resolved_topic.c_str());
  //       print_topics = true;
  //     }
  //   }
  // }

  // if (print_topics) {
  //   RCLCPP_WARN(node_->get_logger(), "Currently available topics are: %s",
  //     published_topics_ss.str().c_str());
  // }
}

}  // namespace cartographer_ros
