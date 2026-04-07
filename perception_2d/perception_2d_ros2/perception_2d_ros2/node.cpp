// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string>
#include <memory>
#include <utility>
#include <set>
#include <functional>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "perception_2d_ros2/node.hpp"
#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/transform/transform.hpp"

namespace perception_ros2
{

std::vector<std::string> SpliteStr(
  const std::string & pathes)
{
  std::vector<std::string> res;
  std::string current_path;
  for (const auto & c : pathes) {
    if (c == ' ' || c == ',') {
      res.push_back(current_path);
      current_path.clear();
    } else {
      current_path.push_back(c);
    }
  }
  if (!current_path.empty()) {
    res.push_back(current_path);
  }
  return res;
}

PerceptionTopics DefaultSensorTopics()
{
  PerceptionTopics topics;
  topics.laser_scan_topic = "scan";
  topics.odometry_topic = "odom";
  topics.update_feature_map_topic = "/updateFeatureMap";
  return topics;
}

template<typename MessageType>
rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
  void (Node::* handler)(
    const std::string &,
    const typename MessageType::ConstSharedPtr &),
  const std::string & topic, 
  std::shared_ptr<rclcpp::Node> node_handle,
  Node * const node)
{
  auto callback = [node, handler, topic](const typename MessageType::ConstSharedPtr msg) {
    (node->*handler)(topic, msg);
  };
  
  return node_handle->create_subscription<MessageType>(
    topic, rclcpp::QoS(10), callback);
}

Node::Node(
  std::shared_ptr<rclcpp::Node> node,
  const NodeOptions & node_options,
  std::unique_ptr<perception_2d::PerceptionProcessorInterface> perception_processor,
  tf2_ros::Buffer * const tf_buffer)
: node_handle_(node),
  perception_processor_bridge_(node_options, std::move(perception_processor), tf_buffer),
  is_activate_(false)
{
  LOG(INFO) << "node start";
  
  target_map_pub = node_handle_->create_publisher<nav_interfaces::msg::TargetMapInfo>(
    "/target_map", rclcpp::QoS(1));
    
  target_vis_pub = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/visualization_marker", rclcpp::QoS(1));
    
  feature_detected_pub_ = node_handle_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/feature_detected_2", rclcpp::QoS(1));

  line_pub_ = node_handle_->create_publisher<nav_interfaces::msg::LineArray>(
    "/laser_line", rclcpp::QoS(1));

  reflector_pub_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud>(
    "/reflector", rclcpp::QoS(1));

  docking_info_pub_ = node_handle_->create_publisher<nav_interfaces::msg::DockingInfo>(
    "/docking_target_info", rclcpp::QoS(1));

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);
  
  // Create services
  active_perception_server_ = node_handle_->create_service<std_srvs::srv::Empty>(
    "activate_perception",
    std::bind(&Node::ActivatePerceptionCallback, this, 
              std::placeholders::_1, std::placeholders::_2));
  
  deactive_perception_server_ = node_handle_->create_service<std_srvs::srv::Empty>(
    "deactivate_perception",
    std::bind(&Node::DeactivatePerceptionCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  PerceptionTopics topics;
  topics = DefaultSensorTopics();
  std::vector<std::string> topic_names;
  std::string scan_topics;
  
  // Get scan topics parameter
  node_handle_->declare_parameter("scan_topics", std::string(""));
  node_handle_->get_parameter("scan_topics", scan_topics);
  
  topic_names = SpliteStr(scan_topics);
  LOG(INFO) << "scan_topics = " << scan_topics;
  if (topic_names.empty()) {
    LOG(INFO) << "scan topics from remap";
    topic_names = ComputeRepeatedTopicNames(
      topics.laser_scan_topic,
      node_options.num_laser_scans);
  }
  for(const std::string & topic : topic_names){
    LOG(INFO) << "topic = " << topic;
    std::string pub_topic, line_topic, ref_topic;
    pub_topic = topic + "_sync";
    line_topic = topic + "_laser_line";
    ref_topic = topic + "_reflector"; 
    
    point_cloud_pubs_.insert(std::make_pair(pub_topic,
      node_handle_->create_publisher<sensor_msgs::msg::PointCloud>(
        pub_topic, rclcpp::QoS(1))));
    
    line_pubs_.insert(std::make_pair(pub_topic,
      node_handle_->create_publisher<nav_interfaces::msg::LineArray>(
        line_topic, rclcpp::QoS(1))));
    
    ref_pubs_.insert(std::make_pair(pub_topic,
      node_handle_->create_publisher<sensor_msgs::msg::PointCloud>(
        ref_topic, rclcpp::QoS(1))));
  }
  
  // Create timers
  wall_timers_.push_back(
    node_handle_->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Node::PublishTargetTF, this)));

  // wall_timers_.push_back(
  //   node_handle_->create_wall_timer(
  //     std::chrono::milliseconds(200),
  //     std::bind(&Node::PublishLineAndRef, this)));

  // wall_timers_.push_back(
  //   node_handle_->create_wall_timer(
  //     std::chrono::milliseconds(200),
  //     std::bind(&Node::PublishPrePoint, this)));
}

Node::~Node(){}

void Node::PublishLineAndRef(){
  if (!is_activate_) {
    return;
  }
  std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> feature_msg_with_origin;
  perception_processor_bridge_.GetLineAndRef(feature_msg_with_origin);

  std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> line_data;
  std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> ref_data;
  for(auto feature_msg_v : feature_msg_with_origin){
    if(feature_msg_v->target_name == "line"){
      line_data.push_back(feature_msg_v);
    }
    if(feature_msg_v->target_name == "reflector"){
      ref_data.push_back(feature_msg_v);
    }
  }

  if(line_data.size() != 0){
    visualization_msgs::msg::MarkerArray feature_detected_markers;
    int marker_id = 0;
    for(auto line_vector : line_data){
      std::vector<perception_2d::LineData> line_data_tmp;
      nav_interfaces::msg::LineArray line_msg;
      line_vector->time = line_vector->time - perception_2d::common::FromSeconds(0.00001);
      line_msg.header.stamp = ToRos(line_vector->time);
      line_msg.header.frame_id = line_vector->frame_id;
      for(auto feature_msg : line_vector->MsgVec){
        line_data_tmp.push_back(
          perception_2d::LineData{TransferLine(feature_msg.line, line_vector->origin).first,
            TransferLine(feature_msg.line, line_vector->origin).second});
        nav_interfaces::msg::Line line;
        line.start.x = feature_msg.line.first[0];
        line.start.y = feature_msg.line.first[1];
        line.start.z = 0;

        line.end.x = feature_msg.line.second[0];
        line.end.y = feature_msg.line.second[1];
        line.end.z = 0;

        line_msg.lines.push_back(line);
      }
      for (auto topic_frame : topic_frame_map_){
         if (topic_frame.first.find(line_vector->frame_id) != std::string::npos) {
          line_msg.header.frame_id = topic_frame.second;
        }      
      }
      for (auto line_publish : line_pubs_) {
        if (line_publish.first.find(line_vector->frame_id) != std::string::npos) {
          auto publisher = std::static_pointer_cast<
            rclcpp::Publisher<nav_interfaces::msg::LineArray>>(line_publish.second);
          publisher->publish(line_msg);
        }
      }
      std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> line_feature =
        CreateLineFeatureMsg(ToRos(line_vector->time), "base_link", marker_id,
            line_data_tmp, false);
      feature_detected_markers.markers.insert(feature_detected_markers.markers.end(),
          line_feature->begin(), line_feature->end());
      marker_id++;
    }
    if (feature_detected_markers.markers.size() != 0) {
      feature_detected_pub_->publish(feature_detected_markers);
    }
  }

  if(ref_data.size() != 0){
    nav_interfaces::msg::TargetMapInfo reflector_map_msg;
    for(auto point_cloud : ref_data){
      // TODO: Convert to PointCloud2 in ROS2
      // sensor_msgs::msg::PointCloud cloud;
      sensor_msgs::msg::PointCloud2 cloud;
      point_cloud->time = point_cloud->time -
        perception_2d::common::FromSeconds(0.00001);
      cloud.header.stamp = ToRos(point_cloud->time);
      cloud.header.frame_id = point_cloud->frame_id;
      // cloud.points.resize(point_cloud->MsgVec.size());
      // cloud.channels.resize(1);
      // cloud.channels[0].name = "intensities";
      // cloud.channels[0].values.resize(point_cloud->MsgVec.size());
      // TODO: Fill PointCloud2 data properly
      /*
      for(size_t i = 0; i < point_cloud->MsgVec.size(); i++){
        cloud.points[i].x = point_cloud->MsgVec[i].x;
        cloud.points[i].y = point_cloud->MsgVec[i].y;
        cloud.points[i].z = 0.0;
        cloud.channels[0].values[i] = 100;      
      }
      */
      for (auto topic_frame : topic_frame_map_) {
        if (topic_frame.first.find(point_cloud->frame_id) != std::string::npos) {
          cloud.header.frame_id = topic_frame.second;
        }
      }
      for (auto ref_publish : ref_pubs_) {
        if (ref_publish.first.find(point_cloud->frame_id) != std::string::npos) {
          auto publisher = std::static_pointer_cast<
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(ref_publish.second);
          publisher->publish(cloud);
        }
      }
    }
  }

  perception_2d::sensor::SyncPointCloudWithOdom sync_point_cloud = 
    perception_processor_bridge_.GetSyncPointCloud();
  for (auto point_cloud : sync_point_cloud.sync_point_clouds) {
    // TODO: Convert to PointCloud2 in ROS2
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = ToRos(sync_point_cloud.stamp);
    for (auto topic_frame : topic_frame_map_) {
      if (topic_frame.first.find(point_cloud.frame_id) != std::string::npos) {
        cloud.header.frame_id = topic_frame.second;
      }
    }
    // TODO: Fill PointCloud2 data properly
    /*
    cloud.points.resize(point_cloud.sync_points_base_link.size());
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(point_cloud.intensities.size());
    for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
      point_cloud.sync_points_base_link[i] =
        point_cloud.origin.cast<float>().inverse() * point_cloud.sync_points_base_link[i];
      cloud.points[i].x = point_cloud.sync_points_base_link[i][0];
      cloud.points[i].y = point_cloud.sync_points_base_link[i][1];
      cloud.points[i].z = 0.0;
      cloud.channels[0].values[i] = point_cloud.intensities[i];
    }
    */
    for (auto publish : point_cloud_pubs_) {
      if (publish.first.find(point_cloud.frame_id) != std::string::npos) {
        // publish.second->publish(cloud);
      }
    }
  }

}

void Node::PublishTargetTF()
{
  if (!is_activate_) {
    return;
  }

  perception_2d::TargetMap target_poses = perception_processor_bridge_.GetTargetPose();

  if (target_poses.pre_point_msg.target_msgs.size() == 0 &&
    target_poses.reflector_msg.feature_msgs.size() == 0 &&
    target_poses.vmark_msg.target_msgs.size() == 0)
  {
    return;
  }

  for (auto vis_info : target_poses.pre_point_msg.feature_map_msgs) {
    linesAndTextVisual(vis_info, "pre_point", node_handle_->now());
  }

  for (auto vis_info : target_poses.pre_point_msg.target_map_msgs) {
    linesAndTextVisual(vis_info, "tar_point", node_handle_->now());
  }

  nav_interfaces::msg::TargetMapInfo target_map_msg;
  for (auto target : target_poses.pre_point_msg.target_msgs) {
    nav_interfaces::msg::TargetInfo target_msg;
    target_msg.x = target.x;
    target_msg.y = target.y;
    target_msg.theta = target.theta;
    target_msg.name = target.target_name;
    target_map_msg.target_map.push_back(target_msg);

    LOG(INFO) << "target name = " << target.target_name << " x = " << target.x << " y = " <<
      target.y <<
      " theta = " << target.theta;

    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.stamp = node_handle_->now();
    stamped_transform.header.frame_id = "map";
    stamped_transform.child_frame_id = target.target_name;
    stamped_transform.transform.rotation.w = perception_2d::transform::RollPitchYaw(
      0, 0,
      target.theta).w();
    stamped_transform.transform.rotation.x = perception_2d::transform::RollPitchYaw(
      0, 0,
      target.theta).x();
    stamped_transform.transform.rotation.y = perception_2d::transform::RollPitchYaw(
      0, 0,
      target.theta).y();
    stamped_transform.transform.rotation.z = perception_2d::transform::RollPitchYaw(
      0, 0,
      target.theta).z();

    stamped_transform.transform.translation.x = target.x;
    stamped_transform.transform.translation.y = target.y;
    stamped_transform.transform.translation.z = 0;
    tf_broadcaster_->sendTransform(stamped_transform);
  }
  target_map_pub->publish(target_map_msg);

  auto target = target_poses.pre_point_msg.target_msgs[0];
  nav_interfaces::msg::DockingInfo docking_info;
  if (target.target_name == "ChargeStation") {
    docking_info.model_type = nav_interfaces::msg::DockingInfo::CHARGE_STATION;
  } else {
    docking_info.model_type = nav_interfaces::msg::DockingInfo::CHARGE_STATION;
  }

  docking_info.pose.header.stamp = node_handle_->now();
  docking_info.pose.header.frame_id = "map";
  docking_info.pose.pose.position.x = target.x;
  docking_info.pose.pose.position.y = target.y;
  docking_info.pose.pose.position.z = 0.0;
  
  // Create quaternion from yaw
  tf2::Quaternion quat;
  quat.setRPY(0, 0, target.theta);
  docking_info.pose.pose.orientation = tf2::toMsg(quat);

  docking_info_pub_->publish(docking_info);
}

void Node::StartRecoginizeWithDefaultTopics(const NodeOptions & node_options)
{
  PerceptionTopics topics;
  topics = DefaultSensorTopics();
  const std::set<perception_2d::LaserPerception2DInterface::SensorId>
  expected_sensor_ids = ComputeExpectedSensorIds(node_options, topics);
  perception_processor_bridge_.InitPerception(expected_sensor_ids, node_options);
  LaunchSubscribers(topics, node_options);
}

void Node::HandleOdometryMessage(
  const std::string & sensor_id,
  const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  if (!is_activate_) {
    return;
  }
  perception_processor_bridge_.sensor_bridge_->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(
  const std::string & sensor_id,
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg)
{
  if (!is_activate_) {
    return;
  }
  if (topic_frame_map_.count(sensor_id) == 0) {
    topic_frame_map_.insert(std::make_pair(sensor_id, msg->header.frame_id));
  }
  perception_processor_bridge_.sensor_bridge_->HandleLaserScanMessage(sensor_id, msg);
  PublishLineAndRef();
}

void Node::HandleFeatureMapInfoMessage(
  const std::string & topic,
  const nav_interfaces::msg::TargetInfo::ConstSharedPtr & msg)
{
  LOG(INFO) << "update feature map info :" << msg->name << " " << msg->x <<
    " " << msg->y << " " << msg->theta;
  perception_processor_bridge_.feature_map_bridge_->UpdateFeatureMap(topic, msg);
}

void Node::LaunchSubscribers(const PerceptionTopics & topics, const NodeOptions & options)
{
  std::vector<std::string> topic_names;
  std::string scan_topics;  // = "/sensor/lidar/front,/sensor/lidar/back";
  // ROS2: Get parameter from node
  scan_topics = node_handle_->declare_parameter<std::string>("scan_topics", "");
  topic_names = SpliteStr(scan_topics);
  if (topic_names.empty()) {
    LOG(INFO) << "scan topics from remap";
    topic_names = ComputeRepeatedTopicNames(
      topics.laser_scan_topic,
      options.num_laser_scans);
  }
  for (const std::string & topic : topic_names)
  {
    subscribers_.push_back(
      {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
          &Node::HandleLaserScanMessage, topic, node_handle_, this),
        topic});
    LOG(INFO) << "subscribe topic = " << topic;
  }

  std::string odom_topic = topics.odometry_topic;
  subscribers_.push_back(
    {SubscribeWithHandler<nav_msgs::msg::Odometry>(
        &Node::HandleOdometryMessage,
        odom_topic, node_handle_, this),
      odom_topic});

  std::string update_feature_map_topic = topics.update_feature_map_topic;
  subscribers_.push_back(
    {SubscribeWithHandler<nav_interfaces::msg::TargetInfo>(
        &Node::HandleFeatureMapInfoMessage, update_feature_map_topic, node_handle_, this),
      update_feature_map_topic});
}

std::set<perception_2d::LaserPerception2DInterface::SensorId>
Node::ComputeExpectedSensorIds(
  const NodeOptions & options,
  const PerceptionTopics & topics) const
{
  using SensorId = perception_2d::LaserPerception2DInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  std::vector<std::string> topic_names;
  std::string scan_topics;  // = "/sensor/lidar/front,/sensor/lidar/back";
  // ROS2: Use already declared parameter
  if(node_handle_->has_parameter("scan_topics")) {
    scan_topics = node_handle_->get_parameter("scan_topics").as_string();
  }
  topic_names = SpliteStr(scan_topics);
  if (topic_names.empty()) {
    LOG(INFO) << "scan topics from remap";
    topic_names = ComputeRepeatedTopicNames(
      topics.laser_scan_topic,
      options.num_laser_scans);
  }
  // Subscribe to all laser scan, multexpected_topicsi echo laser scan, and point cloud topics.
  for (const std::string & topic : topic_names)
  {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(
      SensorId{SensorType::ODOMETRY, topics.odometry_topic});
  }
  return expected_topics;
}

void Node::linesAndTextVisual(
  perception_2d::LinesAndTextInfo lines_text_info,
  std::string ns, rclcpp::Time stamp)
{
  visualization_msgs::msg::MarkerArray lines_text_marker;

  visualization_msgs::msg::Marker line_markers, text_marker;
  line_markers.header.frame_id = text_marker.header.frame_id = lines_text_info.frame;
  line_markers.header.stamp = text_marker.header.stamp = stamp;
  line_markers.ns = text_marker.ns = ns;
  line_markers.action = text_marker.action = visualization_msgs::msg::Marker::ADD;
  line_markers.pose.orientation.w = text_marker.pose.orientation.w = 1.0;

  line_markers.id = 0;
  text_marker.id = 1;
  line_markers.type = visualization_msgs::msg::Marker::LINE_LIST;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

  line_markers.scale.x = 0.005;
  text_marker.scale.z = 0.1;

  line_markers.color.r = text_marker.color.r = color_list[lines_text_info.color][0];
  line_markers.color.g = text_marker.color.g = color_list[lines_text_info.color][1];
  line_markers.color.b = text_marker.color.b = color_list[lines_text_info.color][2];
  line_markers.color.a = text_marker.color.a = 1.0;

  text_marker.text = lines_text_info.text;
  text_marker.pose.position.x = lines_text_info.text_position.x;
  text_marker.pose.position.y = lines_text_info.text_position.y;

  for (auto line : lines_text_info.lines) {
    geometry_msgs::msg::Point p;
    p.x = line.first.x;
    p.y = line.first.y;
    line_markers.points.push_back(p);
    p.x = line.second.x;
    p.y = line.second.y;
    line_markers.points.push_back(p);
  }

  lines_text_marker.markers.push_back(text_marker);
  lines_text_marker.markers.push_back(line_markers);
  target_vis_pub->publish(lines_text_marker);
}


void Node::ActivatePerceptionCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;  // Unused parameter
  (void)res;  // Unused parameter
  // 重置一下检测器
  perception_processor_bridge_.ResetPerception();
  is_activate_ = true;
  RCLCPP_INFO(node_handle_->get_logger(), "Perception activated");
}


void Node::DeactivatePerceptionCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;  // Unused parameter
  (void)res;  // Unused parameter
  is_activate_ = false;
  RCLCPP_INFO(node_handle_->get_logger(), "Perception deactivated");
}


}  // namespace perception_ros2
