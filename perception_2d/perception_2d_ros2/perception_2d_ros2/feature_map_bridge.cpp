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
#include <vector>
#include "perception_2d_ros2/sensor_bridge.hpp"

#include "perception_2d_ros2/msg_conversion.hpp"
#include "perception_2d_ros2/time_conversion.hpp"
#include "perception_2d_ros2/feature_map_bridge.hpp"

namespace perception_ros2
{


using perception_2d::transform::Rigid3d;

namespace
{

const std::string & CheckNoLeadingSlash(const std::string & frame_id)
{
  if (frame_id.size() > 0) {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id <<
      " should not start with a /. See 1.7 in "
      "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

}  // namespace

FeatureMapBridge::FeatureMapBridge(
  const std::string & tracking_frame,
  const double lookup_transform_timeout_sec,
  tf2_ros::Buffer * const tf_buffer,
  perception_2d::LaserPerception2DInterface * const laser_perception_2d)
: tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
  laser_perception_2d_(laser_perception_2d) {}

void FeatureMapBridge::UpdateFeatureMap(
  const std::string & topic,
  const nav_interfaces::msg::TargetInfo::ConstSharedPtr & msg)
{
  std::vector<perception_2d::FeatureInfo> feature_map;
  // feature_map.time = FromRos(rclcpp::Time().now());

  // perception_2d::FeatureMsg feature_msg;
  perception_2d::FeatureInfo feature_msg;
  feature_msg.target_name = msg->name + std::to_string(msg->module_id);
  feature_msg.pose.x = msg->x;
  feature_msg.pose.y = msg->y;
  feature_msg.pose.z = msg->theta;

  if (feature_msg.target_name != "0") {
    feature_map.push_back(feature_msg);
  }

  laser_perception_2d_->HandleFeatureMapInfoMessage(topic, feature_map);
}

void FeatureMapBridge::UpdateTransformFromOdomToMap(
  const perception_2d::transform::Rigid3d & odom_to_map){}

perception_2d::TargetMap FeatureMapBridge::GetTargetPose()
{
  return laser_perception_2d_->GetTargetMsg();
}

bool FeatureMapBridge::GetLineAndRef(std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> & feature_map)
{
  return laser_perception_2d_->GetLineAndRef(feature_map);
}

perception_2d::sensor::SyncPointCloudWithOdom 
  FeatureMapBridge::GetSyncPointCloud(){
  return laser_perception_2d_->GetSyncPointCloud();
}

// std::tuple<perception_2d::LinesAndTextInfo, perception_2d::LinesAndTextInfo>
//    FeatureMapBridge::GetPrePointVisInfo(){
//   return laser_perception_2d_->GetPrePointVisInfo();
// }

// const TfBridge& SensorBridge::tf_bridge() const { return tf_bridge_; }

}  // namespace perception_ros2
