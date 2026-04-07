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
#include <utility>
#include <set>
#include <memory>
#include "perception_2d_ros2/perception_processor_bridge.hpp"

namespace perception_ros2
{
PerceptionProcessorBridge::PerceptionProcessorBridge(
  const NodeOptions & node_options,
  std::unique_ptr<perception_2d::PerceptionProcessorInterface> perception_processor,
  tf2_ros::Buffer * tf_buffer)
: perception_processor_interface_(std::move(perception_processor)),
  node_options_(node_options),
  tf_buffer_(tf_buffer)
{
  //  LOG(INFO) << "perceptoin processor bridge";
}
void PerceptionProcessorBridge::InitPerception(
  const std::set<perception_2d::LaserPerception2DInterface::SensorId> & expected_sensor_ids,
  const NodeOptions & node_options)
{
  perception_processor_interface_->InitPerception(expected_sensor_ids);

  sensor_bridge_ = std::make_unique<SensorBridge>(
    node_options.sensor_bridge_options.num_subdivisions_per_laser_scan,
    node_options.sensor_bridge_options.tracking_frame,
    node_options.lookup_transform_timeout_sec, tf_buffer_,
    perception_processor_interface_->GetLaserPerception2D());

  feature_map_bridge_ = std::make_unique<FeatureMapBridge>(
    node_options.sensor_bridge_options.tracking_frame,
    node_options.lookup_transform_timeout_sec, tf_buffer_,
    perception_processor_interface_->GetLaserPerception2D());

  LOG(INFO) << "PerceptionProcessorBridge::InitPerception";
}

void PerceptionProcessorBridge::ResetPerception()
{
  perception_processor_interface_->ResetPerception();
}

perception_2d::TargetMap PerceptionProcessorBridge::GetTargetPose()
{
  return feature_map_bridge_->GetTargetPose();
}

// const std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>>&
//   PerceptionProcessorBridge::GetLineAndRef(){
//   return feature_map_bridge_->GetLineAndRef();
// }

bool PerceptionProcessorBridge::GetLineAndRef(std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> & feature_map)
{
  return feature_map_bridge_->GetLineAndRef(feature_map);
}

perception_2d::sensor::SyncPointCloudWithOdom 
  PerceptionProcessorBridge::GetSyncPointCloud(){
  return feature_map_bridge_->GetSyncPointCloud();
}



// std::tuple<perception_2d::LinesAndTextInfo, perception_2d::LinesAndTextInfo>
//     PerceptionProcessorBridge::GetPrePointVisInfo(){
//     return feature_map_bridge_->GetPrePointVisInfo();
// }

}  // namespace perception_ros2
