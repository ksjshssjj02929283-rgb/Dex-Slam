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

#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_PROCESSOR_BRIDGE_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_PROCESSOR_BRIDGE_HPP_
#include <set>
#include <memory>
#include "perception_2d_ros2/sensor_bridge.hpp"
#include "perception_2d/laser_perception_2d/perception_processor_interface.hpp"
#include "perception_2d_ros2/node_option.hpp"
#include "perception_2d_ros2/feature_map_bridge.hpp"


namespace perception_ros2
{

class PerceptionProcessorBridge
{
public:
  PerceptionProcessorBridge(
    const NodeOptions & node_options,
    std::unique_ptr<perception_2d::PerceptionProcessorInterface> perception_processor,
    tf2_ros::Buffer * tf_buffer);

  PerceptionProcessorBridge(const PerceptionProcessorBridge &) = delete;
  PerceptionProcessorBridge & operator=(const PerceptionProcessorBridge &) = delete;

  void InitPerception(
    const std::set<perception_2d::LaserPerception2DInterface::SensorId> & expected_sensor_ids,
    const NodeOptions & node_options);

  void ResetPerception();

  perception_2d::TargetMap GetTargetPose();
  // const std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>>& GetLineAndRef();
  bool GetLineAndRef(std::vector<std::shared_ptr<perception_2d::FeatureMsgWithOrigin>> & feature_map);

  perception_2d::sensor::SyncPointCloudWithOdom GetSyncPointCloud();
  // std::tuple<perception_2d::LinesAndTextInfo, perception_2d::LinesAndTextInfo>
  //   GetPrePointVisInfo();
  std::unique_ptr<perception_2d::PerceptionProcessorInterface> perception_processor_interface_;
  std::unique_ptr<SensorBridge> sensor_bridge_;
  std::unique_ptr<FeatureMapBridge> feature_map_bridge_;

private:
  NodeOptions node_options_;
  tf2_ros::Buffer * const tf_buffer_;
};


}  // namespace perception_ros2
#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_PROCESSOR_BRIDGE_HPP_
