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
#ifndef PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_CONSTANTS_HPP_
#define PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_CONSTANTS_HPP_

#include <string>
#include <vector>

namespace perception_ros2
{
std::vector<std::string> ComputeRepeatedTopicNames(
  const std::string & topic,
  int num_topics);
}

#endif  // PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__PERCEPTION_2D_ROS2__NODE_CONSTANTS_HPP_
