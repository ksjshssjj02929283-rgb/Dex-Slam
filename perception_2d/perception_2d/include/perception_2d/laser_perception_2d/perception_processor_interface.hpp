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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_INTERFACE_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_INTERFACE_HPP_

#include <set>
#include "perception_2d/laser_perception_2d/laser_perception_2d_interface.hpp"

namespace perception_2d
{

class PerceptionProcessorInterface
{
public:
  PerceptionProcessorInterface() {}
  virtual ~PerceptionProcessorInterface() {}

  using SensorId = LaserPerception2DInterface::SensorId;

  PerceptionProcessorInterface(const PerceptionProcessorInterface &) = delete;
  PerceptionProcessorInterface & operator=(const PerceptionProcessorInterface &) = delete;

  virtual void InitPerception(const std::set<SensorId> & expected_sensor_id) = 0;
  virtual LaserPerception2DInterface * GetLaserPerception2D() = 0;

  virtual void ResetPerception() = 0;

  virtual void clear() = 0;
};


}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_INTERFACE_HPP_
