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
#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_HPP_

#include <set>
#include <memory>
#include <vector>
#include <string>
#include "perception_2d/laser_perception_2d/perception_processor_interface.hpp"

namespace perception_2d
{

struct PerceptionOptions
{
  Config config;
};

class PerceptionProcessor : public PerceptionProcessorInterface
{
public:
  explicit PerceptionProcessor(const PerceptionOptions & perception_options);
  void InitPerception(const std::set<SensorId> & expected_sensor_id);

  LaserPerception2DInterface * GetLaserPerception2D();
  void clear() override;

  void ResetPerception() override;

private:
  std::unique_ptr<LaserPerception2DInterface> laser_perception_2d_;
  PerceptionOptions perception_options_;
};

std::vector<std::string> SelectRangeSensorIds(
  const std::set<PerceptionProcessor::SensorId> & expected_sensor_ids);
}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__PERCEPTION_PROCESSOR_HPP_
