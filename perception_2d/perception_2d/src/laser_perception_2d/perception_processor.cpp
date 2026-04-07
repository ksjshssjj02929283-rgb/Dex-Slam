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
#include <set>
#include <memory>
#include "perception_2d/laser_perception_2d/perception_processor.hpp"
#include "perception_2d/laser_perception_2d/laser_perception_2d.hpp"
namespace perception_2d
{

std::vector<std::string> SelectRangeSensorIds(
  const std::set<PerceptionProcessor::SensorId> & expected_sensor_ids)
{
  std::vector<std::string> range_sensor_ids;
  for (const PerceptionProcessor::SensorId & sensor_id : expected_sensor_ids) {
    if (sensor_id.type == PerceptionProcessor::SensorId::SensorType::RANGE) {
      range_sensor_ids.push_back(sensor_id.id);
    }
  }
  return range_sensor_ids;
}

PerceptionProcessor::PerceptionProcessor(
  const PerceptionOptions & perception_options)
: perception_options_(perception_options) {}

void PerceptionProcessor::InitPerception(const std::set<SensorId> & expected_sensor_id)
{
  std::vector<std::string> expected_range_sensor_ids =
    SelectRangeSensorIds(expected_sensor_id);
  laser_perception_2d_.reset();
  laser_perception_2d_ = std::make_unique<LaserPerception2D>(
    expected_range_sensor_ids, perception_options_.config);
  LOG(INFO) << "PerceptionProcessor::InitPerception";
}

void PerceptionProcessor::ResetPerception()
{
  laser_perception_2d_->ResetStatus();
}

LaserPerception2DInterface * PerceptionProcessor::GetLaserPerception2D()
{
  return laser_perception_2d_.get();
}

void PerceptionProcessor::clear()
{
  laser_perception_2d_->clear();
}



}  // namespace perception_2d
