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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__REC_BY_REFLEC_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__REC_BY_REFLEC_HPP_

// #include <sensor_msgs/LaserScan.h>
#include <vector>
#include <utility>
#include "perception_2d/laser_perception_2d/config.hpp"
#include "perception_2d/laser_perception_2d/model_manager.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d/sensor/sync_timed_point_cloud.hpp"

namespace perception_2d
{

class RecgByReflec
{
public:
  explicit RecgByReflec(const Config config);
  std::vector<FeatureMsg> getReflector(
    const sensor::TimedPointCloudOriginData & timed_point_cloud_data,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);
  std::vector<FeatureMsg> getReflector(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);
  void UpdateParams(const std::string & param_name, double & value);

private:
  double reflector_error_;
  double reflector_radius_;
};
}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__REC_BY_REFLEC_HPP_
