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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_
#define CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_

#include <memory>
#include <utility>
#include <vector>

#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/flat_cloud_detector.h"
#include "cartographer/laser_perception/laser_perception_options.h"
#include "cartographer/laser_perception/line_detector_x.h"
#include "cartographer/sensor/sync_point_cloud.h"

namespace cartographer
{
namespace laser_perception
{

struct FeatureMsgWithOrigin
{
  Eigen::Vector3d origin;
  std::vector<FeatureMsg> MsgVec;
};

class LaserPerception
{
public:
  explicit LaserPerception(const proto::LaserPerceptionOptions & options);
  ~LaserPerception();

  std::vector<std::pair<int, std::vector<int>>> laserCluster(
    const std::vector<std::pair<int, Eigen::Vector3f>> & msg);
  std::vector<FeatureMsgWithOrigin> laserUpdate(const sensor::PointCloudWithOrigin & cloud);

  std::shared_ptr<sensor::PointCloud> getHighlightCloud() const { return point_cloud_highlight_; }
  std::shared_ptr<sensor::PointCloud> getLessFlatCloud() const { return point_cloud_less_flat_; }

private:
  int min_cluster_points_;
  double min_range_;
  double max_range_;
  int highlight_intensity_threshold_;
  double cluster_distance_threshold_;

  LineDetectorX * lineDetectorX_;
  FlatCloudDetector * flatCloudDetector_;

  std::shared_ptr<sensor::PointCloud> point_cloud_highlight_;
  std::shared_ptr<sensor::PointCloud> point_cloud_less_flat_;
};

}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_