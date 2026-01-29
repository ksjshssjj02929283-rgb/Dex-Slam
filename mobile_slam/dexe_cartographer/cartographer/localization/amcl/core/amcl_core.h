/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_LOCALIZATION_AMCL_CORE_AMCL_CORE_H_
#define CARTOGRAPHER_LOCALIZATION_AMCL_CORE_AMCL_CORE_H_

#include <chrono>
#include <memory>
#include <mutex>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/internal/box_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/laser_perception.h"
#include "cartographer/localization/proto/pose_estimator_options_2d.pb.h"
#include "cartographer/localization/2d/grid_map_2d.h"
#include "cartographer/localization/2d/cloud_map_2d.h"
#include "cartographer/localization/internal/2d/pose_initializer_2d.h"
#include "cartographer/localization/amcl/sensors/amcl_cloud.h"
#include "cartographer/localization/amcl/sensors/amcl_odom.h"
#include "cartographer/localization/amcl/pf/pf.h"
#include "cartographer/localization/amcl/core/amcl_core_options.h"

namespace cartographer {
namespace localization {
namespace amcl {

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;
  // Mean of pose esimate
  pf_vector_t pf_pose_mean;
  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;
} amcl_hyp_t;

class AMCLCore {
public:
  explicit AMCLCore(
    const proto::AMCLCoreOptions& options);
  ~AMCLCore();

  AMCLCore(const AMCLCore&) = delete;
  AMCLCore& operator=(const AMCLCore&) = delete;

  void SetGrid(const mapping::Grid2D* grid);

  bool SetInitialPose(const transform::Rigid2d& init_pose);

  // Returns 'MatchingResult' when range data accumulation completed,
  // otherwise 'nullptr'. Range data must be approximately horizontal
  // for 2D SLAM. `TimedPointCloudData::time` is when the last point in
  // `range_data` was acquired, `TimedPointCloudData::ranges` contains the
  // relative time of point with respect to `TimedPointCloudData::time`.
  bool UpdateWithSensorData(const common::Time time,
                            const sensor::PointCloud& point_cloud,
                            const sensor::OdometryData& odometry_data);

  std::shared_ptr<transform::Rigid2d> GetPose2D() const { return amcl_pose_; };
  Eigen::Matrix3d GetCovariance() const { return covariance_; };

  pf_sample_set_t* GetParticleSet() const;

  std::vector<transform::Rigid2d> GetParticlePoses();

  bool IsInitialized() const { return is_initialized_; }

private:
  void InitParticleFilter();
  void InitAMCLModel();

  static pf_vector_t UniformPoseGenerator(void* arg);
  static std::vector<std::pair<int, int>> free_space_indices_;

private:
  const proto::AMCLCoreOptions options_;

  const mapping::Grid2D* grid_ = nullptr;

  std::shared_ptr<sensor::OdometryData> last_odom_data_ = nullptr;

  std::unique_ptr<AMCLOdom> amcl_odom_ = nullptr;
  std::unique_ptr<AMCLCloud> amcl_cloud_ = nullptr;

  bool use_uniform_sampling_ = true;
  pf_t* particle_filter_ = nullptr;

  std::shared_ptr<transform::Rigid2d> amcl_pose_ = nullptr;
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Zero();

  bool is_initialized_ = false;
};

}  // namespace amcl
}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_AMCL_CORE_AMCL_CORE_H_
