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

#ifndef CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_
#define CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include "cartographer/mapping/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/mapping/internal/range_data_collator.h"
#include "cartographer/metrics/family_factory.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/preprocess/data_preprocessor.h"
#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/laser_perception.h"
#include "cartographer/localization/proto/pose_initializer_options_2d.pb.h"
#include "cartographer/localization/2d/grid_map_2d.h"
#include "cartographer/localization/2d/cloud_map_2d.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/ikd_tree/ikd_tree.h"

namespace cartographer {
namespace localization {

inline double normalized_angle(const double angle)
{
  double res = angle;
  while (res > M_PI) {
    res -= 2.0 * M_PI;
  }
  while (res <= -M_PI) {
    res += 2.0 * M_PI;
  }
  return res;
}

enum class PoseInitializerStatus {
  NotInitialized = -2,
  Failed = -1,
  Idle = 0,
  LocalEstimating = 1,     // 局部重定位中
  GlobalEstimating = 2,    // 全局重定位中
  Success = 3
};

// Wires up the local SLAM stack (i.e. pose extrapolator, scan matching, etc.)
// without loop closure.
// TODO(gaschler): Add test for this class similar to the 3D test.
class PoseInitializer {
 public:
  explicit PoseInitializer(
	  const proto::PoseInitializerOptions2D& options);
  ~PoseInitializer();

  PoseInitializer(const PoseInitializer&) = delete;
  PoseInitializer& operator=(const PoseInitializer&) = delete;

  void Initialize();
  void ResetStatus();

  void InitGlobalGridMap(const std::shared_ptr<GridMap2D>& grid_map);
  void InitGlobalCloudMap(const std::shared_ptr<CloudMap2D>& cloud_map);
  void InitGlobalLineMap(const std::shared_ptr<mapping::LineMap2DList>& line_map);

  bool SetInitialPose(const transform::Rigid2d& initial_pose);

  // 触发全局重定位，不需要初始位姿
  void TriggerGlobalRelocalization();

  // 加入点云数据和线特征数据
  void UpdateSensorData(const sensor::PointCloud& scan_cloud,
                     const std::vector<sensor::LineData>& line_data);
  const PoseInitializerStatus GetInitializerStatus() const { return initializer_status_; }

  std::shared_ptr<transform::Rigid2d> GetEstimatedPose();

  void start();
  void stop();

 private:
  void Run();
  void RelocalzationOnce();
  void GlobalRelocalzationOnce();
  void CreateRelocalzationThread();

  bool IsRelocalizationSuccess();  bool IsPoseValid(const transform::Rigid2d& pose,
                   const transform::Rigid2d& target,
                   const double dist_th = 0.5,
                   const double yaw_th = 0.765 /*45 degrees*/);

private:
  const proto::PoseInitializerOptions2D options_;

  std::shared_ptr<GridMap2D> global_grid_map_ = nullptr;
  std::shared_ptr<CloudMap2D> global_cloud_map_ = nullptr;
  std::shared_ptr<ikd_tree::iKdTree<pcl::PointXYZ>> global_cloud_ikdtree_ = nullptr;
  std::shared_ptr<mapping::LineMap2DList> global_line_map_ = nullptr;

  std::shared_ptr<transform::Rigid2d> initial_pose_;
  std::shared_ptr<transform::Rigid2d> global_pose_estimate_;
  bool need_apply_initial_pose_ = false;
  bool need_global_relocalization_ = false;  // 标识需要进行全局重定位

  bool working_ = false;

  std::unique_ptr<std::thread> relocalization_thread_;
  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  std::shared_ptr<sensor::PointCloud> curr_cloud_;
  std::shared_ptr<std::vector<sensor::LineData>> curr_line_data_;

  std::unique_ptr<mapping::scan_matching::FastCorrelativeScanMatcher2D>
  fast_correlative_scan_matcher_;

  std::unique_ptr<mapping::scan_matching::RealTimeCorrelativeScanMatcher2D>
  real_time_correlative_scan_matcher_;

  std::unique_ptr<mapping::scan_matching::CeresScanMatcher2D>
  ceres_scan_matcher_;

  std::unique_ptr<mapping::scan_matching::LineMatcher2D> line_matcher_;

  PoseInitializerStatus initializer_status_ = PoseInitializerStatus::NotInitialized;
};

}  // namespace localization
}  // namespace localization

#endif  // CARTOGRAPHER_LOCALIZATION_INTERNAL_2D_POSE_INITIALIZER_2D_H_
