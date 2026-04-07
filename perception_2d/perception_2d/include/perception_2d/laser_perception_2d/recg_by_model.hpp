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

#ifndef  PERCEPTION_2D__LASER_PERCEPTION_2D__RECG_BY_MODEL_HPP_
#define  PERCEPTION_2D__LASER_PERCEPTION_2D__RECG_BY_MODEL_HPP_
#include <opencv2/opencv.hpp>
#include <mutex>
#include <utility>
#include <vector>
#include <string>
#include "perception_2d/laser_perception_2d/config.hpp"
#include "perception_2d/laser_perception_2d/model_manager.hpp"
#include "perception_2d/laser_perception_2d/model_cloud_generator.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "perception_2d/transform/transform.hpp"
#include "perception_2d/sensor/odometry_data.hpp"
#include "perception_2d/sensor/timed_point_cloud_data.hpp"
#include "perception_2d/laser_perception_2d/visualization.hpp"

namespace perception_2d
{

class RecgByModel
{
// struct FirstSearchParam
// {
//   double search_dist;
//   double search_angle;
//   double dist_resolution = 0.2;
//   double angle_resolution = 0.03;
// };

public:
  explicit RecgByModel(const Config config);
  PrePointMsg  GetTargetByPredictPose(
    std::vector<std::pair<int,
    std::vector<cv::Point2d>>> globalPointSet);
  PrePointMsg GetChargeStationPose(
    std::vector<std::pair<int,
    std::vector<cv::Point2d>>> globalPointSet);
  PrePointMsg GetChargeStationPoseByPredict(
    const cv::Point3d& predict_pose,
    cv::Point3d& result_pose,
    std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet);

  double GetTargetPosition(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines, cv::Point3d & targetPosition);

  bool UpdateOdomData(const sensor::OdometryData & odom);

  bool UpdateOdomToMapTrans(const transform::Rigid3d & odom_in_map);

  void UpdateParams(const std::string & param_name, double & value);

  LinesAndTextInfo feature_map_vis_info_;
  LinesAndTextInfo target_map_vis_info_;
  LinesAndTextInfo pre_info_;
  RackInfo rack_info_;

private:
  std::vector<FeatureInfo> target_list_;
  ModelManager * model_manager_;
  double search_distance_;
  double max_dis_error_;
  double max_ang_error_;
  double start_rec_dis_;
  int min_select_points_;
  transform::Rigid3d odom_in_map_;
  sensor::OdometryData odom_;
  std::string model_file_;
  std::mutex update_target_list_mutex_;

  bool is_first_search_ = true;
  cv::Point3d last_target_pose_;
  // Visualization visualization_;
};

}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__RECG_BY_MODEL_HPP_
