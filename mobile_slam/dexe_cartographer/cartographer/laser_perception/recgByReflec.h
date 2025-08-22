// Copyright (c) 2021 Geekplus Inc
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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__RECGBYREFLEC_H_
#define CARTOGRAPHER__LASER_PERCEPTION__RECGBYREFLEC_H_
#include <vector>
#include <utility>
#include "cartographer/laser_perception/model_manager.h"
#include "cartographer/laser_perception/proto/laser_perception_options.pb.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "glog/logging.h"

namespace cartographer
{
namespace laser_perception
{
class RecgByReflec
{
public:
  explicit RecgByReflec(const proto::LaserPerceptionOptions & options);
  std::vector<FeatureMsg> getReflector(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);
  std::vector<FeatureMsg> getShapePose(
    const sensor::PointCloudWithOrigin & msg,
    const sensor::OdometryData odom,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);
  std::vector<FeatureMsg> getShapePoseInLaser(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet,
    std::vector<std::vector<int>> & point_set_r,
    std::vector<Line> & lines_r,
    std::vector<cv::Point2d> & vis_input_point);
  std::vector<FeatureMsg> getShelfPoseInLaser(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet,
    std::vector<std::vector<int>> & point_set_r,
    std::vector<Line> & lines_r,
    std::vector<cv::Point2d> & vis_input_point);
  std::vector<FeatureMsg> getTargetByPredictPose(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet,
    transform::Rigid3d predict_target_pose,
    std::vector<std::vector<int>> & point_set_r,
    std::vector<Line> & lines_r,
    std::vector<cv::Point2d> & vis_input_point);
  std::vector<FeatureMsg> getFeaturePoseFromPredict(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet,
    std::vector<std::vector<int>> & point_set_r,
    std::vector<Line> & lines_r,
    std::vector<cv::Point2d> & vis_input_point);
  double getTargetPosition(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition,
    std::vector<cv::Point2d> & vis_input_point);
  double getTargetPositionOriginal(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition,
    std::vector<cv::Point2d> & vis_input_points);
  double getTargetPosition_tmp(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition);
  double getTargetPosition_first(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition,
    std::vector<cv::Point2d> & vis_points);
  double findInitTransPose(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition);
  double findCorner(
    const std::vector<cv::Point2d> selectTargetPoints,
    const std::vector<Line> lines,
    cv::Point3d & targetPosition, cv::Point2d & corner_in_line,
    cv::Point2d & corner_in_points);
  double findBestAngleOfSet(
    const std::vector<cv::Point2d> selectTargetPoints,
    Model & model,
    const cv::Point3d & targetPosition,
    std::vector<Line> & return_lines,
    cv::Point3d & return_target_position);
  std::vector<FeatureMsg> testICP(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::vector<int>> & select_points_set,
    std::vector<Line> & lines,
    Model & model);
  std::vector<FeatureMsg> testSetMatching(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::vector<int>> & select_points_set,
    std::vector<Line> & lines,
    Model & model,
    std::vector<cv::Point2d> & vis_input_point);
  std::vector<FeatureMsg> testMatchingUseDisLimit(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::vector<int>> & select_points_set,
    std::vector<Line> & lines,
    Model & model);
  std::vector<FeatureMsg> testSearchBestSet(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::vector<int>> & select_points_set,
    std::vector<Line> & lines,
    Model & model,
    std::vector<cv::Point2d> & vis_input_point);
  std::vector<FeatureMsg> testShelfRec(
    const sensor::PointCloudWithOrigin & msg,
    std::vector<std::vector<int>> & select_points_set,
    std::vector<Line> & lines,
    Model & model,
    std::vector<cv::Point2d> & vis_input_point);
  double ComputeScoreOfAngle(
    const std::vector<cv::Point2d> & points,
    const std::vector<Line> & lines);
  bool ComputePointSetDis(const std::vector<std::vector<cv::Point2d>> & points);
  std::vector<FeatureMsg> SlideShapeStruct(const std::vector<FeatureMsg> & target_map);

private:
  double reflector_error_;
  double reflector_radius_;
  double shape_match_score_;
  double shape_match_close_degree_;
  double shape_dis_to_robot_center_;
  double reflector_error_range_;
  ModelManager * model_manager_;
};
}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__RECGBYREFLEC_H_
