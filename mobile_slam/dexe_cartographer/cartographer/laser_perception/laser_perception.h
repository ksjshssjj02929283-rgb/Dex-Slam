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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_
#define CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_
#include <vector>
#include <mutex>
#include <cstdio>
#include <utility>
#include <list>

#include "opencv2/opencv.hpp"
#include "cartographer/laser_perception/laser_perception_options.h"
#include "cartographer/laser_perception/line_detector.h"
#include "cartographer/laser_perception/line_detector_x.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/laser_perception/data_type.h"
#include "cartographer/laser_perception/model_manager.h"
#include "cartographer/laser_perception/recgByReflec.h"
// #include "nlohmann/json.h"
#include "cartographer/laser_perception/angle_dectector.h"
#include "cartographer/laser_perception/flat_cloud_detector.h"
#include "cartographer/laser_perception/proto/laser_perception_options.pb.h"
// using Json = nlohmann::json;

namespace cartographer
{
namespace laser_perception
{

struct FeatureMsgWithOrigin
{
  Eigen::Vector3d origin;
  std::vector<FeatureMsg> MsgVec;
};
// struct AngleStructCandidate{
//   std::pair<Eigen::Vector3f, Eigen::Vector3f> first_line;
//   std::pair<Eigen::Vector3f, Eigen::Vector3f> second_line;
// };

// struct AngleStruct{
//   Eigen::Vector3d origin;
//   Eigen::Vector3d pose;
//   std::pair<Eigen::Vector3f, Eigen::Vector3f> first_line;
//   std::pair<Eigen::Vector3f, Eigen::Vector3f> second_line;
//   double angle;
// };
class LaserPerception
{
public:
  explicit LaserPerception(const proto::LaserPerceptionOptions & options);
  ~LaserPerception();
  std::vector<std::pair<int, std::vector<int>>> laserCluster(
    const std::vector<std::pair<int, Eigen::Vector3f>> & msg);
  std::vector<FeatureMsgWithOrigin> laserUpdate(const sensor::SyncPointCloudWithOdom & msg);
  std::vector<FeatureMsgWithOrigin> laserUpdateWithPredictPose(
    const sensor::SyncPointCloudWithOdom & msg);
  std::vector<AngleStructCandidate> computeAllPossibleStruct(std::list<Segment> lines);
  double computeMinDisBetweenLines(AngleStructCandidate candidate);
  bool detectAngleStruct(const AngleStructCandidate & candidate, AngleStruct & angle_struct);
  bool detectAngle(
    const Eigen::Vector2d & inter_point,
    const AngleStructCandidate & candidate,
    double & pose_angle,
    double & angle);
  // std::vector<FeatureMsg> SlideShapeStruct(const std::vector<FeatureMsg>& target_map);

  std::shared_ptr<sensor::PointCloud> getHighlightCloud() const { return point_cloud_highlight_; }
  std::shared_ptr<sensor::PointCloud> getFlatCloud() const { return point_cloud_flat_; }
  std::shared_ptr<sensor::PointCloud> getLessFlatCloud() const { return point_cloud_less_flat_; }

private:
  cv::KalmanFilter * KF_chargeStation;
  cv::KalmanFilter * KF_shelf;
  Config config_;
  bool predict_rec_;
  bool shape_rec_;
  bool reflect_rec_;
  bool line_rec_;
  bool angle_rec_;
  double intensity_threshold_;
  double connect_threshold_;
  int intensity_max_threshold_;
  double intensity_max_range_threshold_;
  double intensity_min_range_threshold_;
  int min_cluster_points_;
  int laser_intensity_type_;
  double angle_increment_;

  LineDetector * lineDetector_;
  LineDetectorX * lineDetectorX_;

  RecgByReflec * recgByReflec_;
  AngleDectector * angleDectector_;

  FlatCloudDetector * flatCloudDetector_;

  std::shared_ptr<sensor::PointCloud> point_cloud_flat_;
  std::shared_ptr<sensor::PointCloud> point_cloud_less_flat_;

  std::shared_ptr<sensor::PointCloud> point_cloud_highlight_;

public:
  transform::Rigid3d map_to_odom;
  std::vector<FeatureInfo> target_list_;
  // TFManager tf_manager_;
};
}  // namespace laser_perception
}  // cartographer


#endif  // CARTOGRAPHER__LASER_PERCEPTION__LASER_PERCEPTION_H_
