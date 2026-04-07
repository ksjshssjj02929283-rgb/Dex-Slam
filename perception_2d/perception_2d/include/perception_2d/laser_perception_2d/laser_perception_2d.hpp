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
#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_HPP_

#include <string>
#include <vector>
#include <utility>
#include <list>
#include <tuple>
#include <mutex>
#include "perception_2d/laser_perception_2d/laser_perception_2d_interface.hpp"
#include "perception_2d/laser_perception_2d/range_data_collator.hpp"
#include "perception_2d/feature_map/feature_map.hpp"
#include "perception_2d/laser_perception_2d/math_tool.hpp"
#include "perception_2d/laser_perception_2d/recg_by_pre_point.hpp"
#include "perception_2d/laser_perception_2d/recg_by_model.hpp"
#include "perception_2d/common/lua_parameter_dictionary.hpp"
#include "perception_2d/laser_perception_2d/rec_by_reflec.hpp"
#include "perception_2d/laser_sync/laser_sync.hpp"
#include "perception_2d/laser_perception_2d/line_detector.hpp"
#include "perception_2d/laser_perception_2d/line_detector_x.hpp"

namespace perception_2d
{
Config CreateLaserPerception2DOption(
  common::LuaParameterDictionary * const
  lua_parameter_dictionary,
  const std::string & model_file_path);

using GlobalPoints = std::vector<std::pair<int, std::vector<cv::Point2d>>>;

class LaserPerception2D : public LaserPerception2DInterface
{
public:
  explicit LaserPerception2D(
    const std::vector<std::string> & expected_range_sensor_ids,
    const Config & config);
  bool AddSensorData(
    const std::string & sensor_id,
    // const sensor::TimedPointCloudDataWithIntensities& timed_point_cloud_data);
    const sensor::PointCloudDataWithIntensities & timed_point_cloud_data);
  bool AddSensorData(
    const std::string & sensor_id,
    const common::Time & time,
    const transform::Rigid3d & sensor_to_tracking,
    const sensor::TimedPointCloudWithIntensities& timed_point_cloud_data);
  bool AddSensorData(
    const std::string & sensor_id,
    const sensor::SyncTimedPointCloudData & range_data_identity);
  void AddSensorData(
    const std::string & sensor_id,
    const sensor::OdometryData & odometry_data);
  void HandleFeatureMapInfoMessage(
    const std::string & topic,
    const std::vector<FeatureInfo> & feature_map);
  void UpdateTransformFromOdomToMap(const transform::Rigid3d & odom_to_map);

  TargetMap GetTargetMsg();

  bool GetLineAndRef(std::vector<std::shared_ptr<FeatureMsgWithOrigin>> & feature_map) override;

  void ResetStatus() override;

  // std::tuple<LinesAndTextInfo, LinesAndTextInfo> GetPrePointVisInfo();

  sensor::SyncPointCloudWithOdom GetSyncPointCloud()
  {
    return update_msg_;
  }

  sensor::OdometryData GetLatestOdom()
  {
    return odom_;
  }

  transform::Rigid3d GetLastestTransFromOdomToMap()
  {
    return odom_in_map_;
  }

  void UpdateParams(const std::string & param_name, double & value);
  void UpdateParams(const std::string & param_name, int & value);
  void UpdateParams(const std::string & param_name, bool & value);

  std::vector<std::pair<int, std::vector<int>>> LaserClustering(
    const sensor::TimedPointCloudOriginData & point_cloud);
  std::vector<std::pair<int, std::vector<int>>> LaserCluster(
    const std::vector<std::pair<int, Eigen::Vector3f>> & msg);

  TargetMap LaserUpdate(
    const sensor::TimedPointCloudOriginData & point_cloud);
  // std::vector<FeatureMsgWithOrigin>
  // std::tuple<std::vector<FeatureMsgWithOrigin>, TargetMap> LaserUpdate(
  //   const sensor::SyncPointCloudWithOdom & msg);
  // std::tuple<std::vector<std::shared_ptr<FeatureMsgWithOrigin>> , TargetMap> 
  // LaserUpdate(
  //       const sensor::SyncPointCloudWithOdom & msg);

  bool LaserUpdate(
    const sensor::SyncPointCloudWithOdom & msg,
    std::vector<std::shared_ptr<FeatureMsgWithOrigin>>& target_map_full,
    TargetMap& pre_model_target_map) override;
  // LinesAndTextInfo pre_info_;
  // bool GetRackInfo(RackInfo & rack_info);

  void clear() override;

private:
  std::vector<std::pair<int, std::vector<cv::Point2d>>> LaserPointToOdom(
    const sensor::TimedPointCloudOriginData & point_cloud,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);

  std::vector<std::pair<int, std::vector<cv::Point2d>>> LaserPointToOdom(
    const sensor::PointCloudWithOrigin & point_cloud,
    std::vector<std::pair<int, std::vector<int>>> laserPointSet);

  std::pair<Eigen::Vector3f, Eigen::Vector3f>
  TransferLine(
    const std::pair<Eigen::Vector3f, Eigen::Vector3f> & line,
    const Eigen::Vector3d & origin);

  FeatureMsg SlideTargetMap(
    const FeatureMsg & feature_msg,
    std::list<FeatureMsg> & slide_vector);
  std::list<GlobalPoints> slide_points;
  int num_accumulated_ = 0;
  // sensor::RangeData accumulated_range_data_;
  Config config_;
  bool predict_rec_, reflect_rec_, vmark_rec_;
  RangeDataCollator range_data_collator_;
  RecgByModel * recg_by_model_;
  RecgByReflec * recg_by_reflec_;
  LineDetector * line_detector_;
  LineDetectorX * line_detector_x_;
  transform::Rigid3d odom_in_map_;
  sensor::SyncPointCloudWithOdom update_msg_;
  sensor::TimedPointCloudOriginData update_point_cloud_;
  sensor::OdometryData odom_;
  std::vector<FeatureInfo> feature_map_;
  TargetMap target_map_;
  std::list<FeatureMsg> pre_point_slide_vector_;
  std::list<FeatureMsg> vmark_sldie_vector_;
  std::vector<std::shared_ptr<FeatureMsgWithOrigin>> line_and_ref_target_map_;
  LaserSync * laser_sync_;

  std::mutex data_mutex_;
};

}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__LASER_PERCEPTION_2D_HPP_
