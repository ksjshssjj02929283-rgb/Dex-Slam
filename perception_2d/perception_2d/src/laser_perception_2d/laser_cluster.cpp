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
#include <utility>
#include <list>
#include "perception_2d/laser_perception_2d/laser_cluster.hpp"

namespace perception_2d
{

// Config CreateLaserPerception2DOption(
//   common::LuaParameterDictionary * const
//   lua_parameter_dictionary,
//   const std::string & model_file_path)
// {
//   Config config;
//   config.predict_rec = lua_parameter_dictionary->GetBool("predict_rec");
//   config.reflect_rec = lua_parameter_dictionary->GetBool("reflect_rec");
//   config.line_rec = lua_parameter_dictionary->GetBool("line_rec");
//   config.shape_rec = lua_parameter_dictionary->GetBool("shape_rec");
//   config.vmark_rec = lua_parameter_dictionary->GetBool("vmark_rec");
//   config.intensity_threshold = lua_parameter_dictionary->GetInt("intensities_min");
//   config.search_distance = lua_parameter_dictionary->GetDouble("search_distance");
//   config.max_ang_error = lua_parameter_dictionary->GetDouble("search_angle");
//   config.max_dis_error = lua_parameter_dictionary->GetDouble("max_error");
//   config.min_select_points = lua_parameter_dictionary->GetInt("min_select_points");
//   config.min_cluster_points = lua_parameter_dictionary->GetInt("min_cluster_points");
//   config.reflector_error = lua_parameter_dictionary->GetDouble("reflector_error");
//   config.reflector_radius = lua_parameter_dictionary->GetDouble("reflector_radius");
//   config.range_max = lua_parameter_dictionary->GetDouble("range_max");
//   config.range_min = lua_parameter_dictionary->GetDouble("range_min");
//   config.start_rec_dis = lua_parameter_dictionary->GetDouble("start_rec_dis");
//   config.model_file = model_file_path;
//   LOG(INFO) << "config.model_file = " << config.model_file;
//   return config;
// }

LaserCluster::LaserCluster(
  const std::vector<std::string> & expected_range_sensor_ids,
  const Config & config)
: config_(config), range_data_collator_(expected_range_sensor_ids)
{
  predict_rec_ = config_.predict_rec;
  reflect_rec_ = config_.reflect_rec;
  vmark_rec_ = config_.vmark_rec;
  laser_sync_ = new LaserSync();
  recg_by_pre_point_ = new RecgByPrePoint(config_);
  recg_by_reflec_ = new RecgByReflec(config_);
  line_detector_ = new LineDetector(config_);
}

bool LaserCluster::AddSensorData(
  const std::string & sensor_id,
  const sensor::PointCloudDataWithIntensities & timed_point_cloud_data)
{
  // const sensor::TimedPointCloudData& timed_point_cloud_data){
  auto synchronized_data =
    range_data_collator_.AddRangeData(sensor_id, timed_point_cloud_data);
  if (synchronized_data.ranges.empty()) {
    LOG(INFO) << "Range data collator filling buffer.";
    return false;
  }
  // LOG(INFO) << "target_map " << target_map.size();
  return true;
}

bool LaserCluster::AddSensorData(
  const std::string & sensor_id,
  const sensor::SyncTimedPointCloudData & range_data_identity)
{
  sensor::SyncPointCloudWithOdom sync_point_cloud;
  std::vector<std::shared_ptr<FeatureMsgWithOrigin>> target_map;
  target_map.resize(0);
  if (laser_sync_->GetSyncPointCloud(
      sensor_id, range_data_identity, sync_point_cloud))
  {
    line_and_ref_target_map_ = target_map;
  }

  return true;
}

void LaserCluster::AddSensorData(
  const std::string & sensor_id,
  const sensor::OdometryData & odometry_data)
{
  // LOG(INFO) << "odom_ x = " << odometry_data.pose.translation()[0]
  //           << " y = " << odometry_data.pose.translation()[1];
  laser_sync_->AddOdometryData(odometry_data);
  recg_by_pre_point_->UpdateOdomData(odometry_data);
  odom_ = odometry_data;
}

void LaserCluster::UpdateTransformFromOdomToMap(const transform::Rigid3d & odom_to_map)
{
  recg_by_pre_point_->UpdateOdomToMapTrans(odom_to_map);
  odom_in_map_ = odom_to_map;
}

void LaserCluster::HandleFeatureMapInfoMessage(
  const std::string & topic,
  const std::vector<FeatureInfo> & feature_map)
{
  recg_by_pre_point_->UpdateTargetList(feature_map);
  feature_map_ = feature_map;
}

std::vector<std::pair<int, std::vector<int>>> LaserCluster::LaserClustering(
  const sensor::TimedPointCloudOriginData & point_cloud)
{
  // 对激光点云进行聚类
  std::vector<int> highlightPointSet;
  std::vector<int> darkPointSet;
  std::vector<std::pair<int, std::vector<int>>> pointSetList;

  for (size_t i = 0; i < point_cloud.ranges.size(); i++) {
    double range = point_cloud.ranges[i].point_time.position.norm();
    if (range > config_.range_max || range < config_.range_min) {
      continue;
    }

    if (point_cloud.ranges[i].intensity < config_.intensity_threshold) {
      double dark_range = 0;
      if (darkPointSet.size() > 0) {
        dark_range = point_cloud.ranges[darkPointSet.back()].point_time.position.norm();
      }
      if (darkPointSet.size() > 0 &&
        chordLength_point(
          point_cloud.ranges[darkPointSet.back()].point_time.position,
          point_cloud.ranges[i].point_time.position) >=
        config_.connect_threshold + dark_range * sin(0.00224))
      {
        if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
          if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
            if (static_cast<int>(highlightPointSet.back()) < darkPointSet.front()) {
              pointSetList.push_back(std::make_pair(2, highlightPointSet));
              pointSetList.push_back(std::make_pair(1, darkPointSet));
            } else {
              pointSetList.push_back(std::make_pair(1, darkPointSet));
              pointSetList.push_back(std::make_pair(2, highlightPointSet));
            }
          } else {
            pointSetList.push_back(std::make_pair(1, darkPointSet));
          }
          highlightPointSet.clear();
        }
        darkPointSet.clear();
      }
      darkPointSet.push_back(i);
    } else {
      if (highlightPointSet.size() > 0 &&
        chordLength_point(
          point_cloud.ranges[highlightPointSet.back()].point_time.position,
          point_cloud.ranges[i].point_time.position) >= config_.connect_threshold)
      {
        if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
          if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
            if (highlightPointSet.back() < darkPointSet.front()) {
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
              pointSetList.push_back(std::make_pair(1, darkPointSet));
            } else {
              pointSetList.push_back(std::make_pair(1, darkPointSet));
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
            }
          } else {
            pointSetList.push_back(std::make_pair(2, highlightPointSet));
          }
          darkPointSet.clear();
        }
        highlightPointSet.clear();
      }
      highlightPointSet.push_back(i);
    }
  }

  if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
    pointSetList.push_back(std::make_pair(1, darkPointSet));
  }
  if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
    pointSetList.push_back(std::make_pair(2, highlightPointSet));
  }

  return pointSetList;
}

std::vector<std::pair<int, std::vector<int>>> LaserCluster::LaserClustering(
  const std::vector<std::pair<int, Eigen::Vector3f>> & msg)
{
  // 对激光点云进行聚类
  std::vector<int> highlightPointSet;
  std::vector<int> darkPointSet;
  std::vector<std::pair<int, std::vector<int>>> pointSetList;
  for (size_t i = 0; i < msg.size(); i++) {
    double range = msg[i].second.norm();
    if (range > config_.range_max || range < config_.range_min) {
      continue;
    }
    if (msg[i].first < config_.intensity_threshold) {
      double dark_range;
      if (darkPointSet.size() > 0) {
        dark_range = sqrt(
          msg[darkPointSet.back()].second[0] * msg[darkPointSet.back()].second[0] +
          msg[darkPointSet.back()].second[1] * msg[darkPointSet.back()].second[1]);
      }
      if (darkPointSet.size() > 0 &&
        chordLength_point(
          msg[darkPointSet.back()].second,
          msg[i].second) >= config_.connect_threshold + dark_range * sin(0.00224))
      {
        if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
          if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
            if (highlightPointSet.back() < darkPointSet.front()) {
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
              pointSetList.push_back(std::make_pair(1, darkPointSet));
            } else {
              pointSetList.push_back(std::make_pair(1, darkPointSet));
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
            }
          } else {
            pointSetList.push_back(std::make_pair(1, darkPointSet));
          }
          highlightPointSet.clear();
        }
        darkPointSet.clear();
      }
      darkPointSet.push_back(i);
    } else {
      if (highlightPointSet.size() > 0 &&
        chordLength_point(
          msg[highlightPointSet.back()].second,
          msg[i].second) >= config_.connect_threshold)
      {
        if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
          if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
            if (highlightPointSet.back() < darkPointSet.front()) {
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
              pointSetList.push_back(std::make_pair(1, darkPointSet));
            } else {
              pointSetList.push_back(std::make_pair(1, darkPointSet));
              pointSetList.push_back(std::make_pair(2, highlightPointSet));  // 2
            }
          } else {
            pointSetList.push_back(std::make_pair(2, highlightPointSet));
          }
          darkPointSet.clear();
        }
        highlightPointSet.clear();
      }
      highlightPointSet.push_back(i);
    }
  }

  if (static_cast<int>(darkPointSet.size()) > config_.min_cluster_points) {
    pointSetList.push_back(std::make_pair(1, darkPointSet));
  }
  if (static_cast<int>(highlightPointSet.size()) > config_.min_cluster_points) {
    pointSetList.push_back(std::make_pair(2, highlightPointSet));
  }
  return pointSetList;
}

std::vector<std::pair<int, std::vector<cv::Point2d>>> LaserCluster::LaserPointToOdom(
  const sensor::TimedPointCloudOriginData & point_cloud,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet;
  for (auto item : laserPointSet) {
    std::vector<cv::Point2d> pointSet;
    for (auto index : item.second) {
      Eigen::Vector3f point_b = point_cloud.ranges[index].point_time.position;
      Eigen::Vector3f point_o = odom_.pose.cast<float>() * point_b;
      pointSet.push_back(cv::Point2d(point_o[0], point_o[1]));
    }
    globalPointSet.push_back(std::make_pair(item.first, pointSet));
  }
  return globalPointSet;
}


TargetMap LaserCluster::GetTargetMsg()
{
  return target_map_;
}

bool LaserCluster::GetLineAndRef(std::vector<std::shared_ptr<FeatureMsgWithOrigin>> & feature_map)
{
  feature_map = line_and_ref_target_map_;
  return true;
}

}  // namespace perception_2d
