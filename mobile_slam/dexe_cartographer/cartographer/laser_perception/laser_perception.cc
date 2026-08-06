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

#include "cartographer/laser_perception/laser_perception.h"

#include <cmath>
#include <list>
#include <utility>
#include <vector>

namespace cartographer
{
namespace laser_perception
{

LaserPerception::LaserPerception(const proto::LaserPerceptionOptions & options)
  : min_cluster_points_(options.min_cluster_points()),
    min_range_(options.min_range()),
    max_range_(options.max_range()),
    highlight_intensity_threshold_(options.highlight_intensity_threshold()),
    cluster_distance_threshold_(options.cluster_distance_threshold())
{
  lineDetectorX_ = new LineDetectorX();
  const auto & line_options = options.line_detector();
  lineDetectorX_->set_least_threshold(line_options.least_thresh());
  lineDetectorX_->set_min_line_length(line_options.min_line_length());
  lineDetectorX_->set_predict_distance(line_options.predict_distance());
  lineDetectorX_->set_seed_line_points(line_options.seed_line_points());
  lineDetectorX_->set_min_line_points(line_options.min_line_points());
  flatCloudDetector_ = new FlatCloudDetector();
}

LaserPerception::~LaserPerception()
{
  delete lineDetectorX_;
  delete flatCloudDetector_;
}

std::vector<std::pair<int, std::vector<int>>> LaserPerception::laserCluster(
  const std::vector<std::pair<int, Eigen::Vector3f>> & msg)
{
  std::vector<int> highlightPointSet;
  std::vector<int> darkPointSet;
  std::vector<std::pair<int, std::vector<int>>> pointSetList;
  for (size_t i = 0; i < msg.size(); i++) {
    double range = msg[i].second.norm();
    if (range > max_range_ || range < min_range_) {
      continue;
    }
    if (msg[i].first < highlight_intensity_threshold_) {
      double dark_range;
      if (darkPointSet.size() > 0) {
        dark_range = std::sqrt(
          msg[darkPointSet.back()].second[0] * msg[darkPointSet.back()].second[0] +
          msg[darkPointSet.back()].second[1] * msg[darkPointSet.back()].second[1]);
      }
      if (darkPointSet.size() > 0 &&
        (msg[darkPointSet.back()].second - msg[i].second).norm() >=
        cluster_distance_threshold_ + dark_range * std::sin(0.00224))
      {
        if (static_cast<int>(darkPointSet.size()) > min_cluster_points_) {
          if (static_cast<int>(highlightPointSet.size()) > min_cluster_points_) {
            if (highlightPointSet.back() < darkPointSet.front()) {
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
        (msg[highlightPointSet.back()].second - msg[i].second).norm() >= 900)
      {
        if (static_cast<int>(highlightPointSet.size()) > min_cluster_points_) {
          if (static_cast<int>(darkPointSet.size()) > min_cluster_points_) {
            if (highlightPointSet.back() < darkPointSet.front()) {
              pointSetList.push_back(std::make_pair(2, highlightPointSet));
              pointSetList.push_back(std::make_pair(1, darkPointSet));
            } else {
              pointSetList.push_back(std::make_pair(1, darkPointSet));
              pointSetList.push_back(std::make_pair(2, highlightPointSet));
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

  if (static_cast<int>(darkPointSet.size()) > min_cluster_points_) {
    pointSetList.push_back(std::make_pair(1, darkPointSet));
  }
  if (static_cast<int>(highlightPointSet.size()) > min_cluster_points_) {
    pointSetList.push_back(std::make_pair(2, highlightPointSet));
  }
  return pointSetList;
}

std::vector<FeatureMsgWithOrigin> LaserPerception::laserUpdate(
  const sensor::PointCloudWithOrigin & cloud)
{
  std::vector<FeatureMsgWithOrigin> target_map;
  sensor::PointCloudWithOrigin point_cloud = cloud;
  std::vector<std::pair<int, Eigen::Vector3f>> point_cloud_in_laser;
  for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
    Eigen::Vector3f point_base_link = point_cloud.sync_points_base_link[i];
    Eigen::Vector3d origin = point_cloud.origin;
    Eigen::Vector3f point;
    point << point_base_link[0] * std::cos(origin[2]) + point_base_link[1] * std::sin(origin[2]) -
    (origin[0] * std::cos(origin[2]) + origin[1] * std::sin(origin[2])),
      -point_base_link[0] * std::sin(origin[2]) + point_base_link[1] * std::cos(origin[2]) -
    (-origin[0] * std::sin(origin[2]) + origin[1] * std::cos(origin[2])),
      0;
    point_cloud_in_laser.push_back(
      std::pair<int, Eigen::Vector3f>(
        point_cloud.intensities[i],
        point));
    point_cloud.sync_points_base_link[i] = point;
  }
  std::vector<std::pair<int, std::vector<int>>> laserPointSet = laserCluster(point_cloud_in_laser);

  point_cloud_highlight_ = std::make_shared<sensor::PointCloud>();
  for (const auto & point_set : laserPointSet) {
    if (point_set.first == 2) {
      for (const auto ind : point_set.second) {
        sensor::PointCloud::PointType p;
        p.position = point_cloud_in_laser[ind].second;
        point_cloud_highlight_->push_back(p);
      }
    }
  }

  sensor::PointCloud point_cloud_flat;
  point_cloud_less_flat_ = std::make_shared<sensor::PointCloud>();
  flatCloudDetector_->DetectFlatCloud(point_cloud, point_cloud_flat, *point_cloud_less_flat_);

  std::list<Point> input_points;
  for (const auto & item : laserPointSet) {
    for (const auto index : item.second) {
      input_points.push_back(
        Point(
          point_cloud_in_laser[index].second[0],
          point_cloud_in_laser[index].second[1]));
    }
  }
  std::list<Segment> lines = lineDetectorX_->detectLines(input_points);

  FeatureMsgWithOrigin line_features;
  line_features.origin = point_cloud.origin;
  for (const auto & line : lines) {
    FeatureMsg feature;
    feature.target_name = "line";
    feature.line = std::make_pair(
      Eigen::Vector3f(line.first_point.x, line.first_point.y, 0),
      Eigen::Vector3f(line.last_point.x, line.last_point.y, 0));
    line_features.MsgVec.push_back(feature);
  }
  target_map.push_back(line_features);

  return target_map;
}

}  // namespace laser_perception
}  // namespace cartographer