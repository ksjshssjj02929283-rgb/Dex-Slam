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

#include "cartographer/laser_perception/laser_perception.h"

#include <math.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <list>

#include "cartographer/laser_perception/util.h"
#include "cartographer/laser_perception/math_tool.h"
namespace cartographer
{
namespace laser_perception
{
LaserPerception::LaserPerception(const proto::LaserPerceptionOptions & options)
{
  KF_chargeStation = new cv::KalmanFilter(6, 3, 0);
  KF_chargeStation->transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1);
  setIdentity(KF_chargeStation->measurementMatrix);
  setIdentity(KF_chargeStation->processNoiseCov, cv::Scalar::all(1e-4));
  setIdentity(KF_chargeStation->measurementNoiseCov, cv::Scalar::all(1e-2));
  setIdentity(KF_chargeStation->errorCovPost, cv::Scalar::all(1));

  KF_shelf = new cv::KalmanFilter(6, 3, 0);
  KF_shelf->transitionMatrix = (cv::Mat_<float>(6, 6) << 1, 0, 0, 1, 0, 0,
    0, 1, 0, 0, 1, 0,
    0, 0, 1, 0, 0, 1,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1);
  setIdentity(KF_shelf->measurementMatrix);
  setIdentity(KF_shelf->processNoiseCov, cv::Scalar::all(1e-2));
  setIdentity(KF_shelf->measurementNoiseCov, cv::Scalar::all(1e-4));
  setIdentity(KF_shelf->errorCovPost, cv::Scalar::all(1));
  predict_rec_ = options.predict_rec();
  shape_rec_ = options.shape_rec();
  reflect_rec_ = options.reflect_rec();
  line_rec_ = options.line_rec();
  angle_rec_ = options.angle_rec();

  intensity_threshold_ = options.intensity_threshold();
  connect_threshold_ = options.connect_threshold();
  min_cluster_points_ = options.min_cluster_points();
  intensity_max_threshold_ = options.intensity_max_threshold();
  intensity_max_range_threshold_ = options.intensity_max_range_threshold();
  intensity_min_range_threshold_ = options.intensity_min_range_threshold();
  laser_intensity_type_ = options.laser_intensity_type();
  angle_increment_ = options.angle_increment();

  lineDetector_ = new LineDetector(options);
  lineDetectorX_ = new LineDetectorX();
  recgByReflec_ = new RecgByReflec(options);
  angleDectector_ = new AngleDectector();
  flatCloudDetector_ = new FlatCloudDetector();
}

LaserPerception::~LaserPerception()
{
  if (lineDetector_ != nullptr) {
    delete lineDetector_;
    lineDetector_ = nullptr;
  }
  if (lineDetectorX_ != nullptr) {
    delete lineDetectorX_;
    lineDetectorX_ = nullptr;
  }
  if (recgByReflec_ != nullptr) {
    delete recgByReflec_;
    recgByReflec_ = nullptr;
  }
  if (angleDectector_ != nullptr) {
    delete angleDectector_;
    angleDectector_ = nullptr;
  }
  if (flatCloudDetector_ != nullptr) {
    delete flatCloudDetector_;
    flatCloudDetector_ = nullptr;
  }
  if (KF_chargeStation != nullptr) {
    delete KF_chargeStation;
    KF_chargeStation = nullptr;
  }
  if (KF_shelf != nullptr) {
    delete KF_shelf;
    KF_shelf = nullptr;
  }
}

std::vector<std::pair<int, std::vector<int>>> LaserPerception::laserCluster(
  const std::vector<std::pair<int, Eigen::Vector3f>> & msg)
{
  // 对激光点云进行聚类
  std::vector<int> highlightPointSet;
  std::vector<int> darkPointSet;
  std::vector<std::pair<int, std::vector<int>>> pointSetList;
  for (size_t i = 0; i < msg.size(); i++) {
    double range = msg[i].second.norm();
    if (range > 10 || range < 0.1) {
      continue;
    }
    if (msg[i].first < 900) {
      double dark_range;
      if (darkPointSet.size() > 0) {
        dark_range = sqrt(
          msg[darkPointSet.back()].second[0] * msg[darkPointSet.back()].second[0] +
          msg[darkPointSet.back()].second[1] * msg[darkPointSet.back()].second[1]);
      }
      if (darkPointSet.size() > 0 &&
        chordLength_point(
          msg[darkPointSet.back()].second,
          msg[i].second) >= 0.05 + dark_range * sin(0.00224))
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
          msg[i].second) >= 900)
      {
        if (static_cast<int>(highlightPointSet.size()) > 10) {
          if (static_cast<int>(darkPointSet.size()) > 10) {
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

  if (static_cast<int>(darkPointSet.size()) > 10) {
    pointSetList.push_back(std::make_pair(1, darkPointSet));
  }
  if (static_cast<int>(highlightPointSet.size()) > 10) {
    pointSetList.push_back(std::make_pair(2, highlightPointSet));
  }
  return pointSetList;
}

std::vector<FeatureMsgWithOrigin> LaserPerception::laserUpdateWithPredictPose(
  const sensor::SyncPointCloudWithOdom & msg)
{
  std::vector<FeatureMsgWithOrigin> target_map;
  for (size_t i = 0; i < msg.sync_point_clouds.size(); i++) {
    sensor::PointCloudWithOrigin point_cloud = msg.sync_point_clouds[i];
    std::vector<std::pair<int, Eigen::Vector3f>> point_cloud_in_laser;
    for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
      Eigen::Vector3f point_base_link = point_cloud.sync_points_base_link[i];
      Eigen::Vector3d origin = point_cloud.origin;
      Eigen::Vector3f point;
      point << point_base_link[0] * cos(origin[2]) + point_base_link[1] * sin(origin[2]) -
      (origin[0] * cos(origin[2]) + origin[1] * sin(origin[2])),
        -point_base_link[0] * sin(origin[2]) + point_base_link[1] * cos(origin[2]) -
      (-origin[0] * sin(origin[2]) + origin[1] * cos(origin[2])),
        0;
      point_cloud_in_laser.push_back(
        std::pair<int, Eigen::Vector3f>(
          point_cloud.intensities[i],
          point));
      point_cloud.sync_points_base_link[i] = point;
    }
    std::vector<std::pair<int, std::vector<int>>>
      laserPointSet = laserCluster(point_cloud_in_laser);
      
    // }
    std::vector<FeatureInfo> target_list;
    target_list = target_list_;
    // LOG(INFO) << "TARGET_LIST SIZE = " << target_list.size();
    for (auto target : target_list) {
      transform::Rigid3d robot_odom = msg.odom.pose;
      transform::Rigid3d map_to_odom_ = map_to_odom;
      transform::Rigid3d robot_pose = map_to_odom_ * robot_odom;
      if (ppDistance(
          cv::Point2d(robot_pose.translation()[0], robot_pose.translation()[1]),
          cv::Point2d(target.pose.x, target.pose.y)) > 4.0)
      {
        // LOG(INFO) << "OUT OF IDENTIFICATION";
        continue;
      }

      transform::Rigid3d target_odom_;
      Eigen::Quaterniond target_quaterniond = transform::RollPitchYaw(0, 0, target.pose.z);
      // Eigen::Quaterniond target_quaterniond = transform::RollPitchYaw(0,0,0);
      transform::Rigid3d target_map_pose(Eigen::Vector3d(
          target.pose.x, target.pose.y,
          0), target_quaterniond);
      //  target_odom_ = map_to_odom_.inverse()*target_map_pose;
      transform::Rigid3d target_predict = robot_pose.inverse() * target_map_pose;
      // bool shelf_rec_ = false;
      if (true) {
        // LOG(INFO) << "START SHELF REC";
        std::vector<FeatureMsg> target_list;
        std::vector<std::vector<int>> point_set_r;
        std::vector<Line> lines_r;
        std::vector<cv::Point2d> vis_input_point;
        target_list = recgByReflec_->getTargetByPredictPose(
          point_cloud, laserPointSet,
          target_predict, point_set_r, lines_r,
          vis_input_point);
        // target_list =
        //   recgByReflec_->getShelfPoseInLaser(
        //   point_cloud, laserPointSet, point_set_r, lines_r, vis_input_point);
        // LOG(INFO) << "target list size = " << target_list.size();
        // LOG(INFO) << "reflector vis_input_point size = " << vis_input_point.size();
        std::vector<FeatureMsg> target_map3;
        for (auto line : lines_r) {
          // std::cout << "x1: " << line.first_point.x << "y1: " <<
          //   line.first_point.y << "x2: " << line.last_point.x << "y2: " <<
          //   line.last_point.y <<std::endl;
          FeatureMsg feature;
          feature.target_name = "line";
          feature.line = std::make_pair(
            Eigen::Vector3f(line.line.first.x, line.line.first.y, 0),
            Eigen::Vector3f(line.line.second.x, line.line.second.y, 0));
          // LOG(INFO) << "line x = " << line.line.first.x << " y = " << line.line.first.y;
          target_map3.push_back(feature);
        }
        FeatureMsgWithOrigin tmp_target_r;
        tmp_target_r.origin = point_cloud.origin;
        tmp_target_r.MsgVec = target_map3;
        target_map.push_back(tmp_target_r);
        for (size_t i = 0; i < vis_input_point.size(); i++) {
          FeatureMsg t_msg;
          t_msg.id = 0;
          t_msg.target_name = "reflector";
          t_msg.x = vis_input_point[i].x;
          t_msg.y = vis_input_point[i].y;
          target_list.push_back(t_msg);
        }
        FeatureMsgWithOrigin tmp_target;
        for (size_t i = 0; i < target_list.size(); i++) {
          tmp_target.origin = point_cloud.origin;
          tmp_target.MsgVec.push_back(target_list[i]);
        }
        target_map.push_back(tmp_target);
      }
    }
  }

  return target_map;
}

std::vector<FeatureMsgWithOrigin> LaserPerception::laserUpdate(
  const sensor::SyncPointCloudWithOdom & msg)
{
//  TimingTool laserTime;
  std::vector<FeatureMsgWithOrigin> target_map;
  for (size_t i = 0; i < msg.sync_point_clouds.size(); i++) {
    sensor::PointCloudWithOrigin point_cloud = msg.sync_point_clouds[i];
    std::vector<std::pair<int, Eigen::Vector3f>> point_cloud_in_laser;
    for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
      Eigen::Vector3f point_base_link = point_cloud.sync_points_base_link[i];
      Eigen::Vector3d origin = point_cloud.origin;
      Eigen::Vector3f point;
      point << point_base_link[0] * cos(origin[2]) + point_base_link[1] * sin(origin[2]) -
      (origin[0] * cos(origin[2]) + origin[1] * sin(origin[2])),
        -point_base_link[0] * sin(origin[2]) + point_base_link[1] * cos(origin[2]) -
      (-origin[0] * sin(origin[2]) + origin[1] * cos(origin[2])),
        0;
      point_cloud_in_laser.push_back(
        std::pair<int, Eigen::Vector3f>(
          point_cloud.intensities[i],
          point));
      point_cloud.sync_points_base_link[i] = point;
    }
    std::vector<std::pair<int, std::vector<int>>>
      laserPointSet = laserCluster(point_cloud_in_laser);

    point_cloud_highlight_ = std::make_shared<sensor::PointCloud>();
    for (const auto point_set : laserPointSet) {
      if (point_set.first == 2) {
        for (const auto ind : point_set.second) {
          sensor::PointCloud::PointType p;
          p.position = point_cloud_in_laser[ind].second;
          point_cloud_highlight_->push_back(p);
        }
      }
    }    

    point_cloud_flat_ = std::make_shared<sensor::PointCloud>();
    point_cloud_less_flat_ = std::make_shared<sensor::PointCloud>();
    flatCloudDetector_->DetectFlatCloud(point_cloud, *point_cloud_flat_, *point_cloud_less_flat_);

    line_rec_ = true;
    if (line_rec_) {
      std::vector<FeatureMsg> target_map3;
      std::list<Point> input_points;
      for (auto item : laserPointSet) {
        for (auto index : item.second) {
          input_points.push_back(
            Point(
              point_cloud_in_laser[index].second[0],
              point_cloud_in_laser[index].second[1]));
        }
      }
      // std::list<Segment> lines = lineDetector_->detectLines(input_points, laserPointSet);
      std::list<Segment> lines = lineDetectorX_->detectLines(input_points);

      for (auto line : lines) {
        // std::cout << "x1: " << line.first_point.x << "y1: " <<
        //   line.first_point.y << "x2: " << line.last_point.x << "y2: " <<
        //   line.last_point.y <<std::endl;
        FeatureMsg feature;
        feature.target_name = "line";
        feature.line = std::make_pair(
          Eigen::Vector3f(line.first_point.x, line.first_point.y, 0),
          Eigen::Vector3f(line.last_point.x, line.last_point.y, 0));
        target_map3.push_back(feature);
      }
      FeatureMsgWithOrigin tmp_target;
      tmp_target.origin = point_cloud.origin;
      tmp_target.MsgVec = target_map3;
      target_map.push_back(tmp_target);
      // bool test_angle_detect = false;

      angle_rec_ = true;
      if (angle_rec_) {
        // LOG(INFO) << "LINES SIZE = " << lines.size();
        if (lines.size() > 1) {
          std::vector<AngleStructCandidate> candidates = angleDectector_->computeAllPossibleStruct(
            lines);
          // LOG(INFO) << "CANDIDATES SIZE = " << candidates.size();
          std::vector<AngleStruct> angle_struct_v;
          if (candidates.size() < 1) {
            // LOG(INFO) << "no angleStruct detected ";
          } else {
            for (size_t i = 0; i < candidates.size(); i++) {
              AngleStruct angle_struct;
              bool angle_detect = angleDectector_->detectAngleStruct(candidates[i], angle_struct);
              if (angle_detect == false) {
                continue;
              }
              angle_struct.origin = point_cloud.origin;
              angle_struct_v.push_back(angle_struct);
            }
          }
          FeatureMsgWithOrigin tmp_target;
          // LOG(INFO) << "angle_struct_v size = " << angle_struct_v.size();
          for (size_t i = 0; i < angle_struct_v.size(); i++) {
            tmp_target.origin = point_cloud.origin;
            FeatureMsg msg;
            msg.target_name = "shape_struct";
            msg.x = angle_struct_v[i].pose[0];
            msg.y = angle_struct_v[i].pose[1];
            msg.theta = angle_struct_v[i].pose[2];
            msg.corner_struct_angle = angle_struct_v[i].angle;
            tmp_target.MsgVec.push_back(msg);
            // LOG(INFO) << "detect angle = " << angle_struct_v[i].angle;
            // LOG(INFO) << "angle_struct pose = " << angle_struct_v[i].pose;
            // LOG(INFO) << "line 1 first = " << angle_struct_v[i].first_line.first[0] << " " <<
            //   " " << angle_struct_v[i].first_line.first[1];
            // LOG(INFO) << "line 1 second = " << angle_struct_v[i].first_line.second[0] << " " <<
            //   " " << angle_struct_v[i].first_line.second[1];
            // LOG(INFO) << "line 2 first = " << angle_struct_v[i].second_line.first[0] << " " <<
            //   " " << angle_struct_v[i].second_line.first[1];
            // LOG(INFO) << "line 2 second = " << angle_struct_v[i].second_line.second[0] << " " <<
            //   " " << angle_struct_v[i].second_line.second[1];
          }
          target_map.push_back(tmp_target);
        }
      }
    }

    reflect_rec_ = true;
    if (reflect_rec_) {
      std::vector<FeatureMsg> target_map3;
      target_map3 = recgByReflec_->getReflector(point_cloud, laserPointSet);
      FeatureMsgWithOrigin tmp_target;
      tmp_target.origin = point_cloud.origin;
      tmp_target.MsgVec = target_map3;
      target_map.push_back(tmp_target);
    }
    bool shelf_rec_ = false;
    if (shelf_rec_) {
      LOG(INFO) << "START SHELF REC";
      std::vector<FeatureMsg> target_list;
      std::vector<std::vector<int>> point_set_r;
      std::vector<Line> lines_r;
      std::vector<cv::Point2d> vis_input_point;
      target_list = recgByReflec_->getShelfPoseInLaser(
        point_cloud, laserPointSet, point_set_r,
        lines_r, vis_input_point);
      LOG(INFO) << "target list size = " << target_list.size();
      LOG(INFO) << "reflector vis_input_point size = " << vis_input_point.size();
      std::vector<FeatureMsg> target_map3;
      for (auto line : lines_r) {
        // std::cout << "x1: " << line.first_point.x << "y1: " <<
        //   line.first_point.y << "x2: " << line.last_point.x << "y2: " <<
        //   line.last_point.y <<std::endl;
        FeatureMsg feature;
        feature.target_name = "line";
        feature.line = std::make_pair(
          Eigen::Vector3f(line.line.first.x, line.line.first.y, 0),
          Eigen::Vector3f(line.line.second.x, line.line.second.y, 0));
        // LOG(INFO) << "line x = " << line.line.first.x << " y = " << line.line.first.y;
        target_map3.push_back(feature);
      }
      FeatureMsgWithOrigin tmp_target_r;
      tmp_target_r.origin = point_cloud.origin;
      tmp_target_r.MsgVec = target_map3;
      target_map.push_back(tmp_target_r);
      for (size_t i = 0; i < vis_input_point.size(); i++) {
        FeatureMsg t_msg;
        t_msg.id = 0;
        t_msg.target_name = "reflector";
        t_msg.x = vis_input_point[i].x;
        t_msg.y = vis_input_point[i].y;
        target_list.push_back(t_msg);
      }
      FeatureMsgWithOrigin tmp_target;
      for (size_t i = 0; i < target_list.size(); i++) {
        tmp_target.origin = point_cloud.origin;
        tmp_target.MsgVec.push_back(target_list[i]);
      }
      target_map.push_back(tmp_target);
    }

    if (shape_rec_) {
      std::vector<FeatureMsg> target_list;
      std::vector<std::vector<int>> point_set_r;
      std::vector<Line> lines_r;
      std::vector<cv::Point2d> vis_input_point;
      std::vector<FeatureInfo> target_list_pre;
      target_list_pre = target_list_;
      for (auto target : target_list_pre) {
        target_list = recgByReflec_->getShapePoseInLaser(
          point_cloud, laserPointSet, point_set_r,
          lines_r, vis_input_point);
        static int kalman_i = 0;
        if (kalman_i == 0) {
          for (size_t j = 0; j < target_list.size(); j++) {
            if (target_list[j].target_name == "shape_struct") {
              KF_chargeStation->statePost.at<float>(0) = target_list[j].x;
              KF_chargeStation->statePost.at<float>(1) = target_list[j].y;
              KF_chargeStation->statePost.at<float>(2) = target_list[j].theta;
              kalman_i++;
              break;
            }
          }
        } else {
          for (size_t i = 0; i < target_list.size(); i++) {
            if (target_list[i].target_name == "shape_struct") {
              // LOG(INFO) << "********************KALMAN_FILTER********************";
              cv::Point3d predict_pose;
              kfPredict(*KF_chargeStation, predict_pose);
              double dlt_angle =
                atan2(
                sin(target_list[i].theta - predict_pose.z),
                cos(target_list[i].theta - predict_pose.z));
              kfMeasure(
                *KF_chargeStation,
                cv::Point3d(target_list[i].x, target_list[i].y, predict_pose.z + dlt_angle));

              // target_list[i].x = KF_chargeStation->statePost.at<float>(0);
              // target_list[i].y = KF_chargeStation->statePost.at<float>(1);
              // target_list[i].theta = KF_chargeStation->statePost.at<float>(2);
            }
          }
        }

        std::vector<FeatureMsg> target_map3;
        for (auto line : lines_r) {
          FeatureMsg feature;
          feature.target_name = "line";
          feature.line = std::make_pair(
            Eigen::Vector3f(line.line.first.x, line.line.first.y, 0),
            Eigen::Vector3f(line.line.second.x, line.line.second.y, 0));
          // target_map3.push_back(feature);
        }
        FeatureMsgWithOrigin tmp_target_r;
        tmp_target_r.origin = point_cloud.origin;
        tmp_target_r.MsgVec = target_map3;
        target_map.push_back(tmp_target_r);
        // std::vector<FeatureMsg> target_tmp;
        // LOG(INFO) << "reflector point_set_r size = " << point_set_r.size();
        // for(int i = 0; i < point_set_r.size(); i++)
        // {
        //   for(int j = 0; j < point_set_r[i].size(); j++)
        //   {
        //     FeatureMsg t_msg;
        //     t_msg.id = 0;
        //     t_msg.target_name = "reflector";
        //     t_msg.x = point_cloud.sync_points_base_link[point_set_r[i][j]][0];
        //     t_msg.y = point_cloud.sync_points_base_link[point_set_r[i][j]][1];
        //     // target_list.push_back(t_msg);
        //   }
        // }
        // LOG(INFO) << "reflector vis_input_point size = " << vis_input_point.size();
        // for(int i = 0; i < vis_input_point.size(); i++)
        // {
        //   FeatureMsg t_msg;
        //   t_msg.id = 0;
        //   t_msg.target_name = "reflector";
        //   t_msg.x = vis_input_point[i].x;
        //   t_msg.y = vis_input_point[i].y;
        //   target_list.push_back(t_msg);
        // }
        FeatureMsgWithOrigin tmp_target;
        for (size_t i = 0; i < target_list.size(); i++) {
          tmp_target.origin = point_cloud.origin;
          tmp_target.MsgVec.push_back(target_list[i]);
        }
        // tmp_target.MsgVec.push_back(target_tmp);
        // FeatureMsg target_msg;
        // target_msg.x =
        // tmp_target.origin = point_cloud.origin;
        // tmp_target.MsgVec.push_back()
        target_map.push_back(tmp_target);
      }
    }
  }
  return target_map;
}

// std::vector<FeatureMsg>
// LaserPerception::SlideShapeStruct(const std::vector<FeatureMsg>& target_map)
// {
//   static std::list<FeatureMsg> slide_vector;
//   std::vector<FeatureMsg> return_map;
//   return_map = target_map;
//   static int slide_i = 0;
//   if(slide_i < 10){
//     return return_map;
//   }else{
//     for(int i = 0; i < target_map.size(); i++){
//       if(target_map[i].target_name == "angle_struct"){
//         if(slide_vector.size() < 10){
//           slide_vector.push_back(target_map[i]);
//           return return_map;
//         }else{
//           slide_vector.pop_front();
//           slide_vector.push_back(target_map[i]);
//           double sum_x = 0;
//           double sum_y = 0;
//           double sum_theta = 0;
//           for(int j = 0; j < slide_vector.size(); j++){
//             sum_x += sum_x;
//             sum_y += sum_y;
//             sum_theta += sum_theta;
//           }
//           return_map[i].x = sum_x/slide_vector.size();
//           return_map[i].y = sum_y/slide_vector.size();
//           return_map[i].theta = sum_theta/slide_vector.size();
//           return return_map;
//         }
//       }
//     }
//   }

//   return return_map;
// }

std::vector<AngleStructCandidate> LaserPerception::computeAllPossibleStruct(
  std::list<Segment> lines)
{
  std::vector<AngleStructCandidate> candidates;
  std::vector<AngleStructCandidate> final_candidates;
  std::vector<Segment> seg_v;
  for (const auto & it : lines) {
    seg_v.push_back(it);
  }
  for (size_t i = 0; i < seg_v.size() - 1; i++) {
    AngleStructCandidate line_;
    line_.first_line = std::make_pair(
      Eigen::Vector3f(seg_v[i].first_point.x, seg_v[i].first_point.y, 0),
      Eigen::Vector3f(seg_v[i].last_point.x, seg_v[i].last_point.y, 0));
    for (size_t j = i + 1; j < seg_v.size(); j++) {
      line_.second_line = std::make_pair(
        Eigen::Vector3f(seg_v[j].first_point.x, seg_v[j].first_point.y, 0),
        Eigen::Vector3f(seg_v[j].last_point.x, seg_v[j].last_point.y, 0));
      candidates.push_back(line_);
    }
  }
  LOG(INFO) << "TMP_CANDIDATES SIZE = " << candidates.size();
  for (size_t i = 0; i < candidates.size(); i++) {
    double dis = computeMinDisBetweenLines(candidates[i]);
    LOG(INFO) << "DIS = " << dis;
    if (dis < 0.1) {
      final_candidates.push_back(candidates[i]);
    }
  }
  return final_candidates;
}

double LaserPerception::computeMinDisBetweenLines(AngleStructCandidate candidate)
{
  std::vector<std::pair<double, std::pair<Eigen::Vector2d, Eigen::Vector2d>>> dis_vec;
  Eigen::Vector2d first_line_1(candidate.first_line.first[0], candidate.first_line.first[1]);
  Eigen::Vector2d first_line_2(candidate.first_line.second[0], candidate.first_line.second[1]);
  Eigen::Vector2d second_line_1(candidate.second_line.first[0], candidate.second_line.first[1]);
  Eigen::Vector2d second_line_2(candidate.second_line.second[0], candidate.second_line.second[1]);
  std::vector<double> dis_v;
  double dis_1 = sqrt(
    (first_line_1[0] - second_line_1[0]) * (first_line_1[0] - second_line_1[0]) +
    (first_line_1[1] - second_line_1[1]) * (first_line_1[1] - second_line_1[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_1[0] - second_line_2[0]) * (first_line_1[0] - second_line_2[0]) +
    (first_line_1[1] - second_line_2[1]) * (first_line_1[1] - second_line_2[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_2[0] - second_line_1[0]) * (first_line_2[0] - second_line_1[0]) +
    (first_line_2[1] - second_line_1[1]) * (first_line_2[1] - second_line_1[1]));
  dis_v.push_back(dis_1);
  dis_1 = sqrt(
    (first_line_2[0] - second_line_2[0]) * (first_line_2[0] - second_line_2[0]) +
    (first_line_2[1] - second_line_2[1]) * (first_line_2[1] - second_line_2[1]));
  dis_v.push_back(dis_1);
  double min_dis;
  min_dis = dis_v[0];
  for (size_t i = 1; i < dis_v.size(); i++) {
    double tmp_dis = dis_v[i];
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
    }
  }
  return min_dis;
}

bool LaserPerception::detectAngleStruct(
  const AngleStructCandidate & candidate,
  AngleStruct & angle_struct)
{
  double detect_angle = 0;
  if (candidate.first_line.first[0] == candidate.first_line.second[0]) {
    if (candidate.second_line.first[0] == candidate.second_line.second[0]) {
      LOG(INFO) << "the two lines are parallel";
      return false;
    }
    double k_2 = (candidate.second_line.second[1] - candidate.second_line.first[1]) /
      (candidate.second_line.second[0] - candidate.second_line.first[0]);
    double b_2 = candidate.second_line.first[1] - k_2 * candidate.second_line.first[0];
    double x = candidate.first_line.first[0];
    double y = k_2 * x + b_2;
    LOG(INFO) << "k_1 = " << k_2 << " b_1 = " << b_2 << " ";
    LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      LOG(INFO) << "can't detect angleStruct ";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  } else if (candidate.second_line.first[0] == candidate.second_line.second[0]) {
    double k_1 = (candidate.first_line.second[1] - candidate.first_line.first[1]) /
      (candidate.first_line.second[0] - candidate.first_line.first[0]);
    double b_1 = candidate.first_line.first[1] - k_1 * candidate.first_line.first[0];
    double x = candidate.second_line.first[0];
    double y = k_1 * x + b_1;
    LOG(INFO) << "k_1 = " << k_1 << " b_1 = " << b_1 << " ";
    LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      LOG(INFO) << "can't detect angleStruct ";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  } else {
    double k_2 = (candidate.second_line.second[1] - candidate.second_line.first[1]) /
      (candidate.second_line.second[0] - candidate.second_line.first[0]);
    double b_2 = candidate.second_line.first[1] - k_2 * candidate.second_line.first[0];
    double k_1 = (candidate.first_line.second[1] - candidate.first_line.first[1]) /
      (candidate.first_line.second[0] - candidate.first_line.first[0]);
    double b_1 = candidate.first_line.first[1] - k_1 * candidate.first_line.first[0];
    if (k_1 == k_2) {
      LOG(INFO) << "the two lines are parallel";
      return false;
    }
    double x = (b_2 - b_1) / (k_1 - k_2);
    double y = k_1 * x + b_1;
    LOG(INFO) << "k_1 = " << k_1 << " b_1 = " << b_1 << " ";
    LOG(INFO) << "k_2 = " << k_2 << " b_2 = " << b_2 << " ";
    LOG(INFO) << "x = " << x << " y = " << y;
    Eigen::Vector2d inter_point(x, y);
    double angle = 0;
    bool angle_detect = detectAngle(inter_point, candidate, angle, detect_angle);
    if (angle_detect == false) {
      LOG(INFO) << "can't detect angleStruct";
      return false;
    }
    angle_struct.angle = detect_angle;
    angle_struct.pose << inter_point[0], inter_point[1], angle;
    angle_struct.first_line = candidate.first_line;
    angle_struct.second_line = candidate.second_line;
  }
  return true;
}

bool LaserPerception::detectAngle(
  const Eigen::Vector2d & inter_point,
  const AngleStructCandidate & candidate,
  double & pose_angle,
  double & angle)
{
  Eigen::Vector2d v_1, v_2;
  // LOG(INFO) << "INTER_POINT = " << inter_point;
  double dis_1 = sqrt(
    (inter_point[0] - candidate.first_line.first[0]) *
    (inter_point[0] - candidate.first_line.first[0]) +
    (inter_point[1] - candidate.first_line.first[1]) *
    (inter_point[1] - candidate.first_line.first[1]));
  double dis_2 = sqrt(
    (inter_point[0] - candidate.first_line.second[0]) *
    (inter_point[0] - candidate.first_line.second[0]) +
    (inter_point[1] - candidate.first_line.second[1]) *
    (inter_point[1] - candidate.first_line.second[1]));
  if (dis_1 > dis_2) {
    if (dis_2 > 0.1) {
      LOG(INFO) << "the dis between inter point and lines are too larget. dis = " << dis_2;
      return false;
    }
    v_1 << candidate.first_line.first[0] - inter_point[0],
      candidate.first_line.first[1] - inter_point[1];
  } else {
    if (dis_1 > 0.1) {
      LOG(INFO) << "the dis between inter point and lines are too larget. dis = " << dis_1;
      return false;
    }
    v_1 << candidate.first_line.second[0] - inter_point[0],
      candidate.first_line.second[1] - inter_point[1];
  }

  dis_1 = sqrt(
    (inter_point[0] - candidate.second_line.first[0]) *
    (inter_point[0] - candidate.second_line.first[0]) +
    (inter_point[1] - candidate.second_line.first[1]) *
    (inter_point[1] - candidate.second_line.first[1]));
  dis_2 = sqrt(
    (inter_point[0] - candidate.second_line.second[0]) *
    (inter_point[0] - candidate.second_line.second[0]) +
    (inter_point[1] - candidate.second_line.second[1]) *
    (inter_point[1] - candidate.second_line.second[1]));
  if (dis_1 > dis_2) {
    if (dis_2 > 0.05) {
      LOG(INFO) << "the dis between inter point and lines are too larget dis = " << dis_2;
      return false;
    }
    v_2 << candidate.second_line.first[0] - inter_point[0],
      candidate.second_line.first[1] - inter_point[1];
  } else {
    if (dis_1 > 0.05) {
      LOG(INFO) << "the dis between inter point and lines are too larget dis = " << dis_1;
      return false;
    }
    v_2 << candidate.second_line.second[0] - inter_point[0],
      candidate.second_line.second[1] - inter_point[1];
  }
  // LOG(INFO) << "V_1 = " << v_1[0] << " " << v_1[1];
  // LOG(INFO) << "V_2 = " << v_2[0] << " " << v_2[1];
  Eigen::Vector2d v_3 = v_1.normalized() + v_2.normalized();
  angle = acos(v_1.normalized().transpose() * v_2.normalized());
  // LOG(INFO) << "V_3 = " << v_3[0] << " " << v_3[1];
  if (v_3[0] == 0 && v_3[1] > 0) {
    pose_angle = 3.1415926 / 2;
  } else if (v_3[0] == 0 && v_3[1] < 0) {
    pose_angle = -3.1415925 / 2;
  } else if (v_3[0] < 0 && v_3[1] > 0) {
    pose_angle = 3.1415926 / 2 + atan(fabs(v_3[0] / v_3[1]));
  } else if (v_3[0] < 0 && v_3[1] < 0) {
    pose_angle = -3.1415925 / 2 - atan(fabs(v_3[0] / v_3[1]));
  } else {
    pose_angle = atan(v_3[1] / v_3[0]);
  }
  return true;
}
}  // namespace laser_perception
}  // cartographer
