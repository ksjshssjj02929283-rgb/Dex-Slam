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

#include "cartographer/laser_perception/recgByReflec.h"
#include <vector>
#include <utility>
#include <string>
#include <list>
#include "opencv2/core/types.hpp"
#include "cartographer/laser_perception/math_tool.h"
#include "cartographer/laser_perception/data_type.h"
namespace cartographer
{
namespace laser_perception
{
RecgByReflec::RecgByReflec(const proto::LaserPerceptionOptions & options)
{
  reflector_error_ = options.reflector_error();
  reflector_radius_ = options.reflector_radius();
  shape_match_score_ = options.shape_match_score();
  shape_match_close_degree_ = options.shape_match_close_degree();
  shape_dis_to_robot_center_ = options.shape_dis_to_robot_center();
  reflector_error_range_ = options.reflector_error_range();
  model_manager_ = ModelManager::instance(options);
}
std::vector<FeatureMsg> RecgByReflec::getReflector(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<FeatureMsg> target_map;

  for (auto points : laserPointSet) {
    if (points.first != 2) {
      continue;
    }
    double length_model = 2 * reflector_radius_;

    double length_scan = chordLength_point(
      msg.sync_points_base_link[points.second.front()],
      msg.sync_points_base_link[points.second.back()]);
    double length_error = length_model - length_scan;
    double tmp_reflector_error = reflector_error_;
    double range = msg.sync_points_base_link[
      points.second[points.second.size() / 2]].norm();
    if (range < reflector_error_range_) {
      tmp_reflector_error = reflector_error_ + 0.01;
    }
    if (fabs(length_error) < tmp_reflector_error) {
      double x = msg.sync_points_base_link[points.second[points.second.size() / 2]][0];
      double y = msg.sync_points_base_link[points.second[points.second.size() / 2]][1];
      double center_x = x + reflector_radius_ * (x / sqrt(x * x + y * y));
      double center_y = y + reflector_radius_ * (y / sqrt(x * x + y * y));
      FeatureMsg feature;
      feature.id = 0;
      feature.target_name = "reflector";
      feature.x = center_x;
      feature.y = center_y;
      feature.theta = 0;
      target_map.push_back(feature);
    } else {
      // LOG(INFO) << "Too large length error length_model: " <<
      //   length_model << " length_scan: " << length_scan;
    }
  }
  return target_map;
}

std::vector<FeatureMsg> RecgByReflec::getShapePose(
  const sensor::PointCloudWithOrigin & msg,
  const sensor::OdometryData odom,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<FeatureInfo> shape_pose_map;
  double start_rec_dis = 4.0;
  double search_distance_ = 0.25;
  double max_dis_error_ = 0.04;
  double max_ang_error_ = 0.2;

  for (auto target : shape_pose_map) {
    if (ppDistance(
        cv::Point2d(odom.pose.translation()[0], odom.pose.translation()[1]),
        cv::Point2d(target.pose.x, target.pose.y)) > start_rec_dis)
    {
      std::cout << "robot x: " << msg.origin[0] << " y: " << msg.origin[1] << std::endl;
      std::cout << "target x: " << target.pose.x << " y: " << target.pose.y << std::endl;
      std::cout << "start dis : " << start_rec_dis << std::endl;
      continue;
    }

    Model model;
    if (!model_manager_->getModel(target.target_name, model)) {
      continue;
    }

    Eigen::Quaterniond q = transform::RollPitchYaw(0, 0, target.pose.z);
    transform::Rigid3d target_pose_map(Eigen::Vector3d(target.pose.x, target.pose.y, 0), q);
    Eigen::Quaterniond q_laser = transform::RollPitchYaw(0, 0, msg.origin[2]);
    transform::Rigid3d scan_transform(Eigen::Vector3d(
        msg.origin[0], msg.origin[1],
        msg.origin[2]), q_laser);
    transform::Rigid3d shape_pose_in_laser = scan_transform.inverse() * odom.pose.inverse() *
      target_pose_map;

    Model model_update = model;
    for (size_t i = 0; i < model.components.size(); i++) {
      // Eigen::Quaterniond q_model_point =
      //     transform::RollPitchYaw(0, 0, model.components[i].theta);
      transform::Rigid3d model_point(Eigen::Vector3d(
          model.components[i].x,
          model.components[i].y, 0), transform::RollPitchYaw(0, 0, model.components[i].theta));
      transform::Rigid3d trans_model_point = shape_pose_in_laser * model_point;
      model_update.components[i].x = trans_model_point.translation()[0];
      model_update.components[i].y = trans_model_point.translation()[1];
      model_update.components[i].theta = transform::GetYaw(trans_model_point.rotation());
    }
    std::vector<std::vector<std::vector<int>>> component_points;
    for (auto component : model_update.components) {
      std::vector<std::vector<int>> possible_points;
      LOG(INFO) << "laserPointSet.size = " << laserPointSet.size();
      for (auto pointSet : laserPointSet) {
        LOG(INFO) << "type = " << pointSet.first;
        if (pointSet.first != component.type) {
          continue;
        }
        double x = 0, y = 0;
        cv::Point2d midPoint;
        for (size_t i = 0; i < pointSet.second.size(); i++) {
          x += msg.sync_points_base_link[pointSet.second[i]][0];
          y += msg.sync_points_base_link[pointSet.second[i]][1];
        }
        midPoint.x = x / pointSet.second.size();
        midPoint.y = y / pointSet.second.size();
        double distance_error = ppDistance(midPoint, cv::Point2d(component.x, component.y));
        if (distance_error < search_distance_) {
          possible_points.push_back(pointSet.second);
        }
      }
      if (possible_points.size() > 0) {
        component_points.push_back(possible_points);
      }
      std::cout << "possible points size = " << possible_points.size() << std::endl;
    }

    std::vector<Line> lines = model_manager_->getModelLines(model_update);

    if (component_points.size() < 1) {
      std::cout << "component points size is 0" << std::endl;
      continue;
    }

    std::vector<std::vector<int>> select_points_set;
    for (size_t k = 0; k < component_points[0].size(); k++) {
      select_points_set.push_back(component_points[0][k]);
    }
    for (size_t i = 0; i < component_points.size(); i++) {
      std::vector<std::vector<int>> tmp_points_set;
      for (size_t j = 0; j < select_points_set.size(); j++) {
        for (size_t k = 0; k < component_points[i].size(); k++) {
          std::vector<int> points;
          points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());
          points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
          tmp_points_set.push_back(points);
        }
      }
      select_points_set = tmp_points_set;
    }
    LOG(INFO) << "SELECT_POINTS_SET SIZE = " << select_points_set.size();
    std::vector<std::pair<double, cv::Point3d>> possible_target;
    std::vector<cv::Point2d> vis_input_point;
    for (size_t i = 0; i < select_points_set.size(); i++) {
      // std::vector<int> select_points = select_points_set[i];
      std::vector<cv::Point2d> select_points;
      for (size_t j = 0; j < select_points_set[i].size(); j++) {
        select_points.push_back(
          cv::Point2d(
            msg.sync_points_base_link[select_points_set[i][j]][0],
            msg.sync_points_base_link[select_points_set[i][j]][1]));
      }
      if (select_points.size() > 2 &&
        ppDistance(select_points.back(), select_points.front()) < 0.2)
      {
        std::cout << "select points max distance less than 0.2" << std::endl;
        continue;
      }

      cv::Point3d target_pose(shape_pose_in_laser.translation()[0],
        shape_pose_in_laser.translation()[1],
        transform::GetYaw(shape_pose_in_laser.rotation()));
      double error = getTargetPosition(select_points, lines, target_pose, vis_input_point);
      possible_target.push_back(std::make_pair(error, target_pose));
    }

    if (possible_target.size() > 0) {
      cv::Point3d target_pose;
      std::vector<int> select_target_list;
      cv::Point2d predict;
      predict.x = shape_pose_in_laser.translation()[0];
      predict.y = shape_pose_in_laser.translation()[1];
      for (size_t i = 0; i < possible_target.size(); i++) {
        cv::Point2d target_pose;
        target_pose.x = possible_target[i].second.x;
        target_pose.y = possible_target[i].second.y;
        double angle_diff = fabs(
          transform::GetYaw(shape_pose_in_laser.rotation()) -
          possible_target[i].second.z);
        if (possible_target[i].first > max_dis_error_) {
          std::cout << "match distance error = " << possible_target[i].first << std::endl;
        } else if (ppDistance(target_pose, predict) > search_distance_) {
          std::cout << "search distance error = " << ppDistance(target_pose, predict) << std::endl;
        } else if (angle_diff > max_ang_error_) {
          std::cout << "match angle error = " << angle_diff << std::endl;
        } else {
          select_target_list.push_back(i);
        }
      }
      if (select_target_list.size() < 1) {
        std::cout << "select target list size = 0 " << std::endl;
        continue;
      }
      for (size_t i = 0; i < select_target_list.size(); i++) {
        cv::Point2d target1;
        target1.x = possible_target[select_target_list[i]].second.x;
        target1.y = possible_target[select_target_list[i]].second.y;
        for (size_t j = i + 1; j < select_target_list.size(); j++) {
          cv::Point2d target2;
          target2.x = possible_target[select_target_list[j]].second.x;
          target2.y = possible_target[select_target_list[j]].second.y;
          if (ppDistance(target1, predict) > ppDistance(target2, predict)) {
            int tmp = select_target_list[i];
            select_target_list[i] = select_target_list[j];
            select_target_list[j] = tmp;
          }
        }
      }

      double min_error = possible_target[select_target_list.front()].first;
      target_pose = possible_target[select_target_list.front()].second;
      if (min_error < max_dis_error_) {
        std::cout << "target pose update" << std::endl;
        FeatureMsg feature_msg;
        feature_msg.id = 0;
        feature_msg.x = target_pose.x;
        feature_msg.y = target_pose.y;
        feature_msg.theta = target_pose.z;
        feature_msg.target_name = target.target_name;
        shape_pose_return.push_back(feature_msg);
      } else {
        std::cout << "match error  = " << min_error << std::endl;
      }
    } else {
      std::cout << "possible target size = 0" << std::endl;
    }
  }
  return shape_pose_return;
}

bool RecgByReflec::ComputePointSetDis(const std::vector<std::vector<cv::Point2d>> & points)
{
  for (size_t i = 0; i < points.size(); i++) {
    for (size_t j = i + 1; j < points.size(); j++) {
      double dis = ppDistance(getPointsCenter(points[i]), getPointsCenter(points[j]));
      if (dis > 1.8 || dis < 0.6) {
        return false;
      }
    }
  }
  return true;
}

std::vector<FeatureMsg> RecgByReflec::getTargetByPredictPose(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet,
  transform::Rigid3d predict_target_pose,
  std::vector<std::vector<int>> & point_set_r,
  std::vector<Line> & lines_r,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> shelf_pose_return;
  std::string target_name = "shapeStruct0";
  Model model;
  double max_dis_error_ = 0.5;
  model_manager_->getModel(target_name, model);
  LOG(INFO) << "laserPointSet size = " << laserPointSet.size();
  std::vector<std::vector<cv::Point2d>> possible_points_vector;
  std::vector<std::vector<std::vector<cv::Point2d>>> component_points;
  Eigen::Quaterniond scan_quaterniond = transform::RollPitchYaw(0, 0, msg.origin[2]);
  transform::Rigid3d scan_to_base_link(Eigen::Vector3d(
      msg.origin[0], msg.origin[1],
      0), scan_quaterniond);
  transform::Rigid3d target_predict = scan_to_base_link.inverse() * predict_target_pose;
  // cv::Point3d target_odom_(
  //   predict_target_pose.translation()[0],
  //   predict_target_pose.translation()[1],
  //   transform::GetYaw(predict_target_pose));
  cv::Point3d target_odom_(target_predict.translation()[0],
    target_predict.translation()[1], transform::GetYaw(target_predict));
  LOG(INFO) << "target_odom = " << target_odom_.x << " " << target_odom_.y << " " << target_odom_.z;
  Model target_model = model_manager_->updateTargetModel(model, target_odom_);
  for (auto component : target_model.components) {
    for (auto pointSet : laserPointSet) {
      std::vector<cv::Point2d> points_tmp;
      for (size_t i = 0; i < pointSet.second.size(); i++) {
        points_tmp.push_back(
          cv::Point2d(
            msg.sync_points_base_link[pointSet.second[i]][0],
            msg.sync_points_base_link[pointSet.second[i]][1]));
      }
      cv::Point2d center = getPointsCenter(points_tmp);
      double distance_error = ppDistance(center, cv::Point2d(component.x, component.y));
      if (distance_error < 0.5) {
        possible_points_vector.push_back(points_tmp);
      }
    }
    if (possible_points_vector.size() > 0) {
      component_points.push_back(possible_points_vector);
    }
    vis_input_point.push_back(cv::Point2d(component.x, component.y));
  }

  LOG(INFO) << "POSSIBLE_POINTS SIZE =" << possible_points_vector.size();
  for (size_t i = 0; i < possible_points_vector.size(); i++) {
    for (size_t j = 0; j < possible_points_vector[i].size(); j++) {
      vis_input_point.push_back(possible_points_vector[i][j]);
    }
  }

  if (component_points.size() < 1) {
    std::cout << "component points sizee is 0" << std::endl;
    return shelf_pose_return;
  }

  std::vector<std::vector<cv::Point2d>> select_points_vector;

  for (size_t k = 0; k < component_points[0].size(); k++) {
    select_points_vector.push_back(component_points[0][k]);
  }

  for (size_t i = 1; i < component_points.size(); i++) {
    std::vector<std::vector<cv::Point2d>> tmp_points_set;
    for (size_t j = 0; j < select_points_vector.size(); j++) {
      for (size_t k = 0; k < component_points[i].size(); k++) {
        std::vector<cv::Point2d> points;
        points.insert(points.end(), select_points_vector[j].begin(), select_points_vector[j].end());
        points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
        tmp_points_set.push_back(points);
      }
    }
    select_points_vector = tmp_points_set;
  }

  LOG(INFO) << "SELECT_POINTS_VECTOR SIZE = " << select_points_vector.size();
  std::vector<Line> lines = model_manager_->getModelLines(target_model);
  lines_r = lines;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  double min_error_ = 1000;
  std::vector<cv::Point2d> tmp_vis_point;
  for (size_t i = 0; i < select_points_vector.size(); i++) {
    if (select_points_vector[i].size() > 2 &&
      ppDistance(select_points_vector[i].back(), select_points_vector[i].front()) < 0.2)
    {
      continue;
    }
    cv::Point3d target_pose = target_odom_;
    std::vector<Line> lines_tmp;
    lines_tmp = model_manager_->getModelLines(target_model);
    double error = getTargetPositionOriginal(
      select_points_vector[i], lines_tmp, target_pose,
      tmp_vis_point);
    if (min_error_ > error) {
      min_error_ = error;
      // vis_input_point = select_points_vector[i];
    }
    for (size_t i = 0; i < tmp_vis_point.size(); i++) {
      vis_input_point.push_back(tmp_vis_point[i]);
    }
    possible_target.push_back(std::make_pair(error, target_pose));
  }

  LOG(INFO) << "min error = " << min_error_;
  LOG(INFO) << "VIS INPUT POINT SIZE = " << vis_input_point.size();
  LOG(INFO) << "possible_target size = " << possible_target.size();
  if (possible_target.size() == 0) {
    return shelf_pose_return;
  }
  auto min_target_pose = possible_target.front();
  // double min_error = possible_target.front().first;
  for (size_t i = 0; i < possible_target.size(); i++) {
    LOG(INFO) << "POSSIBLE TARGET ERROR = " << possible_target[i].first;
    if (min_target_pose.first > possible_target[i].first) {
      min_target_pose = possible_target[i];
    }
  }
  LOG(INFO) << "min error = " << min_target_pose.first;
  if (min_target_pose.first < max_dis_error_) {
    LOG(INFO) << "target pose update";
    FeatureMsg feature_msg;
    feature_msg.id = 0;
    feature_msg.x = min_target_pose.second.x;
    feature_msg.y = min_target_pose.second.y;
    feature_msg.theta = min_target_pose.second.z;
    feature_msg.target_name = "shape_struct";
    shelf_pose_return.push_back(feature_msg);
  }
  LOG(INFO) << "shelf_pose_return pose = " << shelf_pose_return[0].x << " " <<
    shelf_pose_return[0].y <<
    " " << shelf_pose_return[0].theta;

  return shelf_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::getFeaturePoseFromPredict(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet,
  std::vector<std::vector<int>> & point_set_r,
  std::vector<Line> & lines_r,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> return_msg;
  std::string target_name = "shapeStruct0";
  Model model;
  // double max_dis_error_ = 0.5;
  model_manager_->getModel(target_name, model);
  LOG(INFO) << "laserPointSet size = " << laserPointSet.size();

  std::vector<std::vector<std::vector<int>>> component_points;
  for (auto component : model.components) {
    std::vector<std::vector<int>> possible_points;
    for (auto pointSet : laserPointSet) {
      if (pointSet.first != component.type) {
        continue;
      }
      if (pointSet.second.size() < 10) {
        continue;
      }
      possible_points.push_back(pointSet.second);
      point_set_r.push_back(pointSet.second);
    }
    if (possible_points.size() > 0) {
      component_points.push_back(possible_points);
    }
    std::cout << "possible points size = " << possible_points.size() << std::endl;
  }

  std::vector<std::vector<cv::Point2d>> possible_points_vector;
  for (auto pointSet : laserPointSet) {
    std::vector<cv::Point2d> points_tmp;
    for (size_t i = 0; i < pointSet.second.size(); i++) {
      points_tmp.push_back(
        cv::Point2d(
          msg.sync_points_base_link[pointSet.second[i]][0],
          msg.sync_points_base_link[pointSet.second[i]][1]));
    }
    possible_points_vector.push_back(points_tmp);
  }

  LOG(INFO) << "POSSIBLE_POINTS SIZE =" << possible_points_vector.size();
  std::vector<Line> lines = model_manager_->getModelLines(model);
  lines_r = lines;

  if (component_points.size() < 1) {
    std::cout << "component points size is 0" << std::endl;
    return return_msg;
  }
  std::vector<std::vector<int>> select_points_set;
  for (size_t k = 0; k < component_points[0].size(); k++) {
    select_points_set.push_back(component_points[0][k]);
  }
  for (size_t i = 1; i < component_points.size(); i++) {
    std::vector<std::vector<int>> tmp_points_set;
    for (size_t j = 0; j < select_points_set.size(); j++) {
      for (size_t k = 0; k < component_points[i].size(); k++) {
        std::vector<int> points;
        points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());
        points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
        tmp_points_set.push_back(points);
      }
    }
    select_points_set = tmp_points_set;
  }


  return return_msg;
}


std::vector<FeatureMsg> RecgByReflec::getShelfPoseInLaser(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet,
  std::vector<std::vector<int>> & point_set_r,
  std::vector<Line> & lines_r,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> shelf_pose_return;
  std::string target_name = "shelf0";
  Model model;
  double max_dis_error_ = 0.5;
  model_manager_->getModel(target_name, model);
  LOG(INFO) << "laserPointSet size = " << laserPointSet.size();
  // for(int i = 0; i < laserPointSet.size(); i++)
  // {
  //   LOG(INFO) << "size =" << laserPointSet[i].second.size();
  // }
  std::vector<std::vector<cv::Point2d>> possible_points_vector;
  for (auto pointSet : laserPointSet) {
    std::vector<cv::Point2d> points_tmp;
    for (size_t i = 0; i < pointSet.second.size(); i++) {
      points_tmp.push_back(
        cv::Point2d(
          msg.sync_points_base_link[pointSet.second[i]][0],
          msg.sync_points_base_link[pointSet.second[i]][1]));
    }
    // if(points_tmp.size() > 10)
    // {
    //   continue;
    // }
    // cv::Point2d center = getPointsCenter(points_tmp);
    // double distance_error = ppDistance(center, cv::Point2d(0,0));//));
    // if (distance_error < 3.0) {
    possible_points_vector.push_back(points_tmp);
    // }
  }
  LOG(INFO) << "POSSIBLE_POINTS SIZE =" << possible_points_vector.size();
  std::vector<std::vector<cv::Point2d>> select_points_vector;
  for (size_t i = 0; i < possible_points_vector.size(); i++) {
    // std::vector<std::vector<cv::Point2d>> shelf_points;
    // shelf_points.push_back(possible_points_vector[i]);
    std::vector<cv::Point2d> points_1 = possible_points_vector[i];
    for (size_t j = i + 1; j < possible_points_vector.size(); j++) {
      // shelf_points.push_back(possible_points_vector[j]);
      std::vector<cv::Point2d> points_2 = possible_points_vector[j];
      double distance_error = ppDistance(getPointsCenter(points_1), getPointsCenter(points_2));
      if (distance_error > 1.8 || distance_error < 0.6) {
        // shelf_points.pop_back();
        continue;
      }
      for (size_t m = j + 1; m < possible_points_vector.size(); m++) {
        std::vector<cv::Point2d> points_3 = possible_points_vector[m];
        // shelf_points.push_back(possible_points_vector[m]);
        std::vector<std::vector<cv::Point2d>> shelf_points;
        shelf_points.push_back(points_1);shelf_points.push_back(points_2);shelf_points.push_back(
          points_3);
        bool push_shelf_points = ComputePointSetDis(shelf_points);
        if (push_shelf_points == false) {
          // shelf_points.pop_back();
          continue;
        }
        for (size_t n = m + 1; n < possible_points_vector.size(); n++) {
          std::vector<cv::Point2d> points_4 = possible_points_vector[n];
          std::vector<std::vector<cv::Point2d>> shelf_points_f;
          shelf_points_f.push_back(points_1);shelf_points_f.push_back(points_2);
          shelf_points_f.push_back(points_3);shelf_points_f.push_back(points_4);
          // shelf_points.push_back(possible_points_vector[n]);
          push_shelf_points = ComputePointSetDis(shelf_points_f);
          if (push_shelf_points == false) {
            // shelf_points.pop_back();
            continue;
          }
          // LOG(INFO) << "SHELF_POINTS SIZE = " << shelf_points_f.size();
          std::vector<cv::Point2d> points_tmp;
          for (size_t k = 0; k < shelf_points_f.size(); k++) {
            points_tmp.insert(points_tmp.end(), shelf_points_f[k].begin(), shelf_points_f[k].end());
          }
          select_points_vector.push_back(points_tmp);
        }
      }
    }
  }
  LOG(INFO) << "SELECT_POINTS_VECTOR SIZE = " << select_points_vector.size();
  // LOG(INFO) << "element size = " << select_points_vector[0].size();
  std::vector<Line> lines = model_manager_->getModelLines(model);
  lines_r = lines;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  double min_error_ = 1000;
  std::vector<cv::Point2d> tmp_vis_point;
  for (size_t i = 0; i < select_points_vector.size(); i++) {
    cv::Point3d target_pose;
    cv::Point2d center = getPointsCenter(select_points_vector[i]);
    std::vector<cv::Point2d> lines_point_set;
    for (size_t j = 0; j < lines.size(); j++) {
      lines_point_set.push_back(lines[j].line.first);
      lines_point_set.push_back(lines[j].line.second);
    }
    cv::Point2d line_set_center = getPointsCenter(lines_point_set);
    cv::Point3d init_pose;
    init_pose.x = center.x - line_set_center.x;
    init_pose.y = center.y - line_set_center.y;
    init_pose.z = 0;
    std::vector<Line> lines_tmp;
    Model model_ = model_manager_->updateTargetModel(model, init_pose);
    lines_tmp = model_manager_->getModelLines(model_);
    double error = getTargetPositionOriginal(
      select_points_vector[i], lines_tmp, target_pose,
      tmp_vis_point);
    if (min_error_ > error) {
      // if( error > 0.4) //&& error > 0.35)
      min_error_ = error;
      vis_input_point = select_points_vector[i];
    }
    cv::Point3d end_pose;
    end_pose.x = init_pose.x * cos(target_pose.z) - init_pose.y * sin(target_pose.z) +
      target_pose.x;
    end_pose.y = init_pose.x * sin(target_pose.z) + init_pose.y * cos(target_pose.z) +
      target_pose.y;
    end_pose.z = init_pose.z + target_pose.z;
    possible_target.push_back(std::make_pair(error, end_pose));
  }
  // for(int i = 0; i < tmp_vis_point.size(); i++)
  // {
  //   vis_input_point.push_back(tmp_vis_point[i]);
  // }


  // LOG(INFO) << "min error = " << min_error_;
  // LOG(INFO) << "VIS INPUT POINT SIZE = " << vis_input_point.size();
  // LOG(INFO) << "possible_target size = " << possible_target.size();


/*
  std::vector<std::vector<std::vector<int>>> component_points;
  for(auto component : model.components)
  {
      std::vector<std::vector<int> > possible_points;
      for(auto pointSet : laserPointSet)
      {
        if(pointSet.first != component.type)
        {
          continue;
        }
        std::vector<cv::Point2d> points_tmp;
        points_tmp.resize(0);
        for(int i = 0; i < pointSet.second.size(); i++)
        {
          points_tmp.push_back(cv::Point2d(msg.sync_points_base_link[pointSet.second[i]][0],
                                           msg.sync_points_base_link[pointSet.second[i]][1]));
        }
        cv::Point2d center = getPointsCenter(points_tmp);
        double distance_error = ppDistance(center, cv::Point2d(0,0));//));
        if (distance_error < 3.0) {
          possible_points.push_back(pointSet.second);
        }
        // possible_points.push_back(pointSet.second);
        // point_set_r.push_back(pointSet.second);
      }
      if(possible_points.size() > 0)
      {
        component_points.push_back(possible_points);
      }
      std::cout << "possible points size = " << possible_points.size() << std::endl;
  }
    std::vector<Line> lines = model_manager_->getModelLines(model);
    lines_r = lines;
    LOG(INFO) << "lines size = " << lines.size();
    if(component_points.size() < 1)
    {
      std::cout << "component points size is 0" << std::endl;
      return shelf_pose_return;
    }
    LOG(INFO) << "component_points size = " << component_points.size();
    // LOG(INFO) << "component points size = " << component_points.size();
    std::vector<std::vector<int> > select_points_set;
    for (int k = 0; k < component_points[0].size(); k++) {
      select_points_set.push_back(component_points[0][k]);
      // LOG(INFO) << "component_points[0][k] size = " << component_points[0][k].size() ;
    }
    LOG(INFO) << "select_points_set size 1 = " << select_points_set.size();
    for(int i = 1; i < component_points.size(); i++)
    {
      std::vector<std::vector<int> > tmp_points_set;
      for(int j = 0; j < select_points_set.size(); j++)
      {
        for(int k = 0; k < component_points[i].size(); k++)
        {
          //LOG(INFO) << "component_points[i].size = " << component_points[i].size();
          std::vector<int> points;
          points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());
          points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
          tmp_points_set.push_back(points);
        }
      }
      select_points_set = tmp_points_set;
    }
    LOG(INFO) << "select_points_set size = " << select_points_set.size();
    std::vector<std::pair<double,cv::Point3d>> possible_target;

    for(int i = 0; i < select_points_set.size(); i++){
      cv::Point3d target_pose;
      std::vector<cv::Point2d> select_points;
      for(int j = 0; j < select_points_set[i].size(); j++)
      {
        select_points.push_back(cv::Point2d(msg.sync_points_base_link[select_points_set[i][j]][0],
                                            msg.sync_points_base_link[select_points_set[i][j]][1]));
      }
      double error = getTargetPositionOriginal(select_points,lines,target_pose);
      possible_target.push_back(std::make_pair(error,target_pose));
    }
  */
  auto min_target_pose = possible_target.front();
  // double min_error = possible_target.front().first;
  for (size_t i = 0; i < possible_target.size(); i++) {
    LOG(INFO) << "POSSIBLE TARGET ERROR = " << possible_target[i].first;
    if (min_target_pose.first > possible_target[i].first) {
      // if(possible_target[i].first  > 0.4 )
      min_target_pose = possible_target[i];
    }
  }
  LOG(INFO) << "min error = " << min_target_pose.first;
  if (min_target_pose.first < max_dis_error_) {
    LOG(INFO) << "target pose update";
    FeatureMsg feature_msg;
    feature_msg.id = 0;
    feature_msg.x = min_target_pose.second.x;
    feature_msg.y = min_target_pose.second.y;
    feature_msg.theta = min_target_pose.second.z;
    feature_msg.target_name = "shape_struct";
    shelf_pose_return.push_back(feature_msg);
  }
  LOG(INFO) << "shelf_pose_return pose = " << shelf_pose_return[0].x << " " <<
    shelf_pose_return[0].y <<
    " " << shelf_pose_return[0].theta;

  return shelf_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::SlideShapeStruct(const std::vector<FeatureMsg> & target_map)
{
  static std::list<FeatureMsg> slide_vector;
  std::vector<FeatureMsg> return_map;
  return_map = target_map;
  std::cout << "target_map size = " << target_map.size();
  std::cout << "gogogoogogogogogogogogogogo" << std::endl;
  static int slide_i = 0;
  std::cout << "slide_i = " << slide_i << std::endl;
  // if(slide_i < 10){
  //   return return_map;
  // }else{
  for (size_t i = 0; i < target_map.size(); i++) {
    if (slide_vector.size() < 10) {
      slide_vector.push_back(target_map[i]);
      slide_i++;
      return return_map;
    } else {
      slide_vector.pop_front();
      slide_vector.push_back(target_map[i]);
      double sum_x = 0;
      double sum_y = 0;
      double sum_theta = 0;
      for (auto target : slide_vector) {
        sum_x += target.x;
        sum_y += target.y;
        sum_theta += target.theta;
      }
      return_map[i].x = sum_x / slide_vector.size();
      return_map[i].y = sum_y / slide_vector.size();
      return_map[i].theta = sum_theta / slide_vector.size();
      std::cout << "return x = " << return_map[i].x <<
        "return y = " << return_map[i].y <<
        "return theta = " << return_map[i].theta <<
        std::endl;
      std::cout << "=======================filte success========================" << std::endl;
      return return_map;
    }
  }
  // }
  return return_map;
}

std::vector<FeatureMsg> RecgByReflec::getShapePoseInLaser(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet,
  std::vector<std::vector<int>> & point_set_r,
  std::vector<Line> & lines_r,
  std::vector<cv::Point2d> & vis_input_point)
{
  LOG(INFO) << "start 2";
  std::vector<FeatureMsg> shape_pose_return;
  std::string target_name = "shapeStruct0";
  Model model;
  // double max_dis_error_ = 0.8;
  model_manager_->getModel(target_name, model);

  std::vector<std::vector<std::vector<int>>> component_points;
  for (auto component : model.components) {
    std::vector<std::vector<int>> possible_points;
    for (auto pointSet : laserPointSet) {
      if (pointSet.first != component.type) {
        continue;
      }
      if (pointSet.second.size() < 20) {
        continue;
      }
      possible_points.push_back(pointSet.second);
      point_set_r.push_back(pointSet.second);
    }
    if (possible_points.size() > 0) {
      component_points.push_back(possible_points);
    }
    std::cout << "possible points size = " << possible_points.size() << std::endl;
  }

  std::vector<Line> lines = model_manager_->getModelLines(model);
  lines_r = lines;
  if (component_points.size() < 1) {
    std::cout << "component points size is 0" << std::endl;
    return shape_pose_return;
  }
  std::vector<std::vector<int>> select_points_set;
  for (size_t k = 0; k < component_points[0].size(); k++) {
    select_points_set.push_back(component_points[0][k]);
  }
  for (size_t i = 1; i < component_points.size(); i++) {
    std::vector<std::vector<int>> tmp_points_set;
    for (size_t j = 0; j < select_points_set.size(); j++) {
      for (size_t k = 0; k < component_points[i].size(); k++) {
        std::vector<int> points;
        points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());
        points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
        tmp_points_set.push_back(points);
      }
    }
    select_points_set = tmp_points_set;
  }
  // shape_pose_return = testMatchingUseDisLimit(msg,select_points_set,lines,model);
  // shape_pose_return = testSetMatching(msg,select_points_set,lines,model,vis_input_point);
  shape_pose_return = testSearchBestSet(msg, select_points_set, lines, model, vis_input_point);
  // std::vector<FeatureMsg> filte_pose_return = SlideShapeStruct(shape_pose_return);
  //  shape_pose_return = testICP(msg,select_points_set,lines,model);
  LOG(INFO) << "target_pose_return size = " << shape_pose_return.size();
  return shape_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::testICP(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::vector<int>> & select_points_set,
  std::vector<Line> & lines,
  Model & model)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  // std::vector<std::pair<double,std::vector<cv::Point2d> > > possible_point;
  std::vector<std::vector<cv::Point2d>> possible_point;

  LOG(INFO) << "select_points_set size = " << select_points_set.size();
  for (size_t i = 0; i < select_points_set.size(); i++) {
    // std::vector<int> select_points = select_points_set[i];
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      select_points.push_back(
        cv::Point2d(
          msg.sync_points_base_link[select_points_set[i][j]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }

    cv::Point3d target_pose(0, 0, 0);
    double error = getTargetPosition_tmp(select_points, lines, target_pose);
    // LOG(INFO) << "error = " << error ;
    // LOG(INFO) << "target_pose.theta = " << target_pose.z;
    possible_target.push_back(std::make_pair(error, target_pose));
    possible_point.push_back(select_points);
  }
  LOG(INFO) << "possible_target size = " << possible_target.size();
  std::vector<cv::Point2d> target_points;
  if (possible_target.size() > 0) {
    cv::Point3d target_pose;
    std::vector<int> select_target_list;
    cv::Point2d predict;
    predict.x = 0;
    predict.y = 0;
    for (size_t i = 0; i < possible_target.size(); i++) {
      cv::Point2d target_pose;
      target_pose.x = possible_target[i].second.x;
      target_pose.y = possible_target[i].second.y;
      if (possible_target[i].first > 0.8) {
        std::cout << "match distance error = " << possible_target[i].first << std::endl;
      } else {
        select_target_list.push_back(i);
      }
    }
    if (select_target_list.size() < 1) {
      std::cout << "select target list size = 0 " << std::endl;
      return shape_pose_return;
    }
    for (size_t i = 0; i < select_target_list.size(); i++) {
      cv::Point2d target1;
      target1.x = possible_target[select_target_list[i]].second.x;
      target1.y = possible_target[select_target_list[i]].second.y;
      for (size_t j = i + 1; j < select_target_list.size(); j++) {
        cv::Point2d target2;
        target2.x = possible_target[select_target_list[j]].second.x;
        target2.y = possible_target[select_target_list[j]].second.y;
        // if(ppDistance(target1,predict) > ppDistance(target2, predict))
        // {
        //   int tmp = select_target_list[i];
        //   select_target_list[i] = select_target_list[j];
        //   select_target_list[j] = tmp;
        // }
        if (possible_target[select_target_list[j]].first <
          possible_target[select_target_list[i]].first)
        {
          int tmp = select_target_list[i];
          select_target_list[i] = select_target_list[j];
          select_target_list[j] = tmp;
          std::vector<cv::Point2d> tmp_points = possible_point[i];
          possible_point[i] = possible_point[j];
          possible_point[j] = tmp_points;
          // target_points = possible_point[select_target_list[j]].second;
        }
      }
    }
    std::vector<cv::Point2d> vis_input_point;
    double min_error = possible_target[select_target_list.front()].first;
    target_pose = possible_target[select_target_list.front()].second;
    target_points = possible_point.front();
    if (min_error < 0.8) {
      Model model_ = model_manager_->updateTargetModel(model, target_pose);
      lines = model_manager_->getModelLines(model_);
      cv::Point3d target_tmp_pose;
      // double error = getTargetPosition(target_points, lines, target_tmp_pose, vis_input_point);
      cv::Point3d final_target_pose;
      final_target_pose.x = target_pose.x * cos(target_tmp_pose.z) - target_pose.y * sin(
        target_tmp_pose.z) + target_tmp_pose.x;
      final_target_pose.y = target_pose.x * sin(target_tmp_pose.z) + target_pose.y * cos(
        target_tmp_pose.z) + target_tmp_pose.y;
      final_target_pose.z = target_pose.z + target_tmp_pose.z;
      std::cout << "target pose update" << std::endl;
      LOG(INFO) << "min error = " << min_error;
      LOG(INFO) << "target pose x = " << final_target_pose.x << " y = " << final_target_pose.y <<
        " theta = " << final_target_pose.z;
      FeatureMsg feature_msg;
      feature_msg.id = 0;
      feature_msg.x = final_target_pose.x;
      feature_msg.y = final_target_pose.y;
      feature_msg.theta = final_target_pose.z;
      LOG(INFO) << "target_pose.z = " << final_target_pose.z;
      feature_msg.target_name = "angle_struct";
      shape_pose_return.push_back(feature_msg);
    } else {
      std::cout << "match error  = " << min_error << std::endl;
    }
  } else {
    std::cout << "possible target size = 0" << std::endl;
  }
  LOG(INFO) << "target_pose_return size = " << shape_pose_return.size();
  return shape_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::testSetMatching(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::vector<int>> & select_points_set,
  std::vector<Line> & lines,
  Model & model,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  // std::vector<std::pair<double,std::vector<cv::Point2d> > > possible_point;
  std::vector<std::vector<cv::Point2d>> possible_point;
  std::vector<std::pair<double, std::vector<cv::Point2d>>> vis_input_v;
  std::vector<cv::Point2d> tmp_vis_point;
  std::vector<cv::Point2d> corner_points;
  std::vector<Line> lines_tmp = lines;

  LOG(INFO) << "select_points_set size = " << select_points_set.size();
  for (size_t i = 0; i < select_points_set.size(); i++) {
    // std::vector<int> select_points = select_points_set[i];
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      select_points.push_back(
        cv::Point2d(
          msg.sync_points_base_link[select_points_set[i][j]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }

    cv::Point3d target_tmp_pose(0, 0, 0);
    cv::Point2d point_test, model_point_test;
    // findCorner(select_points,lines,target_tmp_pose,point_test,model_point_test);
    getTargetPosition_first(select_points, lines, target_tmp_pose, corner_points);
    // findInitTransPose(select_points,lines,target_tmp_pose);
    Model model_ = model_manager_->updateTargetModel(model, target_tmp_pose);
    lines_tmp = model_manager_->getModelLines(model_);
    // cv::Point3d final_target_pose;
    // final_target_pose.x =
    //   target_pose.x * cos(target_tmp_pose.z) -
    //   target_pose.y*sin(target_tmp_pose.z) +
    //   target_tmp_pose.x;
    // final_target_pose.y =
    //   target_pose.x * sin(target_tmp_pose.z) +
    //   target_pose.y*cos(target_tmp_pose.z) +
    //   target_tmp_pose.y;
    // final_target_pose.z = target_pose.z + target_tmp_pose.z;

    cv::Point3d target_pose(0, 0, 0);
    cv::Point3d final_target_pose(0, 0, 0);
    double score = getTargetPosition(select_points, lines_tmp, final_target_pose, tmp_vis_point);
    for (size_t m = 0; m < tmp_vis_point.size(); m++) {
      Eigen::Matrix3d m_matrix;
      m_matrix << cos(target_tmp_pose.z), -sin(target_tmp_pose.z), target_tmp_pose.x,
        sin(target_tmp_pose.z), cos(target_tmp_pose.z), target_tmp_pose.y,
        0, 0, 1;
      Eigen::Vector3d m_vetcor(tmp_vis_point[m].x, tmp_vis_point[m].y, 1);
      Eigen::Vector3d m_final = m_matrix.inverse() * m_vetcor;
      cv::Point2d tmp_point;
      tmp_point.x = m_final(0);
      tmp_point.y = m_final(1);
      // tmp_point.x = tmp_vis_point[m].x * cos(-target_tmp_pose.z) -
      //   tmp_vis_point[m].y * sin(-target_tmp_pose.z) - final_target_pose.x;
      // tmp_point.y = tmp_vis_point[m].x * sin(-target_tmp_pose.z) +
      //   tmp_vis_point[m].y * cos(-target_tmp_pose.z) - final_target_pose.y;
      tmp_vis_point[m] = tmp_point;
    }
    target_pose.x = target_tmp_pose.x * cos(final_target_pose.z) - target_tmp_pose.y * sin(
      final_target_pose.z) + final_target_pose.x;
    target_pose.y = target_tmp_pose.x * sin(final_target_pose.z) + target_tmp_pose.y * cos(
      final_target_pose.z) + final_target_pose.y;
    target_pose.z = target_tmp_pose.z + final_target_pose.z;
    std::cout << "point size = " << select_points.size() << std::endl;
    LOG(INFO) << "score = " << score;
    LOG(INFO) << "target_pose.theta = " << target_pose.z;
    possible_target.push_back(std::make_pair(score, target_pose));
    possible_point.push_back(select_points);
    vis_input_v.push_back(std::make_pair(score, tmp_vis_point));
  }
  LOG(INFO) << "possible_target size = " << possible_target.size();
  vis_input_point = corner_points;
  std::vector<cv::Point2d> target_points;
  if (possible_target.size() > 0) {
    cv::Point3d target_pose;
    std::vector<int> select_target_list;
    cv::Point2d predict;
    predict.x = 0;
    predict.y = 0;
    for (size_t i = 0; i < possible_target.size(); i++) {
      cv::Point2d target_pose;
      target_pose.x = possible_target[i].second.x;
      target_pose.y = possible_target[i].second.y;
      if (possible_target[i].first < 0.3) {
        std::cout << "match distance error = " << possible_target[i].first << std::endl;
      } else {
        select_target_list.push_back(i);
      }
    }
    if (select_target_list.size() < 1) {
      std::cout << "select target list size = 0 " << std::endl;
      return shape_pose_return;
    }
    for (size_t i = 0; i < select_target_list.size(); i++) {
      cv::Point2d target1;
      target1.x = possible_target[select_target_list[i]].second.x;
      target1.y = possible_target[select_target_list[i]].second.y;
      for (size_t j = i + 1; j < select_target_list.size(); j++) {
        cv::Point2d target2;
        target2.x = possible_target[select_target_list[j]].second.x;
        target2.y = possible_target[select_target_list[j]].second.y;
        // if(ppDistance(target1,predict) > ppDistance(target2, predict))
        // {
        //   int tmp = select_target_list[i];
        //   select_target_list[i] = select_target_list[j];
        //   select_target_list[j] = tmp;
        // }
        if (possible_target[select_target_list[j]].first >
          possible_target[select_target_list[i]].first)
        {
          int tmp = select_target_list[i];
          select_target_list[i] = select_target_list[j];
          select_target_list[j] = tmp;
          std::vector<cv::Point2d> tmp_points = possible_point[i];
          possible_point[i] = possible_point[j];
          possible_point[j] = tmp_points;
          // target_points = possible_point[select_target_list[j]].second;
        }
      }
    }

    for (size_t i = 0; i < vis_input_v.size(); i++) {
      for (size_t j = i + 1; j < vis_input_v.size(); j++) {
        if (vis_input_v[j].first > vis_input_v[i].first) {
          std::pair<double, std::vector<cv::Point2d>> tmp_vis = vis_input_v[i];
          vis_input_v[i] = vis_input_v[j];
          vis_input_v[j] = tmp_vis;
        }
      }
    }
    for (size_t i = 0; i < vis_input_v.front().second.size(); i++) {
      vis_input_point.push_back(vis_input_v.front().second[i]);
    }
    double min_error = possible_target[select_target_list.front()].first;
    std::cout << "best score = " << min_error << std::endl;
    target_pose = possible_target[select_target_list.front()].second;
    target_points = possible_point.front();
    // vis_input_point = corner_points;
    if (min_error > 0.7) {
      // Model model_ = model_manager_->updateTargetModel(model,target_pose);
      // lines = model_manager_->getModelLines(model_);
      // cv::Point3d target_tmp_pose;
      // double error = getTargetPosition(target_points, lines, target_tmp_pose);
      // cv::Point3d final_target_pose;
      // final_target_pose.x = target_pose.x * cos(target_tmp_pose.z) -
      //   target_pose.y*sin(target_tmp_pose.z) + target_tmp_pose.x;
      // final_target_pose.y = target_pose.x * sin(target_tmp_pose.z) +
      //   target_pose.y*cos(target_tmp_pose.z) + target_tmp_pose.y;
      // final_target_pose.z = target_pose.z + target_tmp_pose.z;
      // std::cout << "target pose update" << std::endl;
      // LOG(INFO) << "min error = " << min_error;
      // LOG(INFO) << "target pose x = " <<
      //   final_target_pose.x << " y = " << final_target_pose.y
      // << " theta = " << final_target_pose.z ;
      FeatureMsg feature_msg;
      feature_msg.id = 0;
      feature_msg.x = target_pose.x;
      feature_msg.y = target_pose.y;
      feature_msg.theta = target_pose.z;
      // LOG(INFO) << "target_pose.z = " << target_pose.z;
      feature_msg.target_name = "angle_struct";
      shape_pose_return.push_back(feature_msg);
      // vis_input_point = vis_input_v.front().second;
      // vis_input_point = corner_points;
      std::cout << "vis_input_point score = " << vis_input_v.front().first << std::endl;
      std::cout << "vis_input_point size = " << vis_input_point.size() << std::endl;
      std::cout << "possible target size = " << possible_point.size() << std::endl;
      std::cout << "vis_input_v size = " << vis_input_v.size() << std::endl;
    } else {
      std::cout << "match score  = " << min_error << std::endl;
    }
  } else {
    std::cout << "possible target size = 0" << std::endl;
  }
  LOG(INFO) << "target_pose_return size = " << shape_pose_return.size();
  return shape_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::testMatchingUseDisLimit(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::vector<int>> & select_points_set,
  std::vector<Line> & lines,
  Model & model)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  std::vector<std::vector<cv::Point2d>> possible_point;
  std::vector<cv::Point2d> points;
  std::vector<cv::Point2d> vis_input_point;
  for (auto line : lines) {
    points.push_back(line.line.first);
    points.push_back(line.line.second);
  }
  double feature_dis_model = 0;
  for (size_t i = 0; i < points.size(); i++) {
    for (size_t j = i + 1; j < points.size(); j++) {
      double tmp_dis = hypot(
        points[j].x - points[i].x,
        points[j].y - points[i].y);
      if (tmp_dis > feature_dis_model) {
        feature_dis_model = tmp_dis;
      }
    }
  }

  double feature_dis_points = 0;
  for (size_t i = 0; i < select_points_set.size(); i++) {
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      for (size_t k = j + 1; k < select_points_set[i].size(); k++) {
        double tmp_dis = hypot(
          msg.sync_points_base_link[select_points_set[i][j]][0] -
          msg.sync_points_base_link[select_points_set[i][k]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1] -
          msg.sync_points_base_link[select_points_set[i][k]][1]);
        if (tmp_dis > feature_dis_points) {
          feature_dis_points = tmp_dis;
        }
      }
    }
    if (fabs(feature_dis_model - feature_dis_points) > 0.1) {
      std::cout << "feature_dis not match!" << std::endl;
      continue;
    }
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      select_points.push_back(
        cv::Point2d(
          msg.sync_points_base_link[select_points_set[i][j]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }

    cv::Point3d target_pose(0, 0, 0);
    double score = getTargetPosition(select_points, lines, target_pose, vis_input_point);
    LOG(INFO) << "error = " << score;
    LOG(INFO) << "target_pose.theta = " << target_pose.z;
    possible_target.push_back(std::make_pair(score, target_pose));
    possible_point.push_back(select_points);
  }

  LOG(INFO) << "possible_target size = " << possible_target.size();
  std::vector<cv::Point2d> target_points;
  if (possible_target.size() > 0) {
    cv::Point3d target_pose;
    std::vector<int> select_target_list;
    cv::Point2d predict;
    predict.x = 0;
    predict.y = 0;
    for (size_t i = 0; i < possible_target.size(); i++) {
      cv::Point2d target_pose;
      target_pose.x = possible_target[i].second.x;
      target_pose.y = possible_target[i].second.y;
      if (possible_target[i].first > 0.8) {
        std::cout << "match distance error = " << possible_target[i].first << std::endl;
      } else {
        select_target_list.push_back(i);
      }
    }
    if (select_target_list.size() < 1) {
      std::cout << "select target list size = 0 " << std::endl;
      return shape_pose_return;
    }
    for (size_t i = 0; i < select_target_list.size(); i++) {
      cv::Point2d target1;
      target1.x = possible_target[select_target_list[i]].second.x;
      target1.y = possible_target[select_target_list[i]].second.y;
      for (size_t j = i + 1; j < select_target_list.size(); j++) {
        cv::Point2d target2;
        target2.x = possible_target[select_target_list[j]].second.x;
        target2.y = possible_target[select_target_list[j]].second.y;
        // if(ppDistance(target1,predict) > ppDistance(target2, predict))
        // {
        //   int tmp = select_target_list[i];
        //   select_target_list[i] = select_target_list[j];
        //   select_target_list[j] = tmp;
        // }
        if (possible_target[select_target_list[j]].first >
          possible_target[select_target_list[i]].first)
        {
          int tmp = select_target_list[i];
          select_target_list[i] = select_target_list[j];
          select_target_list[j] = tmp;
          std::vector<cv::Point2d> tmp_points = possible_point[i];
          possible_point[i] = possible_point[j];
          possible_point[j] = tmp_points;
          // target_points = possible_point[select_target_list[j]].second;
        }
      }
    }
    double min_error = possible_target[select_target_list.front()].first;
    target_pose = possible_target[select_target_list.front()].second;
    target_points = possible_point.front();
    if (min_error > 0.5) {
      FeatureMsg feature_msg;
      feature_msg.id = 0;
      feature_msg.x = target_pose.x;
      feature_msg.y = target_pose.y;
      feature_msg.theta = target_pose.z;
      LOG(INFO) << "target_pose.z = " << target_pose.z;
      feature_msg.target_name = "angle_struct";
      shape_pose_return.push_back(feature_msg);
    } else {
      std::cout << "match error  = " << min_error << std::endl;
    }
  } else {
    std::cout << "possible target size = 0" << std::endl;
  }

  return shape_pose_return;
}

double RecgByReflec::getTargetPosition(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition,
  std::vector<cv::Point2d> & vis_input_point)
{
  double R[4] = {1.f, 0.f, 0.f, 1.f}, T[2] = {0., 0.};
  cv::Mat r = cv::Mat(2, 2, CV_64F, R);
  cv::Mat t = cv::Mat(2, 1, CV_64F, T);
  double err =
    icp_test(
    selectTargetPoints, lines, r, t, cv::TermCriteria(
      cv::TermCriteria::Type::COUNT, 1000,
      0.0001), vis_input_point,
    shape_match_close_degree_);
  LOG(INFO) << "err = " << err;
  // Eigen::Isometry2d iso2d_1 = Eigen::Isometry2d::Identity();
  // iso2d_1.rotate(targetPosition.z);
  // iso2d_1.pretranslate(Eigen::Vector2d(targetPosition.x,targetPosition.y));
  // Eigen::Isometry2d iso2d_2 = Eigen::Isometry2d::Identity();
  // Eigen::Matrix2d rotation_m;
  // rotation_m << R[0],R[1],R[2],R[3];
  // Eigen::Rotation2Dd rotation_2d(rotation_m);
  // LOG(INFO) << "err = " << err;
  // iso2d_2.rotate(rotation_m);
  // iso2d_2.pretranslate(Eigen::Vector2d(T[0],T[1]));
  // Eigen::Isometry2d iso2d_3 = iso2d_2.inverse()*iso2d_1;
  // targetPosition.x = iso2d_3.translation()[0];
  // targetPosition.y = iso2d_3.translation()[1];
  // Eigen::Rotation2Dd rotation_final(iso2d_3.rotation());
  // targetPosition.z = rotation_final.angle();


  double x = (R[0] * targetPosition.x + R[1] * targetPosition.y + T[0]);
  double y = (R[2] * targetPosition.x + R[3] * targetPosition.y + T[1]);
  double theta = targetPosition.z + atan2(-R[1], R[0]);
  targetPosition.x = x;
  targetPosition.y = y;
  targetPosition.z = theta;
  return err;
}

double RecgByReflec::getTargetPositionOriginal(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition,
  std::vector<cv::Point2d> & vis_input_point)
{
  double R[4] = {1.f, 0.f, 0.f, 1.f}, T[2] = {0., 0.};
  cv::Mat r = cv::Mat(2, 2, CV_64F, R);
  cv::Mat t = cv::Mat(2, 1, CV_64F, T);
  double err =
    icp_original(
    selectTargetPoints, lines, r, t,
    cv::TermCriteria(cv::TermCriteria::Type::COUNT, 1000, 0.0001), vis_input_point);
  double x = (R[0] * targetPosition.x + R[1] * targetPosition.y + T[0]);
  double y = (R[2] * targetPosition.x + R[3] * targetPosition.y + T[1]);
  double theta = targetPosition.z + atan2(-R[1], R[0]);
  targetPosition.x = x;
  targetPosition.y = y;
  targetPosition.z = theta;
  return err;
}

double RecgByReflec::getTargetPosition_tmp(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition)
{
  double R[4] = {1.f, 0.f, 0.f, 1.f}, T[2] = {0., 0.};
  cv::Mat r = cv::Mat(2, 2, CV_64F, R);
  cv::Mat t = cv::Mat(2, 1, CV_64F, T);
  double err =
    icp_tmp(
    selectTargetPoints, lines, r, t,
    cv::TermCriteria(cv::TermCriteria::Type::COUNT, 1000, 0.0001));
  double x = (R[0] * targetPosition.x + R[1] * targetPosition.y + T[0]);
  double y = (R[2] * targetPosition.x + R[3] * targetPosition.y + T[1]);
  double theta = targetPosition.z + atan2(-R[1], R[0]);
  Eigen::Isometry2d iso2d_1 = Eigen::Isometry2d::Identity();
  iso2d_1.rotate(targetPosition.z);
  iso2d_1.pretranslate(Eigen::Vector2d(targetPosition.x, targetPosition.y));
  Eigen::Isometry2d iso2d_2 = Eigen::Isometry2d::Identity();
  Eigen::Matrix2d rotation_m;
  rotation_m << R[0], R[1], R[2], R[3];
  Eigen::Rotation2Dd rotation_2d(rotation_m);
  LOG(INFO) << "icp rotation theta = " << rotation_2d.angle();
  iso2d_2.rotate(rotation_m);
  iso2d_2.pretranslate(Eigen::Vector2d(T[0], T[1]));
  Eigen::Isometry2d iso2d_3 = iso2d_2 * iso2d_1;
  targetPosition.x = x;
  targetPosition.y = y;
  targetPosition.z = theta;
  targetPosition.x = iso2d_3.translation()[0];
  targetPosition.y = iso2d_3.translation()[1];
  Eigen::Rotation2Dd rotation_final(iso2d_3.rotation());
  targetPosition.z = rotation_final.angle();
  return err;
}

double RecgByReflec::getTargetPosition_first(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition,
  std::vector<cv::Point2d> & vis_points)
{
  cv::Point2d corner_in_lines;
  cv::Point2d corner_in_points;
  cv::Point3d targetPosition_3;
  findCorner(selectTargetPoints, lines, targetPosition_3, corner_in_lines, corner_in_points);
  std::vector<cv::Point2d> first_input_point;
  std::vector<cv::Point2d> first_model_point;
  first_input_point.push_back(selectTargetPoints.back());
  // first_input_point.push_back(selectTargetPoints[selectTargetPoints.size()/2]);
  first_input_point.push_back(corner_in_points);
  first_input_point.push_back(selectTargetPoints.front());
  first_model_point.push_back(
    cv::Point2d(lines.front().line.first.x, lines.front().line.first.y));
  // if(lines.size()/2 == 0)
  // {
  //   first_model_point.push_back(cv::Point2d(lines[lines.size()/2].line.first));
  // }else{
  // cv::Point2d point(
  //   (lines[lines.size()/2].line.first.x + lines[lines.size()/2].line.second.x)/2,
  //   (lines[lines.size()/2].line.first.y + lines[lines.size()/2].line.second.y)/2);
  //   first_model_point.push_back(point);
  // }
  // first_model_point.push_back(cv::Point2d(lines[lines.size()/2].line.first));
  first_model_point.push_back(corner_in_lines);
  first_model_point.push_back(cv::Point2d(lines.back().line.second.x, lines.back().line.second.y));
  // cv::Mat r,t;
  // Model model_ = model_manager)_->updateTargetModel(model,target_pose);
  // lines = model_manager_->getModelLines(model_);
  for (size_t i = 0; i < first_input_point.size(); i++) {
    vis_points.push_back(first_input_point[i]);
  }
  for (size_t i = 0; i < first_model_point.size(); i++) {
    vis_points.push_back(first_model_point[i]);
  }
  double R[4] = {1.f, 0.f, 0.f, 1.f}, T[2] = {0., 0.};
  cv::Mat r = cv::Mat(2, 2, CV_64F, R);
  cv::Mat t = cv::Mat(2, 1, CV_64F, T);
  getRTMatrixSVD(
    &first_model_point[0], &first_input_point[0], first_input_point.size(),
    r, t);
  // double err =
  //   icp_tmp(selectTargetPoints, lines, r, t,
  //   cv::TermCriteria(cv::TermCriteria::Type::COUNT, 1000, 0.0001));
  double x = (R[0] * targetPosition.x + R[1] * targetPosition.y + T[0]);
  double y = (R[2] * targetPosition.x + R[3] * targetPosition.y + T[1]);
  double theta = targetPosition.z + atan2(-R[1], R[0]);
  Eigen::Isometry2d iso2d_1 = Eigen::Isometry2d::Identity();
  iso2d_1.rotate(targetPosition.z);
  iso2d_1.pretranslate(Eigen::Vector2d(targetPosition.x, targetPosition.y));
  Eigen::Isometry2d iso2d_2 = Eigen::Isometry2d::Identity();
  Eigen::Matrix2d rotation_m;
  rotation_m << R[0], R[1], R[2], R[3];
  Eigen::Rotation2Dd rotation_2d(rotation_m);
  // LOG(INFO) << "icp rotation theta = " << rotation_2d.angle();
  iso2d_2.rotate(rotation_m);
  iso2d_2.pretranslate(Eigen::Vector2d(T[0], T[1]));
  Eigen::Isometry2d iso2d_3 = iso2d_2 * iso2d_1;
  targetPosition.x = x;
  targetPosition.y = y;
  targetPosition.z = theta;
  targetPosition.x = iso2d_3.translation()[0];
  targetPosition.y = iso2d_3.translation()[1];
  Eigen::Rotation2Dd rotation_final(iso2d_3.rotation());
  targetPosition.z = rotation_final.angle();
  vis_points.push_back(cv::Point2d(targetPosition.x, targetPosition.y));
  return 0;
}

double RecgByReflec::findInitTransPose(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition)
{
  double p_x = 0, p_y = 0;
  double l_x = 0, l_y = 0;
  for (size_t i = 0; i < selectTargetPoints.size(); i++) {
    double x = selectTargetPoints[i].x;
    double y = selectTargetPoints[i].y;
    p_x += x;
    p_y += y;
  }
  p_x = p_x / selectTargetPoints.size();
  p_y = p_y / selectTargetPoints.size();

  for (auto line : lines) {
    double x = line.line.first.x;
    l_x += x;
    x = line.line.second.x;
    l_x += x;

    double y = line.line.second.y;
    l_y += y;
    y = line.line.second.y;
    l_y += y;
  }
  l_x = l_x / (2 * lines.size());
  l_y = l_y / (2 * lines.size());
  targetPosition.x = p_x - l_x;
  targetPosition.y = p_y - l_y;
  targetPosition.z = 0;
  return 0;
}

double RecgByReflec::findCorner(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition, cv::Point2d & corner_in_line,
  cv::Point2d & corner_in_points)
{
  cv::Point2d point_begin = selectTargetPoints.front();
  cv::Point2d point_end = selectTargetPoints.back();
  cv::Point2d point_to_match;
  double point_max_dis = 0;
  // int find_i = 0;
  std::vector<cv::Point2d> first_set_of_whole_set;
  std::vector<cv::Point2d> second_set_of_whole_set;
  for (size_t i = 0; i < selectTargetPoints.size(); i++) {
    cv::Point2d nearest_point;
    double seg_dis;
    seg_dis = pointToSegDist(selectTargetPoints[i], point_begin, point_end, nearest_point);
    if (point_max_dis < seg_dis) {
      point_max_dis = seg_dis;
      point_to_match = selectTargetPoints[i];
      // find_i = i;
    }
  }

  // std::vector<cv::Point2d> tmp_v = selectTargetPoints;
  // while((selectTargetPoints.size()/2 - find_i) > 10){
  //   auto it = tmp_v.begin();
  //   for(int i = 0; i < find_i; i++){
  //     it++;
  //   }
  //   it = tmp_v.erase(it);
  //   for(int i = 0; i < tmp_v.size(); i++)
  //   {
  //     cv::Point2d nearest_point;
  //     double seg_dis;
  //     seg_dis = pointToSegDist(tmp_v[i],point_begin,point_end,nearest_point);
  //     if(point_max_dis < seg_dis)
  //     {
  //       point_max_dis = seg_dis;
  //       point_to_match = tmp_v[i];
  //       find_i = i;
  //     }
  //   }
  // }
  // for(int i = 0; i < selectTargetPoints.size(); i++)
  // {
  //   if(i < find_i)
  //   {
  //     first_set_of_whole_set.push_back(selectTargetPoints[i]);
  //   }else{
  //     second_set_of_whole_set.push_back(selectTargetPoints[i]);
  //   }

  // }
  cv::Point2d line_begin = lines.front().line.first;
  cv::Point2d line_end = lines.back().line.second;

  std::vector<cv::Point2d> line_point;
  for (auto line : lines) {
    line_point.push_back(line.line.first);
    line_point.push_back(line.line.second);
  }

  double line_max_dis = 0;
  cv::Point2d line_point_to_match;
  for (size_t i = 0; i < line_point.size(); i++) {
    cv::Point2d nearest_point;
    double seg_dis;
    seg_dis = pointToSegDist(line_point[i], line_begin, line_end, nearest_point);
    if (line_max_dis < seg_dis) {
      line_max_dis = seg_dis;
      line_point_to_match = line_point[i];
    }
  }
  corner_in_line = line_point_to_match;
  corner_in_points = point_to_match;
  targetPosition.x = point_to_match.x - line_point_to_match.x;
  targetPosition.y = point_to_match.y - line_point_to_match.y;
  targetPosition.z = 0;
  return 0;
}

std::vector<FeatureMsg> RecgByReflec::testShelfRec(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::vector<int>> & select_points_set,
  std::vector<Line> & lines,
  Model & model,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> shelf_pose_return;

  for (size_t i = 0; i < select_points_set.size(); i++) {
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      select_points.push_back(
        cv::Point2d(
          msg.sync_points_base_link[select_points_set[i][j]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }
    return shelf_pose_return;
// msg.sync_points_base_link[select_points_set[i][j]][0],
// msg.sync_points_base_link[select_points_set[i][j]][1]
  }
  return shelf_pose_return;
}

std::vector<FeatureMsg> RecgByReflec::testSearchBestSet(
  const sensor::PointCloudWithOrigin & msg,
  std::vector<std::vector<int>> & select_points_set,
  std::vector<Line> & lines,
  Model & model,
  std::vector<cv::Point2d> & vis_input_point)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  std::vector<std::vector<cv::Point2d>> possible_point;
  std::vector<std::vector<cv::Point2d>> vis_input_v_2;
  std::vector<Line> lines_tmp = lines;

  LOG(INFO) << "select_points_set size = " << select_points_set.size();
  for (size_t i = 0; i < select_points_set.size(); i++) {
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      // LOG(INFO) << "SELECT_POINTS_SET.SIZE = " << select_points_set[i].size();
      select_points.push_back(
        cv::Point2d(
          msg.sync_points_base_link[select_points_set[i][j]][0],
          msg.sync_points_base_link[select_points_set[i][j]][1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }
    LOG(INFO) << "========================SELECT_POINTS SIZE = " << select_points.size();
    cv::Point3d targetPosition_, targetPosition_tmp;
    cv::Point2d corner_in_line;
    cv::Point2d corner_in_points;
    findCorner(select_points, lines, targetPosition_, corner_in_line, corner_in_points);
    Model model_ = model_manager_->updateTargetModel(model, targetPosition_);
    lines_tmp = model_manager_->getModelLines(model_);
    vis_input_point.push_back(cv::Point2d(targetPosition_.x, targetPosition_.y));
    vis_input_point.push_back(corner_in_line);
    vis_input_point.push_back(corner_in_points);
    // double theta = 0.0;
    findBestAngleOfSet(select_points, model, targetPosition_, lines_tmp, targetPosition_tmp);
    // theta = targetPosition_tmp.z;
    for (auto line : lines) {
      vis_input_point.push_back(line.line.first);
      vis_input_point.push_back(line.line.second);
    }
    std::vector<cv::Point2d> tmp_vis_point;
    cv::Point3d end_target_pose(0, 0, 0);
    cv::Point3d target_pose(0, 0, 0);
    // double score = getTargetPosition(select_points,lines_tmp,end_target_pose,tmp_vis_point);
    double score = getTargetPosition(select_points, lines_tmp, targetPosition_tmp, tmp_vis_point);
    for (size_t m = 0; m < tmp_vis_point.size(); m++) {
      Eigen::Matrix3d m_matrix;
      m_matrix << cos(targetPosition_tmp.z), -sin(targetPosition_tmp.z), targetPosition_tmp.x,
        sin(targetPosition_tmp.z), cos(targetPosition_tmp.z), targetPosition_tmp.y,
        0, 0, 1;
      Eigen::Vector3d m_vetcor(tmp_vis_point[m].x, tmp_vis_point[m].y, 1);
      Eigen::Vector3d m_final = m_matrix.inverse() * m_vetcor;
      cv::Point2d tmp_point;
      tmp_point.x = m_final(0);
      tmp_point.y = m_final(1);
      // tmp_point.x = tmp_vis_point[m].x * cos(-target_tmp_pose.z) -
      //   tmp_vis_point[m].y * sin(-target_tmp_pose.z) - final_target_pose.x;
      // tmp_point.y = tmp_vis_point[m].x * sin(-target_tmp_pose.z) +
      //   tmp_vis_point[m].y * cos(-target_tmp_pose.z) - final_target_pose.y;
      tmp_vis_point[m] = tmp_point;
    }
    // target_pose.x = targetPosition_tmp.x * cos(end_target_pose.z) -
    //   targetPosition_tmp.y*sin(end_target_pose.z) + end_target_pose.x;
    // target_pose.y = targetPosition_tmp.x * sin(end_target_pose.z) +
    //   targetPosition_tmp.y*cos(end_target_pose.z) + end_target_pose.y;
    // target_pose.z = targetPosition_tmp.z + end_target_pose.z;

    target_pose.x = targetPosition_tmp.x;
    target_pose.y = targetPosition_tmp.y;
    target_pose.z = targetPosition_tmp.z;

    // LOG(INFO) << "end_target_pose = " << end_target_pose.x << " " << end_target_pose.y
    //           << " " << end_target_pose.z;
    possible_target.push_back(std::make_pair(score, target_pose));
    possible_point.push_back(select_points);
    vis_input_v_2.push_back(tmp_vis_point);
    // vis_input_v.push_back(std::pair<double, std::vector<cv::Point2d>>(score,tmp_vis_point));
    // vis_input_v.push_back(std::make_pair(score,tmp_vis_point));
    // vis_input_v.push_back(std::make_pair(score,select_points));
  }
  // LOG(INFO) << "possible_target size = " << possible_target.size();
  for (size_t i = 0; i < possible_target.size(); i++) {
    if (possible_target[i].first < shape_match_score_) {
      // std::cout << "score is too low" << std::endl;
    } else {
      cv::Point3d target_pose;
      target_pose = possible_target[i].second;
      FeatureMsg feature_msg;
      feature_msg.id = 0;
      feature_msg.x = target_pose.x;
      feature_msg.y = target_pose.y;
      feature_msg.theta = target_pose.z;

      // feature_msg.target_name = "angle_struct";
      feature_msg.target_name = "shape_struct";
      shape_pose_return.push_back(feature_msg);
      // std::cout << "score = " << possible_target[i].first << std::endl;
      for (size_t j = 0; j < vis_input_v_2[i].size(); j++) {
        vis_input_point.push_back(vis_input_v_2[i][j]);
      }
      // LOG(INFO) << "msg.sync_points_base_link.size = " << msg.sync_points_base_link.size();
    }
    // for(int j = 0; j < vis_input_v[j].second.size(); j++)
    // {
    //   vis_input_point.push_back(vis_input_v[i].second[j]);
    // }
  }

  // LOG(INFO) << "SHAPE_POSE_RETURN POSE = " << shape_pose_return.size();
  return shape_pose_return;
}

double RecgByReflec::ComputeScoreOfAngle(
  const std::vector<cv::Point2d> & points,
  const std::vector<Line> & lines)
{
  double score = 0;
  std::vector<cv::Point2d> intersection_set;
  std::vector<cv::Point2d> union_set;
  for (size_t i = 0; i < points.size(); i++) {
    // double tmp_dis = DBL_MAX;
    // double min_dis = DBL_MAX;
    double tmpDist = 0.0;
    double nearestDist = DBL_MAX;
    bool in_line = false;
    bool tmp_in_line = false;
    cv::Point2d tmpPoint;
    cv::Point2d nearestPoint;
    for (auto line : lines) {
      if (line.type == "line") {
        tmpDist = pointToLine(points[i], line.line.first, line.line.second, tmpPoint, tmp_in_line);
      }
      if (nearestDist > tmpDist) {
        nearestDist = tmpDist;
        nearestPoint = tmpPoint;
        in_line = tmp_in_line;
      }
    }
    if (nearestDist < 0.005 && in_line == true) {
      intersection_set.push_back(nearestPoint);
    }
  }
  double intersection_sum_dis = 0;
  for (size_t i = 1; i < intersection_set.size(); i++) {
    double dis = hypot(
      intersection_set[i].x - intersection_set[i - 1].x,
      intersection_set[i].y - intersection_set[i - 1].y);
    if (dis < 0.05) {
      intersection_sum_dis += dis;
    }
  }
  double model_sum_dis = 0;
  for (auto line : lines) {
    double dis = hypot(
      line.line.first.x - line.line.second.x,
      line.line.first.y - line.line.second.y);
    model_sum_dis += dis;
  }
  score = intersection_sum_dis / model_sum_dis;
  return score;
}

double RecgByReflec::findBestAngleOfSet(
  const std::vector<cv::Point2d> selectTargetPoints,
  Model & model,
  const cv::Point3d & targetPosition,
  std::vector<Line> & return_lines,
  cv::Point3d & return_target_position)
{
  cv::Point3d targetPosition_tmp = targetPosition;
  std::vector<Line> lines;
  Model model_ = model_manager_->updateTargetModel(model, targetPosition_tmp);
  lines = model_manager_->getModelLines(model_);
  double max_score = ComputeScoreOfAngle(selectTargetPoints, lines);
  double step = 0.005;
  double condition_ = 0;
  double angle = targetPosition.z + step;
  targetPosition_tmp.z = angle;
  model_ = model_manager_->updateTargetModel(model, targetPosition_tmp);
  lines = model_manager_->getModelLines(model_);
  double score = ComputeScoreOfAngle(selectTargetPoints, lines);
  if (score < max_score) {
    step = -step;
  }
  if (step > 0) {
    condition_ = targetPosition.z + 6.28;
    for (; angle < condition_; angle = angle + step) {
      targetPosition_tmp.z = angle;
      model_ = model_manager_->updateTargetModel(model, targetPosition_tmp);
      lines = model_manager_->getModelLines(model_);
      score = ComputeScoreOfAngle(selectTargetPoints, lines);
      if (score > max_score) {
        max_score = score;
        return_lines = lines;
        return_target_position = targetPosition_tmp;
      }
    }
  } else {
    condition_ = targetPosition.z - 6.28;
    for (; angle > condition_; angle = angle + step) {
      targetPosition_tmp.z = angle;
      model_ = model_manager_->updateTargetModel(model, targetPosition_tmp);
      lines = model_manager_->getModelLines(model_);
      score = ComputeScoreOfAngle(selectTargetPoints, lines);
      if (score > max_score) {
        max_score = score;
        return_lines = lines;
        return_target_position = targetPosition_tmp;
      }
    }
  }
  return 0;
}


}  // namespace laser_perception
}  // cartographer
