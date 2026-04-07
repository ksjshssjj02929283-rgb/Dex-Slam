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
#include <utility>
#include <vector>
#include <string>
#include "perception_2d/laser_perception_2d/recg_by_pre_point.hpp"
#include "perception_2d/laser_perception_2d/math_tool.hpp"

namespace perception_2d
{
RecgByPrePoint::RecgByPrePoint(const Config config)
{
  search_distance_ = config.search_distance;
  max_dis_error_ = config.max_dis_error;
  max_ang_error_ = config.max_ang_error;
  start_rec_dis_ = config.start_rec_dis;
  min_select_points_ = config.min_select_points;
  model_file_ = config.model_file;
  model_manager_ = ModelManager::instance(config);
}

void RecgByPrePoint::UpdateParams(
  const std::string & param_name, double & value)
{
  if (param_name == "search_distance") {
    search_distance_ = value;
  } else if (param_name == "max_error") {
    max_dis_error_ = value;
  } else if (param_name == "search_angle") {
    max_ang_error_ = value;
  } else if (param_name == "start_rec_dis") {
    start_rec_dis_ = value;
  }
}

PrePointMsg RecgByPrePoint::GetTargetByPredictPose(
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet)
{
//  printf("get target by predict pose\n");
  PrePointMsg target_map;
  std::vector<FeatureInfo> target_list;
  {
    std::lock_guard<std::mutex> lock(update_target_list_mutex_);
    target_list = target_list_;
  }

  // 对于每一个目标模型
  for (auto target : target_list) {
    // 识别距离判断
    // cv::Point3d robot_pose;
    transform::Rigid3d robot_pose;
    robot_pose = odom_in_map_ * odom_.pose;
    // 判断该模型是否在识别范围内
    if (ppDistance(
        cv::Point2d(robot_pose.translation()[0], robot_pose.translation()[1]),
        cv::Point2d(target.pose.x, target.pose.y)) > start_rec_dis_)
    {
      LOG(INFO) << "robot x: " << robot_pose.translation()[0] <<
        " y: " << robot_pose.translation()[1];
      LOG(INFO) << "target x:" << target.pose.x << " y: " << target.pose.y;
      //  printf("start distance:%lf\n", start_rec_dis_);
      LOG(INFO) << "out of identification";
      continue;
    }

    transform::Rigid3d target_rigid_odom_;
    target_rigid_odom_ = odom_in_map_.inverse() *
      transform::Rigid3d(
      Eigen::Vector3d(target.pose.x, target.pose.y, 0),
      transform::RollPitchYaw(0, 0, target.pose.z));
    // 目标点在odom坐标系下的坐标
    cv::Point3d target_odom_(target_rigid_odom_.translation()[0],
      target_rigid_odom_.translation()[1],
      transform::GetYaw(target_rigid_odom_.rotation()));

    // 如果model manager中没有该模型，则跳过
    // target_odom_ = tf_manager_.TransTargetToOdom(target.pose);
    Model model;
    if (!model_manager_->getModel(target.target_name, model)) {
      continue;
    } 
    Model target_model = model_manager_->updateTargetModel(model, target_odom_);

    // 如果模型是货架或者托盘，则获取其转换到里程计坐标系下
    std::vector<Line> lines;
    if (target.target_name.find("shelf") != std::string::npos ||
      target.target_name.find("holder") != std::string::npos)
    {
      lines = model_manager_->getModelLines(
        target_model,
        cv::Point3d(
          odom_.pose.translation()[0],
          odom_.pose.translation()[1],
          transform::GetYaw(odom_.pose.rotation())));
    } else {  // 否则
      lines = model_manager_->getModelLines(target_model);
    }


    // 将每个模型的line包装成lines_text_info
    LinesAndTextInfo lines_text_info;
    lines_text_info.frame = "odom";
    lines_text_info.color = blue;
    lines_text_info.text = "Pre:" + target.target_name + " X:" + std::to_string(target.pose.x) +
      " Y:" + std::to_string(target.pose.y) + " T:" + std::to_string(target.pose.z);
    lines_text_info.text_position = cv::Point2d(target_odom_.x, target_odom_.y);
    for (auto line : lines) {
      lines_text_info.lines.push_back(line.line);
    }
    pre_info_ = lines_text_info;
    // feature_map_vis_info_ = lines_text_info;
    target_map.feature_map_msgs.push_back(lines_text_info);
    // visualization_.linesAndTextVisual(lines_text_info, "pre_point", ros::Time::now());

    std::vector<std::vector<std::vector<cv::Point2d>>> component_points;
    for (auto component : target_model.components) {  // 遍历模型的component
      std::vector<std::vector<cv::Point2d>> possible_points;

      for (auto pointSet : globalPointSet) {
        if (pointSet.first != component.type) {
          continue;
        }

        Pose element_point = target_model.elements[component.element_id].poses.front();
        // 模型都是对称的，所以使用乘以2
        double model_length =
          2 * sqrt(element_point.x * element_point.x + element_point.y * element_point.y);
        double point_set_length = ppDistance(pointSet.second.front(), pointSet.second.back());
        double length_error = point_set_length - model_length;  // 聚类与模型长度的误差

        if (target.target_name.find("shelf") != std::string::npos &&
            length_error > 0.05)
        {
          continue;
        }

        cv::Point2d center = getPointsCenter(pointSet.second);  // 聚类中心
        double distance_error = ppDistance(center, cv::Point2d(component.x, component.y));
        if (distance_error < search_distance_) {
          possible_points.push_back(pointSet.second);
        }
      }
      if (possible_points.size() > 0) {
        component_points.push_back(possible_points);
      }
      LOG(INFO) << "possible points size: " << possible_points.size();
      // printf("possible points size:%d\n", static_cast<int>(possible_points.size()));
    }


    if (component_points.size() < 1) {
      LOG(INFO) << "component points size is 0";
      continue;
    }

    std::vector<std::vector<cv::Point2d>> select_points_set;
    for (size_t k = 0; k < component_points[0].size(); k++) {
      select_points_set.push_back(component_points[0][k]);
    }

    for (size_t i = 1; i < component_points.size(); i++) {
      std::vector<std::vector<cv::Point2d>> tmp_points_set;
      for (size_t j = 0; j < select_points_set.size(); j++) {
        for (size_t k = 0; k < component_points[i].size(); k++) {
          std::vector<cv::Point2d> points;
          points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());
          points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());
          tmp_points_set.push_back(points);
        }
      }
      select_points_set = tmp_points_set;
    }
    std::vector<std::pair<double, cv::Point3d>> possible_target;

    for (size_t i = 0; i < select_points_set.size(); i++) {
      std::vector<cv::Point2d> select_points = select_points_set[i];
      if (select_points.size() > 2 &&
        ppDistance(select_points.back(), select_points.front()) < 0.2)
      {
        LOG(INFO) << "select points max distance less than 0.2";
        continue;
      }

      // ICP计算目标的位姿
      //  TimingTool icpTime;
      //   icpTime.timingStart();
      cv::Point3d target_pose = target_odom_;
      double error = GetTargetPosition(select_points, lines, target_pose);
      possible_target.push_back(std::make_pair(error, target_pose));
    }

    if (possible_target.size() > 0) {
      cv::Point3d target_pose;

      std::vector<int> select_target_list;
      cv::Point2d predict;
      predict.x = target_odom_.x;
      predict.y = target_odom_.y;
      for (size_t i = 0; i < possible_target.size(); i++) {
        cv::Point2d target_pose;
        target_pose.x = possible_target[i].second.x;
        target_pose.y = possible_target[i].second.y;
        double angle_dif = fabs(target_odom_.z - possible_target[i].second.z);

        if (possible_target[i].first > max_dis_error_) {
          LOG(INFO) << "match distance error: " << possible_target[i].first;
        } else if (ppDistance(target_pose, predict) > search_distance_) {
          LOG(INFO) << "search distance error: " << ppDistance(target_pose, predict);
        } else if (angle_dif > max_ang_error_) {
          LOG(INFO) << "match distance error: " << angle_dif;
        } else {
          select_target_list.push_back(i);
        }
      }
      if (select_target_list.size() < 1) {
        LOG(INFO) << "select target list size is 0";
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
        LOG(INFO) << "target pose update";
        FeatureMsg feature_msg;
        feature_msg.id = 0;
        transform::Rigid3d feature_odom(Eigen::Vector3d(target_pose.x, target_pose.y, 0),
          transform::RollPitchYaw(0, 0, target_pose.z));
        transform::Rigid3d feature_map = odom_in_map_ * feature_odom;
        feature_msg.x = feature_map.translation()[0];
        feature_msg.y = feature_map.translation()[1];
        Eigen::Vector3d eulerAngle = feature_map.rotation().matrix().eulerAngles(0, 1, 2);
        feature_msg.theta = eulerAngle[2];
        feature_msg.target_name = target.target_name;
        target_map.target_msgs.push_back(feature_msg);
        Model recog_model =
          model_manager_->updateTargetModel(
          model,
          cv::Point3d(target_pose.x, target_pose.y, target_pose.z));
        std::vector<Line> lines = model_manager_->getModelLines(recog_model);
        LinesAndTextInfo lines_text_info;
        lines_text_info.frame = "odom";
        lines_text_info.color = green;
        lines_text_info.text = "Rec:" + target.target_name;
        lines_text_info.text_position = cv::Point2d(target_pose.x, target_pose.y);
        for (auto line : lines) {
          lines_text_info.lines.push_back(line.line);
        }
        // target_map_vis_info_ = lines_text_info;
        target_map.target_map_msgs.push_back(lines_text_info);
        // visualization_.linesAndTextVisual(lines_text_info, "tar_point", ros::Time::now());
      } else {
        LOG(INFO) << "maching error is " << min_error;
      }
    } else {
      LOG(INFO) << "possible target size is 0";
    }
  }
  // target_list_.resize(0);
  return target_map;
}

double RecgByPrePoint::GetTargetPosition(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines,
  cv::Point3d & targetPosition)
{
  double R[4] = {1.f, 0.f, 0.f, 1.f}, T[2] = {0., 0.};
  cv::Mat r = cv::Mat(2, 2, CV_64F, R);
  cv::Mat t = cv::Mat(2, 1, CV_64F, T);
  double err =
    icp(
    selectTargetPoints, lines, r, t,
    cv::TermCriteria(cv::TermCriteria::Type::COUNT, 1000, 0.0001));
  double x = (R[0] * targetPosition.x + R[1] * targetPosition.y + T[0]);
  double y = (R[2] * targetPosition.x + R[3] * targetPosition.y + T[1]);
  double theta = targetPosition.z + atan2(-R[1], R[0]);
  targetPosition.x = x;
  targetPosition.y = y;
  targetPosition.z = theta;
  return err;
}

bool RecgByPrePoint::UpdateTargetList(const std::vector<FeatureInfo> & target_list)
{
  std::lock_guard<std::mutex> lock(update_target_list_mutex_);
  target_list_ = target_list;
  return true;
}

bool RecgByPrePoint::GetRackInfo(RackInfo & rack_info)
{
  LOG(INFO) << "target_list size = " << target_list_.size();
  if (target_list_.size() == 0) {
    return false;
  }
  for (auto target : target_list_) {
    LOG(INFO) << "target name " << target.target_name;
    if (target.target_name.find("shelf") != std::string::npos ||
      target.target_name.find("holder") != std::string::npos)
    {
      rack_info = model_manager_->getModelSize(target.target_name);
      return true;
    }
  }
  return false;
}

bool RecgByPrePoint::UpdateOdomData(const sensor::OdometryData & odom)
{
  odom_ = odom;
  return true;
}

bool RecgByPrePoint::UpdateOdomToMapTrans(const transform::Rigid3d & odom_in_map)
{
  odom_in_map_ = odom_in_map;
  return true;
}

std::vector<FeatureMsg> RecgByPrePoint::GetVmark(
  const sensor::TimedPointCloudOriginData & timed_point_cloud_data,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<FeatureInfo> target_list;
  {
    std::lock_guard<std::mutex> lock(update_target_list_mutex_);
    target_list = target_list_;
  }
  for (auto target : target_list) {
    // 识别距离判断
    // cv::Point3d robot_pose;
    transform::Rigid3d robot_pose;
    robot_pose = odom_in_map_ * odom_.pose;
    if (ppDistance(
        cv::Point2d(robot_pose.translation()[0], robot_pose.translation()[1]),
        cv::Point2d(target.pose.x, target.pose.y)) > 5.0)
    {
      printf("robot x:%lf\t y:%lf\n", robot_pose.translation()[0], robot_pose.translation()[1]);
      printf("target x:%lf\t y:%lf\n", target.pose.x, target.pose.y);
      //  printf("start distance:%lf\n", start_rec_dis_);
      printf("out of identification\n");
      continue;
    }

    transform::Rigid3d target_rigid_odom_;
    target_rigid_odom_ = odom_in_map_.inverse() *
      transform::Rigid3d(
      Eigen::Vector3d(target.pose.x, target.pose.y, 0),
      transform::RollPitchYaw(0, 0, target.pose.z));
    cv::Point3d target_odom_(target_rigid_odom_.translation()[0],
      target_rigid_odom_.translation()[1],
      transform::GetYaw(target_rigid_odom_.rotation()));

    LOG(INFO) << "died";
    // target_odom_ = tf_manager_.TransTargetToOdom(target.pose);
    Model model;
    if (!model_manager_->getModel(target.target_name, model)) {
      continue;
    }
    Model target_model = model_manager_->updateTargetModel(model, target_odom_);
    // std::vector<std::vector<std::vector<cv::Point2d>>> component_points;
    std::vector<std::vector<std::vector<int>>> component_points;
    for (auto component : target_model.components) {
      // std::vector<std::vector<cv::Point2d>> possible_points;
      std::vector<std::vector<int>> possible_points;
      // LOG(INFO) << "laserPointSet size = " << laserPointSet.size();
      for (auto pointSet : laserPointSet) {
        if (pointSet.first != component.type) {
          continue;
        }
        // cv::Point2d center = getPointsCenter(pointSet.second);
        // double distance_error = ppDistance(center, cv::Point2d(component.x, component.y));
        // if (distance_error < search_distance_) {
        possible_points.push_back(pointSet.second);
        // }
      }
      if (possible_points.size() > 0) {
        component_points.push_back(possible_points);
      }
      printf("possible points size:%d\n", static_cast<int>(possible_points.size()));
    }
    std::vector<Line> lines = model_manager_->getModelLines(target_model);
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
    shape_pose_return = TestSearchBestSet(
      timed_point_cloud_data, select_points_set, lines, model);
    LOG(INFO) << "target_pose_return size = " << shape_pose_return.size();
  }
  return shape_pose_return;
}

std::vector<FeatureMsg> RecgByPrePoint::TestSearchBestSet(
  const sensor::TimedPointCloudOriginData & msg,
  std::vector<std::vector<int>> & select_points_set, std::vector<Line> & lines,
  Model & model)
{
  std::vector<FeatureMsg> shape_pose_return;
  std::vector<std::pair<double, cv::Point3d>> possible_target;
  std::vector<std::vector<cv::Point2d>> possible_point;
  std::vector<Line> lines_tmp = lines;

  LOG(INFO) << "select_points_set size = " << select_points_set.size();
  for (size_t i = 0; i < select_points_set.size(); i++) {
    std::vector<cv::Point2d> select_points;
    for (size_t j = 0; j < select_points_set[i].size(); j++) {
      select_points.push_back(
        cv::Point2d(
          msg.ranges[select_points_set[i][j]].point_time.position[0],
          msg.ranges[select_points_set[i][j]].point_time.position[1]));
    }
    if (select_points.size() > 2 && ppDistance(select_points.back(), select_points.front()) < 0.2) {
      std::cout << "select points max distance less than 0.2" << std::endl;
    }
    LOG(INFO) << "========================SELECT_POINTS SIZE = " << select_points.size();
    cv::Point3d targetPosition_, targetPosition_tmp;
    cv::Point2d corner_in_line;
    cv::Point2d corner_in_points;
    FindCorner(select_points, lines, targetPosition_, corner_in_line, corner_in_points);
    Model model_ = model_manager_->updateTargetModel(model, targetPosition_);
    lines_tmp = model_manager_->getModelLines(model_);
    FindBestAngleOfSet(select_points, model, targetPosition_, lines_tmp, targetPosition_tmp);

    std::vector<cv::Point2d> tmp_vis_point;
    cv::Point3d target_pose(0, 0, 0);
    double score = GetTargetPosition(select_points, lines_tmp, targetPosition_tmp);
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
      tmp_vis_point[m] = tmp_point;
    }

    target_pose.x = targetPosition_tmp.x;
    target_pose.y = targetPosition_tmp.y;
    target_pose.z = targetPosition_tmp.z;

    possible_target.push_back(std::make_pair(score, target_pose));
    possible_point.push_back(select_points);
  }
  for (size_t i = 0; i < possible_target.size(); i++) {
    if (possible_target[i].first < 0.8 /*shape_match_score_*/) {
    } else {
      cv::Point3d target_pose;
      target_pose = possible_target[i].second;
      FeatureMsg feature_msg;
      feature_msg.id = 0;
      feature_msg.x = target_pose.x;
      feature_msg.y = target_pose.y;
      feature_msg.theta = target_pose.z;
      feature_msg.target_name = "shape_struct";
      shape_pose_return.push_back(feature_msg);
    }
  }
  return shape_pose_return;
}

double RecgByPrePoint::FindCorner(
  const std::vector<cv::Point2d> selectTargetPoints,
  const std::vector<Line> lines, cv::Point3d & targetPosition,
  cv::Point2d & corner_in_line, cv::Point2d & corner_in_points)
{
  cv::Point2d point_begin = selectTargetPoints.front();
  cv::Point2d point_end = selectTargetPoints.back();
  cv::Point2d point_to_match;
  double point_max_dis = 0;
  int find_i = 0;
  std::vector<cv::Point2d> first_set_of_whole_set;
  std::vector<cv::Point2d> second_set_of_whole_set;
  for (size_t i = 0; i < selectTargetPoints.size(); i++) {
    cv::Point2d nearest_point;
    double seg_dis;
    seg_dis = pointToSegDist(selectTargetPoints[i], point_begin, point_end, nearest_point);
    if (point_max_dis < seg_dis) {
      point_max_dis = seg_dis;
      point_to_match = selectTargetPoints[i];
      find_i = i;
    }
  }

  std::vector<cv::Point2d> tmp_v = selectTargetPoints;
  while ((selectTargetPoints.size() / 2 - find_i) > 10) {
    auto it = tmp_v.begin();
    for (int i = 0; i < find_i; i++) {
      it++;
    }
    it = tmp_v.erase(it);
    for (size_t i = 0; i < tmp_v.size(); i++) {
      cv::Point2d nearest_point;
      double seg_dis;
      seg_dis = pointToSegDist(tmp_v[i], point_begin, point_end, nearest_point);
      if (point_max_dis < seg_dis) {
        point_max_dis = seg_dis;
        point_to_match = tmp_v[i];
        find_i = i;
      }
    }
  }
  for (size_t i = 0; i < selectTargetPoints.size(); i++) {
    if (i < (size_t)find_i) {
      first_set_of_whole_set.push_back(selectTargetPoints[i]);
    } else {
      second_set_of_whole_set.push_back(selectTargetPoints[i]);
    }
  }
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

double RecgByPrePoint::FindBestAngleOfSet(
  const std::vector<cv::Point2d> selectTargetPoints,
  Model & model, const cv::Point3d & targetPosition,
  std::vector<Line> & return_lines, cv::Point3d & return_target_position)
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

double RecgByPrePoint::ComputeScoreOfAngle(
  const std::vector<cv::Point2d> & points,
  const std::vector<Line> & lines)
{
  double score = 0;
  std::vector<cv::Point2d> intersection_set;
  std::vector<cv::Point2d> union_set;
  for (size_t i = 0; i < points.size(); i++) {
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
}  // namespace perception_2d
