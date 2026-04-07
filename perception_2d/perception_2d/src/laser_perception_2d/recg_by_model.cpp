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
#include "perception_2d/laser_perception_2d/recg_by_model.hpp"
#include "perception_2d/laser_perception_2d/math_tool.hpp"

namespace perception_2d
{
RecgByModel::RecgByModel(const Config config)
{
  search_distance_ = config.search_distance;
  max_dis_error_ = config.max_dis_error;
  max_ang_error_ = config.max_ang_error;
  start_rec_dis_ = config.start_rec_dis;
  min_select_points_ = config.min_select_points;
  model_file_ = config.model_file;
  model_manager_ = ModelManager::instance(config);
}

void RecgByModel::UpdateParams(
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


PrePointMsg RecgByModel::GetChargeStationPose(
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet)
{
  PrePointMsg target_map;

  double start_dist = 0.6;
  double search_dist = 0.6;
  double search_angle = 30 * M_PI / 180;
  double dist_res = 0.1;
  double ang_res = 3 * M_PI / 180;
  if (is_first_search_) {
    for (double ang = -search_angle; ang < search_angle; ang += ang_res) {
      for (double start = start_dist - 0.2; start < start_dist + search_dist + 0.2; start += dist_res) {
        cv::Point3d predict_pose(start, 0, ang);
        std::cout << "predict_pose: " << predict_pose << std::endl;
        cv::Point3d result_pose;
        target_map = GetChargeStationPoseByPredict(predict_pose, result_pose, globalPointSet);
        std::cout << "result_pose: " << result_pose << std::endl;

        if (result_pose.x != 0) {
          is_first_search_ = false;
          last_target_pose_ = result_pose;
          break;
        }
      }
    }
  } else {
    cv::Point3d predict_pose = last_target_pose_;
    target_map = GetChargeStationPoseByPredict(predict_pose, last_target_pose_, globalPointSet);
  }
  return target_map;
}

PrePointMsg RecgByModel::GetChargeStationPoseByPredict(
  const cv::Point3d& predict_pose,
  cv::Point3d& result_pose,
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet)
{
  // 搜索的角度范围，搜索的距离范围，搜索的分辨率

  PrePointMsg target_map;
  // Step: 1. 按照一定的分辨率进行检索
  FeatureInfo target;
  target.target_name = "chargeStation";
  target.pose = predict_pose;
  // 初始的位姿x y theta
  // 获取model，将model转换到odom坐标系下
  std::string target_name = "chargeStation";
  Model model;
  if (!model_manager_->getModel(target.target_name, model)) {
    return target_map;
  }

  transform::Rigid3d target_rigid_odom;
  // target_rigid_odom = odom_in_map_.inverse() *
  //   transform::Rigid3d(
  //   Eigen::Vector3d(target.pose.x, target.pose.y, 0),
  //   transform::RollPitchYaw(0, 0, target.pose.z));

  target_rigid_odom = odom_.pose * transform::Rigid3d(
                                Eigen::Vector3d(target.pose.x, target.pose.y, 0),
                                transform::RollPitchYaw(0, 0, target.pose.z));
  // std::cout << target_rigid_odom << std::endl;
  // 目标点在odom坐标系下的坐标
  cv::Point3d target_odom(target_rigid_odom.translation()[0],
    target_rigid_odom.translation()[1],
    transform::GetYaw(target_rigid_odom.rotation()));

  // 将model转换到odom坐标系下
  Model target_model = model_manager_->updateTargetModel(model, target_odom);
  sensor::PointCloud target_cloud = model_manager_->getModelCloud(target_model);

  // std::cout << "target_cloud.size():" << target_cloud.size() << std::endl;
  // for (auto p : target_cloud) {
  //   std::cout << "p.x:" << p.position.x() << " p.y:" << p.position.y() << std::endl;
  // }
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
  lines_text_info.text = "Pre:" + target.target_name + " X:" + std::to_string(target_odom.x) +
    " Y:" + std::to_string(target_odom.y) + " T:" + std::to_string(target_odom.z);
  lines_text_info.text_position = cv::Point2d(target_odom.x + 0.1, target_odom.y);
  for (auto line : lines) {
    lines_text_info.lines.push_back(line.line);
  }
  pre_info_ = lines_text_info;
  // feature_map_vis_info_ = lines_text_info;
  target_map.feature_map_msgs.push_back(lines_text_info);
  LOG(INFO) << "target_map.feature_map_msgs: " << target_map.feature_map_msgs.size() << std::endl;

  std::vector<std::vector<std::vector<cv::Point2d>>> component_points;
  for (auto component : target_model.components) {
    std::vector<std::vector<cv::Point2d>> possible_points;
    for (auto pointSet : globalPointSet) {
      if (pointSet.first != component.type) {
        continue;
      }
      double point_set_length = ppDistance(pointSet.second.front(), pointSet.second.back());

      if (point_set_length < 0.05) {
        continue;
      }

      Pose element_point = target_model.elements[component.element_id].poses.front();
      // 模型都是对称的，所以使用乘以2
      double model_length =
        2 * sqrt(element_point.x * element_point.x + element_point.y * element_point.y);
      double length_error = std::fabs(point_set_length - model_length);  // 聚类与模型长度的误差

      if (length_error > 0.1) {
        continue;
      }
      // std::cout << "model_length: " << model_length << std::endl;
      // std::cout << "point_set_length: " << point_set_length << std::endl;
      // std::cout << "length_error: " << length_error << std::endl;

      cv::Point2d center = getPointsCenter(pointSet.second);  // 聚类中心
      double distance_error = ppDistance(center, cv::Point2d(component.x, component.y));

      if (distance_error < search_distance_) {
        possible_points.push_back(pointSet.second);
        // std::cout << "center: " << center << std::endl;
        // std::cout << "point: " << cv::Point2d(component.x, component.y) << std::endl;

        // std::cout << "distance_error: " << distance_error << std::endl << std::endl;
      }
    }
    // possible_points里面存放的都是当前component的候选点
    if (possible_points.size() > 0) {
      component_points.push_back(possible_points);
    }

    if (component_points.empty() || possible_points.empty()) {
      continue;
    }
    // 当当前的model是一个component的时候，component_points.size() == 1，多个时为 == n
    // 将第0个component的point_set都划入到select_points_set中
    std::vector<std::vector<cv::Point2d>> select_points_set;
    for (size_t k = 0; k < component_points[0].size(); k++) {
      select_points_set.push_back(component_points[0][k]);
    }

    // 将所有的点都划入到select_points_set
    // 对于其他component的点
    for (size_t i = 1; i < component_points.size(); i++) {
      std::vector<std::vector<cv::Point2d>> tmp_points_set;
      for (size_t j = 0; j < select_points_set.size(); j++) {  // 对于第0个component对应的可能点
        for (size_t k = 0; k < component_points[i].size(); k++) {  // 第i个component对应的可能点
          std::vector<cv::Point2d> points;
          points.insert(points.end(), select_points_set[j].begin(), select_points_set[j].end());  // 添加第0个component对应的第j个可能点
          points.insert(points.end(), component_points[i][k].begin(), component_points[i][k].end());  // 添加第i个component对应的第k个可能点
          // points里就是一个完成model多个component对应的点集
          tmp_points_set.push_back(points);
        }
      }
      select_points_set = tmp_points_set;  // 变相的有多个可能的模型点集
    }
    std::vector<std::pair<double, cv::Point3d>> possible_target;

    for (size_t i = 0; i < select_points_set.size(); i++) {  // 对于每一个可能的模型点集
      std::vector<cv::Point2d> select_points = select_points_set[i];
      // 筛选掉无效的点集
      if (select_points.size() > 2 &&
        ppDistance(select_points.back(), select_points.front()) < 0.2)
      {
        // LOG(INFO) << "select points max distance less than 0.2";
        continue;
      }
      // ICP计算目标的位姿
      //  TimingTool icpTime;
      //   icpTime.timingStart();
      cv::Point3d target_pose = target_odom;
      double error = GetTargetPosition(select_points, lines, target_pose);
      // if (target_pose.z > M_PI_2) {
      //   target_pose.z = target_pose.z - M_PI;
      // }
      // if (target_pose.z < -M_PI_2) {
      //   target_pose.z = target_pose.z + M_PI;
      // }

      possible_target.push_back(std::make_pair(error, target_pose));
      LOG(INFO) << "target_pose: " << target_pose << std::endl;
    }


    if (possible_target.size() > 0) {
      cv::Point3d target_pose;

      std::vector<int> select_target_list;
      cv::Point2d predict;
      predict.x = target_odom.x;
      predict.y = target_odom.y;
      for (size_t i = 0; i < possible_target.size(); i++) {
        cv::Point2d target_pose;
        target_pose.x = possible_target[i].second.x;
        target_pose.y = possible_target[i].second.y;
        double angle_diff = fabs(target_odom.z - possible_target[i].second.z);

        if (possible_target[i].first > max_dis_error_) {
          LOG(INFO) << "match distance error: " << possible_target[i].first;
        } else if (ppDistance(target_pose, predict) > search_distance_) {
          LOG(INFO) << "search distance error: " << ppDistance(target_pose, predict);
        } else if (angle_diff > max_ang_error_) {
          LOG(INFO) << "match distance error: " << angle_diff;
        } else {
          select_target_list.push_back(i);
        }
        select_target_list.push_back(i);
      }
      LOG(INFO) << "select_target_list.size: " << select_target_list.size() << std::endl;
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

        transform::Rigid3d feature_base = odom_.pose.inverse() * feature_odom;
        result_pose.x = feature_base.translation().x();
        result_pose.y = feature_base.translation().y();
        result_pose.z = transform::GetYaw(feature_base.rotation());

        transform::Rigid3d feature_map = odom_in_map_ * feature_odom;
        feature_msg.x = feature_map.translation()[0];
        feature_msg.y = feature_map.translation()[1];
        feature_msg.theta = 2 * atan(feature_map.rotation().z() / feature_map.rotation().w());
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
        lines_text_info.text_position = cv::Point2d(target_pose.x - 0.1, target_pose.y);
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
  return target_map;
  // 计算匹配结果
  // 所有的结果都在odom坐标系下进行
  // 1. 先将模型点云转换到odom坐标系下
  // 2. 再将scan 点云转换到odom坐标系下
  // 3. 进行plicp匹配
  // 4. 匹配结果是scan在odom系下的漂移量
  // 5. 将这个点发布出去
}


PrePointMsg RecgByModel::GetTargetByPredictPose(
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

double RecgByModel::GetTargetPosition(
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


bool RecgByModel::UpdateOdomData(const sensor::OdometryData & odom)
{
  odom_ = odom;
  return true;
}


bool RecgByModel::UpdateOdomToMapTrans(const transform::Rigid3d & odom_in_map)
{
  odom_in_map_ = odom_in_map;
  return true;
}

}  // namespace perception_2d
