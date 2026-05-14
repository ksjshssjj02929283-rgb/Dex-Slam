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
#include <tuple>
#include "perception_2d/laser_perception_2d/laser_perception_2d.hpp"

namespace perception_2d
{

// 从 Lua 读取算法层默认值
// Lua key 与 perception_2d_ros2/params/params_perception_2d.yaml 保持一致
// node_main.cpp::InitParams 通过 declare_parameter(name, lua_default) 注入 ROS2 参数体系
Config CreateLaserPerception2DOption(
  common::LuaParameterDictionary * const
  lua_parameter_dictionary,
  const std::string & model_file_path)
{
  Config config;
  // 检测器开关
  config.predict_rec = lua_parameter_dictionary->GetBool("predict_rec");
  config.reflect_rec = lua_parameter_dictionary->GetBool("reflect_rec");
  config.line_rec = lua_parameter_dictionary->GetBool("line_rec");
  config.shape_rec = lua_parameter_dictionary->GetBool("shape_rec");
  config.vmark_rec = lua_parameter_dictionary->GetBool("vmark_rec");
  config.angle_rec = lua_parameter_dictionary->GetBool("angle_rec");

  // 数据预处理
  config.intensity_threshold = lua_parameter_dictionary->GetInt("intensities_min");
  config.connect_threshold = lua_parameter_dictionary->GetDouble("connect_threshold");
  config.range_max = lua_parameter_dictionary->GetDouble("range_max");
  config.range_min = lua_parameter_dictionary->GetDouble("range_min");
  config.min_cluster_points = lua_parameter_dictionary->GetInt("min_cluster_points");

  // 匹配阈值
  config.search_distance = lua_parameter_dictionary->GetDouble("search_distance");
  config.max_ang_error = lua_parameter_dictionary->GetDouble("search_angle");
  config.max_dis_error = lua_parameter_dictionary->GetDouble("max_error");
  config.start_rec_dis = lua_parameter_dictionary->GetDouble("start_rec_dis");
  config.min_select_points = lua_parameter_dictionary->GetInt("min_select_points");

  // 反光板
  config.reflector_error = lua_parameter_dictionary->GetDouble("reflector_error");
  config.reflector_radius = lua_parameter_dictionary->GetDouble("reflector_radius");

  // 直线检测
  config.min_line_length = lua_parameter_dictionary->GetDouble("min_line_length");

  // 形状匹配（shape_rec=true 时生效）
  config.shape_match_score = lua_parameter_dictionary->GetDouble("shape_match_score");
  config.shape_match_close_degree =
    lua_parameter_dictionary->GetDouble("shape_match_close_degree");
  config.shape_dis_to_robot_center =
    lua_parameter_dictionary->GetDouble("shape_dis_to_robot_center");

  // 模型文件：优先 launch 的 -model_basename gflag；Lua model_file 为备用
  const std::string lua_model_file = lua_parameter_dictionary->GetString("model_file");
  config.model_file = lua_model_file.empty() ? model_file_path : lua_model_file;
  LOG(INFO) << "config.model_file = " << config.model_file;
  return config;
}

LaserPerception2D::LaserPerception2D(
  const std::vector<std::string> & expected_range_sensor_ids,
  const Config & config)
: config_(config), range_data_collator_(expected_range_sensor_ids)
{
  predict_rec_ = config_.predict_rec;
  reflect_rec_ = config_.reflect_rec;
  vmark_rec_ = config_.vmark_rec;
  laser_sync_ = new LaserSync();
  recg_by_model_ = new RecgByModel(config_);
  recg_by_reflec_ = new RecgByReflec(config_);
  line_detector_ = new LineDetector(config_);
  line_detector_x_ = new LineDetectorX();
}

bool LaserPerception2D::AddSensorData(
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


  TargetMap target_map;

  target_map = LaserUpdate(synchronized_data);

  target_map_ = target_map;

  return true;
}

FeatureMsg LaserPerception2D::SlideTargetMap(
  const FeatureMsg & feature_msg,
  std::list<FeatureMsg> & slide_vector)
{
  FeatureMsg feature_msg_o;
  feature_msg_o = feature_msg;

  if (slide_vector.size() < 20) {
    slide_vector.push_back(feature_msg);
  } else {
    slide_vector.pop_front();
    slide_vector.push_back(feature_msg);
    auto sort_slide_vector = slide_vector;

    double sum_x = 0;
    double sum_y = 0;
    Eigen::Vector2d sum_theta_vector(0, 0);
    Eigen::Vector2d average_theta_vector(0, 0);

    for (auto item : sort_slide_vector) {
      sum_x += item.x;
      sum_y += item.y;

      sum_theta_vector[0] += cos(item.theta);
      sum_theta_vector[1] += sin(item.theta);
    }

    average_theta_vector[0] = sum_theta_vector[0] / sort_slide_vector.size();
    average_theta_vector[1] = sum_theta_vector[1] / sort_slide_vector.size();

    feature_msg_o.x = sum_x / sort_slide_vector.size();
    feature_msg_o.y = sum_y / sort_slide_vector.size();
    feature_msg_o.theta = atan2(
      average_theta_vector[1],
      average_theta_vector[0]);
  }
  return feature_msg_o;
}

bool LaserPerception2D::AddSensorData(
  const std::string & sensor_id,
  const common::Time & time,
  const transform::Rigid3d & sensor_to_tracking,
  const sensor::TimedPointCloudWithIntensities& timed_point_cloud_data)
{
  const auto range_data = perception_2d::sensor::SyncTimedPointCloudData{
              time, sensor_to_tracking/*Eigen::Vector3d(sensor_to_tracking->translation()[0],
              sensor_to_tracking->translation()[1],
              perception_2d::transform::GetYaw(sensor_to_tracking->rotation()))*/,
              perception_2d::sensor::TimedPointCloudWithIntensities{
                    timed_point_cloud_data.points,
                    timed_point_cloud_data.intensities}};

  AddSensorData(sensor_id, range_data);
  return false;
}

bool LaserPerception2D::AddSensorData(
  const std::string & sensor_id,
  const sensor::SyncTimedPointCloudData & range_data_identity)
{
  sensor::SyncPointCloudWithOdom sync_point_cloud;
  std::tuple<std::vector<std::shared_ptr<FeatureMsgWithOrigin>>, TargetMap> target_map_test;
  std::unique_lock<std::mutex> lock(data_mutex_);

  if (laser_sync_->GetSyncPointCloud(
      sensor_id, range_data_identity, sync_point_cloud))
  {
    //  return true;
    update_msg_ = sync_point_cloud;
    LaserUpdate(sync_point_cloud, line_and_ref_target_map_, target_map_);
  }
  return false;
}

void LaserPerception2D::AddSensorData(
  const std::string & sensor_id,
  const sensor::OdometryData & odometry_data)
{
  // LOG(INFO) << "odom_ x = " << odometry_data.pose.translation()[0]
  //           << " y = " << odometry_data.pose.translation()[1];
  laser_sync_->AddOdometryData(odometry_data);
  recg_by_model_->UpdateOdomData(odometry_data);
  odom_ = odometry_data;
}

void LaserPerception2D::UpdateTransformFromOdomToMap(const transform::Rigid3d & odom_to_map)
{
  recg_by_model_->UpdateOdomToMapTrans(odom_to_map);
  odom_in_map_ = odom_to_map;
}

void LaserPerception2D::HandleFeatureMapInfoMessage(
  const std::string & topic,
  const std::vector<FeatureInfo> & feature_map)
{
  if (feature_map_.size() == 0) {
    feature_map_ = feature_map;
    // recg_by_model_->UpdateTargetList(feature_map);
    pre_point_slide_vector_.resize(0);
    LOG(INFO) << "init feature_map";
    return;
  }
  LOG(INFO) << "feature_map_ name = " << feature_map_[0].target_name <<
    " x : " << feature_map_[0].pose.x <<
    " y : " << feature_map_[0].pose.y <<
    " theta : " << feature_map_[0].pose.z;
  LOG(INFO) << "feature_map name = " << feature_map[0].target_name <<
    " x : " << feature_map[0].pose.x <<
    " y : " << feature_map[0].pose.y <<
    " theta : " << feature_map[0].pose.z;
  if (feature_map[0].target_name == feature_map_[0].target_name) {
    LOG(INFO) << "fabs x = " << fabs(feature_map[0].pose.x - feature_map_[0].pose.x);
    LOG(INFO) << "fabs y = " << fabs(feature_map[0].pose.y - feature_map_[0].pose.y);
    LOG(INFO) << "fabs z = " << fabs(feature_map[0].pose.z - feature_map_[0].pose.z);
    if (fabs(feature_map[0].pose.x - feature_map_[0].pose.x) < 0.3 &&
      fabs(feature_map[0].pose.y - feature_map_[0].pose.y) < 0.3 &&
      fabs(feature_map[0].pose.z - feature_map_[0].pose.z) < 0.2)
    {
      LOG(INFO) << "keep feature_map";
      return;
    }
  }
  // recg_by_model_->UpdateTargetList(feature_map);
  feature_map_ = feature_map;
  pre_point_slide_vector_.resize(0);
  LOG(INFO) << "Update feature_map";
}

std::vector<std::pair<int, std::vector<int>>> LaserPerception2D::LaserClustering(
  const sensor::TimedPointCloudOriginData & point_cloud)
{
  // 对激光点云进行聚类
  std::vector<int> highlightPointSet;   // 高反的聚类点
  std::vector<int> darkPointSet;        // 非高反的聚类点
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

std::vector<std::pair<int, std::vector<int>>> LaserPerception2D::LaserCluster(
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

std::vector<std::pair<int, std::vector<cv::Point2d>>> LaserPerception2D::LaserPointToOdom(
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

std::vector<std::pair<int, std::vector<cv::Point2d>>> LaserPerception2D::LaserPointToOdom(
  const sensor::PointCloudWithOrigin & point_cloud,
  std::vector<std::pair<int, std::vector<int>>> laserPointSet)
{
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet;
  // LOG(INFO) << "origin = " << point_cloud.origin.cast<float>().translation()[0]
  //           << " " << point_cloud.origin.cast<float>().translation()[1];
  for (auto item : laserPointSet) {
    std::vector<cv::Point2d> pointSet;
    for (auto index : item.second) {
      // Eigen::Vector3f point_b = point_cloud.ranges[index].point_time.position;
      Eigen::Vector3f point_b = point_cloud.origin.cast<float>() *
        point_cloud.sync_points_base_link[index];
      Eigen::Vector3f point_o = odom_.pose.cast<float>() * point_b;
      // point_cloud.sync_points_base_link[index];
      pointSet.push_back(cv::Point2d(point_o[0], point_o[1]));
      // LOG(INFO) << "odom point x = " << point_o[0] << " y = " << point_o[1];
      // LOG(INFO) << "odom laser x = " << point_cloud.origin_points_odom[index][0]
      //           << " y = " << point_cloud.origin_points_odom[index][1];
    }
    globalPointSet.push_back(std::make_pair(item.first, pointSet));
  }
  return globalPointSet;
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> LaserPerception2D::TransferLine(
  const std::pair<Eigen::Vector3f, Eigen::Vector3f> & line,
  const Eigen::Vector3d & origin)
{
  std::pair<Eigen::Vector3f, Eigen::Vector3f> return_line;
  transform::Rigid3d origin_trans(Eigen::Vector3d(origin[0], origin[1], 0),
    transform::RollPitchYaw(0, 0, origin[2]));

  Eigen::Vector3d begin_point(static_cast<double>(line.first[0]),
    static_cast<double>(line.first[1]),
    0);

  Eigen::Vector3d end_point(static_cast<double>(line.second[0]),
    static_cast<double>(line.second[1]),
    0);

  Eigen::Vector3f trans_begin_point{
    static_cast<float>((origin_trans * begin_point)[0]),
    static_cast<float>((origin_trans * begin_point)[1]),
    0};
  Eigen::Vector3f trans_end_point{
    static_cast<float>((origin_trans * end_point)[0]),
    static_cast<float>((origin_trans * end_point)[1]),
    0};

  return_line.first = trans_begin_point;
  return_line.second = trans_end_point;
  return return_line;
}

bool LaserPerception2D::LaserUpdate(
  const sensor::SyncPointCloudWithOdom & msg,
  std::vector<std::shared_ptr<FeatureMsgWithOrigin>>& target_map_full,
  TargetMap& pre_model_target_map)
{
  target_map_full.clear();

  // 将点云从base link转到laser link下
  for (size_t i = 0; i < msg.sync_point_clouds.size(); i++) {
    sensor::PointCloudWithOrigin point_cloud = msg.sync_point_clouds[i];
    std::vector<std::pair<int, Eigen::Vector3f>> point_cloud_in_laser;
    for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
      Eigen::Vector3f point_base_link = point_cloud.sync_points_base_link[i];
      // Eigen::Vector3d origin = point_cloud.origin;
      transform::Rigid3d origin = point_cloud.origin;
      // LOG(INFO) << "origin x = " << origin[0] << " y = " << origin[1];
      Eigen::Vector3f point;
      // point << point_base_link[0] * cos(origin[2]) + point_base_link[1] * sin(origin[2]) -
      // (origin[0] * cos(origin[2]) + origin[1] * sin(origin[2])),
      //   -point_base_link[0] * sin(origin[2]) + point_base_link[1] * cos(origin[2]) -
      // (-origin[0] * sin(origin[2]) + origin[1] * cos(origin[2])),
      //   0;
      point = origin.cast<float>().inverse() * point_base_link;
      point_cloud_in_laser.push_back(
        std::pair<int, Eigen::Vector3f>(
          point_cloud.intensities[i],
          point));
      point_cloud.sync_points_base_link[i] = point;
    }

    std::vector<std::pair<int, std::vector<int>>> laserPointSet =
      LaserCluster(point_cloud_in_laser);
    // for(size_t i = 0; i < point_cloud_in_laser.size(); i++){
    //   LOG(INFO) << " point clout in laser x =" << point_cloud_in_laser[i].second[0];
    //   LOG(INFO) << " point cloud x = " << point_cloud.sync_points_base_link[i][0];
    // }
    // std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet =
    //   LaserPointToOdom(point_cloud, laserPointSet);
    // recg_by_model_->GetChargeStationPose(globalPointSet);
    // LOG(INFO) << "laserPointSet.size = " << laserPointSet.size();
    // if(config_.predict_rec){
    //   std::vector<FeatureMsg> target_map3;
    //   target_map3 =
    // }
    // 获取精准对接结构在odom下的位姿
    if (true) {
      PrePointMsg pre_point_msg;
      std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet =
        LaserPointToOdom(point_cloud, laserPointSet);
      pre_point_msg = recg_by_model_->GetChargeStationPose(globalPointSet);
      pre_info_ = recg_by_model_->pre_info_;
      pre_point_msg.time = msg.stamp;
      pre_point_msg.frame_id = "map";
      LOG(INFO) << "pre_point_msg.feature_map_msgs : " << pre_point_msg.feature_map_msgs.size() << std::endl;
      for (auto & pre_msg : pre_point_msg.target_msgs) {
        pre_msg = SlideTargetMap(pre_msg, pre_point_slide_vector_);
        LOG(INFO) << "pre_msg x = " << pre_msg.x << " y = " << pre_msg.y;
      }
      pre_model_target_map.pre_point_msg = pre_point_msg;
    }

    if (config_.line_rec) {
      std::vector<FeatureMsg> target_map3;
      std::list<Point> input_points;
      for (auto point : point_cloud_in_laser) {
        input_points.push_back(
          Point(
              point.second[0],
              point.second[1]));
      }

      // 线特征检测
      // std::list<Segment> lines = line_detector_->detectLines(input_points, laserPointSet);
      std::list<Segment> lines = line_detector_x_->detectLines(input_points);

      for (auto line : lines) {
        FeatureMsg feature;
        feature.target_name = "line";
        feature.line = std::make_pair(
          Eigen::Vector3f(line.first_point.x, line.first_point.y, 0.0),
          Eigen::Vector3f(line.last_point.x, line.last_point.y, 0.0));
        // // feature.line = TransferLine(feature.line, point_cloud.origin);
        // LOG(INFO) << "line.first_point.x = " << line.first_point.x << " y = " << line.first_point.y;
        // LOG(INFO) << "line.last_point.x = " << line.last_point.x << " y = " << line.last_point.y;
        target_map3.push_back(feature);
      }

      std::shared_ptr<FeatureMsgWithOrigin> tmp_target(new FeatureMsgWithOrigin());
      tmp_target->time = msg.stamp;
      tmp_target->target_name = "line";
      // tmp_target.frame_id = "base_link";
      // tmp_target.origin = Eigen::Vector3d(0, 0, 0);
      tmp_target->frame_id = msg.sync_point_clouds[i].frame_id;
      tmp_target->origin = msg.sync_point_clouds[i].origin;
      tmp_target->MsgVec = target_map3;
      target_map_full.push_back(tmp_target);
    }

    if (config_.reflect_rec) {
      std::vector<FeatureMsg> target_map3;
      target_map3 = recg_by_reflec_->getReflector(point_cloud, laserPointSet);
      // for (auto & feature_msg : target_map3) {
      //   Eigen::Vector3d feature_v(feature_msg.x, feature_msg.y, 0);
      //   transform::Rigid3d origin_trans(
      //     Eigen::Vector3d(point_cloud.origin[0], point_cloud.origin[1], 0),
      //     transform::RollPitchYaw(0, 0, point_cloud.origin[2]));
      //   feature_msg.x = (origin_trans * feature_v)[0];
      //   feature_msg.y = (origin_trans * feature_v)[1];
      // }
      std::shared_ptr<FeatureMsgWithOrigin> tmp_target(new FeatureMsgWithOrigin());
      tmp_target->target_name = "reflector";
      tmp_target->time = msg.stamp;
      // tmp_target.frame_id = "base_link";
      // tmp_target.origin = Eigen::Vector3d(0, 0, 0);
      tmp_target->frame_id = msg.sync_point_clouds[i].frame_id;
      tmp_target->origin = msg.sync_point_clouds[i].origin;
      tmp_target->MsgVec = target_map3;
      target_map_full.push_back(tmp_target);
    }
  }

  // return std::make_tuple(target_map_full, pre_model_target_map);
  return true;
}

TargetMap LaserPerception2D::LaserUpdate(
  const sensor::TimedPointCloudOriginData & point_cloud)
{
  // std::vector<FeatureMsg> target_map;
  TargetMap target_map;
  std::vector<std::pair<int, std::vector<int>>> laserPointSet = LaserClustering(point_cloud);
  std::vector<FeatureMsg> target_list;
  std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet =
    LaserPointToOdom(point_cloud, laserPointSet);

  // if (config_.predict_rec) {
  //   PrePointMsg pre_point_msg;
  //   pre_point_msg = recg_by_model_->GetTargetByPredictPose(globalPointSet);
  //   pre_point_msg.time = point_cloud.time;
  //   pre_point_msg.frame_id = "odom";
  //   for (auto & pre_msg : pre_point_msg.target_msgs) {
  //     pre_msg = SlideTargetMap(pre_msg, pre_point_slide_vector_);
  //   }
  //   target_map.pre_point_msg = pre_point_msg;
  //   // target_map.insert(target_map.end(), target_list.begin(), target_list.end());
  // }
  LOG(INFO) << "process is fine";
  return target_map;
}


void LaserPerception2D::UpdateParams(
  const std::string & param_name, double & value)
{
  LOG(INFO) << param_name << " = " << value;
  recg_by_model_->UpdateParams(param_name, value);
  recg_by_reflec_->UpdateParams(param_name, value);
  line_detector_->UpdateParams(param_name, value);
}

void LaserPerception2D::UpdateParams(
  const std::string & param_name, int & value)
{
  LOG(INFO) << param_name << " = " << value;
  if (param_name == "intensities_min") {
    config_.intensity_threshold = value;
  } else if (param_name == "min_cluster_points") {
    config_.min_cluster_points = value;
  }
}

void LaserPerception2D::UpdateParams(
  const std::string & param_name, bool & value)
{
  LOG(INFO) << param_name << " = " << value;
  if (param_name == "predict_rec") {
    config_.predict_rec = value;
  } else if (param_name == "reflect_rec") {
    config_.reflect_rec = value;
  } else if (param_name == "line_rec") {
    config_.line_rec = value;
  }
}

void LaserPerception2D::clear()
{}

TargetMap LaserPerception2D::GetTargetMsg()
{
  TargetMap pre_model_target_map;
  LOG(INFO) << "!!!!!!!!!!!!!!!!!!!!!!!!!!";
  for (size_t i = 0; i < update_msg_.sync_point_clouds.size(); i++) {
    LOG(INFO) << "update size = " << update_msg_.sync_point_clouds.size();
    sensor::PointCloudWithOrigin point_cloud = update_msg_.sync_point_clouds[i];
    std::vector<std::pair<int, Eigen::Vector3f>> point_cloud_in_laser;
    for (size_t i = 0; i < point_cloud.sync_points_base_link.size(); i++) {
      Eigen::Vector3f point_base_link = point_cloud.sync_points_base_link[i];
      transform::Rigid3d origin = point_cloud.origin;
      Eigen::Vector3f point;
      point = origin.cast<float>().inverse() * point_base_link;
      point_cloud_in_laser.push_back(
        std::pair<int, Eigen::Vector3f>(
          point_cloud.intensities[i],
          point));
      point_cloud.sync_points_base_link[i] = point;
    }
    std::vector<std::pair<int, std::vector<int>>> laserPointSet =
      LaserCluster(point_cloud_in_laser);
    if (config_.predict_rec) {
      PrePointMsg pre_point_msg;
      std::vector<std::pair<int, std::vector<cv::Point2d>>> globalPointSet =
        LaserPointToOdom(point_cloud, laserPointSet);
      pre_point_msg = recg_by_model_->GetChargeStationPose(globalPointSet);
      pre_info_ = recg_by_model_->pre_info_;
      pre_point_msg.time = update_msg_.stamp;
      pre_point_msg.frame_id = "odom";
      for (auto & pre_msg : pre_point_msg.target_msgs) {
        pre_msg = SlideTargetMap(pre_msg, pre_point_slide_vector_);
        LOG(INFO) << "pre_msg x = " << pre_msg.x << " y = " << pre_msg.y;
        LOG(INFO) << "pre_msg theta = " << pre_msg.theta;
      }
      pre_model_target_map.pre_point_msg = pre_point_msg;
      LOG(INFO) << "target_msgs size = " << pre_model_target_map.pre_point_msg.target_msgs.size();
      if (pre_point_msg.target_msgs.size() == 0) {
        continue;
      }
      break;
    }
  }


  return pre_model_target_map;
}

bool LaserPerception2D::GetLineAndRef(std::vector<std::shared_ptr<FeatureMsgWithOrigin>> & feature_map)
{
  std::unique_lock<std::mutex> lock(data_mutex_);

  feature_map.clear();
  feature_map = line_and_ref_target_map_;
  return true;
}

void LaserPerception2D::ResetStatus()
{
  std::unique_lock<std::mutex> lock(data_mutex_);
  predict_rec_ = config_.predict_rec;
  reflect_rec_ = config_.reflect_rec;
  vmark_rec_ = config_.vmark_rec;
  laser_sync_ = new LaserSync();
  recg_by_model_ = new RecgByModel(config_);
  recg_by_reflec_ = new RecgByReflec(config_);
  line_detector_ = new LineDetector(config_);
  line_detector_x_ = new LineDetectorX();
}

// std::tuple<LinesAndTextInfo, LinesAndTextInfo>
//     LaserPerception2D::GetPrePointVisInfo(){
//     return std::make_tuple(recg_by_model_->feature_map_vis_info_,
//                             recg_by_model_->target_map_vis_info_);
// }

}  // namespace perception_2d
