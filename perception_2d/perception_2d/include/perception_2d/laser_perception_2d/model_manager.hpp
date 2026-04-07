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

#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_MANAGER_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_MANAGER_HPP_

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include "glog/logging.h"
#include "perception_2d/laser_perception_2d/config.hpp"
#include "perception_2d/laser_perception_2d/model_cloud_generator.hpp"

namespace perception_2d
{
struct Pose
{
  double x;
  double y;
};

struct Element
{
  std::string type;
  std::vector<Pose> poses;
};

struct Component
{
  int element_id;
  int type;
  double x;
  double y;
  double theta;
};

struct Model
{
  std::string name;
  std::map<int, Element> elements;
  std::vector<Component> components;
};

struct RackInfo
{
  double type;
  double rack_length;
  double rack_width;
  double rack_leg_length;
  double rack_leg_width;
};

struct ReflectorInfo
{
  double radian;
  double radius;
};

struct Line
{
  std::string type;
  std::pair<cv::Point2d, cv::Point2d> line;
};

struct FeatureInfo
{
  std::string target_name;
  cv::Point3d pose;
};

// struct FeatureMsg {
//   int id;
//   std::string target_name;
//   std::pair<Eigen::Vector3f, Eigen::Vector3f> line;
//   double x;
//   double y;
//   double theta;
//   double corner_struct_angle;
// };

using CallbackFunc = std::function<void ()>;

class ModelManager
{
public:
  explicit ModelManager(const Config config);
  bool loadModelFile(std::string model_file_dir);
  bool loadRackModel(std::string name, RackInfo rack_info);
  bool loadReflectorModel(std::string name, ReflectorInfo reflector_info);
  bool loadSingleModel(Model model);
  bool getModel(std::string model_name, Model & model);
  Model updateTargetModel(const Model & model, cv::Point3d targetPose);
  bool lineCross(Line l1, Line l2);
  double mul(cv::Point2d a, cv::Point2d b, cv::Point2d c);
  std::vector<Line> getModelLines(Model model);
  std::vector<Line> getModelLines(Model model, cv::Point3d robot_pose);
  sensor::PointCloud getModelCloud(Model model);

  RackInfo getModelSize(const std::string & name);

  static ModelManager * instance(const Config config);

private:
  static ModelManager * model_manager_ins_;
  std::map<std::string, Model> model_list_;
};
}  // namespace perception_2d
#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_MANAGER_HPP_
