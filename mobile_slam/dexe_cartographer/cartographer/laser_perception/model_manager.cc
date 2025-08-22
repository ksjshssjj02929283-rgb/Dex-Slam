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

#include "cartographer/laser_perception/model_manager.h"
#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <vector>
#include <utility>
#include <glog/logging.h>

namespace cartographer
{
namespace laser_perception
{
ModelManager * ModelManager::model_manager_ins_ = nullptr;
ModelManager::ModelManager(const proto::LaserPerceptionOptions & config)
{
  loadModelFile(config.model_file());
}

// 加载目标的特征文件列表
bool ModelManager::loadModelFile(std::string model_file_dir)
{
  // model_file_dir = "/home/young/data/model.yaml";
  // model_file_dir = "/root/model.yaml";
  try {
    YAML::Node model_file = YAML::LoadFile(model_file_dir);
    if (model_file["shelf"].IsDefined()) {
      YAML::Node shelf_models = model_file["shelf"];
      for (YAML::Node model : shelf_models) {
        std::string model_name = model["name"].as<std::string>();
        printf("load model:%s\n", model_name.c_str());
        RackInfo rack_info;
        rack_info.type = model["type"].as<int>();
        rack_info.rack_length = model["rack_length"].as<double>();
        rack_info.rack_width = model["rack_width"].as<double>();
        rack_info.rack_leg_length = model["rack_leg_length"].as<double>();
        rack_info.rack_leg_width = model["rack_leg_width"].as<double>();
        loadRackModel(model_name, rack_info);
      }
    }

    if (model_file["reflector"].IsDefined()) {
      YAML::Node reflector_models = model_file["reflector"];
      ReflectorInfo reflector_info;
      reflector_info.radius = reflector_models["radius"].as<double>();
      reflector_info.radian = reflector_models["radian"].as<double>();
      printf("load model: reflector\n");
      loadReflectorModel("reflector", reflector_info);
    }

    if (model_file["chargingStation"].IsDefined()) {
      YAML::Node chargingStation_models = model_file["chargingStation"];
      for (YAML::Node chargingStation : chargingStation_models) {
        Model model;
        model.name = chargingStation["name"].as<std::string>();
        printf("load model:%s\n", model.name.c_str());
        YAML::Node elements = chargingStation["elements"];
        for (YAML::Node element_node : elements) {
          Element element;
          element.type = "polygonal";
          int id = element_node["id"].as<int>();
          YAML::Node points = element_node["points"];
          for (YAML::Node point : points) {
            Pose pose;
            pose.x = point["x"].as<double>();
            pose.y = point["y"].as<double>();
            element.poses.push_back(pose);
          }
          model.elements[id] = element;
        }

        YAML::Node components = chargingStation["components"];
        for (YAML::Node component_node : components) {
          Component component;
          component.element_id = component_node["elementId"].as<int>();
          component.x = component_node["x"].as<double>();
          component.y = component_node["y"].as<double>();
          component.theta = component_node["theta"].as<double>();
          component.type = component_node["type"].as<int>();
          model.components.push_back(component);
        }
        loadSingleModel(model);
      }
    }
    if (model_file["shapeStruct"].IsDefined()) {
      YAML::Node chargingStation_models = model_file["shapeStruct"];
      for (YAML::Node chargingStation : chargingStation_models) {
        Model model;
        model.name = chargingStation["name"].as<std::string>();
        printf("load model:%s\n", model.name.c_str());
        YAML::Node elements = chargingStation["elements"];
        for (YAML::Node element_node : elements) {
          Element element;
          element.type = "polygonal";
          int id = element_node["id"].as<int>();
          YAML::Node points = element_node["points"];
          for (YAML::Node point : points) {
            Pose pose;
            pose.x = point["x"].as<double>();
            pose.y = point["y"].as<double>();
            element.poses.push_back(pose);
          }
          model.elements[id] = element;
        }

        YAML::Node components = chargingStation["components"];
        for (YAML::Node component_node : components) {
          Component component;
          component.element_id = component_node["elementId"].as<int>();
          component.x = component_node["x"].as<double>();
          component.y = component_node["y"].as<double>();
          component.theta = component_node["theta"].as<double>();
          component.type = component_node["type"].as<int>();
          model.components.push_back(component);
        }
        loadSingleModel(model);
      }
    }
    return true;
  } catch (...) {  // 文件为非yaml格式抛出异常
    LOG(INFO) << "error" << std::endl;
    return false;
  }
}

bool ModelManager::loadRackModel(std::string name, RackInfo rack_info)
{
  Model rack_model;
  rack_model.name = name;
  int multi_table[5][2] = {{-1, -1}, {-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
  Element element;
  element.type = "polygonal";
  for (int i = 0; i < 5; i++) {
    Pose pose;
    pose.x = multi_table[i][0] * rack_info.rack_leg_length / 2;
    pose.y = multi_table[i][1] * rack_info.rack_leg_width / 2;
    element.poses.push_back(pose);
  }
  rack_model.elements[0] = element;

  for (int i = 0; i < 4; i++) {
    Component component;
    component.x = multi_table[i][0] * rack_info.rack_length / 2;
    component.y = multi_table[i][1] * rack_info.rack_width / 2;
    component.type = rack_info.type;
    component.element_id = 0;
    rack_model.components.push_back(component);
  }
  model_list_[name] = rack_model;
  return true;
}

bool ModelManager::loadReflectorModel(std::string name, ReflectorInfo reflector_info)
{
  Model reflector_model;
  reflector_model.name = name;
  Element element;
  element.type = "circle";

  Pose pose;
  pose.x = reflector_info.radius;
  pose.y = reflector_info.radian;
  element.poses.push_back(pose);
  reflector_model.elements[0] = element;


  Component component;
  component.x = 0;
  component.y = 0;
  component.type = 2;
  component.element_id = 0;
  reflector_model.components.push_back(component);

  model_list_[name] = reflector_model;
  return true;
}

bool ModelManager::loadSingleModel(Model model)
{
  model_list_[model.name] = model;
  return true;
}

bool ModelManager::getModel(std::string model_name, Model & model)
{
  // std::map<std::string, Model>::iterator it = model_list_.begin();
  if (model_list_.find(model_name) == model_list_.end()) {
    printf("don't have this model");
    return false;
  } else {
    model = model_list_[model_name];
    return true;
  }
}

std::vector<Line> ModelManager::getModelLines(Model model)
{
  std::vector<Line> lines;
  for (auto component : model.components) {
    Element element = model.elements[component.element_id];
    if (element.type == "circle") {
      Line line;
      line.type = "circle";
      line.line =
        std::make_pair(
        cv::Point2d(component.x, component.y),
        cv::Point2d(element.poses[0].x, element.poses[0].y));
      lines.push_back(line);
    }
    if (element.type == "polygonal") {
      for (int i = 0; i < static_cast<int>(element.poses.size()) - 1; i++) {
        cv::Point2d start_pose;
        start_pose.x = element.poses[i].x * cos(component.theta) - element.poses[i].y * sin(
          component.theta) + component.x;
        start_pose.y = element.poses[i].x * sin(component.theta) + element.poses[i].y * cos(
          component.theta) + component.y;
        cv::Point2d end_pose;
        end_pose.x = element.poses[i + 1].x * cos(component.theta) - element.poses[i + 1].y * sin(
          component.theta) + component.x;
        end_pose.y = element.poses[i + 1].x * sin(component.theta) + element.poses[i + 1].y * cos(
          component.theta) + component.y;
        Line line;
        line.type = "line";
        line.line = std::make_pair(start_pose, end_pose);
        lines.push_back(line);
      }
    }
  }
  return lines;
}

Model ModelManager::updateTargetModel(const Model & model, cv::Point3d targetPose)
{
  Model model_update = model;
  for (int i = 0; i < static_cast<int>(model.components.size()); i++) {
    cv::Point2d tmp;
    double x = model.components[i].x;
    double y = model.components[i].y;
    double theta = targetPose.z;

    tmp.x = x * cos(theta) - y * sin(theta) + targetPose.x;
    tmp.y = x * sin(theta) + y * cos(theta) + targetPose.y;
    theta += model.components[i].theta;
    model_update.components[i].x = tmp.x;
    model_update.components[i].y = tmp.y;
    model_update.components[i].theta = theta;
  }
  return model_update;
}


RackInfo ModelManager::getModelSize(const std::string & name)
{
  RackInfo rack_info;
  Model model = model_list_[name];
  rack_info.rack_leg_length = fabs(model.elements[0].poses[0].x * 2);
  rack_info.rack_leg_width = fabs(model.elements[0].poses[0].y * 2);
  rack_info.rack_length = fabs(model.components[0].x * 2) + rack_info.rack_leg_length;
  rack_info.rack_width = fabs(model.components[0].y * 2) + rack_info.rack_leg_width;
  return rack_info;
}

ModelManager * ModelManager::instance(const proto::LaserPerceptionOptions & config)
{
  if (model_manager_ins_ == nullptr) {
    static ModelManager model_manager(config);
    model_manager_ins_ = &model_manager;
  }
  return model_manager_ins_;
}

}  // namespace laser_perception
}  // cartographer
