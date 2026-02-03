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

#ifndef CARTOGRAPHER__LASER_PERCEPTION__MODEL_MANAGER_H_
#define CARTOGRAPHER__LASER_PERCEPTION__MODEL_MANAGER_H_

#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include "cartographer/laser_perception/proto/laser_perception_options.pb.h"
#include "cartographer/laser_perception/data_type.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Core"
namespace cartographer
{
namespace laser_perception
{


using CallbackFunc = std::function<void ()>;

class ModelManager
{
public:
  explicit ModelManager(const proto::LaserPerceptionOptions & config);
  bool loadModelFile(std::string model_file_dir);
  bool loadRackModel(std::string name, RackInfo rack_info);
  bool loadReflectorModel(std::string name, ReflectorInfo reflector_info);
  bool loadSingleModel(Model model);
  bool getModel(std::string model_name, Model & model);
  Model updateTargetModel(const Model & model, cv::Point3d targetPose);
  std::vector<Line> getModelLines(Model model);
  RackInfo getModelSize(const std::string & name);

  static ModelManager * instance(const proto::LaserPerceptionOptions & config);

private:
  static ModelManager * model_manager_ins_;
  std::map<std::string, Model> model_list_;
};
}  // namespace laser_perception
}  // cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__MODEL_MANAGER_H_
