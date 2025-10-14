// Copyright (c) 2021 dcm360 Inc
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

// simple ini library address & reference
// https://github.com/brofield/simpleini

#ifndef POSE_SAVE_TOOL_HPP_
#define POSE_SAVE_TOOL_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>

#include "glog/logging.h"

#include "file_manager.hpp"
#include "converter.hpp"

namespace dcm
{

namespace utility
{

class PoseSaveTool
{
  typedef enum
  {
    KITTI = 1,
    TUM = 2
  } PoseSaveFormat;

public:
  PoseSaveTool(const std::string & file_path, const std::string & save_format);
  ~PoseSaveTool() = default;

  bool AddPoseData(const Eigen::Matrix4d & pose);
  bool AddPoseData(const Eigen::Vector3d & pose_2d);

  bool AddPoseData(const Eigen::Matrix4d & pose, const double & time);
  bool AddPoseData(const Eigen::Vector3d & pose_2d, const double & time);

  bool AddPoseData(const Eigen::Vector3d & t, const Eigen::Quaterniond & q);
  bool AddPoseData(const Eigen::Vector3d & t, const Eigen::Quaterniond & q, const double & time);

private:
  void SetPoseSaveFormat(std::string format_string);

  void SavePoseKITTI(std::ofstream & ofs, const Eigen::Matrix4d & pose);
  void SavePoseKITTI(std::ofstream & ofs, const Eigen::Vector3d & t, const Eigen::Quaterniond & q);

  void SavePoseTUM(
    std::ofstream & ofs,
    const Eigen::Matrix4d & pose,
    const double & time);

  void SavePoseTUM(
    std::ofstream & ofs,
    const Eigen::Vector3d & t,
    const Eigen::Quaterniond & q,
    const double & time);

private:
  PoseSaveFormat pose_save_format_ = PoseSaveFormat::KITTI;
  std::string pose_file_path_;
  std::ofstream pose_file_;
};

}  // namespace utility

}  // namespace dcm

#endif  // POSE_SAVE_TOOL_HPP_
