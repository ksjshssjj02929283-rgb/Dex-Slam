// Copyright (c) 2026 dexforce Inc
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

#include <string>
#include <algorithm>

#include "cartographer/utility/include/pose_save_tool.hpp"

namespace cartographer
{

namespace utility
{

void PoseSaveTool::SetPoseSaveFormat(std::string format_string)
{
  // 将字符串转换为小写
  std::transform(format_string.begin(), format_string.end(), format_string.end(), ::tolower);
  if (format_string == "kitti") {
    pose_save_format_ = PoseSaveFormat::KITTI;
  } else if (format_string == "tum") {
    pose_save_format_ = PoseSaveFormat::TUM;
    pose_file_ << "# timestamp tx ty tz qx qy qz qw\n";
  } else {
    pose_save_format_ = PoseSaveFormat::KITTI;
  }
}


PoseSaveTool::PoseSaveTool(const std::string & file_path, const std::string & save_format)
{
  if (!FileManager::CreateFile(pose_file_, file_path)) {
    LOG(ERROR) << "Cannot create file at : " << file_path <<
      ". Please check your path is vaild." << std::endl;
    return;
  }
  SetPoseSaveFormat(save_format);
}


bool PoseSaveTool::AddPoseData(const Eigen::Matrix4d & pose)
{
  if (pose_save_format_ == PoseSaveFormat::KITTI) {
    SavePoseKITTI(pose_file_, pose);
  } else if (pose_save_format_ == PoseSaveFormat::TUM) {
    LOG(ERROR) << "for saving tum format, please add time stamp." << std::endl;
    return false;
  }
  return true;
}


bool PoseSaveTool::AddPoseData(const Eigen::Vector3d & pose_2d)
{
  Eigen::Matrix4d pose_3d;
  Converter::ConvertPose2DTo3D(pose_2d, pose_3d);
  AddPoseData(pose_3d);
  return true;
}


bool PoseSaveTool::AddPoseData(const Eigen::Matrix4d & pose, const double & time)
{
  if (pose_save_format_ == PoseSaveFormat::TUM) {
    SavePoseTUM(pose_file_, pose, time);
  } else if (pose_save_format_ == PoseSaveFormat::KITTI) {
    SavePoseKITTI(pose_file_, pose);
  }
  return true;
}

bool PoseSaveTool::AddPoseData(const Eigen::Vector3d & pose_2d, const double & time)
{
  Eigen::Matrix4d pose_3d;
  Converter::ConvertPose2DTo3D(pose_2d, pose_3d);
  AddPoseData(pose_3d, time);
  return true;
}

/**
 * @brief 通过translation和quaternion添加位姿数据，只用于KITTI格式
 * 
 * @param t translation
 * @param q quaternion
 * @return true 
 * @return false 
 */
bool PoseSaveTool::AddPoseData(const Eigen::Vector3d & t, const Eigen::Quaterniond & q)
{
  if (pose_save_format_ == PoseSaveFormat::KITTI) {
    SavePoseKITTI(pose_file_, t, q);
  } else if (pose_save_format_ == PoseSaveFormat::TUM) {
    LOG(ERROR) << "for saving tum format, please add time stamp." << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief 通过translation和quaternion添加位姿数据
 * 
 * @param t translation
 * @param q quaternion
 * @param time 位姿时间戳
 * @return true 
 * @return false 
 */
bool PoseSaveTool::AddPoseData(const Eigen::Vector3d & t, const Eigen::Quaterniond & q, const double & time)
{
  if (pose_save_format_ == PoseSaveFormat::TUM) {
    SavePoseTUM(pose_file_, t, q, time);
  } else if (pose_save_format_ == PoseSaveFormat::KITTI) {
    SavePoseKITTI(pose_file_, t, q);
  }
  return true;
}

/**
 * @brief 保存位姿到KITTI格式文件(matrix4d格式的位姿)
 * 
 * @param ofs ofstream文件流
 * @param pose matrix4d格式的位姿
 */
void PoseSaveTool::SavePoseKITTI(std::ofstream & ofs, const Eigen::Matrix4d & pose)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);
      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }
}


/**
 * @brief 保存位姿到KITTI格式文件(translation和quaternion格式的位姿)
 * 
 * @param ofs ofstream文件流
 * @param t translation
 * @param q quaternion
 */
void PoseSaveTool::SavePoseKITTI(std::ofstream & ofs, const Eigen::Vector3d & t, const Eigen::Quaterniond & q)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(t);
  pose.rotate(q);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      ofs << pose(i, j);
      if (i == 2 && j == 3) {
        ofs << std::endl;
      } else {
        ofs << " ";
      }
    }
  }
}

/**
 * @brief 保存位姿到TUM格式文件(matrix4d格式的位姿)
 * 
 * @param ofs ofstream文件流
 * @param pose matrix4d格式的位姿
 * @param time 位姿时间戳
 */
void PoseSaveTool::SavePoseTUM(
  std::ofstream & ofs,
  const Eigen::Matrix4d & pose,
  const double & time)
{
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  Eigen::Vector3d trans(pose.block<3, 1>(0, 3));
  ofs << std::to_string(time) << " " << trans.x() << " " << trans.y() << " " << trans.z() << " " <<
    quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
}

/**
 * @brief 保存位姿到TUM格式文件(translation和quaternion格式的位姿)
 * 
 * @param ofs ofstream文件流
 * @param trans translation
 * @param quat quaternion
 * @param time 时间戳
 */
void PoseSaveTool::SavePoseTUM(
  std::ofstream & ofs,
  const Eigen::Vector3d & t,
  const Eigen::Quaterniond & q,
  const double & time)
{
  Eigen::Vector3d trans = t;
  Eigen::Quaterniond quat = q;
  ofs << std::to_string(time) << " " << trans.x() << " " << trans.y() << " " << trans.z() << " " <<
    quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;
}

}  // namespace utility

}  // namespace cartographer
