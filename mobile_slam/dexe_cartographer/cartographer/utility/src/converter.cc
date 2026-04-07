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

#include <limits>

#include "cartographer/utility/include/converter.hpp"

namespace cartographer
{

namespace utility
{

void Converter::ConvertPose3DTo2D(
  const Eigen::Matrix4d & pose_3d, Eigen::Vector3d & pose_2d)
{
  pose_2d.setZero();
  pose_2d.x() = pose_3d(0, 3);
  pose_2d.y() = pose_3d(1, 3);

  double yaw;
  Eigen::Quaterniond quat(pose_3d.block<3, 3>(0, 0));

  quat.normalize();
  if (std::fabs(quat.x()) < std::numeric_limits<double>::epsilon() &&
    std::fabs(quat.y()) < std::numeric_limits<double>::epsilon())
  {
    // 说明旋转是Z轴的二维旋转，可以直接作用四元数转换
    yaw = 2 * atan(quat.z() / quat.w());
  } else {
    yaw = pose_3d.block<3, 3>(0, 0).eulerAngles(2, 1, 0).x();
  }

  pose_2d.z() = yaw;
}

void Converter::ConvertPose2DTo3D(
  const Eigen::Vector3d & pose_2d, Eigen::Matrix4d & pose_3d)
{
  Eigen::Isometry3d iso_mat = Eigen::Isometry3d::Identity();
  iso_mat.translate(Eigen::Vector3d(pose_2d.x(), pose_2d.y(), 0.0));
  iso_mat.rotate(Eigen::AngleAxisd(pose_2d.z(), Eigen::Vector3d::UnitZ()));
  pose_3d = iso_mat.matrix();
}

double Converter::GetYawFromQuaternion(
  const Eigen::Quaterniond & quat)
{
  double yaw;
  if (std::fabs(quat.x()) < std::numeric_limits<double>::epsilon() &&
    std::fabs(quat.y()) < std::numeric_limits<double>::epsilon())
  {
    // 说明旋转是Z轴的二维旋转，可以直接作用四元数转换
    yaw = 2 * atan(quat.z() / quat.w());
  } else {
    yaw = quat.toRotationMatrix().eulerAngles(2, 1, 0).x();
  }

  return yaw;
}

void Converter::EulerAngleToRotationMatrix(
  const Eigen::Vector3d & rot_vec, Eigen::Matrix3d & rot_mat)
{
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rot_vec(0), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rot_vec(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rot_vec(2), Eigen::Vector3d::UnitZ()));

  rot_mat = yawAngle * pitchAngle * rollAngle;
}

}  // namespace utility

}  // namespace cartographer
