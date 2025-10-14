/*
 * @Author: Buddy_X guo_linfeng@foxmail.com
 * @Date: 2023-05-25 11:42:19
 * @LastEditors: Buddy_X guo_linfeng@foxmail.com
 * @LastEditTime: 2023-06-02 14:36:59
 * @FilePath: /0_dcm_ws/src/utility/include/converter.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

#ifndef CONVERTER_HPP_
#define CONVERTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <limits>

namespace dcm
{

namespace utility
{

class Converter
{
public:
  static void ConvertPose3DTo2D(const Eigen::Matrix4d & pose_3d, Eigen::Vector3d & pose_2d);

  static void ConvertPose2DTo3D(const Eigen::Vector3d & pose_2d, Eigen::Matrix4d & pose_3d);

  static double GetYawFromQuaternion(const Eigen::Quaterniond & quat);

  static void EulerAngleToRotationMatrix(
    const Eigen::Vector3d & rot_vec, Eigen::Matrix3d & rot_mat);
};

}  // namespace utility

}  // namespace dcm

#endif  // CONVERTER_HPP_
