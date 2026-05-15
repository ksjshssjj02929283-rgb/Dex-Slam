// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// E2: 反射强度异常法（Wang 2017 三参数）
//
// 来源：raw_sources/02. 激光反射强度异常法.md
// 阈值默认值采用 Wang 2017 论文初值，BlueSea 雷达需现场标定。
// 优点：能补 E1 的"闭合玻璃门"盲区
// 缺点：依赖 intensity 通道；不同厂商强度量纲不一致

#ifndef GLASS_DETECTOR__ALGORITHMS__E2_INTENSITY_HPP_
#define GLASS_DETECTOR__ALGORITHMS__E2_INTENSITY_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace glass_detector
{

struct E2Params
{
  bool   enabled           = true;
  double trigger_intensity = 8000.0;   // 强度触发阈值（孤峰）
  double intensity_delta   = 4000.0;   // 玻璃前沿与邻域最低强度差
  int    profile_width     = 5;        // 玻璃区段最小射线数（窗口）
};

/// 在 is_glass 上 OR-merge E2 检测结果。
/// 若 scan.intensities 为空或长度与 ranges 不一致，则直接返回（不报错）。
void detectE2Intensity(
  const sensor_msgs::msg::LaserScan & scan,
  const E2Params & params,
  std::vector<bool> & is_glass);

}  // namespace glass_detector

#endif  // GLASS_DETECTOR__ALGORITHMS__E2_INTENSITY_HPP_
