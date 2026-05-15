// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// E1: 几何不连续法（Tibebu 2021 风格的两步滤波）
//
// 来源：raw_sources/01. 激光几何不连续法.md
// 优点：不依赖 intensity 通道，对任何 2D LiDAR 适用，W1 PRO 主力路线
// 缺点：对"整面闭合玻璃门"几何无突变信号弱（用 E2 补）

#ifndef GLASS_DETECTOR__ALGORITHMS__E1_GEOMETRIC_HPP_
#define GLASS_DETECTOR__ALGORITHMS__E1_GEOMETRIC_HPP_

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace glass_detector
{

struct E1Params
{
  bool   enabled       = true;
  double std_threshold = 0.05;   // m，邻域距离标准差阈值
  double range_jump    = 0.30;   // m，邻接射线距离跳变阈值
  int    window_size   = 5;      // 邻域窗口（用于局部 std 计算）
};

/// 在 is_glass 上 OR-merge E1 检测结果（不清空已有 true 标记）。
/// is_glass 大小自动对齐到 scan.ranges.size()。
void detectE1Geometric(
  const sensor_msgs::msg::LaserScan & scan,
  const E1Params & params,
  std::vector<bool> & is_glass);

}  // namespace glass_detector

#endif  // GLASS_DETECTOR__ALGORITHMS__E1_GEOMETRIC_HPP_
