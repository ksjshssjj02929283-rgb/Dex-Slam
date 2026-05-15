// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// E1 几何不连续法单测

#include <gtest/gtest.h>

#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "glass_detector/algorithms/e1_geometric.hpp"

namespace
{

sensor_msgs::msg::LaserScan makeScan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = ranges;
  scan.range_min = 0.1f;
  scan.range_max = 50.0f;
  return scan;
}

}  // namespace

TEST(E1Geometric, FlatWallNoGlass)
{
  // 平直墙面，全 1.0m 无跳变
  auto scan = makeScan({1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F});
  std::vector<bool> is_glass(scan.ranges.size(), false);
  glass_detector::E1Params p;
  p.range_jump = 0.3;
  p.std_threshold = 0.05;
  glass_detector::detectE1Geometric(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E1Geometric, LargeRangeJumpDetected)
{
  // 前 4 束打到 1m 墙，后 6 束穿透到 10m（玻璃后方）
  auto scan = makeScan({1.0F, 1.0F, 1.0F, 1.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F, 10.0F});
  std::vector<bool> is_glass(scan.ranges.size(), false);
  glass_detector::E1Params p;
  p.range_jump = 0.3;
  glass_detector::detectE1Geometric(scan, p, is_glass);
  // 跳变位置（3 → 4）两束都被标记
  EXPECT_TRUE(is_glass[3]);
  EXPECT_TRUE(is_glass[4]);
}

TEST(E1Geometric, DisabledNoOp)
{
  auto scan = makeScan({1.0F, 10.0F, 1.0F, 10.0F, 1.0F});
  std::vector<bool> is_glass(scan.ranges.size(), false);
  glass_detector::E1Params p;
  p.enabled = false;
  glass_detector::detectE1Geometric(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E1Geometric, EmptyScanNoCrash)
{
  sensor_msgs::msg::LaserScan scan;
  scan.range_min = 0.1F;
  scan.range_max = 50.0F;
  std::vector<bool> is_glass;
  glass_detector::E1Params p;
  glass_detector::detectE1Geometric(scan, p, is_glass);
  EXPECT_TRUE(is_glass.empty());
}

TEST(E1Geometric, IsGlassResizedToScanSize)
{
  auto scan = makeScan({1.0F, 1.0F, 1.0F});
  std::vector<bool> is_glass;  // 空，函数应自动 resize
  glass_detector::E1Params p;
  glass_detector::detectE1Geometric(scan, p, is_glass);
  EXPECT_EQ(is_glass.size(), 3U);
}
