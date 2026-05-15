// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// E2 强度异常法单测

#include <gtest/gtest.h>

#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "glass_detector/algorithms/e2_intensity.hpp"

namespace
{

sensor_msgs::msg::LaserScan makeScan(
  const std::vector<float> & ranges,
  const std::vector<float> & intensities)
{
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = ranges;
  scan.intensities = intensities;
  scan.range_min = 0.1F;
  scan.range_max = 50.0F;
  return scan;
}

}  // namespace

TEST(E2Intensity, EmptyIntensitiesNoOp)
{
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {1.0F, 1.0F, 1.0F, 1.0F, 1.0F};
  std::vector<bool> is_glass(scan.ranges.size(), false);
  glass_detector::E2Params p;
  glass_detector::detectE2Intensity(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E2Intensity, LowIntensityNoOp)
{
  // 全低强度（远低于 trigger 8000）
  std::vector<float> r(20, 1.0F);
  std::vector<float> ii(20, 100.0F);
  auto scan = makeScan(r, ii);
  std::vector<bool> is_glass(20, false);
  glass_detector::E2Params p;
  p.trigger_intensity = 8000.0;
  glass_detector::detectE2Intensity(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E2Intensity, IntensitySpikeDetected)
{
  // 背景强度 1000，玻璃前沿三束孤峰到 12000（高于 trigger + Δ）
  std::vector<float> r(20, 1.0F);
  std::vector<float> ii(20, 1000.0F);
  ii[10] = 12000.0F;
  ii[11] = 12000.0F;
  ii[12] = 12000.0F;

  auto scan = makeScan(r, ii);
  std::vector<bool> is_glass(20, false);
  glass_detector::E2Params p;
  p.trigger_intensity = 8000.0;
  p.intensity_delta = 4000.0;
  p.profile_width = 5;
  glass_detector::detectE2Intensity(scan, p, is_glass);

  // 峰值附近至少一束被标记
  bool any_marked = is_glass[9] || is_glass[10] || is_glass[11] || is_glass[12] || is_glass[13];
  EXPECT_TRUE(any_marked);
}

TEST(E2Intensity, IntensityDeltaTooSmallNoOp)
{
  // 强度高于 trigger 但邻域同样高 → Δ 不足，不算玻璃前沿
  std::vector<float> r(20, 1.0F);
  std::vector<float> ii(20, 11000.0F);  // 整体高强度（如全是反光物）
  auto scan = makeScan(r, ii);
  std::vector<bool> is_glass(20, false);
  glass_detector::E2Params p;
  p.trigger_intensity = 8000.0;
  p.intensity_delta = 4000.0;
  p.profile_width = 5;
  glass_detector::detectE2Intensity(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E2Intensity, DisabledNoOp)
{
  std::vector<float> r(20, 1.0F);
  std::vector<float> ii(20, 12000.0F);
  auto scan = makeScan(r, ii);
  std::vector<bool> is_glass(20, false);
  glass_detector::E2Params p;
  p.enabled = false;
  glass_detector::detectE2Intensity(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}

TEST(E2Intensity, SizeMismatchNoOp)
{
  // ranges 和 intensities 长度不一致 → 跳过
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = std::vector<float>(20, 1.0F);
  scan.intensities = std::vector<float>(15, 12000.0F);
  std::vector<bool> is_glass(20, false);
  glass_detector::E2Params p;
  glass_detector::detectE2Intensity(scan, p, is_glass);
  for (bool g : is_glass) {
    EXPECT_FALSE(g);
  }
}
