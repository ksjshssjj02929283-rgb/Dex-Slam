// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0

#include "glass_detector/single_laser_detector.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace glass_detector
{

SingleLaserDetector::SingleLaserDetector(rclcpp::Node * node, const std::string & ns)
: node_(node), ns_(ns)
{
  declareParams();
  loadParams();

  // LaserScan 通常 best_effort，与 bluesea2 驱动一致
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  qos.best_effort();

  sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, qos,
    std::bind(&SingleLaserDetector::onScan, this, std::placeholders::_1));

  pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(corrected_topic_, qos);

  RCLCPP_INFO(
    node_->get_logger(),
    "[glass_detector/%s] sub=%s pub=%s E1=%s E2=%s",
    ns_.c_str(), scan_topic_.c_str(), corrected_topic_.c_str(),
    e1_.enabled ? "on" : "off", e2_.enabled ? "on" : "off");
}

void SingleLaserDetector::declareParams()
{
  node_->declare_parameter<std::string>(ns_ + ".scan_topic", "/laser_scan/" + ns_);
  node_->declare_parameter<std::string>(
    ns_ + ".corrected_topic", "/laser_scan/" + ns_ + "_corrected");

  node_->declare_parameter<bool>(ns_ + ".enable_e1", true);
  node_->declare_parameter<bool>(ns_ + ".enable_e2", true);

  node_->declare_parameter<double>(ns_ + ".e1_std_threshold", 0.05);
  node_->declare_parameter<double>(ns_ + ".e1_range_jump", 0.30);
  node_->declare_parameter<int>(ns_ + ".e1_window_size", 5);

  node_->declare_parameter<double>(ns_ + ".e2_trigger_intensity", 8000.0);
  node_->declare_parameter<double>(ns_ + ".e2_intensity_delta", 4000.0);
  node_->declare_parameter<int>(ns_ + ".e2_profile_width", 5);
}

void SingleLaserDetector::loadParams()
{
  scan_topic_ = node_->get_parameter(ns_ + ".scan_topic").as_string();
  corrected_topic_ = node_->get_parameter(ns_ + ".corrected_topic").as_string();

  e1_.enabled = node_->get_parameter(ns_ + ".enable_e1").as_bool();
  e1_.std_threshold = node_->get_parameter(ns_ + ".e1_std_threshold").as_double();
  e1_.range_jump = node_->get_parameter(ns_ + ".e1_range_jump").as_double();
  e1_.window_size = static_cast<int>(node_->get_parameter(ns_ + ".e1_window_size").as_int());

  e2_.enabled = node_->get_parameter(ns_ + ".enable_e2").as_bool();
  e2_.trigger_intensity = node_->get_parameter(ns_ + ".e2_trigger_intensity").as_double();
  e2_.intensity_delta = node_->get_parameter(ns_ + ".e2_intensity_delta").as_double();
  e2_.profile_width = static_cast<int>(node_->get_parameter(ns_ + ".e2_profile_width").as_int());
}

void SingleLaserDetector::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 全局开关 use_glass_detection（节点级参数）：false 时 pass-through
  bool use_glass = true;
  if (node_->has_parameter("use_glass_detection")) {
    use_glass = node_->get_parameter("use_glass_detection").as_bool();
  }
  if (!use_glass) {
    pub_->publish(*msg);
    return;
  }

  const std::size_t N = msg->ranges.size();
  if (N == 0) {
    pub_->publish(*msg);
    return;
  }

  std::vector<bool> is_glass(N, false);
  if (e1_.enabled) {
    detectE1Geometric(*msg, e1_, is_glass);
  }
  if (e2_.enabled) {
    detectE2Intensity(*msg, e2_, is_glass);
  }

  // 完整复制 header / 角度 / 强度 / 时间字段，仅改 ranges 部分元素
  auto corrected = std::make_unique<sensor_msgs::msg::LaserScan>(*msg);
  for (std::size_t i = 0; i < N; ++i) {
    if (is_glass[i]) {
      corrected->ranges[i] = estimateGlassFront(*msg, i, is_glass);
    }
  }

  pub_->publish(std::move(corrected));
}

float SingleLaserDetector::estimateGlassFront(
  const sensor_msgs::msg::LaserScan & scan,
  std::size_t i,
  const std::vector<bool> & is_glass) const
{
  const int N = static_cast<int>(scan.ranges.size());

  // 找左侧第一个非玻璃且有效的射线
  int left = static_cast<int>(i) - 1;
  while (left >= 0) {
    if (!is_glass[static_cast<std::size_t>(left)] &&
      std::isfinite(scan.ranges[static_cast<std::size_t>(left)]))
    {
      break;
    }
    --left;
  }

  // 右侧
  int right = static_cast<int>(i) + 1;
  while (right < N) {
    if (!is_glass[static_cast<std::size_t>(right)] &&
      std::isfinite(scan.ranges[static_cast<std::size_t>(right)]))
    {
      break;
    }
    ++right;
  }

  const bool has_left = (left >= 0);
  const bool has_right = (right < N);

  if (has_left && has_right) {
    const float lr = scan.ranges[static_cast<std::size_t>(left)];
    const float rr = scan.ranges[static_cast<std::size_t>(right)];
    return 0.5f * (lr + rr);
  }
  if (has_left) {
    return scan.ranges[static_cast<std::size_t>(left)];
  }
  if (has_right) {
    return scan.ranges[static_cast<std::size_t>(right)];
  }

  // Case 3：完全没有邻居 → 标 range_max 让 SLAM 当无效射线
  return scan.range_max;
}

}  // namespace glass_detector
