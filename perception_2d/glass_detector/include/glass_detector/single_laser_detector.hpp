// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// SingleLaserDetector：处理单路雷达的检测 + 修正 + 发布。
// 一个节点持有 2 个 instance（front / rear），双雷达完全独立处理。

#ifndef GLASS_DETECTOR__SINGLE_LASER_DETECTOR_HPP_
#define GLASS_DETECTOR__SINGLE_LASER_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <string>
#include <vector>

#include "glass_detector/algorithms/e1_geometric.hpp"
#include "glass_detector/algorithms/e2_intensity.hpp"

namespace glass_detector
{

class SingleLaserDetector
{
public:
  /// 构造时声明并加载 ns 子命名空间下的参数（front. / rear.），
  /// 然后建立 subscription 和 publisher。
  SingleLaserDetector(rclcpp::Node * node, const std::string & ns);

private:
  void declareParams();
  void loadParams();
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /// 玻璃前沿距离估算（方案 §4.2）：
  /// Case 1：相邻非玻璃射线插值 → 取均值
  /// Case 3：无相邻有效射线 → 返回 range_max（让 SLAM 当作无效射线丢弃）
  float estimateGlassFront(
    const sensor_msgs::msg::LaserScan & scan,
    std::size_t i,
    const std::vector<bool> & is_glass) const;

  rclcpp::Node * node_;
  std::string ns_;
  std::string scan_topic_;
  std::string corrected_topic_;

  E1Params e1_;
  E2Params e2_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

}  // namespace glass_detector

#endif  // GLASS_DETECTOR__SINGLE_LASER_DETECTOR_HPP_
