// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0
//
// GlassDetectorNode：单节点 + 双 SingleLaserDetector instance（front / rear）。

#ifndef GLASS_DETECTOR__GLASS_DETECTOR_NODE_HPP_
#define GLASS_DETECTOR__GLASS_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "glass_detector/single_laser_detector.hpp"

namespace glass_detector
{

class GlassDetectorNode : public rclcpp::Node
{
public:
  GlassDetectorNode();

private:
  std::unique_ptr<SingleLaserDetector> front_;
  std::unique_ptr<SingleLaserDetector> rear_;
};

}  // namespace glass_detector

#endif  // GLASS_DETECTOR__GLASS_DETECTOR_NODE_HPP_
