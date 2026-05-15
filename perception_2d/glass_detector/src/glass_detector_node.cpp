// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0

#include "glass_detector/glass_detector_node.hpp"

#include <memory>

namespace glass_detector
{

GlassDetectorNode::GlassDetectorNode()
: rclcpp::Node("glass_detector")
{
  // 全局开关：true 时启用玻璃检测，false 时 pass-through
  declare_parameter<bool>("use_glass_detection", true);

  // 双雷达完全独立 instance（参数命名空间 front. / rear.）
  front_ = std::make_unique<SingleLaserDetector>(this, "front");
  rear_ = std::make_unique<SingleLaserDetector>(this, "rear");

  RCLCPP_INFO(
    get_logger(),
    "glass_detector_node up — front + rear instances loaded; "
    "use_glass_detection=%s",
    get_parameter("use_glass_detection").as_bool() ? "true" : "false");
}

}  // namespace glass_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<glass_detector::GlassDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
