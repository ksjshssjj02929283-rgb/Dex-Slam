#include <cmath>
#include <limits>

#include "cartographer_ros/mapping/sensor_bridge.h"
#include "gtest/gtest.h"

namespace cartographer_ros {
namespace {

class InitialScanTest : public ::testing::Test {
 protected:
  InitialScanTest()
      : buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
        health_(std::make_shared<cartographer::mapping::MappingQualityState>()),
        bridge_(1, true, "other_tracking_frame", 0., &buffer_, nullptr,
                health_) {
    buffer_.setUsingDedicatedThread(true);
    scan_.header.frame_id = "base_footprint";
    scan_.range_min = .01f;
    scan_.range_max = 20.f;
    scan_.angle_increment = 0.f;
  }
  void Transform(double x, double yaw) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "base_footprint";
    transform.child_frame_id = "laser";
    transform.transform.translation.x = x;
    transform.transform.rotation.z = std::sin(yaw / 2.);
    transform.transform.rotation.w = std::cos(yaw / 2.);
    ASSERT_TRUE(buffer_.setTransform(transform, "test", true));
    scan_.header.frame_id = "laser";
  }
  tf2_ros::Buffer buffer_;
  std::shared_ptr<cartographer::mapping::MappingQualityState> health_;
  SensorBridge bridge_;
  sensor_msgs::msg::LaserScan scan_;
};

TEST_F(InitialScanTest, StrictMajorityAndIndependentSensors) {
  scan_.ranges = {.2f, 1.f, 1.f};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_FALSE(health_->initial_scans.at("front").insufficient());
  scan_.ranges = {.2f, 1.f};
  bridge_.CheckInitialLaserScan("rear", scan_);
  EXPECT_TRUE(health_->initial_scans.at("rear").insufficient());
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_FALSE(health_->initial_scans.at("front").insufficient());
}

TEST_F(InitialScanTest, InvalidReturnsCountInTotal) {
  scan_.ranges = {0.f,
                  -1.f,
                  30.f,
                  std::numeric_limits<float>::infinity(),
                  std::numeric_limits<float>::quiet_NaN(),
                  1.f};
  bridge_.CheckInitialLaserScan("front", scan_);
  const auto& result = health_->initial_scans.at("front");
  EXPECT_EQ(result.total, 6u);
  EXPECT_EQ(result.outside, 1u);
  EXPECT_TRUE(result.insufficient());
}

TEST_F(InitialScanTest, AllInvalidAndEmptyScansFail) {
  scan_.ranges = {std::numeric_limits<float>::infinity(),
                  std::numeric_limits<float>::quiet_NaN()};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_TRUE(health_->initial_scans.at("front").insufficient());
  EXPECT_EQ(health_->initial_scans.at("front").outside, 0u);
  scan_.ranges.clear();
  bridge_.CheckInitialLaserScan("rear", scan_);
  EXPECT_TRUE(health_->initial_scans.at("rear").insufficient());
  EXPECT_EQ(health_->initial_scans.at("rear").total, 0u);
}

TEST_F(InitialScanTest, OutsideMajorityPassesDespiteInvalidReturns) {
  scan_.ranges = {1.f, 1.f, std::numeric_limits<float>::infinity()};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_FALSE(health_->initial_scans.at("front").insufficient());
  EXPECT_EQ(health_->initial_scans.at("front").outside, 2u);
}

TEST_F(InitialScanTest, AppliesTranslationAndRotationInBaseNotTracking) {
  Transform(.5, 3.141592653589793);
  scan_.ranges = {.4f, .4f, 1.f};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_EQ(health_->initial_scans.at("front").outside, 1u);
  EXPECT_TRUE(health_->initial_scans.at("front").insufficient());
}

TEST_F(InitialScanTest, MissingTransformRetries) {
  scan_.header.frame_id = "laser";
  scan_.ranges = {.2f};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_FALSE(health_->initial_scans.at("front").checked);
  Transform(0., 0.);
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_TRUE(health_->initial_scans.at("front").insufficient());
}

TEST_F(InitialScanTest, OpenRectangleBoundary) {
  // y is exactly the double-precision rectangle boundary after translation.
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base_footprint";
  transform.child_frame_id = "laser";
  transform.transform.translation.y = .3;
  transform.transform.rotation.w = 1.;
  ASSERT_TRUE(buffer_.setTransform(transform, "test", true));
  scan_.header.frame_id = "laser";
  scan_.ranges = {.2f};
  bridge_.CheckInitialLaserScan("front", scan_);
  EXPECT_EQ(health_->initial_scans.at("front").outside, 1u);
  EXPECT_FALSE(health_->initial_scans.at("front").insufficient());
}

TEST_F(InitialScanTest, NewSessionChecksAgain) {
  scan_.ranges = {1.f};
  bridge_.CheckInitialLaserScan("front", scan_);
  auto fresh = std::make_shared<cartographer::mapping::MappingQualityState>();
  SensorBridge next(1, true, "base_footprint", 0., &buffer_, nullptr, fresh);
  scan_.ranges = {.2f};
  next.CheckInitialLaserScan("front", scan_);
  EXPECT_TRUE(fresh->initial_scans.at("front").insufficient());
  EXPECT_FALSE(health_->initial_scans.at("front").insufficient());
}

}  // namespace
}  // namespace cartographer_ros
