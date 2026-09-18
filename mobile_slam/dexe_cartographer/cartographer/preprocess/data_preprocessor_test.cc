// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "cartographer/preprocess/data_preprocessor.h"

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/rigid_transform.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace preprocess {
namespace {

// 里程计陈旧守卫（odom delta 限幅）：短时（≤ max_odometry_extrapolation_seconds）仍按最后 twist
// 外插，超时则保持最后位姿且 twist 置零。背景：底盘掉 OP 后里程计停发，若无上限外插，
// 预测位姿会按最后一瞬的速度匀速跑飞，之后所有恢复都在错误位置搜索。
class DataPreprocessorOdomTest : public ::testing::Test {
 protected:
  DataPreprocessorOdomTest() {
    proto::DataPreprocessorOptions options;
    options.set_use_odometry(true);
    options.set_max_odometry_extrapolation_seconds(0.2);
    preprocessor_ = std::make_unique<DataPreprocessor>(options);
    // 三帧 100 Hz 样本，沿 x 以 1 m/s 前进
    for (int i = 0; i < 3; ++i) {
      const double t = 0.01 * i;
      preprocessor_->AddOdometryData(sensor::OdometryData{
        At(t),
        transform::Rigid3d::Translation(Eigen::Vector3d(t * 1.0, 0., 0.)),
        sensor::Twist{Eigen::Vector3d(1.0, 0., 0.), Eigen::Vector3d::Zero()}});
    }
  }
  static common::Time At(double seconds) {
    return common::FromUniversal(0) + common::FromSeconds(seconds);
  }
  std::unique_ptr<DataPreprocessor> preprocessor_;
};

TEST_F(DataPreprocessorOdomTest, ShortGapStillExtrapolates) {
  // 最新样本 t=0.02，查询 t=0.12（间隔 0.1 s ≤ 0.2）→ 外插 0.1 m
  const auto odom = preprocessor_->GetOdometryAtTime(At(0.12));
  ASSERT_TRUE(odom.has_value());
  EXPECT_NEAR(odom->pose.translation().x(), 0.02 + 0.10, 1e-6);
  EXPECT_NEAR(odom->twist.linear.x(), 1.0, 1e-9);
}

TEST_F(DataPreprocessorOdomTest, StaleOdometryHoldsLastPoseWithZeroTwist) {
  // 查询 t=5.02（间隔 5 s > 0.2）→ 保持 t=0.02 的位姿，twist 置零，不再跑飞
  const auto odom = preprocessor_->GetOdometryAtTime(At(5.02));
  ASSERT_TRUE(odom.has_value());
  EXPECT_NEAR(odom->pose.translation().x(), 0.02, 1e-9);
  EXPECT_NEAR(odom->twist.linear.norm(), 0.0, 1e-12);
  EXPECT_NEAR(odom->twist.angular.norm(), 0.0, 1e-12);
}

TEST_F(DataPreprocessorOdomTest, InterpolationInsideBufferUnchanged) {
  const auto odom = preprocessor_->GetOdometryAtTime(At(0.015));
  ASSERT_TRUE(odom.has_value());
  EXPECT_NEAR(odom->pose.translation().x(), 0.015, 1e-6);
}

}  // namespace
}  // namespace preprocess
}  // namespace cartographer
