// Copyright 2026 Dexforce. Licensed under the Apache License, Version 2.0.

#include "cartographer/localization/internal/2d/pose_estimator_options_2d.h"

#include <string>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/config.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace localization {
namespace {

// 用【真实】配置文件驱动整条 lua → proto 解析链。cartographer 对未消费的 lua 键会 CHECK 失败，
// 解析器少写一行会在机器人启动时才炸，这里在本机就抓住。
proto::PoseEstimatorOptions2D Load(const std::string& extra_lua) {
  const std::string code = "include \"pose_estimator_2d.lua\"\n" + extra_lua +
                           "\nreturn POSE_ESTIMATOR_2D";
  auto file_resolver = ::absl::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string>{
          std::string(common::kSourceDirectory) + "/configuration_files"});
  common::LuaParameterDictionary dict(code, std::move(file_resolver));
  return CreatePoseEstimatorOptions2D(&dict);
}

TEST(PoseEstimatorOptions2DTest, RecoveryFcsmWindowParsedFromRealLua) {
  const auto& fcsm = Load("").consistency_checker_options().fast_correlative_scan_matcher_options();
  EXPECT_DOUBLE_EQ(fcsm.linear_search_window(), 3.0);
  EXPECT_NEAR(fcsm.angular_search_window(), 3.14159265358979, 1e-6);
  EXPECT_EQ(fcsm.branch_and_bound_depth(), 7);
}

TEST(PoseEstimatorOptions2DTest, RecoveryFcsmFallsBackToDefaultsWhenAbsent) {
  // 现场部署覆盖文件可能没有这个子表：旧 lua 也必须能加载
  const auto& fcsm = Load("POSE_ESTIMATOR_2D.consistency_checker.fast_correlative_scan_matcher = nil")
                         .consistency_checker_options().fast_correlative_scan_matcher_options();
  EXPECT_DOUBLE_EQ(fcsm.linear_search_window(), 3.0);
  EXPECT_EQ(fcsm.branch_and_bound_depth(), 7);
}

TEST(PoseEstimatorOptions2DTest, OdometryExtrapolationGuardParsedFromRealLua) {
  EXPECT_NEAR(Load("").data_preprocessor_options().max_odometry_extrapolation_seconds(), 0.2, 1e-6);
  EXPECT_NEAR(Load("POSE_ESTIMATOR_2D.data_preprocessor.max_odometry_extrapolation_seconds = nil")
                  .data_preprocessor_options().max_odometry_extrapolation_seconds(), 0.2, 1e-6);
}

}  // namespace
}  // namespace localization
}  // namespace cartographer
