// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer/mapping/2d/structure_alignment.h"

#include <cmath>
#include <map>
#include <random>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

// 构造以 'mid' 为中心、给定长度与方向的线段。
// 注意 LineData 构造器会按端点到原点的范数交换端点（方向可能翻转 π，
// 估计器按 mod π 归一化，无影响）；|dx|/|dy| < 1e-5 时角度被吸到 0/π/2
// （对刻意构造的轴向线段而言正是期望行为）。
sensor::LineData MakeLine(const Eigen::Vector2d& mid, double length,
                          double angle) {
  const Eigen::Vector2d d =
      0.5 * length * Eigen::Vector2d(std::cos(angle), std::sin(angle));
  return sensor::LineData(mid - d, mid + d);
}

LineMap2DList MakeLineList(const std::vector<sensor::LineData>& lines) {
  std::map<int, MapLine2D> line_map;
  for (size_t i = 0; i < lines.size(); ++i) {
    line_map.emplace(static_cast<int>(i),
                     MapLine2D(static_cast<int>(i), lines[i]));
  }
  return LineMap2DList(static_cast<int>(lines.size()), std::move(line_map));
}

// 结构方向 φ 对应的期望 new_yaw：new_yaw ≡ φ (mod π/2) 且 |new_yaw| ≤ π/4
double FoldYaw(double phi) {
  double y = std::fmod(phi, M_PI_2);
  if (y > M_PI_4) y -= M_PI_2;
  if (y < -M_PI_4) y += M_PI_2;
  return y;
}

// 一族墙：'count' 条等长 'length'、方向 'angle' 的线段
void AddWallFamily(std::vector<sensor::LineData>* lines, int count,
                   double length, double angle) {
  for (int i = 0; i < count; ++i) {
    lines->push_back(
        MakeLine(Eigen::Vector2d(i * 1.5, lines->size() * 0.7), length, angle));
  }
}

// 正交两族墙，无噪声：任意旋转角下都应精确还原（±0.05° 裕量）
TEST(EstimateAlignmentYaw, ExactOrthogonalWallsAtVariousRotations) {
  for (const double phi_deg : {7.0, 23.0, 45.0, 68.0, 89.0}) {
    const double phi = phi_deg * kDegToRad;
    std::vector<sensor::LineData> lines;
    AddWallFamily(&lines, 3, 5.0, phi);             // 主方向族
    AddWallFamily(&lines, 2, 4.0, phi + M_PI_2);    // 次方向族
    const auto yaw =
        EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
    ASSERT_TRUE(yaw.has_value()) << "phi = " << phi_deg;
    EXPECT_NEAR(yaw.value(), FoldYaw(phi), 0.05 * kDegToRad)
        << "phi = " << phi_deg;
  }
}

// 方向带高斯噪声（σ = 0.5°）的正交墙：估计误差应远小于噪声尺度
TEST(EstimateAlignmentYaw, NoisyOrthogonalWalls) {
  const double phi = 17.0 * kDegToRad;
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.5 * kDegToRad);
  std::vector<sensor::LineData> lines;
  for (int i = 0; i < 8; ++i) {
    lines.push_back(MakeLine(Eigen::Vector2d(i * 1.2, 0.3 * i), 2.0 + 0.4 * i,
                             phi + noise(rng)));
  }
  for (int i = 0; i < 8; ++i) {
    lines.push_back(MakeLine(Eigen::Vector2d(0.5 * i, i * 1.1), 2.0 + 0.3 * i,
                             phi + M_PI_2 + noise(rng)));
  }
  const auto yaw =
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(yaw.value(), FoldYaw(phi), 0.5 * kDegToRad);
}

// 两族墙不严格正交（δ = 4°）且支撑相等：结果应为折中 δ/2，两族各留 δ/2
TEST(EstimateAlignmentYaw, ShearedFamiliesFuseToCompromise) {
  const double delta = 4.0 * kDegToRad;
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 4.0, 0.0);                  // w1 = 12
  AddWallFamily(&lines, 2, 6.0, M_PI_2 + delta);       // w2 = 12
  const auto yaw =
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(yaw.value(), delta * 0.5, 0.3 * kDegToRad);
  // 两族各自的残余角都约为 δ/2（而不是次方向独扛全部 δ）
  const double primary_residual = std::abs(yaw.value());
  const double secondary_residual =
      std::abs(yaw.value() + M_PI_2 - (M_PI_2 + delta));
  EXPECT_NEAR(primary_residual, delta * 0.5, 0.3 * kDegToRad);
  EXPECT_NEAR(secondary_residual, delta * 0.5, 0.3 * kDegToRad);
}

// 支撑不等时按支撑加权：次方向弱则折中点偏向主方向
TEST(EstimateAlignmentYaw, ShearWithUnequalSupport) {
  const double delta = 4.0 * kDegToRad;
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 4.0, 0.0);                  // w1 = 12
  AddWallFamily(&lines, 2, 3.0, M_PI_2 + delta);       // w2 = 6
  const auto yaw =
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
  ASSERT_TRUE(yaw.has_value());
  // 期望 r = 0 + δ·w2/(w1+w2) = 4°·6/18 ≈ 1.33°
  EXPECT_NEAR(yaw.value(), delta * 6.0 / 18.0, 0.3 * kDegToRad);
}

// 主峰一侧存在不对称尾部（+6° 斜线，位于旧 ±10° 精化窗内、±2° 窗外）：
// 渐缩窗口应剔除尾部，结果仍贴近真实墙向
TEST(EstimateAlignmentYaw, AsymmetricTailRejected) {
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 6.0, 0.0);          // 主方向族
  AddWallFamily(&lines, 2, 5.0, M_PI_2);       // 次方向族
  AddWallFamily(&lines, 3, 2.0, 6.0 * kDegToRad);  // 不对称尾部
  const auto yaw =
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
  ASSERT_TRUE(yaw.has_value());
  EXPECT_LT(std::abs(yaw.value()), 0.5 * kDegToRad);
}

// 主方向接近竖直（89.3°）：折叠后 new_yaw 应落在 (−π/4, π/4] 内
TEST(EstimateAlignmentYaw, NearVerticalPrimaryFoldsCorrectly) {
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 4.0, 89.3 * kDegToRad);
  AddWallFamily(&lines, 2, 4.0, 179.3 * kDegToRad);
  const auto yaw =
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{});
  ASSERT_TRUE(yaw.has_value());
  EXPECT_NEAR(yaw.value(), -0.7 * kDegToRad, 0.05 * kDegToRad);
}

// 单一方向（无正交结构）：应放弃
TEST(EstimateAlignmentYaw, SingleDirectionReturnsNullopt) {
  std::vector<sensor::LineData> lines;
  for (int i = 0; i < 5; ++i) {
    lines.push_back(MakeLine(Eigen::Vector2d(i * 2.0, 0.0), 4.0,
                             (29.0 + i * 0.5) * kDegToRad));
  }
  EXPECT_FALSE(
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{})
          .has_value());
}

// 次方向支撑过弱（< min_secondary_support_ratio）：应放弃
TEST(EstimateAlignmentYaw, WeakSecondaryReturnsNullopt) {
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 5.0, 0.0);   // 直方图支撑 ≈ 15
  lines.push_back(MakeLine(Eigen::Vector2d(9.0, 9.0), 1.0, M_PI_2));  // ≈ 1
  // 支撑比 ≈ 1/15 < 0.1 → nullopt
  EXPECT_FALSE(
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{})
          .has_value());
}

// 次方向与主方向明显不正交：应放弃
TEST(EstimateAlignmentYaw, NonOrthogonalSecondaryReturnsNullopt) {
  std::vector<sensor::LineData> lines;
  AddWallFamily(&lines, 3, 5.0, 0.0);
  AddWallFamily(&lines, 2, 4.0, 45.0 * kDegToRad);
  EXPECT_FALSE(
      EstimateAlignmentYaw(MakeLineList(lines), StructureAlignmentOptions{})
          .has_value());
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
