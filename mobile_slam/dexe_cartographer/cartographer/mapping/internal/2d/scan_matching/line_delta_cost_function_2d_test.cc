// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer/mapping/internal/2d/scan_matching/line_delta_cost_function_2d.h"

#include <array>
#include <cmath>
#include <memory>

#include "gtest/gtest.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

std::array<double, 3> EvaluateImprovedCost(
  const sensor::LineData & data, const sensor::LineData & target,
  const std::array<double, 3> & pose,
  const double distance_huber_scale = 0.,
  const double angle_huber_scale = 0.,
  const double line_length_reference = 1.,
  const double overlap_gap_weight = 0.5)
{
  std::unique_ptr<ceres::CostFunction> cost(
    CreateImprovedLineDeltaCostFunction2D(
      1., 1., distance_huber_scale, angle_huber_scale,
      line_length_reference, overlap_gap_weight, data, target));
  const double * parameters[] = {pose.data()};
  std::array<double, 3> residuals{};
  EXPECT_TRUE(cost->Evaluate(parameters, residuals.data(), nullptr));
  return residuals;
}

TEST(ImprovedLineDeltaCostFunction2DTest, DoesNotPenalizePartialOverlap)
{
  const sensor::LineData data({0.5, 0.}, {1.5, 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0., 0.}});

  EXPECT_NEAR(residuals[0], 0., 1e-12);
  EXPECT_NEAR(residuals[1], 0., 1e-12);
  EXPECT_NEAR(residuals[2], 0., 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, PenalizesOnlyDisjointGap)
{
  const sensor::LineData data({1.2, 0.}, {2.2, 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0., 0.}});

  EXPECT_NEAR(residuals[0], 0., 1e-12);
  EXPECT_NEAR(residuals[1], 0.1, 1e-12);
  EXPECT_NEAR(residuals[2], 0., 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, UsesSignedNormalDistance)
{
  const sensor::LineData data({0., 0.}, {1., 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0.2, 0.}});

  EXPECT_NEAR(residuals[0], 0.2, 1e-12);
  EXPECT_NEAR(residuals[1], 0., 1e-12);
  EXPECT_NEAR(residuals[2], 0., 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, TreatsLineDirectionAsUndirected)
{
  const sensor::LineData data({0., 0.}, {1., 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0., M_PI}});

  EXPECT_NEAR(residuals[2], 0., 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, AppliesHuberBeforeExternalWeight)
{
  const sensor::LineData data({0., 0.}, {1., 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals =
    EvaluateImprovedCost(
      data, target, std::array<double, 3>{{0., 0.2, 0.}}, 0.1);

  EXPECT_NEAR(residuals[0], std::sqrt(0.03), 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, BoundsLengthConfidenceUsingShorterLine)
{
  const sensor::LineData data({0., 0.}, {0.25, 0.});
  const sensor::LineData target({0., 1.}, {4., 1.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0., 0.}});

  EXPECT_NEAR(residuals[0], -0.5, 1e-12);
}

TEST(ImprovedLineDeltaCostFunction2DTest, IgnoresDegenerateSegments)
{
  const sensor::LineData data({0., 0.}, {0., 0.});
  const sensor::LineData target({0., 0.}, {1., 0.});
  const auto residuals = EvaluateImprovedCost(
    data, target, std::array<double, 3>{{0., 0., 0.}});

  EXPECT_NEAR(residuals[0], 0., 1e-12);
  EXPECT_NEAR(residuals[1], 0., 1e-12);
  EXPECT_NEAR(residuals[2], 0., 1e-12);
}

}  // namespace
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
