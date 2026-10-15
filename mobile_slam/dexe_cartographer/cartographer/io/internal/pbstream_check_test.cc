/*
 * Copyright 2018 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/internal/pbstream_check.h"

#include <cmath>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace cartographer {
namespace io {
namespace pbstream_check_internal {
namespace {

// 在 x 轴上造一条长度为 length 的线段。
void AddLine(double length, int inliers, mapping::proto::LineMap2DList* list) {
  auto* map_line = list->add_line_map_data();
  map_line->set_local_id(list->line_map_data_size());
  map_line->set_global_id(list->line_map_data_size());
  map_line->set_inliers(inliers);
  auto* line = map_line->mutable_line_data();
  line->set_begin_x(0.0);
  line->set_begin_y(0.0);
  line->set_end_x(length);
  line->set_end_y(0.0);
}

// 以 (mid_x, mid_y) 为中心造一条给定长度/方向的线段。
void AddLineAt(double length, int inliers, double angle_rad, double mid_x,
               double mid_y, mapping::proto::LineMap2DList* list) {
  const double dx = 0.5 * length * std::cos(angle_rad);
  const double dy = 0.5 * length * std::sin(angle_rad);
  auto* map_line = list->add_line_map_data();
  map_line->set_local_id(list->line_map_data_size());
  map_line->set_global_id(list->line_map_data_size());
  map_line->set_inliers(inliers);
  auto* line = map_line->mutable_line_data();
  line->set_begin_x(mid_x - dx);
  line->set_begin_y(mid_y - dy);
  line->set_end_x(mid_x + dx);
  line->set_end_y(mid_y + dy);
}

// 正交两族墙，整体朝向 phi（主族 3 条、次族 2 条，参数同
// structure_alignment_test 的精确用例）。
void AddOrthogonalWalls(double phi_rad, mapping::proto::LineMap2DList* list) {
  for (int i = 0; i < 3; ++i) {
    AddLineAt(5.0 - i, 5, phi_rad, i * 1.5,
              list->line_map_data_size() * 0.7, list);
  }
  for (int i = 0; i < 2; ++i) {
    AddLineAt(4.0 - i, 5, phi_rad + M_PI / 2,
              list->line_map_data_size() * 0.7, i * 1.5, list);
  }
}

constexpr CullingCriteria kMappingCriteria{0.3, 7.0, 2};

TEST(SurvivesCullingTest, LengthBoundsAreInclusive) {
  EXPECT_TRUE(SurvivesCulling(0.3, 5, kMappingCriteria));
  EXPECT_TRUE(SurvivesCulling(7.0, 5, kMappingCriteria));
  EXPECT_FALSE(SurvivesCulling(0.29, 5, kMappingCriteria));
  EXPECT_FALSE(SurvivesCulling(7.01, 5, kMappingCriteria));
}

TEST(SurvivesCullingTest, InlierFloorIsIndependentOfConfiguration) {
  // inliers < 3 是 LineDataCulling() 里的硬编码条件，不随 lua 变化。
  EXPECT_FALSE(SurvivesCulling(1.0, kMinLineInliers - 1, kMappingCriteria));
  EXPECT_TRUE(SurvivesCulling(1.0, kMinLineInliers, kMappingCriteria));
}

// 核心回归：建图侧 min_line_length=0.3 存下来的线段，会被定位侧 dynamic submap
// 的类内默认 0.4 额外剔除 —— 两个存活数的差值就是该缺陷的暴露量。
TEST(CollectLineStatsTest, DynamicDefaultsCullMoreThanPbstreamCriteria) {
  mapping::proto::LineMap2DList list;
  AddLine(0.35, 5, &list);  // 只在 0.3 判据下存活
  AddLine(0.36, 5, &list);  // 只在 0.3 判据下存活
  AddLine(1.20, 5, &list);  // 两套判据都存活
  AddLine(6.90, 5, &list);  // 两套判据都存活

  LineStats stats;
  CollectLineStats(list, kMappingCriteria, &stats);

  EXPECT_EQ(stats.count, 4);
  EXPECT_EQ(stats.survive_pbstream, 4);
  EXPECT_EQ(stats.survive_defaults, 2);
  EXPECT_THAT(stats.lengths, ::testing::SizeIs(4));
  EXPECT_THAT(stats.inliers, ::testing::Each(5));
}

TEST(CollectLineStatsTest, TooFewInliersCulledUnderBothCriteria) {
  mapping::proto::LineMap2DList list;
  AddLine(1.0, kMinLineInliers - 1, &list);
  AddLine(1.0, kMinLineInliers, &list);

  LineStats stats;
  CollectLineStats(list, kMappingCriteria, &stats);

  EXPECT_EQ(stats.count, 2);
  EXPECT_EQ(stats.survive_pbstream, 1);
  EXPECT_EQ(stats.survive_defaults, 1);
}

TEST(CollectLineStatsTest, EmptyListYieldsZeroCounts) {
  mapping::proto::LineMap2DList list;
  LineStats stats;
  CollectLineStats(list, kMappingCriteria, &stats);

  EXPECT_EQ(stats.count, 0);
  EXPECT_EQ(stats.survive_pbstream, 0);
  EXPECT_EQ(stats.survive_defaults, 0);
}

TEST(ResolveCriteriaTest, PrefersOptionsEmbeddedInPbstream) {
  ::google::protobuf::RepeatedPtrField<
      mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      all_options;
  auto* submaps_options = all_options.Add()
                              ->mutable_trajectory_builder_options()
                              ->mutable_trajectory_builder_2d_options()
                              ->mutable_submaps_options();
  submaps_options->set_min_line_length(0.3);
  submaps_options->set_max_line_length(7.0);
  submaps_options->set_line_culling_obs(2);

  bool from_pbstream = false;
  const CullingCriteria criteria = ResolveCriteria(all_options, &from_pbstream);

  EXPECT_TRUE(from_pbstream);
  EXPECT_DOUBLE_EQ(criteria.min_length, 0.3);
  EXPECT_DOUBLE_EQ(criteria.max_length, 7.0);
  EXPECT_EQ(criteria.culling_obs, 2);
}

TEST(ResolveCriteriaTest, FallsBackToDefaultsWhenOptionsAbsent) {
  ::google::protobuf::RepeatedPtrField<
      mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      all_options;

  bool from_pbstream = true;
  const CullingCriteria criteria = ResolveCriteria(all_options, &from_pbstream);

  EXPECT_FALSE(from_pbstream);
  EXPECT_DOUBLE_EQ(criteria.min_length, kDefaultMinLineLength);
  EXPECT_DOUBLE_EQ(criteria.max_length, kDefaultMaxLineLength);
  EXPECT_EQ(criteria.culling_obs, kDefaultLineCullingObs);
}

// 老地图的建图配置里没有线特征参数（proto3 默认 0），此时用 0 会让所有线段
// "存活"，掩盖真实加载行为，因此必须回退默认值。
TEST(ResolveCriteriaTest, FallsBackWhenMaxLineLengthIsUnset) {
  ::google::protobuf::RepeatedPtrField<
      mapping::proto::TrajectoryBuilderOptionsWithSensorIds>
      all_options;
  all_options.Add()
      ->mutable_trajectory_builder_options()
      ->mutable_trajectory_builder_2d_options()
      ->mutable_submaps_options();

  bool from_pbstream = true;
  const CullingCriteria criteria = ResolveCriteria(all_options, &from_pbstream);

  EXPECT_FALSE(from_pbstream);
  EXPECT_DOUBLE_EQ(criteria.max_length, kDefaultMaxLineLength);
}

// 轴向正交墙：残余旋转角应近 0（已对齐）。
TEST(EvaluateStructureAlignmentTest, AxisAlignedWallsEvaluateToNearZeroResidual) {
  mapping::proto::LineMap2DList list;
  AddOrthogonalWalls(0.0, &list);

  const AlignmentStats stats = EvaluateStructureAlignment(list);

  EXPECT_TRUE(stats.evaluated);
  EXPECT_LT(stats.residual_yaw_deg, 0.05);
}

// 整体旋转 phi 的墙：残余角应还原出折叠后的 phi（>1° 即未对齐）。
TEST(EvaluateStructureAlignmentTest, RotatedWallsReportFoldedResidual) {
  for (const double phi_deg : {30.0, 44.0}) {
    mapping::proto::LineMap2DList list;
    AddOrthogonalWalls(phi_deg * M_PI / 180.0, &list);

    const AlignmentStats stats = EvaluateStructureAlignment(list);

    ASSERT_TRUE(stats.evaluated) << "phi = " << phi_deg;
    EXPECT_NEAR(stats.residual_yaw_deg, phi_deg, 0.05) << "phi = " << phi_deg;
  }
}

// 单一方向（无正交结构）：不可判定。
TEST(EvaluateStructureAlignmentTest, SingleDirectionNotEvaluable) {
  mapping::proto::LineMap2DList list;
  for (int i = 0; i < 5; ++i) {
    AddLineAt(4.0, 5, 0.0, i * 2.0, 0.0, &list);
  }
  EXPECT_FALSE(EvaluateStructureAlignment(list).evaluated);
}

// 空线地图：不可判定。
TEST(EvaluateStructureAlignmentTest, EmptyListNotEvaluable) {
  mapping::proto::LineMap2DList list;
  EXPECT_FALSE(EvaluateStructureAlignment(list).evaluated);
}

}  // namespace
}  // namespace pbstream_check_internal
}  // namespace io
}  // namespace cartographer
