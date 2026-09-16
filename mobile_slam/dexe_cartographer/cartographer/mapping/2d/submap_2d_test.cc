/*
 * Copyright 2016 The Cartographer Authors
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

#include "cartographer/mapping/2d/submap_2d.h"

#include <map>
#include <memory>
#include <set>
#include <string>

#include "cartographer/common/internal/testing/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/transform/transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

TEST(Submap2DTest, TheRightNumberOfRangeDataAreInserted) {
  constexpr int kNumRangeData = 10;
  auto parameter_dictionary = common::MakeDictionary(
      "return {"
      "num_range_data = " +
      std::to_string(kNumRangeData) +
      ", "
      // 本 fork 给 CreateSubmapsOptions2D 增加了 5 个必填键，上游用例的内联
      // 字典一直没同步，导致该用例在读 cloud_culling_prob 时 CHECK 失败。
      // cloud_culling_prob 取 0.1（CloudDataCulling 的关闭值），使本用例仍然
      // 只度量它原本要度量的「插入了多少帧 range data」。
      "cloud_culling_prob = 0.1, "
      "min_line_length = 0.3, "
      "max_line_length = 7.0, "
      "line_culling_obs = 2, "
      "culling_interval = 5, "
      "grid_options_2d = {"
      "grid_type = \"PROBABILITY_GRID\","
      "resolution = 0.05, "
      "},"
      "range_data_inserter = {"
      "range_data_inserter_type = \"PROBABILITY_GRID_INSERTER_2D\","
      "probability_grid_range_data_inserter = {"
      "insert_free_space = true, "
      "hit_probability = 0.53, "
      "miss_probability = 0.495, "
      "},"
      "tsdf_range_data_inserter = { "
      "truncation_distance = 2.0,"
      "maximum_weight = 10.,"
      "update_free_space = false,"
      "normal_estimation_options = {"
      "num_normal_samples = 2,"
      "sample_radius = 10.,"
      "},"
      "project_sdf_distance_to_scan_normal = false,"
      "update_weight_range_exponent = 0,"
      "update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0,"
      "update_weight_distance_cell_to_hit_kernel_bandwidth = 0,"
      "},"
      "},"
      "}");
  ActiveSubmaps2D submaps{CreateSubmapsOptions2D(parameter_dictionary.get())};
  std::set<std::shared_ptr<const Submap2D>> all_submaps;
  for (int i = 0; i != 1000; ++i) {
    // 获取插入的submaps
    auto insertion_submaps =
        submaps.InsertData({{Eigen::Vector3f::Zero(), {}, {}}}, 
                          {},  // empty cloud_data
                          {},  // empty highlight_cloud_data
                          {},  // empty line_data
                          {}); // empty reflector_data
    // Except for the first, maps should only be returned after enough range
    // data.
    for (const auto& submap : insertion_submaps) {
      all_submaps.insert(submap);
    }
    if (submaps.submaps().size() > 1) {
      EXPECT_LE(kNumRangeData, submaps.submaps().front()->num_range_data());
    }
  }
  EXPECT_EQ(2, submaps.submaps().size());
  int correct_num_finished_submaps = 0;
  int num_unfinished_submaps = 0;
  for (const auto& submap : all_submaps) {
    if (submap->num_range_data() == kNumRangeData * 2) {
      ++correct_num_finished_submaps;
    } else {
      EXPECT_EQ(kNumRangeData, submap->num_range_data());
      ++num_unfinished_submaps;
    }
  }
  // Submaps should not be left without the right number of range data in them.
  EXPECT_EQ(correct_num_finished_submaps, all_submaps.size() - 1);
  EXPECT_EQ(1, num_unfinished_submaps);
}

TEST(Submap2DTest, ToFromProto) {
  MapLimits expected_map_limits(1., Eigen::Vector2d(2., 3.),
                                CellLimits(100, 110));
  ValueConversionTables conversion_tables;
  Submap2D expected(Eigen::Vector2f(4.f, 5.f),
                    absl::make_unique<ProbabilityGrid>(expected_map_limits,
                                                       &conversion_tables),
                    &conversion_tables,
                    absl::make_unique<sensor::PointCloud>(),
                    absl::make_unique<sensor::PointCloud>(),
                    0, std::map<int, cartographer::mapping::MapLine2D>());
  const proto::Submap proto =
      expected.ToProto(true /* include_probability_grid_data */);
  EXPECT_TRUE(proto.has_submap_2d());
  EXPECT_FALSE(proto.has_submap_3d());
  const auto actual = Submap2D(proto.submap_2d(), &conversion_tables);
  EXPECT_TRUE(expected.local_pose().translation().isApprox(
      actual.local_pose().translation(), 1e-6));
  EXPECT_TRUE(expected.local_pose().rotation().isApprox(
      actual.local_pose().rotation(), 1e-6));
  EXPECT_EQ(expected.num_range_data(), actual.num_range_data());
  EXPECT_EQ(expected.insertion_finished(), actual.insertion_finished());
  EXPECT_NEAR(expected.grid()->limits().resolution(),
              actual.grid()->limits().resolution(), 1e-6);
  EXPECT_TRUE(expected.grid()->limits().max().isApprox(
      actual.grid()->limits().max(), 1e-6));
  EXPECT_EQ(expected.grid()->limits().cell_limits().num_x_cells,
            actual.grid()->limits().cell_limits().num_x_cells);
}

// B20260817：LocalMapUpdater2D 从 static submap 深拷贝出 dynamic submap 后，
// 从不调用 SetLineCullingParam，于是这棵 submap 全程使用 LineMap2DList 的类内
// 默认 min 0.4，而建图 lua 配的是 0.3 —— 长度落在 [0.3, 0.4) 的线段会在定位期
// 首次 culling 被静默剔除，并在 dynamic_map_policy = PERSISTENT 下随下一次
// save_map 写回磁盘，即每存一次盘就永久少一批短线段。
// 真实地图 20260711_xmk 实测：108 条线里 2 条命中该区间。
TEST(Submap2DTest, InheritedLineCullingParamKeepsShortLineThatDefaultsWouldCull) {
  constexpr double kShortLineLength = 0.36;  // 落在 [建图 0.3, 类内默认 0.4)
  ValueConversionTables conversion_tables;

  const auto make_submap_with_short_line = [&conversion_tables,
                                            kShortLineLength] {
    const sensor::LineData short_line(Eigen::Vector2d(0.0, 0.0),
                                      Eigen::Vector2d(kShortLineLength, 0.0));
    MapLine2D line(/*local_id=*/0, short_line);
    line.SetInliers(20);  // 远高于 inliers < 3 门槛，确保只有长度判据起作用
    std::map<int, MapLine2D> active_line_data;
    active_line_data.insert({0, line});
    return absl::make_unique<Submap2D>(
        Eigen::Vector2f(0.f, 0.f),
        absl::make_unique<ProbabilityGrid>(
            MapLimits(1., Eigen::Vector2d(2., 3.), CellLimits(100, 110)),
            &conversion_tables),
        &conversion_tables, absl::make_unique<sensor::PointCloud>(),
        absl::make_unique<sensor::PointCloud>(), /*latest_line_id=*/1,
        std::move(active_line_data));
  };

  // 对照组：不继承判据 → 走类内默认 0.4 → Finish() 触发 culling 后线段消失。
  // 这正是缺陷现场。
  auto without_inheritance = make_submap_with_short_line();
  ASSERT_EQ(without_inheritance->line_map_2d_list()->size(), 1u);
  without_inheritance->Finish();
  ASSERT_EQ(without_inheritance->line_map_2d_list()->size(), 0u)
      << "前提失效：该长度在类内默认判据下本应被剔除";

  // 源 submap 持有建图期判据（线上实际来自 pbstream 内嵌 submaps_options）。
  auto source = make_submap_with_short_line();
  source->SetLineCullingParam(/*min_line_length=*/0.3, /*max_line_length=*/7.0,
                              /*culling_obs=*/2);

  auto target = make_submap_with_short_line();
  target->InheritLineCullingParamFrom(*source);
  target->Finish();

  EXPECT_EQ(target->line_map_2d_list()->size(), 1u)
      << "继承源 submap 判据后，[0.3, 0.4) 区间的线段不应在定位期被剔除";
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
