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

#include "cartographer/mapping/2d/line_map_2d.h"

#include <cstring>
#include <map>
#include <new>
#include <vector>

#include "gmock/gmock.h"

namespace cartographer {
namespace mapping {
namespace {

// 构造一条长度合格（1.0m）、内点充足的线段，用于 culling 测试。
MapLine2D MakeQualifiedLine(int local_id) {
  const sensor::LineData line_data(Eigen::Vector2d(0.0, 0.0),
                                   Eigen::Vector2d(1.0, 0.0));
  MapLine2D line(local_id, line_data);
  line.SetInliers(20);  // 远高于 culling 的 inliers < 3 门槛
  return line;
}

// 模拟「从 pbstream 反序列化 / static→dynamic 深拷贝」的入口：
// 只带线段数据，不带 observations_（该字段既不持久化也不随深拷贝传递）。
LineMap2DList MakePersistedList(int line_count) {
  std::map<int, MapLine2D> active_line_data;
  for (int i = 0; i < line_count; ++i) {
    active_line_data.insert({i, MakeQualifiedLine(i)});
  }
  return LineMap2DList(line_count, std::move(active_line_data));
}

// 加载来的线段没有观测记录，这是该构造路径的固有事实，不是缺陷。
TEST(LineMap2DListTest, PersistedLinesHaveNoObservationRecords) {
  const LineMap2DList line_map = MakePersistedList(5);
  EXPECT_EQ(line_map.size(), 5u);
  EXPECT_TRUE(line_map.GetObservations().empty());
}

// 回归核心：加载来的线段观测计数为空，但它们已在建图期通过 culling 门槛，
// 首次 LineDataCulling() 不得以「观测次数不足」为由把它们删掉。
// 一旦删空，Submap2D::ToProto() 会因 line_map_2d_list_ 为空而跳过整段，
// 保存出的 pbstream 将永久丢失线特征。
TEST(LineMap2DListTest, CullingKeepsPersistedLinesWithoutObservationRecords) {
  LineMap2DList line_map = MakePersistedList(5);
  line_map.SetCullingParam(/*min_line_length=*/0.4, /*max_line_length=*/7.0,
                           /*culling_obs=*/2);

  line_map.LineDataCulling();

  EXPECT_EQ(line_map.size(), 5u) << "加载来的线特征在首次 culling 后被删空";
}

// 未调用 SetCullingParam 时 culling_obs_ 取类内默认值 3，
// 深拷贝生成的 dynamic submap 正是这条路径（不设参数直接 Finish）。
TEST(LineMap2DListTest, CullingKeepsPersistedLinesUnderDefaultThreshold) {
  LineMap2DList line_map = MakePersistedList(3);

  line_map.LineDataCulling();

  EXPECT_EQ(line_map.size(), 3u);
}

// 反向保证：本次运行新增的线段有观测记录，观测不足时仍须被正常剔除，
// 修复不得把 culling 整体失效。
TEST(LineMap2DListTest, CullingStillRemovesFreshLinesWithTooFewObservations) {
  LineMap2DList line_map;
  line_map.SetCullingParam(0.4, 7.0, /*culling_obs=*/3);

  // PushBack 会为新线段建立 observations_ 记录（初值 1）。
  const sensor::LineData line_data(Eigen::Vector2d(0.0, 0.0),
                                   Eigen::Vector2d(1.0, 0.0));
  line_map.PushBack(std::vector<sensor::LineData>{line_data});
  ASSERT_EQ(line_map.size(), 1u);
  ASSERT_FALSE(line_map.GetObservations().empty());

  line_map.LineDataCulling();

  EXPECT_EQ(line_map.size(), 0u) << "观测次数不足的新线段应被剔除";
}

// 加载来的线段仍须接受长度与内点检查：
// 「不按观测次数判」只豁免 observations_ 这一项，其余判据照旧。
TEST(LineMap2DListTest, CullingStillRemovesPersistedLinesFailingOtherCriteria) {
  std::map<int, MapLine2D> active_line_data;

  // 0: 合格
  active_line_data.insert({0, MakeQualifiedLine(0)});

  // 1: 过短（< min_line_length 0.4）
  {
    const sensor::LineData short_line(Eigen::Vector2d(0.0, 0.0),
                                      Eigen::Vector2d(0.1, 0.0));
    MapLine2D line(1, short_line);
    line.SetInliers(20);
    active_line_data.insert({1, line});
  }

  // 2: 内点过少（< 3）
  {
    const sensor::LineData line_data(Eigen::Vector2d(0.0, 1.0),
                                     Eigen::Vector2d(1.0, 1.0));
    MapLine2D line(2, line_data);
    line.SetInliers(1);
    active_line_data.insert({2, line});
  }

  LineMap2DList line_map(3, std::move(active_line_data));
  line_map.SetCullingParam(0.4, 7.0, /*culling_obs=*/2);
  ASSERT_EQ(line_map.size(), 3u);

  line_map.LineDataCulling();

  EXPECT_EQ(line_map.size(), 1u) << "长度/内点不合格的加载线段仍应被剔除";
  EXPECT_EQ(line_map.GetLineMapData2DList().count(0), 1u);
}

// culling 判据需要能被读出来，深拷贝的 submap 才能从源 submap 继承它们。
TEST(LineMap2DListTest, CullingParamGettersExposeEffectiveCriteria) {
  LineMap2DList line_map;

  EXPECT_DOUBLE_EQ(line_map.min_line_length(), 0.4);
  EXPECT_DOUBLE_EQ(line_map.max_line_length(), 7.0);
  EXPECT_EQ(line_map.culling_obs(), 3);

  line_map.SetCullingParam(0.3, 6.0, 2);

  EXPECT_DOUBLE_EQ(line_map.min_line_length(), 0.3);
  EXPECT_DOUBLE_EQ(line_map.max_line_length(), 6.0);
  EXPECT_EQ(line_map.culling_obs(), 2);
}

// 在一块被 0xAB 填满的存储上 placement-new 构造，使「未初始化」与「初始化为 0」
// 可区分 —— 否则栈上恰好残留 0 时，测试会假过。
template <typename Ctor>
int InliersOfFreshlyConstructedLine(const Ctor& construct) {
  alignas(MapLine2D) unsigned char storage[sizeof(MapLine2D)];
  std::memset(storage, 0xAB, sizeof(storage));
  MapLine2D* line = construct(storage);
  const int inliers = line->inliers();
  line->~MapLine2D();
  return inliers;
}

// LineDataCulling() 以 `inliers < 3` 作为剔除判据之一，而 MapLine2D 的两个构造
// 函数都没给 inliers_ 赋初值。读到的残留值既可能让未拟合的线段逃过剔除，
// 也可能把合格线段误删，且随构建/调用栈变化，不可复现。
TEST(MapLine2DTest, FreshLineStartsWithZeroInliers) {
  const sensor::LineData line_data(Eigen::Vector2d(0.0, 0.0),
                                   Eigen::Vector2d(1.0, 0.0));

  const int inliers = InliersOfFreshlyConstructedLine(
      [&line_data](void* storage) {
        return new (storage) MapLine2D(/*local_id=*/1, line_data);
      });

  EXPECT_EQ(inliers, 0) << "inliers_ 未初始化，读到的是构造前的残留值";
}

// 另一个构造函数（static_line / updata 路径）同样未初始化 inliers_。
TEST(MapLine2DTest, PersistedLineStartsWithZeroInliers) {
  const sensor::LineData line_data(Eigen::Vector2d(0.0, 0.0),
                                   Eigen::Vector2d(1.0, 0.0));

  const int inliers = InliersOfFreshlyConstructedLine(
      [&line_data](void* storage) {
        return new (storage) MapLine2D(MapLine2D::static_line,
                                       /*global_id=*/7, line_data);
      });

  EXPECT_EQ(inliers, 0) << "inliers_ 未初始化，读到的是构造前的残留值";
}

}  // namespace
}  // namespace mapping
}  // namespace cartographer
