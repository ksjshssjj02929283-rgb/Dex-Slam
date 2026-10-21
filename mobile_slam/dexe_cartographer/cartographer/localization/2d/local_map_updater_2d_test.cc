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

#include "cartographer/localization/2d/local_map_updater_2d.h"

#include <memory>
#include <vector>

#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/xy_index.h"
#include "cartographer/transform/rigid_transform.h"
#include "gmock/gmock.h"

namespace cartographer {
namespace localization {
namespace {

using ::testing::AllOf;
using ::testing::DoubleEq;
using ::testing::Ge;
using ::testing::Le;

// 复现 align 工具产出的地图形态。
//
// map_transform_common.cc 用「节点包围盒 + lidar_range_pad(30m)」的右上角同时
// 作为 Submap2D 的 local_pose 与 MapLimits.max，而 local_pose 恰恰落在数据范围
// 外扩 30m 处。定位侧曾把它当作 ±15m 局部窗口的中心，导致 9640 个点被 100%
// 裁空，调试写盘抛 pcl::io::Exception，启动失败后 100ms 定时器解引用空指针 SIGSEGV。
//
// 这里把数据集中放在原点附近、栅格按 align 的方式外扩 30m，复现同一形态。

constexpr double kResolution = 0.05;
constexpr double kLidarRangePad = 30.0;  // align 侧硬编码值
constexpr int kNumCells = 1200;          // 60m / 0.05m

// MapLimits 只存 max_（右上角）与 cell_limits_，min 需自行反推。
Eigen::Vector2d LimitsMin(const mapping::MapLimits& limits) {
  return limits.max() -
         limits.resolution() *
             Eigen::Vector2d(limits.cell_limits().num_x_cells,
                             limits.cell_limits().num_y_cells);
}

// 构造 align 形态的 global_static_submap：栅格锚在 bbox 右上角，数据在原点附近。
std::shared_ptr<mapping::Submap2D> MakeAlignedSubmap(
    mapping::ValueConversionTables* conversion_tables) {
  // 模拟节点包围盒 = 原点附近一个小方框。
  const Eigen::Vector2d data_min(-1.0, -1.0);
  const Eigen::Vector2d data_max(1.0, 1.0);
  const Eigen::Vector2d pad = Eigen::Vector2d::Constant(kLidarRangePad);
  const Eigen::Vector2d bbox_min = data_min - pad;
  // align 侧还经 SnapMaxToFirstNodeLattice 把 max 锚到首节点格网上；
  // 首节点在原点，故此处直接取 bbox.max() + pad 即可保持同一形态。
  const Eigen::Vector2d bbox_max = data_max + pad;

  auto grid = std::make_unique<mapping::ProbabilityGrid>(
      mapping::MapLimits(kResolution, bbox_max,
                         mapping::CellLimits(kNumCells, kNumCells)),
      conversion_tables);

  // 在原点附近撒点，模拟机器人实际扫到的点。
  // points() 只有 const 重载，故点云须在构造时一次性填好。
  std::vector<sensor::PointCloud::PointType> points;
  for (int i = 0; i < 200; ++i) {
    const double angle = 2.0 * M_PI * i / 200.0;
    const double radius = 0.5 + 0.5 * (i % 10) / 10.0;
    sensor::PointCloud::PointType point;
    point.position.x() = static_cast<float>(radius * std::cos(angle));
    point.position.y() = static_cast<float>(radius * std::sin(angle));
    point.position.z() = 0.0f;
    points.push_back(point);
  }
  auto cloud = std::make_unique<sensor::PointCloud>(std::move(points));

  // 真实地图里扫到的区域在栅格上是占据的。Submap2D::Finish() 会执行
  // CloudDataCulling()，把落在概率 < cloud_culling_prob_ (0.5) 单元里的点删掉，
  // 故必须把点所在单元标为占据，否则点云会在 Finish 阶段被裁光——那与本次要
  // 复现的「窗口与地图不重叠导致裁空」是完全不同的失效模式。
  for (const auto& point : cloud->points()) {
    const auto index = grid->limits().GetCellIndex(
        Eigen::Vector2f(point.position.x(), point.position.y()));
    if (grid->limits().Contains(index) && !grid->IsKnown(index)) {
      grid->SetProbability(index, mapping::kMaxProbability);
    }
  }

  // origin_2d 同时作 local_pose 与 MapLimits.max（Submap2D 构造契约）。
  const Eigen::Vector2f origin_2d(static_cast<float>(bbox_max.x()),
                                  static_cast<float>(bbox_max.y()));
  return std::make_shared<mapping::Submap2D>(
      origin_2d, std::move(grid), conversion_tables, std::move(cloud),
      std::make_unique<sensor::PointCloud>(), 0,
      std::map<int, mapping::MapLine2D>{});
}

mapping::proto::SubmapsOptions2D MakeSubmapOptions() {
  mapping::proto::SubmapsOptions2D options;
  auto* inserter = options.mutable_range_data_inserter_options();
  inserter->set_range_data_inserter_type(
      mapping::proto::RangeDataInserterOptions::
          PROBABILITY_GRID_INSERTER_2D);
  return options;
}

proto::LocalMapUpdaterOptions2D MakeUpdaterOptions() {
  proto::LocalMapUpdaterOptions2D options;
  options.set_min_x(-15.0f);
  options.set_max_x(15.0f);
  options.set_min_y(-15.0f);
  options.set_max_y(15.0f);
  options.set_min_z(-1.0f);
  options.set_max_z(1.0f);
  // 默认为 NONE(N=0)，该策略下 SetGlobalSubmap 会提前返回、不构建 local submap。
  options.set_dynamic_map_policy(proto::DynamicMapPolicy::LONG);
  options.set_save_debug_data(false);
  return options;
}

TEST(LocalMapUpdater2DTest, InitialWindowIsCenteredOnGridNotOnLocalPose) {
  mapping::ValueConversionTables conversion_tables;
  auto global_submap = MakeAlignedSubmap(&conversion_tables);
  ASSERT_NE(nullptr, global_submap->grid());

  LocalMapUpdater2D updater(MakeUpdaterOptions(), MakeSubmapOptions());
  updater.SetGlobalSubmap(global_submap);

  // 根因断言：窗口中心来自栅格几何中心，且落在栅格覆盖范围内。
  const auto origin = updater.origin_pose();
  ASSERT_NE(nullptr, origin);
  const auto& limits = global_submap->grid()->limits();
  const Eigen::Vector2d limits_min = LimitsMin(limits);
  EXPECT_THAT(origin->translation().x(), AllOf(Ge(limits_min.x()),
                                             Le(limits.max().x())));
  EXPECT_THAT(origin->translation().y(), AllOf(Ge(limits_min.y()),
                                             Le(limits.max().y())));

  // 数据没有被裁空——旧代码此处恒为 0 点。
  const auto local_submap = updater.local_submap();
  ASSERT_NE(nullptr, local_submap);
  ASSERT_NE(nullptr, local_submap->cloud());
  EXPECT_GT(local_submap->cloud()->points().size(), 0);
}

// 把「local_pose 不是地图中心」固化为可执行契约，防止有人再次把它当中心使用。
TEST(LocalMapUpdater2DTest, LocalPoseIsNotTheGridCenter) {
  mapping::ValueConversionTables conversion_tables;
  auto global_submap = MakeAlignedSubmap(&conversion_tables);

  const Eigen::Vector2d local_pose =
      global_submap->local_pose().translation().head<2>();
  const Eigen::Vector2d grid_center = global_submap->GridCenterXY();

  EXPECT_LT(grid_center.x(), local_pose.x() - kLidarRangePad / 2.0);
  EXPECT_LT(grid_center.y(), local_pose.y() - kLidarRangePad / 2.0);

  // GridCenterXY 必须严格落在栅格 limits 内部。
  const auto& limits = global_submap->grid()->limits();
  const Eigen::Vector2d limits_min = LimitsMin(limits);
  EXPECT_THAT(grid_center.x(), AllOf(Ge(limits_min.x()), Le(limits.max().x())));
  EXPECT_THAT(grid_center.y(), AllOf(Ge(limits_min.y()), Le(limits.max().y())));
  EXPECT_THAT(grid_center.x(), DoubleEq(0.5 * (limits_min.x() + limits.max().x())));
  EXPECT_THAT(grid_center.y(), DoubleEq(0.5 * (limits_min.y() + limits.max().y())));
}

// 源图 401×401 格，覆盖 [-10.05, 10] m：格数为奇数，栅格几何中心落在格心上；±15 m
// 窗口会伸出源图。按固定规律填概率并留出未知格。点云所在格先标为占据，否则
// Finish() 的 CloudDataCulling 会把点删光（SetProbability 只允许写未知格，故先写点）。
std::shared_ptr<mapping::Submap2D> MakePatternSubmap(
    mapping::ValueConversionTables* conversion_tables) {
  auto grid = std::make_unique<mapping::ProbabilityGrid>(
      mapping::MapLimits(kResolution, Eigen::Vector2d(10., 10.),
                         mapping::CellLimits(401, 401)),
      conversion_tables);
  std::vector<sensor::PointCloud::PointType> points;
  for (int i = 0; i < 100; ++i) {
    sensor::PointCloud::PointType point;
    point.position = Eigen::Vector3f(0.1f * (i % 10) - 0.5f,
                                     0.1f * (i / 10) - 0.5f, 0.f);
    points.push_back(point);
    const auto index = grid->limits().GetCellIndex(point.position.head<2>());
    if (!grid->IsKnown(index)) {
      grid->SetProbability(index, mapping::kMaxProbability);
    }
  }
  for (const Eigen::Array2i& index :
       mapping::XYIndexRangeIterator(grid->limits().cell_limits())) {
    const int key = index.x() * 7 + index.y();
    if (key % 13 == 0 || grid->IsKnown(index)) continue;
    grid->SetProbability(index, 0.2f + (key % 9) * 0.07f);
  }
  return std::make_shared<mapping::Submap2D>(
      Eigen::Vector2f(10.f, 10.f), std::move(grid), conversion_tables,
      std::make_unique<sensor::PointCloud>(std::move(points)),
      std::make_unique<sensor::PointCloud>(), 0,
      std::map<int, mapping::MapLine2D>{});
}

// 局部栅格必须与源图格线对齐：在不落在格线上的位姿重建时，局部图每个格心都与一个
// 源图格心重合、概率和未知状态一致，源图外的格子为 kMinProbability，且覆盖请求窗口。
// 旧实现以位姿为中心另建网格、按格心最近邻查源图，障碍物会平移，最多半格。
TEST(LocalMapUpdater2DTest, LocalGridKeepsSourceCellGeometry) {
  mapping::ValueConversionTables conversion_tables;
  LocalMapUpdater2D updater(MakeUpdaterOptions(), MakeSubmapOptions());
  updater.SetGlobalSubmap(MakePatternSubmap(&conversion_tables));
  const auto* source = dynamic_cast<const mapping::ProbabilityGrid*>(
      updater.GetGlobalSubmap()->grid());
  ASSERT_NE(nullptr, source);
  const auto& source_limits = source->limits();

  for (const Eigen::Vector2d& pose :
       {Eigen::Vector2d(0.537, -0.213), Eigen::Vector2d(-3.025, 4.011),
        Eigen::Vector2d(1., 1.)}) {
    updater.RebuildLocalSubmapAt(transform::Rigid2d(pose, 0.));
    const auto local_submap = updater.local_submap();
    ASSERT_NE(nullptr, local_submap);
    const auto* local =
        dynamic_cast<const mapping::ProbabilityGrid*>(local_submap->grid());
    ASSERT_NE(nullptr, local);
    const auto& limits = local->limits();
    EXPECT_DOUBLE_EQ(limits.resolution(), source_limits.resolution());

    // 边界向外取整，窗口不缩小。源图外的格子是已知格，Finish() 的裁剪不会削掉它们。
    const Eigen::Vector2d limits_min = LimitsMin(limits);
    EXPECT_GE(limits.max().x() + 1e-9, pose.x() + 15.);
    EXPECT_GE(limits.max().y() + 1e-9, pose.y() + 15.);
    EXPECT_LE(limits_min.x() - 1e-9, pose.x() - 15.);
    EXPECT_LE(limits_min.y() - 1e-9, pose.y() - 15.);

    for (const Eigen::Array2i& index :
         mapping::XYIndexRangeIterator(limits.cell_limits())) {
      const Eigen::Vector2f center = limits.GetCellCenter(index);
      const Eigen::Array2i source_index = source_limits.GetCellIndex(center);
      if (!source_limits.Contains(source_index)) {
        ASSERT_NEAR(local->GetProbability(index), mapping::kMinProbability,
                    1e-3);
        continue;
      }
      ASSERT_LT((center - source_limits.GetCellCenter(source_index))
                    .cwiseAbs()
                    .maxCoeff(),
                1e-4)
          << "pose=" << pose.transpose() << " index=" << index.transpose();
      ASSERT_EQ(local->IsKnown(index), source->IsKnown(source_index));
      if (source->IsKnown(source_index)) {
        ASSERT_FLOAT_EQ(local->GetProbability(index),
                        source->GetProbability(source_index));
      }
    }
  }
}

}  // namespace
}  // namespace localization
}  // namespace cartographer
