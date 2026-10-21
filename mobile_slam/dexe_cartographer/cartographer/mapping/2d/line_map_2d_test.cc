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

#include <cmath>
#include <cstring>
#include <map>
#include <new>
#include <random>
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
// 只带线段数据，不带 observations_（该字段既不持久化也不随深拷贝传
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

// ─────────────────────────────────────────────────────────────────────
// 直线拟合（LineFittingRANSAC / OrthogonalRegression）
// ─────────────────────────────────────────────────────────────────────

// 沿方向 'phi' 生成散布在 [−2.5, 2.5] 的点，叠加垂直于直线的高斯噪声
void MakeNoisyLinePoints(double phi, double noise_sigma, int num_points,
                         uint32_t seed, std::vector<double>* xs,
                         std::vector<double>* ys) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> noise(0.0, noise_sigma);
  const Eigen::Vector2d dir(std::cos(phi), std::sin(phi));
  const Eigen::Vector2d normal(-std::sin(phi), std::cos(phi));
  for (int i = 0; i < num_points; ++i) {
    const double t = -2.5 + 5.0 * i / (num_points - 1);
    const Eigen::Vector2d p = t * dir + noise(rng) * normal;
    xs->push_back(p.x());
    ys->push_back(p.y());
  }
}

// 近竖直直线：正交回归应无偏（OLS 会把斜率衰减向 0，此类场景误差 1°+）。
// 直接测 OrthogonalRegression：给定输入完全确定。
TEST(OrthogonalRegressionTest, NearVerticalIsUnbiased) {
  std::vector<double> xs, ys;
  MakeNoisyLinePoints(89.9 * M_PI / 180.0, /*noise_sigma=*/0.01,
                      /*num_points=*/60, /*seed=*/7, &xs, &ys);
  double a, b;
  OrthogonalRegression(xs, ys, a, b);
  EXPECT_NEAR(std::atan(a) * 180.0 / M_PI, 89.9, 0.05);
}

// 浅角度与 45° 直线同样保持高精度
TEST(OrthogonalRegressionTest, AccurateAtShallowAnd45Degrees) {
  for (const double phi_deg : {0.5, 45.0}) {
    std::vector<double> xs, ys;
    MakeNoisyLinePoints(phi_deg * M_PI / 180.0, /*noise_sigma=*/0.01,
                        /*num_points=*/60, /*seed=*/13, &xs, &ys);
    double a, b;
    OrthogonalRegression(xs, ys, a, b);
    EXPECT_NEAR(std::atan(a) * 180.0 / M_PI, phi_deg, 0.05)
        << "phi = " << phi_deg;
  }
}

// 完全竖直（无噪声，Sxy 恒为 0）应走垂直约定：|a| > 1e6 且 −b/a 还原 x̄，
// 与 ProjectPointToLine 垂直分支（x = −b/k）自洽。
// 注意：带噪声时 Sxy 随机波动，斜率是很大但有限的值（无偏，见上一用例），
// 不会进入该分支——这是预期行为。
TEST(OrthogonalRegressionTest, ExactlyVerticalUsesVerticalConvention) {
  std::vector<double> xs, ys;
  for (int i = 0; i < 60; ++i) {
    xs.push_back(2.0);
    ys.push_back(5.0 * i / 59.0);
  }
  double a, b;
  OrthogonalRegression(xs, ys, a, b);
  EXPECT_GT(std::abs(a), 1e6);
  EXPECT_NEAR(-b / a, 2.0, 1e-3);
}

// RANSAC 全流程（假设采样 + 内点筛选 + 正交精化）在近竖直场景下仍无偏。
// 噪声取 σ=0.002m：即使随机抽到相邻点对使假设线偏斜数度，
// 0.1m 内点带仍覆盖足够长的点列，精化结果稳定在 0.1° 内。
TEST(LineFittingRANSACTest, NearVerticalPipelineIsUnbiased) {
  std::vector<double> xs, ys;
  MakeNoisyLinePoints(89.9 * M_PI / 180.0, /*noise_sigma=*/0.002,
                      /*num_points=*/60, /*seed=*/7, &xs, &ys);
  double k, b, r;
  LineFittingRANSAC(xs, ys, &k, &b, &r);
  EXPECT_NEAR(std::atan(k) * 180.0 / M_PI, 89.9, 0.1);
}

// 全流程：MapLine2D 多次 UpdateLineData 后，近竖直墙的角度应保持 ≈ π/2
// （此前 OLS 精化会把近竖直墙角度拉向水平方向）。
// 100 次观测使沿墙散布足够长，TLS 角度波动（∝ σ·L/√N / L²）远小于 0.1°。
TEST(MapLine2DTest, UpdateLineDataNearVerticalWallKeepsAngle) {
  std::mt19937 rng(11);
  std::normal_distribution<double> noise(0.0, 0.01);
  // 种子线段：竖直墙 x = 1.0 上的一小段
  MapLine2D line(0, sensor::LineData(Eigen::Vector2d(1.0, 0.0),
                                     Eigen::Vector2d(1.0, 0.5)));
  // 100 次观测：沿墙不同位置的 0.5m 线段，端点带垂直噪声
  for (int i = 0; i < 100; ++i) {
    const double y0 = 0.1 * i;
    line.UpdateLineData(
        sensor::LineData(Eigen::Vector2d(1.0 + noise(rng), y0),
                         Eigen::Vector2d(1.0 + noise(rng), y0 + 0.5)));
  }
  EXPECT_NEAR(std::abs(line.line_data().angle()), M_PI_2,
              0.1 * M_PI / 180.0);
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
