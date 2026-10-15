#include "cartographer/localization/2d/aligned_probability_grid.h"

#include <limits>

#include "gtest/gtest.h"

namespace cartographer {
namespace localization {
namespace {

TEST(AlignedProbabilityGridTest, RectangularAsymmetricCropPreservesCells) {
  mapping::ValueConversionTables tables;
  mapping::ProbabilityGrid source(
      mapping::MapLimits(0.05, {0.47, -0.13}, mapping::CellLimits(20, 30)),
      &tables);
  for (int row = 0; row < 20; ++row) {
    for (int col = 0; col < 30; ++col) {
      source.SetProbability({row, col}, 0.2f + ((row * 7 + col) % 9) * 0.07f);
    }
  }
  const Eigen::Vector2d requested_min(-0.631, -0.824);
  const Eigen::Vector2d requested_max(0.214, -0.237);
  auto cropped = CreateAlignedProbabilityGrid(source, requested_min,
                                              requested_max, &tables);
  const auto& limits = cropped->limits();
  EXPECT_NE(limits.cell_limits().num_x_cells, limits.cell_limits().num_y_cells);
  EXPECT_GE(limits.max().x(), requested_max.x());
  EXPECT_GE(limits.max().y(), requested_max.y());
  EXPECT_LE(
      limits.max().x() - limits.resolution() * limits.cell_limits().num_y_cells,
      requested_min.x());
  EXPECT_LE(
      limits.max().y() - limits.resolution() * limits.cell_limits().num_x_cells,
      requested_min.y());
  for (int row = 0; row < limits.cell_limits().num_x_cells; ++row) {
    for (int col = 0; col < limits.cell_limits().num_y_cells; ++col) {
      const Eigen::Array2i index(row, col);
      const auto world = limits.GetCellCenter(index);
      const auto original = source.limits().GetCellIndex(world);
      ASSERT_TRUE(source.limits().Contains(original));
      EXPECT_TRUE(
          world.isApprox(source.limits().GetCellCenter(original), 1e-6));
      EXPECT_FLOAT_EQ(cropped->GetProbability(index),
                      source.GetProbability(original));
    }
  }
}

TEST(AlignedProbabilityGridTest, DifferentCentersKeepWorldGeometry) {
  mapping::ValueConversionTables tables;
  mapping::ProbabilityGrid source(
      mapping::MapLimits(0.05, {1., 1.}, mapping::CellLimits(40, 40)), &tables);
  const Eigen::Array2i obstacle(17, 23);
  source.SetProbability(obstacle, 0.8f);
  const auto world = source.limits().GetCellCenter(obstacle);
  for (const double shift : {-0.024, 0., 0.024, 0.051}) {
    auto crop =
        CreateAlignedProbabilityGrid(source, {-0.5 + shift, -0.5 + shift},
                                     {0.5 + shift, 0.5 + shift}, &tables);
    const auto index = crop->limits().GetCellIndex(world);
    ASSERT_TRUE(crop->limits().Contains(index));
    EXPECT_TRUE(world.isApprox(crop->limits().GetCellCenter(index), 1e-6));
    EXPECT_FLOAT_EQ(crop->GetProbability(index),
                    source.GetProbability(obstacle));
  }
}

TEST(AlignedProbabilityGridTest, PreservesUnknownAndOutsideSourceSemantics) {
  mapping::ValueConversionTables tables;
  mapping::ProbabilityGrid source(
      mapping::MapLimits(1., {2., 3.}, mapping::CellLimits(2, 3)), &tables);
  source.SetProbability({0, 0}, 0.7f);
  auto crop =
      CreateAlignedProbabilityGrid(source, {-2., 0.}, {3., 4.}, &tables);
  const auto known =
      crop->limits().GetCellIndex(source.limits().GetCellCenter({0, 0}));
  const auto unknown =
      crop->limits().GetCellIndex(source.limits().GetCellCenter({1, 1}));
  EXPECT_FLOAT_EQ(crop->GetProbability(known), source.GetProbability({0, 0}));
  EXPECT_FALSE(crop->IsKnown(unknown));
  const auto outside = crop->limits().GetCellIndex({2.5f, 3.5f});
  EXPECT_TRUE(crop->IsKnown(outside));
  EXPECT_NEAR(crop->GetProbability(outside), mapping::kMinProbability, 1e-5);
  EXPECT_FALSE(source.IsKnown({1, 1}));
}

TEST(AlignedProbabilityGridTest,
     FloatResolutionDoesNotShrinkRequestedCoverage) {
  mapping::ValueConversionTables tables;
  for (const double resolution : {double(0.05f), 0.1, 0.2}) {
    mapping::ProbabilityGrid source(
        mapping::MapLimits(resolution, {20., 20.},
                           mapping::CellLimits(400, 400)),
        &tables);
    auto crop =
        CreateAlignedProbabilityGrid(source, {-15., -7.}, {15., 7.}, &tables);
    const auto& limits = crop->limits();
    // Allow double arithmetic roundoff (far below one grid cell).
    EXPECT_GE(limits.max().x() + 1e-12, 15.);
    EXPECT_GE(limits.max().y() + 1e-12, 7.);
    EXPECT_LE(limits.max().x() - resolution * limits.cell_limits().num_y_cells,
              -15.);
    EXPECT_LE(limits.max().y() - resolution * limits.cell_limits().num_x_cells,
              -7.);
    EXPECT_LT(limits.max().x() - 15., resolution + 1e-10);
    EXPECT_LT(limits.max().y() - 7., resolution + 1e-10);
  }
}

TEST(AlignedProbabilityGridTest, SourceGrowthKeepsCellPositions) {
  mapping::ValueConversionTables tables;
  mapping::ProbabilityGrid source(
      mapping::MapLimits(0.1, {1., 1.}, mapping::CellLimits(20, 20)), &tables);
  source.SetProbability({10, 10}, 0.75f);
  const auto world = source.limits().GetCellCenter({10, 10});
  auto before =
      CreateAlignedProbabilityGrid(source, {-0.3, -0.4}, {0.4, 0.5}, &tables);
  source.GrowLimits({5.f, -5.f});
  auto after =
      CreateAlignedProbabilityGrid(source, {-0.3, -0.4}, {0.4, 0.5}, &tables);
  EXPECT_TRUE(before->limits().max().isApprox(after->limits().max(), 1e-12));
  EXPECT_FLOAT_EQ(before->GetProbability(before->limits().GetCellIndex(world)),
                  after->GetProbability(after->limits().GetCellIndex(world)));
}

TEST(AlignedProbabilityGridTest, RejectsInvalidBounds) {
  mapping::ValueConversionTables tables;
  mapping::ProbabilityGrid source(
      mapping::MapLimits(1., {1., 1.}, mapping::CellLimits(2, 2)), &tables);
  EXPECT_DEATH(
      CreateAlignedProbabilityGrid(source, {0., 0.}, {0., 1.}, &tables),
      "Check failed");
  EXPECT_DEATH(CreateAlignedProbabilityGrid(
                   source, {0., 0.},
                   {std::numeric_limits<double>::infinity(), 1.}, &tables),
               "Check failed");
}

}  // namespace
}  // namespace localization
}  // namespace cartographer
