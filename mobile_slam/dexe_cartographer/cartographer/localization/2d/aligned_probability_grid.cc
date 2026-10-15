#include "cartographer/localization/2d/aligned_probability_grid.h"

#include <cmath>
#include <limits>

namespace cartographer {
namespace localization {

std::unique_ptr<mapping::ProbabilityGrid> CreateAlignedProbabilityGrid(
    const mapping::ProbabilityGrid& source,
    const Eigen::Vector2d& requested_min, const Eigen::Vector2d& requested_max,
    mapping::ValueConversionTables* conversion_tables) {
  CHECK(requested_min.allFinite());
  CHECK(requested_max.allFinite());
  CHECK((requested_max.array() > requested_min.array()).all());
  const auto& limits = source.limits();
  const double resolution = limits.resolution();
  Eigen::Array2i begin;
  Eigen::Array2i size;
  // MapLimits index 0 is world Y, index 1 is world X. Round boundaries
  // outward, then copy integer indices to avoid float cell-center lookup.
  for (int index_axis = 0; index_axis < 2; ++index_axis) {
    const int world_axis = 1 - index_axis;
    const double first = std::floor(
        (limits.max()[world_axis] - requested_max[world_axis]) / resolution);
    const double end = std::ceil(
        (limits.max()[world_axis] - requested_min[world_axis]) / resolution);
    CHECK(std::isfinite(first));
    CHECK(std::isfinite(end));
    CHECK_GE(first, std::numeric_limits<int>::min());
    CHECK_LE(end, std::numeric_limits<int>::max());
    CHECK_GT(end - first, 0.);
    CHECK_LE(end - first, std::numeric_limits<int>::max());
    begin[index_axis] = static_cast<int>(first);
    size[index_axis] = static_cast<int>(end - first);
  }
  const Eigen::Vector2d aligned_max =
      limits.max() - resolution * Eigen::Vector2d(begin[1], begin[0]);
  auto grid = std::make_unique<mapping::ProbabilityGrid>(
      mapping::MapLimits(resolution, aligned_max,
                         mapping::CellLimits(size[0], size[1])),
      conversion_tables);
  for (int row = 0; row < size[0]; ++row) {
    for (int col = 0; col < size[1]; ++col) {
      const Eigen::Array2i local_index(row, col);
      const Eigen::Array2i source_index = begin + local_index;
      if (!limits.Contains(source_index)) {
        // Preserve the existing outside-source behavior; unknown source
        // cells remain unknown rather than being converted to free space.
        grid->SetProbability(local_index, mapping::kMinProbability);
      } else if (source.IsKnown(source_index)) {
        grid->SetProbability(local_index, source.GetProbability(source_index));
      }
    }
  }
  return grid;
}

}  // namespace localization
}  // namespace cartographer
