#ifndef CARTOGRAPHER_LOCALIZATION_2D_ALIGNED_PROBABILITY_GRID_H_
#define CARTOGRAPHER_LOCALIZATION_2D_ALIGNED_PROBABILITY_GRID_H_

#include <memory>

#include "cartographer/mapping/2d/probability_grid.h"

namespace cartographer {
namespace localization {

// Covers the requested world-coordinate rectangle with whole source cells.
// Copies probabilities without moving cell centers or modifying the source.
std::unique_ptr<mapping::ProbabilityGrid> CreateAlignedProbabilityGrid(
    const mapping::ProbabilityGrid& source,
    const Eigen::Vector2d& requested_min, const Eigen::Vector2d& requested_max,
    mapping::ValueConversionTables* conversion_tables);

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_ALIGNED_PROBABILITY_GRID_H_
