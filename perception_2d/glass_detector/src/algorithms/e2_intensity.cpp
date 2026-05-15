// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0

#include "glass_detector/algorithms/e2_intensity.hpp"

#include <algorithm>
#include <cmath>

namespace glass_detector
{

void detectE2Intensity(
  const sensor_msgs::msg::LaserScan & scan,
  const E2Params & params,
  std::vector<bool> & is_glass)
{
  if (!params.enabled) {
    return;
  }
  const std::size_t N = scan.intensities.size();
  if (N == 0 || N != scan.ranges.size()) {
    return;
  }
  if (is_glass.size() != N) {
    is_glass.assign(N, false);
  }

  const int W = std::max(1, params.profile_width);
  if (N <= static_cast<std::size_t>(W * 2)) {
    return;
  }

  for (std::size_t i = static_cast<std::size_t>(W); i + static_cast<std::size_t>(W) < N; ++i) {
    const float intensity_i = scan.intensities[i];
    if (!std::isfinite(intensity_i)) {
      continue;
    }
    if (intensity_i < params.trigger_intensity) {
      continue;
    }

    // 在前后 W 窗口内找最低强度（不含 i 自身）
    float min_neighbor = intensity_i;
    for (int j = -W; j <= W; ++j) {
      if (j == 0) {
        continue;
      }
      const float v = scan.intensities[i + static_cast<std::size_t>(j)];
      if (std::isfinite(v) && v < min_neighbor) {
        min_neighbor = v;
      }
    }

    if (static_cast<double>(intensity_i) - static_cast<double>(min_neighbor) <
      params.intensity_delta)
    {
      continue;
    }

    // 标记当前及前后 W/2 范围为玻璃
    const int half_w = W / 2;
    const int start = std::max(0, static_cast<int>(i) - half_w);
    const int end = std::min(static_cast<int>(N) - 1, static_cast<int>(i) + half_w);
    for (int j = start; j <= end; ++j) {
      is_glass[static_cast<std::size_t>(j)] = true;
    }
  }
}

}  // namespace glass_detector
