// Copyright 2026 DEXE Robot Team
// Licensed under Apache-2.0

#include "glass_detector/algorithms/e1_geometric.hpp"

#include <algorithm>
#include <cmath>

namespace glass_detector
{

namespace
{

// 在以 center 为中心、宽 window 的邻域内计算 range 的标准差。
// 跳过非有限值（NaN / inf）。若有效样本 < 2，返回 0。
double localStd(const std::vector<float> & ranges, std::size_t center, int window)
{
  const int half = std::max(1, window / 2);
  const int n = static_cast<int>(ranges.size());
  const int start = std::max(0, static_cast<int>(center) - half);
  const int end = std::min(n - 1, static_cast<int>(center) + half);

  double sum = 0.0;
  int cnt = 0;
  for (int j = start; j <= end; ++j) {
    const float r = ranges[static_cast<std::size_t>(j)];
    if (std::isfinite(r)) {
      sum += r;
      ++cnt;
    }
  }
  if (cnt < 2) {
    return 0.0;
  }
  const double mean = sum / cnt;

  double var = 0.0;
  for (int j = start; j <= end; ++j) {
    const float r = ranges[static_cast<std::size_t>(j)];
    if (std::isfinite(r)) {
      const double d = r - mean;
      var += d * d;
    }
  }
  return std::sqrt(var / cnt);
}

}  // namespace

void detectE1Geometric(
  const sensor_msgs::msg::LaserScan & scan,
  const E1Params & params,
  std::vector<bool> & is_glass)
{
  if (!params.enabled) {
    return;
  }
  const std::size_t N = scan.ranges.size();
  if (is_glass.size() != N) {
    is_glass.assign(N, false);
  }
  if (N < 2) {
    return;
  }

  // Step 1+2：联合检测距离跳变 + 局部 std
  for (std::size_t i = 0; i < N; ++i) {
    const float r_i = scan.ranges[i];
    if (!std::isfinite(r_i)) {
      continue;
    }
    if (r_i < scan.range_min || r_i > scan.range_max) {
      continue;
    }

    // 距离跳变：与下一束 |Δr| 比较
    bool jump_hit = false;
    if (i + 1 < N) {
      const float r_next = scan.ranges[i + 1];
      if (std::isfinite(r_next)) {
        const double dr = std::abs(static_cast<double>(r_next) - static_cast<double>(r_i));
        if (dr > params.range_jump) {
          is_glass[i] = true;
          is_glass[i + 1] = true;
          jump_hit = true;
        }
      }
    }
    if (jump_hit) {
      continue;
    }

    // 局部 std 检测
    const double sigma = localStd(scan.ranges, i, params.window_size);
    if (sigma > params.std_threshold) {
      is_glass[i] = true;
    }
  }
}

}  // namespace glass_detector
