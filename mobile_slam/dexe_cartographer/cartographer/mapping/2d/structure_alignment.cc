// Copyright (c) 2026 Dexforce Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cartographer/mapping/2d/structure_alignment.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr double kDegToRad = M_PI / 180.0;

// 线方向无正负之分，归一化到 [0, π)；异常值归 0
double NormalizeDirection(double angle) {
  if (!std::isfinite(angle)) return 0.0;
  double a = std::fmod(angle, M_PI);
  if (a < 0) a += M_PI;
  return a;
}

// 无向线方向（mod π）上的角度差，结果落在 [0, π/2]
double DirectionDistance(double a, double b) {
  const double d = std::fmod(std::abs(a - b), M_PI);
  return std::min(d, M_PI - d);
}

// 循环下标
int WrapIndex(int i, int n) { return ((i % n) + n) % n; }

struct Candidate {
  double length;
  double angle;  // 已归一化到 [0, π)
};

}  // namespace

absl::optional<double> EstimateAlignmentYaw(
    const LineMap2DList& line_map,
    const StructureAlignmentOptions& options) {
  // ── 1. 候选筛选：结构长度门槛 ────────────────────────────────────────
  // （观测次数 / RANSAC 内点等基础质量剔除已由上游 LineDataCulling 完成）
  std::vector<Candidate> candidates;
  for (const auto& kv : line_map.GetLineMapData2DList()) {
    const MapLine2D& map_line = kv.second;
    const sensor::LineData& line = map_line.line_data();
    if (line.length() < options.min_line_length) continue;
    candidates.push_back({line.length(), NormalizeDirection(line.angle())});
  }
  LOG(INFO) << "EstimateAlignmentYaw: " << candidates.size() << " of "
            << line_map.size()
            << " global lines passed structure length gate (min_length="
            << options.min_line_length << " m)";
  // 正交结构至少需要两条方向不同的线段
  if (candidates.size() < 2) {
    LOG(INFO) << "EstimateAlignmentYaw: not enough candidates, giving up.";
    return absl::nullopt;
  }

  // ── 2. 按长度降序取 Top-N（长的墙优先）─────────────────────────────
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.length > b.length;
            });
  if (static_cast<int>(candidates.size()) > options.max_candidate_lines) {
    candidates.resize(options.max_candidate_lines);
  }

  // ── 3. 加权角度直方图（权重 = 线段长度），方向归一化到 [0, π) ──────
  const int num_bins = std::max(
      1, static_cast<int>(
             std::round(M_PI / (options.histogram_bin_deg * kDegToRad))));
  std::vector<double> hist(num_bins, 0.0);
  for (const auto& c : candidates) {
    const int bin = std::min(static_cast<int>(c.angle / M_PI * num_bins),
                             num_bins - 1);
    hist[bin] += c.length;
  }

  // 循环平滑（核 [1 2 3 2 1]/9），避免能量被相邻分箱拆散
  std::vector<double> smooth(num_bins, 0.0);
  for (int i = 0; i < num_bins; ++i) {
    smooth[i] = (hist[WrapIndex(i - 2, num_bins)] +
                 2.0 * hist[WrapIndex(i - 1, num_bins)] + 3.0 * hist[i] +
                 2.0 * hist[WrapIndex(i + 1, num_bins)] +
                 hist[WrapIndex(i + 2, num_bins)]) /
                9.0;
  }

  const double bin_width = M_PI / num_bins;
  const double tol = options.orthogonality_tolerance_deg * kDegToRad;
  const int tol_bins = std::max(1, static_cast<int>(std::ceil(tol / bin_width)));
  // 某峰 ±tol 窗口内的支撑量（平滑后权重之和）
  auto window_support = [&smooth, num_bins, tol_bins](int center) {
    double s = 0.0;
    for (int d = -tol_bins; d <= tol_bins; ++d) {
      s += smooth[WrapIndex(center + d, num_bins)];
    }
    return s;
  };

  // ── 4. 主方向：直方图峰值 + 窗口内倍角加权圆均值精化 ────────────────
  const int peak1 = static_cast<int>(
      std::max_element(smooth.begin(), smooth.end()) - smooth.begin());
  if (smooth[peak1] <= 0.0) {
    LOG(INFO) << "EstimateAlignmentYaw: empty histogram, giving up.";
    return absl::nullopt;
  }
  const double support1 = window_support(peak1);

  double sum_w = 0.0, sum_sin = 0.0, sum_cos = 0.0;
  for (int d = -tol_bins; d <= tol_bins; ++d) {
    const int b = WrapIndex(peak1 + d, num_bins);
    const double a = (b + 0.5) * bin_width;  // 分箱中心角
    sum_w += smooth[b];
    sum_sin += smooth[b] * std::sin(2.0 * a);
    sum_cos += smooth[b] * std::cos(2.0 * a);
  }
  double theta1 = NormalizeDirection(0.5 * std::atan2(sum_sin, sum_cos));

  // ── 5. 次方向：屏蔽主方向窗口后的次峰，须接近主方向 +90° ────────────
  std::vector<double> masked = smooth;
  for (int d = -tol_bins; d <= tol_bins; ++d) {
    masked[WrapIndex(peak1 + d, num_bins)] = 0.0;
  }
  const int peak2 = static_cast<int>(
      std::max_element(masked.begin(), masked.end()) - masked.begin());
  if (masked[peak2] <= 0.0) {
    LOG(INFO) << "EstimateAlignmentYaw: single direction only, no "
                 "orthogonal structure. Giving up.";
    return absl::nullopt;
  }
  const double theta2 = (peak2 + 0.5) * bin_width;
  if (DirectionDistance(theta2, theta1 + M_PI_2) > tol) {
    LOG(INFO) << "EstimateAlignmentYaw: secondary direction ("
              << theta2 / kDegToRad << " deg) is not orthogonal to primary ("
              << theta1 / kDegToRad << " deg). Giving up.";
    return absl::nullopt;
  }
  const double support2 = window_support(peak2);
  if (support2 < options.min_secondary_support_ratio * support1) {
    LOG(INFO) << "EstimateAlignmentYaw: secondary support too weak ("
              << support2 << " vs primary " << support1 << ", required ratio "
              << options.min_secondary_support_ratio << "). Giving up.";
    return absl::nullopt;
  }

  // ── 6. 计算新原点朝向：new_yaw ≡ θ1 (mod π/2)，取 |new_yaw| 最小解 ──
  //    对旧系施加 R(-new_yaw) 后，θ1 方向的线落在坐标轴上，
  //    与之正交的次方向自然落在另一轴上。
  double new_yaw = std::fmod(theta1, M_PI_2);
  if (new_yaw > M_PI_4) new_yaw -= M_PI_2;

  LOG(INFO) << "EstimateAlignmentYaw: primary direction "
            << theta1 / kDegToRad << " deg (support " << support1
            << "), secondary direction " << theta2 / kDegToRad
            << " deg (support " << support2 << ") -> new_yaw " << new_yaw
            << " rad (" << new_yaw / kDegToRad << " deg)";
  return new_yaw;
}

}  // namespace mapping
}  // namespace cartographer
