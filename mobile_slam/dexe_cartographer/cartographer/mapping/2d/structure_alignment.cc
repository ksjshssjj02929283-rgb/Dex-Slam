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

// 逐线精化的最终窗口半径（±2°）：主峰附近的真实墙体方向散布通常在 1–2° 内，
// 更宽的窗口会把漂移/斜向线段等不对称尾部算进均值
constexpr double kRefineHalfWindowRad = 2.0 * kDegToRad;

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

// 角度包裹到 [−π/2, π/2)：线方向 mod π，两方向的正交误差合法范围即此区间
double WrapToHalfPi(double x) {
  x = std::fmod(x, M_PI);
  if (x >= M_PI_2) x -= M_PI;
  if (x < -M_PI_2) x += M_PI;
  return x;
}

// 用候选线段自身的角度（而非直方图分箱中心）精化方向估计：
// 长度加权、倍角圆均值，窗口判定为 DirectionDistance ≤ 半窗。
// 窗口从 first_half_window（以 seed_angle 为中心）开始，每轮减半并以上一轮
// 加权均值为中心重新定位，直到 min_half_window —— 渐缩式收窄能在存在不对称
// 尾部（漂移墙、斜向家具线）时收敛到真实主峰而不是被尾部拖走；
// 一步跳到窄窗则有丢失真实主峰的风险。
// 空窗时保持当前中心。*support_out = 最终窗口内候选长度之和（支撑量）。
double RefineDirection(const std::vector<Candidate>& candidates,
                       double seed_angle, double first_half_window,
                       double min_half_window, double* support_out) {
  double center = NormalizeDirection(seed_angle);
  double support = 0.0;
  double half_window = std::max(first_half_window, min_half_window);
  while (true) {
    double sum_w = 0.0, sum_sin = 0.0, sum_cos = 0.0;
    for (const auto& c : candidates) {
      if (DirectionDistance(c.angle, center) > half_window) continue;
      sum_w += c.length;
      sum_sin += c.length * std::sin(2.0 * c.angle);
      sum_cos += c.length * std::cos(2.0 * c.angle);
    }
    if (sum_w <= 0.0) break;  // 空窗：保持当前中心
    center = NormalizeDirection(0.5 * std::atan2(sum_sin, sum_cos));
    support = sum_w;
    if (half_window <= min_half_window + 1e-12) break;
    half_window = std::max(min_half_window, 0.5 * half_window);
  }
  if (support_out != nullptr) *support_out = support;
  return center;
}

}  // namespace

absl::optional<double> EstimateAlignmentYaw(
    const LineMap2DList& line_map,
    const StructureAlignmentOptions& options,
    StructureAlignmentDiagnostics* diagnostics) {
  // 本地诊断缓冲：调用方不关心时也照常填写，末尾统一回写，
  // 避免每个 return 点重复判空。
  StructureAlignmentDiagnostics diag;
  const auto emit = [&diagnostics, &diag]() {
    if (diagnostics != nullptr) *diagnostics = diag;
  };

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
  diag.candidate_lines = static_cast<int>(candidates.size());
  // 正交结构至少需要两条方向不同的线段
  if (candidates.size() < 2) {
    LOG(INFO) << "EstimateAlignmentYaw: not enough candidates, giving up.";
    emit();
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
  // Top-N 截断后的实际参与数才是「估计用了多少条线」
  diag.candidate_lines = static_cast<int>(candidates.size());

  // ── 3. 加权角度直方图（权重 = 线段长度），仅用于粗找峰 ──────────────
  //    方向值由第 4/5 段用线段自身角度精化给出，直方图不承担估值职责，
  //    故无需平滑：即使密集簇被分箱边界拆散，峰值 bin 与真实模的偏差
  //    也 ≤ 1 个 bin（1°），远小于首遍精化窗口 ±tol。
  const int num_bins = std::max(
      1, static_cast<int>(
             std::round(M_PI / (options.histogram_bin_deg * kDegToRad))));
  std::vector<double> hist(num_bins, 0.0);
  for (const auto& c : candidates) {
    const int bin = std::min(static_cast<int>(c.angle / M_PI * num_bins),
                             num_bins - 1);
    hist[bin] += c.length;
  }

  const double bin_width = M_PI / num_bins;
  const double tol = options.orthogonality_tolerance_deg * kDegToRad;
  const int tol_bins = std::max(1, static_cast<int>(std::ceil(tol / bin_width)));

  // ── 4. 主方向：直方图峰值 → 候选线段角度迭代精化 ────────────────────
  //    精化用线段自身角度（无分箱量化），收窄窗口以剔除不对称尾部；
  //    w1 = 精化窗口内线段长度之和，即主方向支撑量（第 6 段融合与
  //    次方向支撑比门槛共用）。
  const int peak1 = static_cast<int>(
      std::max_element(hist.begin(), hist.end()) - hist.begin());
  if (hist[peak1] <= 0.0) {
    LOG(INFO) << "EstimateAlignmentYaw: empty histogram, giving up.";
    emit();
    return absl::nullopt;
  }

  double w1 = 0.0;
  const double theta1 = RefineDirection(candidates, (peak1 + 0.5) * bin_width,
                                        tol, kRefineHalfWindowRad, &w1);

  // ── 5. 次方向：屏蔽主方向窗口后的次峰，须接近主方向 +90° ────────────
  std::vector<double> masked = hist;
  for (int d = -tol_bins; d <= tol_bins; ++d) {
    masked[WrapIndex(peak1 + d, num_bins)] = 0.0;
  }
  const int peak2 = static_cast<int>(
      std::max_element(masked.begin(), masked.end()) - masked.begin());
  if (masked[peak2] <= 0.0) {
    LOG(INFO) << "EstimateAlignmentYaw: single direction only, no "
                 "orthogonal structure. Giving up.";
    emit();
    return absl::nullopt;
  }
  double w2 = 0.0;
  const double theta2 = RefineDirection(candidates, (peak2 + 0.5) * bin_width,
                                        tol, kRefineHalfWindowRad, &w2);
  if (DirectionDistance(theta2, theta1 + M_PI_2) > tol) {
    LOG(INFO) << "EstimateAlignmentYaw: secondary direction ("
              << theta2 / kDegToRad << " deg) is not orthogonal to primary ("
              << theta1 / kDegToRad << " deg). Giving up.";
    emit();
    return absl::nullopt;
  }
  // 支撑比门槛直接用精化窗口内累计长度 w2/w1
  if (w2 < options.min_secondary_support_ratio * w1) {
    LOG(INFO) << "EstimateAlignmentYaw: secondary support too weak ("
              << w2 << " vs primary " << w1 << ", required ratio "
              << options.min_secondary_support_ratio << "). Giving up.";
    emit();
    return absl::nullopt;
  }

  // ── 6. 双方向支撑加权融合：加权最小二乘折中 ─────────────────────────
  //    实际地图受位姿图漂移影响，两族墙未必精确正交（δ = θ2−θ1−90° ≠ 0）。
  //    最小化 J(r) = w1·(θ1−r)² + w2·(θ2−r−90°)² 得
  //      r = θ1 + δ·w2/(w1+w2)：
  //    等支撑时两族各承担 δ/2 的残余（只用 θ1 会让次方向独扛全部 δ）；
  //    次方向弱时自然退化为仅用 θ1。
  const double delta = WrapToHalfPi(theta2 - theta1 - M_PI_2);
  double r = theta1;
  if (w1 + w2 > 0.0) {
    r = theta1 + delta * w2 / (w1 + w2);
  }
  // new_yaw ≡ r (mod π/2)，取 |new_yaw| 最小解
  double new_yaw = std::fmod(r, M_PI_2);
  if (new_yaw > M_PI_4) new_yaw -= M_PI_2;

  // 诊断量：对齐后两族各自保留的残余角及加权 RMS
  const double w_sum = std::max(w1 + w2, 1e-12);
  const double res1 = std::abs(delta) * w2 / w_sum;  // 主方向族残余
  const double res2 = std::abs(delta) * w1 / w_sum;  // 次方向族残余
  const double rms_residual =
      std::sqrt((w1 * res1 * res1 + w2 * res2 * res2) / w_sum);

  diag.primary_direction = theta1;
  diag.secondary_direction = theta2;
  diag.primary_support = w1;
  diag.secondary_support = w2;
  diag.orthogonality_error = delta;
  diag.rms_axis_residual = rms_residual;
  diag.estimated = true;
  emit();

  LOG(INFO) << "EstimateAlignmentYaw: primary direction "
            << theta1 / kDegToRad << " deg (support " << w1
            << "), secondary direction " << theta2 / kDegToRad
            << " deg (support " << w2 << "), orthogonality error "
            << delta / kDegToRad << " deg, weighted RMS axis residual "
            << rms_residual / kDegToRad << " deg -> new_yaw " << new_yaw
            << " rad (" << new_yaw / kDegToRad << " deg)";
  return new_yaw;
}

}  // namespace mapping
}  // namespace cartographer
