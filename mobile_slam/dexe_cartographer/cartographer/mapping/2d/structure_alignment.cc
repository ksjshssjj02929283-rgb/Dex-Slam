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

struct Candidate {
  double length;
  double angle;  // 已归一化到 [0, π)
};

// 一个互不相同的结构方向及其支撑量
struct DirectionPeak {
  double angle;    // 精化后的方向（弧度，归一化到 [0, π)）
  double support;  // 精化窗口内候选线段长度之和（米）
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

  // ── 4. 枚举所有结构方向 ──────────────────────────────────────────────
  //    旧实现取 hist 的 argmax 当主方向、屏蔽其 ±tol 后再取 argmax 当次方向，
  //    最后才校验正交性。这是无条件 argmax：只要存在一族总长度更大的斜向
  //    线（家具、稀疏回波拟合出的伪长线），次方向的 argmax 就会被它抢走，
  //    正交校验必然失败并放弃对齐 —— 哪怕数据里确实躺着一对清晰的墙。
  //    改为把显著峰全部枚举成方向、各自精化，正交性留给第 5 段在候选对里
  //    挑，斜向族就只能"参与竞争"而无法"顶掉"真正的正交解。
  std::vector<int> bins_by_support(num_bins);
  for (int i = 0; i < num_bins; ++i) bins_by_support[i] = i;
  std::sort(bins_by_support.begin(), bins_by_support.end(),
            [&hist](int a, int b) { return hist[a] > hist[b]; });

  std::vector<DirectionPeak> peaks;
  for (const int bin : bins_by_support) {
    if (hist[bin] <= 0.0) break;  // 已按支撑降序，后面全为 0
    double support = 0.0;
    const double angle = RefineDirection(
        candidates, (bin + 0.5) * bin_width, tol, kRefineHalfWindowRad,
        &support);
    if (support <= 0.0) continue;
    // 已被更强的方向覆盖（同一族墙的邻接 bin，或精化后收敛到同一模）
    bool duplicate = false;
    for (const auto& p : peaks) {
      if (DirectionDistance(angle, p.angle) <= tol) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) peaks.push_back({angle, support});
  }

  LOG(INFO) << "EstimateAlignmentYaw: " << peaks.size()
            << " distinct structure direction(s) above noise";
  if (peaks.size() < 2) {
    LOG(INFO) << "EstimateAlignmentYaw: single direction only, no "
                 "orthogonal structure. Giving up.";
    emit();
    return absl::nullopt;
  }

  // ── 5. 在所有 (主方向, 正交伙伴) 组合里挑联合支撑最大的一对 ──────────
  //    评分 w1·w2：两个方向各自都要有足够支撑才可能胜出，避免一对弱峰
  //    因为恰好接近垂直而战胜真正的两族墙。
  //    "支撑大者为 primary" 不是为了物理含义，而是堵住支撑比门槛的漏洞：
  //    否则 (弱, 强) 这种反向配对会用 w2=强 去比 w1=弱，绕开门槛。交换
  //    primary/secondary 得到的新原点角在 mod π/2 下相同，所以该约束不
  //    损失任何解。
  const DirectionPeak* best_primary = nullptr;
  const DirectionPeak* best_secondary = nullptr;
  double best_score = -1.0;
  for (size_t i = 0; i < peaks.size(); ++i) {
    for (size_t j = 0; j < peaks.size(); ++j) {
      if (i == j) continue;
      if (peaks[i].support < peaks[j].support) continue;
      if (DirectionDistance(peaks[j].angle, peaks[i].angle + M_PI_2) > tol) {
        continue;
      }
      if (peaks[j].support <
          options.min_secondary_support_ratio * peaks[i].support) {
        continue;
      }
      const double score = peaks[i].support * peaks[j].support;
      if (score > best_score) {
        best_score = score;
        best_primary = &peaks[i];
        best_secondary = &peaks[j];
      }
    }
  }

  if (best_primary == nullptr) {
    // 区分「只有一族」与「有多族但两两都不正交」——两者的处置完全不同：
    // 前者是数据不足，后者要去看是不是容差定得太紧。
    LOG(INFO) << "EstimateAlignmentYaw: no pair of structure directions is "
                 "orthogonal within tolerance ("
              << options.orthogonality_tolerance_deg
              << " deg, min secondary support ratio "
              << options.min_secondary_support_ratio << "). Giving up.";
    for (const auto& p : peaks) {
      LOG(INFO) << "  candidate direction " << p.angle / kDegToRad
                << " deg (support " << p.support << " m)";
    }
    emit();
    return absl::nullopt;
  }

  const double theta1 = best_primary->angle;
  const double theta2 = best_secondary->angle;
  const double w1 = best_primary->support;
  const double w2 = best_secondary->support;

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
