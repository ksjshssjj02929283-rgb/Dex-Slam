// Copyright (c) 2021 Geekplus Inc
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

#include "cartographer/mapping/2d/line_map_2d.h"
#include <vector>
#include <map>
#include <utility>
#include <cmath>  
#include <algorithm>  
#include <random>  

namespace cartographer
{
namespace mapping
{
// Eigen::Vector2d ComputeContactPoint(const double& k, const double& b,
//   const Eigen::Vector2d &point) {
//   Eigen::Vector2d res;
//   if (line.style() == sensor::VerticalStyle::X) {
//     res = Eigen::Vector2d{point[0], line.begin()[1]};
//   } else if (line.style() == sensor::VerticalStyle::Y) {
//     res = Eigen::Vector2d{line.begin()[0], point[1]};
//   } else {
//     // double m = point[0] + line.k() * point[1];
//     // res[0] = (m - line.k() * line.b()) / (std::pow(line.k(), 2) + 1);
//     // res[1] = line.k() * res[0] + line.b();
//     double dx = line.begin()[0] - line.end()[0];
//     double dy = line.begin()[1] - line.end()[1];
//     double u = (point[0] - line.begin()[0]) * (line.begin()[0] - line.end()[0]) +
//                (point[1] - line.begin()[1]) * (line.begin()[1] - line.end()[1]);
//     u =  u / ((dx * dx) + (dy * dy));
//     res[0] = line.begin()[0] + u * dx;
//     res[1] = line.begin()[1] + u * dy;
//   }
//   return res;
// }


MapLine2D::MapLine2D(const int local_id, const sensor::LineData & line_data)
: local_id_(local_id), line_data_(line_data), lose_time_(0)
{
  line_data_list_.push_back(line_data);
}

MapLine2D::MapLine2D(LineMapTpye type, const int global_id, const sensor::LineData & line_data)
: global_id_(global_id), line_data_(line_data)
{
}

void MapLine2D::UpdateLineData(const sensor::LineData & line_data)
{
  line_data_list_.push_back(line_data);
  
  // 动态历史数据管理 - 基于数据质量而非固定数量
  if (line_data_list_.size() > 200) {
    // 保留最近的150个数据点，删除最旧的50个
    line_data_list_.erase(line_data_list_.begin(), line_data_list_.begin() + 50);
  }
  
  // 收集所有端点用于拟合
  std::vector<double> xs, ys;
  std::vector<Eigen::Vector2d> all_points;
  
  // 预分配内存提高性能
  const size_t total_points = line_data_list_.size() * 2;
  xs.reserve(total_points);
  ys.reserve(total_points);
  all_points.reserve(total_points);
  
  for (const auto& data : line_data_list_) {
    // 收集起点
    xs.push_back(data.begin()[0]);
    ys.push_back(data.begin()[1]);
    all_points.push_back(data.begin());
    
    // 收集终点
    xs.push_back(data.end()[0]);
    ys.push_back(data.end()[1]);
    all_points.push_back(data.end());
  }
  
  // 使用RANSAC进行鲁棒拟合
  double k, b, r;
  inliers_ = LineFittingRANSAC(xs, ys, &k, &b, &r);
  
  // 改进的端点选择策略
  Eigen::Vector2d optimized_start, optimized_end;
  if (ComputeOptimizedEndpoints(all_points, k, b, optimized_start, optimized_end)) {
    line_data_ = sensor::LineData(optimized_start, optimized_end);
  } else {
    // 回退到简单的端点选择
    line_data_ = sensor::LineData(ComputeEndpointsFromFit(all_points, k, b));
  }
  
  lose_time_ = 0;
}

// 计算优化的端点
bool MapLine2D::ComputeOptimizedEndpoints(const std::vector<Eigen::Vector2d>& points,
                                         double k, double b,
                                         Eigen::Vector2d& start_point,
                                         Eigen::Vector2d& end_point) const
{
  if (points.empty()) return false;
  
  // 检查是否为垂直线
  const double eps = 1e-8;
  bool is_vertical = std::abs(k) > 1e6;  // 斜率过大认为是垂直线
  
  if (is_vertical) {
    // 垂直线：按Y坐标排序
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
    double avg_x = 0.0;
    
    for (const auto& point : points) {
      min_y = std::min(min_y, point[1]);
      max_y = std::max(max_y, point[1]);
      avg_x += point[0];
    }
    avg_x /= points.size();
    
    start_point = Eigen::Vector2d(avg_x, min_y);
    end_point = Eigen::Vector2d(avg_x, max_y);
  } else {
    // 非垂直线：投影到拟合直线上并找到最远的两个点
    std::vector<std::pair<double, Eigen::Vector2d>> projections;
    projections.reserve(points.size());
    
    for (const auto& point : points) {
      // 将点投影到拟合直线上
      Eigen::Vector2d projected = ProjectPointToLine(point, k, b);
      
      // 计算投影点在直线上的参数t（沿直线方向的距离）
      double t = (std::abs(k) < eps) ? projected[0] : projected[0]; // 水平线用x坐标
      if (std::abs(k) > eps) {
        // 对于有斜率的直线，使用更稳定的参数化
        t = projected[0] + k * projected[1];
      }
      
      projections.emplace_back(t, projected);
    }
    
    // 找到参数t的最小值和最大值对应的点
    auto minmax = std::minmax_element(projections.begin(), projections.end(),
                                     [](const auto& a, const auto& b) {
                                       return a.first < b.first;
                                     });
    
    start_point = minmax.first->second;
    end_point = minmax.second->second;
  }
  
  // 验证端点有效性
  double distance = (end_point - start_point).norm();
  return distance > eps && distance < 100.0;  // 合理的距离范围
}

// 将点投影到直线上
Eigen::Vector2d MapLine2D::ProjectPointToLine(const Eigen::Vector2d& point, 
                                             double k, double b) const
{
  const double eps = 1e-8;
  
  if (std::abs(k) > 1e6) {
    // 垂直线 x = c，投影就是 (c, point_y)
    double x_line = -b;  // 对于垂直线，通常表示为 x = -b/a
    return Eigen::Vector2d(x_line, point[1]);
  } else if (std::abs(k) < eps) {
    // 水平线 y = b，投影就是 (point_x, b)
    return Eigen::Vector2d(point[0], b);
  } else {
    // 一般直线 y = kx + b
    // 点到直线的投影公式
    double x_proj = (point[0] + k * (point[1] - b)) / (1 + k * k);
    double y_proj = k * x_proj + b;
    return Eigen::Vector2d(x_proj, y_proj);
  }
}

// 简单的端点计算作为回退方案
sensor::LineData MapLine2D::ComputeEndpointsFromFit(const std::vector<Eigen::Vector2d>& points,
                                                   double k, double b) const
{
  if (points.empty()) {
    return line_data_;  // 返回原有数据
  }
  
  // 找到X坐标的最小值和最大值对应的点
  auto x_minmax = std::minmax_element(points.begin(), points.end(),
                                     [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
                                       return a[0] < b[0];
                                     });
  
  Eigen::Vector2d min_point = *x_minmax.first;
  Eigen::Vector2d max_point = *x_minmax.second;
  
  // 将Y坐标投影到拟合直线上
  min_point[1] = k * min_point[0] + b;
  max_point[1] = k * max_point[0] + b;
  
  return sensor::LineData(min_point, max_point);
}

LineMap2DList::LineMap2DList(
  int latest_local_id,
  std::map<int, MapLine2D> active_line_data)
: latest_local_id_(latest_local_id)
{
  line_map_data_list_ = active_line_data;
}

LineMap2DList::LineMap2DList(
  const std::vector<cartographer::sensor::LaneData> & laser_line_map_2d)
{
  int global_id = 0;
  for (auto it : laser_line_map_2d) {
    line_map_data_list_.insert(
      std::pair<int, MapLine2D>(
        global_id,
        MapLine2D(
          MapLine2D::LineMapTpye::static_line, global_id,
          sensor::LineData{it.line_data.begin, it.line_data.end})));
    global_id++;
  }
}

LineMap2DList::LineMap2DList(
  const
  std::vector<cartographer::sensor::LaneDataWithNormal> & laser_line_map_2d)
{
  int global_id = 0;
  for (auto it : laser_line_map_2d) {
    sensor::LineData line_data(it.data.line_data.begin, it.data.line_data.end);
    line_data.set_normal(it.normal);

    line_map_data_list_.insert(
      std::pair<int, MapLine2D>(
        global_id,
        MapLine2D(
          MapLine2D::LineMapTpye::static_line, global_id,
          line_data)));
    // line_map_data_list_[global_id].line_data().set_normal(it.normal);
    global_id++;
  }
}

LineMap2DList::~LineMap2DList()
{
}

void LineMap2DList::UpdateLoseTime()
{
  for (auto & it : line_map_data_list_) {
    int lose_time = it.second.lose_time();
    it.second.SetLoseTime(lose_time + 1);
  }
}

void LineMap2DList::PushBack(std::vector<sensor::LineData> line_data)
{
  if (line_data.empty()) {
    std::cout << "[ERROR] LineMap2DList PushBack line_data is empty" <<
      std::endl;
    return;
  }
  
  latest_local_id_++;
  // std::cout << "\033[33m PushBack: " << latest_local_id_ << "\033[0m" << std::endl;
  // std::cout << line_data[0].angle() << std::endl;
  // std::cout << line_data[0].begin().transpose() << std::endl;
  // std::cout << line_data[0].end().transpose() << std::endl;
  // std::cout << "line length: " << line_data[0].length() << std::endl; 
  
  // Insert the first line data
  line_map_data_list_.insert(
    std::pair<int, MapLine2D>(
      latest_local_id_,
      MapLine2D(latest_local_id_, line_data[0])));
  observations_.insert(std::pair<int, size_t>(latest_local_id_, 1));
  
  // If there are multiple line data, use UpdateLineMap for the remaining ones
  if (line_data.size() > 1) {
    std::vector<sensor::LineData> remaining_data(line_data.begin() + 1, line_data.end());
    UpdateLineMap(latest_local_id_, remaining_data);
  }
}

void LineMap2DList::PushBack(
  int local_id, std::vector<sensor::LineData> line_data)
{
  // std::cout << "\033[32m PushNewBack: " << local_id << "\033[0m" << std::endl;
  if (line_data.empty()) {
    std::cout << "[ERROR] LineMap2DList PushBack line_data is empty" <<
      std::endl;
    return;
  }
  
  // line_data[0].Transform(transform::Rigid2d::FromVector(origin.cast<double>()));
  // Insert the first line data
  line_map_data_list_.insert(
    std::pair<int, MapLine2D>(
      local_id,
      MapLine2D(local_id, line_data[0])));
  observations_.insert(std::pair<int, size_t>(local_id, 1));
  
  // If there are multiple line data, use UpdateLineMap for the remaining ones
  if (line_data.size() > 1) {
    std::vector<sensor::LineData> remaining_data(line_data.begin() + 1, line_data.end());
    UpdateLineMap(local_id, remaining_data);
  }
}

void LineMap2DList::UpdateLineMap(
  int local_id, std::vector<sensor::LineData> line_data)
{
  if (line_map_data_list_.count(local_id) == 0) {
    this->PushBack(local_id, line_data);
    return;
  }
  // std::cout << "\033[34m !!!!!!!!!!!!!!!!!! UpdateLineMap: "
  //           << local_id << "\033[0m" << std::endl;
  auto it = line_map_data_list_.find(local_id);
  for (int i = 0; i < static_cast<int>(line_data.size()); i++) {
    // std::cout << "this line:" << std::endl; 
    // std::cout << line_data[i].angle() << std::endl;
    // std::cout << line_data[i].begin().transpose() << std::endl;
    // std::cout << line_data[i].end().transpose() << std::endl;
    // std::cout << "line length: " << line_data[i].length() << std::endl; 
    // std::cout << "before update:" << std::endl; 
    // std::cout << it->second.line_data().angle() << std::endl;
    // std::cout << it->second.line_data().begin().transpose() << std::endl;
    // std::cout << it->second.line_data().end().transpose() << std::endl;
    it->second.UpdateLineData(line_data[i]);

    // std::cout << "after update:" << std::endl;
    // std::cout << it->second.line_data().angle() << std::endl;
    // std::cout << it->second.line_data().begin().transpose() << std::endl;
    // std::cout << it->second.line_data().end().transpose() << std::endl;
    // std::cout << "line length: " << it->second.line_data().length() << std::endl;
    it->second.SetLoseTime(0);
  }

  observations_[local_id]++;
}

void LineMap2DList::SetCullingParam(
                      const double& min_line_length,
                      const double& max_line_length,
                      const int& culling_obs)
{
  min_line_length_ = min_line_length;
  max_line_length_ = max_line_length;
  culling_obs_ = culling_obs;
  LOG(INFO) << "Set Line Map Culling Param: " <<
            "min: " << min_line_length_ <<
            " max: " << max_line_length_ <<
            " obs: " << culling_obs_ << std::endl;
}


void LineMap2DList::LineDataCulling()
{
  std::vector<int> to_remove;
  
  // 收集需要删除的线段ID
  for (const auto& it : line_map_data_list_) {
    const int local_id = it.first;
    const double length = it.second.line_data().length();
    const int inliers = it.second.inliers();
    
    // 检查观测次数
    auto obs_it = observations_.find(local_id);
    const size_t obs_count = (obs_it != observations_.end()) ? obs_it->second : 0;
    
    // 删除条件：观测次数不足、长度超出范围、内点数量过少
    if (obs_count < static_cast<size_t>(culling_obs_) ||
        length < min_line_length_ ||
        length > max_line_length_ ||
        inliers < 3) {  // 添加内点数量检查
      to_remove.push_back(local_id);
    }
  }
  
  // 批量删除
  for (int local_id : to_remove) {
    line_map_data_list_.erase(local_id);
    observations_.erase(local_id);  // 同步清理observations_
  }
  // LOG(INFO) << "Line culling: removed " << to_remove.size() 
  //       << "/" << line_map_data_list_.size() + to_remove.size() << " lines, remaining: " << line_map_data_list_.size()
  //       << " (removal rate: " << (100.0 * to_remove.size() / (line_map_data_list_.size() + to_remove.size())) << "%)";
}

void LinearRegression(const std::vector<double> &xs, const std::vector<double> &ys, double &a, double &b) {
  double xmean = 0.0, ymean = 0.0;  
  for (int i = 0; i < static_cast<int>(xs.size()); ++i) {  
    xmean += xs[i];  
    ymean += ys[i];  
  }  
  xmean /= xs.size();  
  ymean /= xs.size();  

  double varx2 = 0.0, varxy = 0.0;  
  for (int i = 0; i < static_cast<int>(xs.size()); ++i) {  
    varx2 += (xs[i] - xmean) * (xs[i] - xmean);  
    varxy += (ys[i] - ymean) * (xs[i] - xmean);  
  }  

  a = varxy / varx2;  
  b = ymean - a * xmean;  
}


int LineFittingRANSAC(  
  const std::vector<double> &xs, const std::vector<double> &ys,  
  double *a, double *b, double *r,  
  int iterations, double threshold, int minInliers)  
{
  if(xs.size() != ys.size() || xs.size() < 2) {  
    throw std::runtime_error("Insufficient or unequal data points.");  
  }

  // Handle small datasets - use simple linear regression
  if (xs.size() < 4) {
    LinearRegression(xs, ys, *a, *b);
    *r = 1.0;
    return static_cast<int>(xs.size());
  }

  std::random_device rd;  
  std::mt19937 gen(rd());  
  std::uniform_int_distribution<> dis(0, xs.size() - 1);  

  int bestInliers = 0;  
  double bestA = 0.0, bestB = 0.0;
  const double eps = 1e-8;  // For numerical stability
  
  // Adaptive iteration count based on data size
  int adaptiveIterations = std::min(iterations, static_cast<int>(xs.size() * 2));
  
  // Early success ratio for termination
  double targetInlierRatio = 0.8;
  int targetInliers = std::max(minInliers, static_cast<int>(xs.size() * targetInlierRatio));

  for (int i = 0; i < adaptiveIterations; ++i) {  
    // Randomly select two points  
    int idx1 = dis(gen);  
    int idx2 = dis(gen);  
    while (idx2 == idx1) idx2 = dis(gen);  

    // Handle vertical lines and numerical stability
    double dx = xs[idx2] - xs[idx1];
    double dy = ys[idx2] - ys[idx1];
    
    if (std::abs(dx) < eps) {
      // Vertical line - skip this iteration
      continue;
    }

    // Fit line to these points  
    double tempA = dy / dx;
    double tempB = ys[idx1] - tempA * xs[idx1];  
    
    // Validate line parameters
    if (!std::isfinite(tempA) || !std::isfinite(tempB)) {
      continue;
    }

    int inliers = 0;  

    // Count inliers with optimized distance calculation
    for (size_t j = 0; j < xs.size(); ++j) {  
      // Use point-to-line distance for better accuracy
      double dist = std::abs(tempA * xs[j] - ys[j] + tempB) / std::sqrt(tempA * tempA + 1.0);
      if (dist < threshold) {  
        inliers++;  
      }  
    }  

    // Update best model if current one is better  
    if(inliers > bestInliers) {  
      bestInliers = inliers;  
      bestA = tempA;  
      bestB = tempB;
      
      // Early termination if we have enough good inliers
      if (bestInliers >= targetInliers) break;
    }
  }  

  // If no good fit found, fall back to standard linear regression
  if (bestInliers < 2) {
    LinearRegression(xs, ys, *a, *b);
    *r = 0.5;  // Low confidence
    return static_cast<int>(xs.size());
  }

  // Final line fitting using inliers from the best model  
  std::vector<double> inlierXs, inlierYs;  
  for (size_t i = 0; i < xs.size(); ++i) {  
    double dist = std::abs(bestA * xs[i] - ys[i] + bestB) / std::sqrt(bestA * bestA + 1.0);
    if (dist < threshold) {  
      inlierXs.push_back(xs[i]);  
      inlierYs.push_back(ys[i]);  
    }  
  }  

  // Refine fit with inliers only
  if (inlierXs.size() >= 2) {
    LinearRegression(inlierXs, inlierYs, bestA, bestB);  
  }

  *a = bestA;  
  *b = bestB;  

  // Calculate R-squared value (coefficient of determination)
  if (inlierXs.size() < 2) {
    *r = 0.0;
    return bestInliers;
  }
  
  double xmean = 0.0, ymean = 0.0;
  for (size_t i = 0; i < inlierXs.size(); ++i) {  
    xmean += inlierXs[i];  
    ymean += inlierYs[i];  
  }  
  xmean /= inlierXs.size();  
  ymean /= inlierXs.size();  
  
  double ssres = 0.0, sstot = 0.0;  // Sum of squares residual and total
  for (size_t i = 0; i < inlierXs.size(); ++i) {  
    double yPred = bestA * inlierXs[i] + bestB;
    ssres += (inlierYs[i] - yPred) * (inlierYs[i] - yPred);
    sstot += (inlierYs[i] - ymean) * (inlierYs[i] - ymean);
  }  
  
  *r = (sstot > eps) ? std::max(0.0, 1.0 - ssres / sstot) : 0.0;

  return bestInliers;
}  


void LineFitting(
  const std::vector<double> & xs, const std::vector<double> & ys,
  double * a, double * b, double * r)
{
  double xmean = 0.0;
  double ymean = 0.0;
  for (int i = 0; i < static_cast<int>(xs.size()); ++i) {
    xmean += xs[i];
    ymean += ys[i];
  }
  xmean /= xs.size();
  ymean /= xs.size();

  double varx2 = 0.0;
  double vary2 = 0.0;
  double varxy = 0.0;
  for (int i = 0; i < static_cast<int>(xs.size()); i++) {
    varx2 += (xs[i] - xmean) * (xs[i] - xmean);
    vary2 += (ys[i] - ymean) * (ys[i] - ymean);
    varxy += (ys[i] - ymean) * (xs[i] - xmean);
  }
  varx2 /= xs.size();
  vary2 /= xs.size();
  varxy /= xs.size();

  *a = varxy / varx2;
  *b = ymean - *a * xmean;

  *r = fabs(varxy / sqrt(varx2 * vary2));
}

}  // namespace mapping
}  // namespace cartographer
