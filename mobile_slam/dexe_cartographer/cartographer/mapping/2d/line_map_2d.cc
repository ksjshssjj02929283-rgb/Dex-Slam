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
  if (line_data_list_.size() > 200) {
    line_data_list_.erase(line_data_list_.begin(), line_data_list_.begin() + 100);
  }
  // 根据所有的xy拟合出直线的参数，然后更新直线的端点
  Eigen::Vector2d max_point = {-1e5, -1e5}, min_point = {1e5, 1e5};
  std::vector<double> xs;
  std::vector<double> ys;
  for (int i = 0; i < static_cast<int>(line_data_list_.size()); i++) {
    xs.push_back(line_data_list_[i].begin()[0]);
    ys.push_back(line_data_list_[i].begin()[1]);
    if (line_data_list_[i].begin()[0] > max_point[0]) {
      max_point = line_data_list_[i].begin();
    }
    if (line_data_list_[i].begin()[0] < min_point[0]) {
      min_point = line_data_list_[i].begin();
    }
    xs.push_back(line_data_list_[i].end()[0]);
    ys.push_back(line_data_list_[i].end()[1]);
    if (line_data_list_[i].end()[0] > max_point[0]) {
      max_point = line_data_list_[i].end();
    }
    if (line_data_list_[i].end()[0] < min_point[0]) {
      min_point = line_data_list_[i].end();
    }
  }
  double k, b, r;
  // LineFitting(xs, ys, &k, &b, &r);
  inliers_ = LineFittingRANSAC(xs, ys, &k, &b, &r);
  max_point[1] = k * max_point[0] + b;
  min_point[1] = k * min_point[0] + b;
  line_data_ = sensor::LineData(max_point, min_point);
  lose_time_ = 0;
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
  for (auto it = line_map_data_list_.begin(); it != line_map_data_list_.end();)
  {
    const int local_id = it->first;
    const double length = it->second.line_data().length();
    const int inliers = it->second.inliers();
    if (observations_[local_id] < culling_obs_ ||
        length < min_line_length_ ||
        length > max_line_length_)
    {
      it = line_map_data_list_.erase(it);
    } else {
      ++it;
    }
  }
  LOG(INFO) << "line size after culling : " << line_map_data_list_.size() << std::endl;
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
