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
#ifndef CARTOGRAPHER__MAPPING__2D__LINE_MAP_2D_HPP_
#define CARTOGRAPHER__MAPPING__2D__LINE_MAP_2D_HPP_
#include "cartographer/sensor/line_data.h"
#include <vector>
#include <map>
#include <array>
#include <utility>
#include <limits>
#include <algorithm>
#include <cmath>

namespace cartographer
{
namespace mapping
{
void LinearRegression(const std::vector<double> &xs, const std::vector<double> &ys, double &a, double &b);
// 正交回归（TLS / PCA 主轴）拟合，输出契约与 LinearRegression 相同。
// 详见 line_map_2d.cc 的实现注释。
void OrthogonalRegression(const std::vector<double> &xs, const std::vector<double> &ys, double &a, double &b);
int LineFittingRANSAC(
  const std::vector<double> &xs, const std::vector<double> &ys,  
  double *a, double *b, double *r,  
  int iterations = 200, double threshold = 0.1, int minInliers = 20);
void LineFitting(
  const std::vector<double> & xs, const std::vector<double> & ys,
  double * a, double * b, double * r);

class MapLine2D
{
private:
  /* data */
  int global_id_, local_id_;
  sensor::LineData line_data_;
  std::vector<sensor::LineData> line_data_list_;
  int lose_time_;
  // 两个构造函数都不在初始化列表里给它赋值，而 LineDataCulling() 以
  // `inliers < 3` 判剔除 —— 不给默认值就会拿栈上残留值去做剔除判定。
  int inliers_ = 0;

public:
  enum LineMapTpye
  {
    static_line,
    updata,
  };
  MapLine2D(const int local_id, const sensor::LineData & line_data);
  MapLine2D(
    LineMapTpye type, const int global_id,
    const sensor::LineData & line_data);
  ~MapLine2D() {}
  const sensor::LineData & line_data() const {return line_data_;}
  const std::vector<sensor::LineData>& line_data_list() const
  {
    return line_data_list_;
  }
  const int local_id() const {return local_id_;}
  const int global_id() const {return global_id_;}
  int lose_time() const {return lose_time_;}

  const int inliers() const {return inliers_;}

  void SetLoseTime(int lose_time) {lose_time_ = lose_time;}
  void SetLocalId(int local_id) {local_id_ = local_id;}
  void SetInliers(int inliers) {inliers_ = inliers;}

  void UpdateLineData(const sensor::LineData & line_data);

private:
  // 优化的端点计算方法
  bool ComputeOptimizedEndpoints(const std::vector<Eigen::Vector2d>& points,
                                double k, double b,
                                Eigen::Vector2d& start_point,
                                Eigen::Vector2d& end_point) const;
  
  // 点投影到直线
  Eigen::Vector2d ProjectPointToLine(const Eigen::Vector2d& point, 
                                    double k, double b) const;
  
  // 简单端点计算（回退方案）
  sensor::LineData ComputeEndpointsFromFit(const std::vector<Eigen::Vector2d>& points,
                                          double k, double b) const;
};

class LineMap2DList
{
public:
  // 每桶 5°，搜索 ±2 桶覆盖 ±10°（等于最大角度阈值）
  static constexpr int kAngleBuckets = 36;

  // 将角度（任意弧度）映射到 [0, kAngleBuckets) 的桶编号
  // NaN/Inf 安全：异常值统一映射到桶 0，避免越界访问
  static int AngleToBucket(double angle) {
    if (!std::isfinite(angle)) return 0;
    // 线段方向无正负之分，归一化到 [0, π)
    double a = std::fmod(angle, M_PI);
    if (a < 0) a += M_PI;
    int b = static_cast<int>(a / M_PI * kAngleBuckets);
    return std::min(b, kAngleBuckets - 1);
  }

  // 返回指定桶的 local_id 列表（供 line_matcher_2d.cc 使用）
  const std::vector<int>& GetAngleBucket(int bucket) const {
    return angle_index_[bucket];
  }

  // 检查角度索引是否与 line_map_data_list_ 保持一致
  // 若返回 false 则说明索引未初始化或过时，调用方应回退到全量遍历
  bool is_angle_index_valid() const {
    return local_id_to_bucket_.size() == line_map_data_list_.size();
  }

private:
  /* data */
  int latest_local_id_;
  std::map<int, MapLine2D> line_map_data_list_;
  // 该线段被观测的次数
  std::map<int, size_t> observations_;
  // std::map<int, MapLine2D> active_line_list_;

  double min_line_length_ = 0.4;
  double max_line_length_ = 7.0;
  int culling_obs_ = 3;

  // 角度分桶索引：将线段方向归一化到 [0, π)，分 kAngleBuckets 个桶
  std::array<std::vector<int>, kAngleBuckets> angle_index_;  // bucket → local_id list
  std::map<int, int> local_id_to_bucket_;  // local_id → bucket，用于 O(1) 查表删除

public:
  LineMap2DList() = default;
  LineMap2DList(int latest_local_id, std::map<int, MapLine2D> active_line_data);
  LineMap2DList(
    const std::vector<cartographer::sensor::LaneData> & laser_line_map_2d);
  LineMap2DList(
    const std::vector<cartographer::sensor::LaneDataWithNormal> &
    laser_line_map_2d);
  ~LineMap2DList();
  const std::map<int, MapLine2D> & GetLineMapData2DList() const
  {
    return line_map_data_list_;
  }

  const std::map<int, size_t> & GetObservations() const
  {
    return observations_;
  }

  void SetCullingParam(const double& min_line_length,
                       const double& max_line_length,
                       const int& culling_obs);

  // 当前生效的 culling 判据。深拷贝出的 submap 需要据此继承源 submap 的判据，
  // 否则退回本类默认值，与建图期 lua 配置不一致（见 Submap2D::InheritLineCullingParamFrom）。
  double min_line_length() const {return min_line_length_;}
  double max_line_length() const {return max_line_length_;}
  int culling_obs() const {return culling_obs_;}

  size_t size() const {return line_map_data_list_.size();}
  void UpdateLoseTime();
  int latest_local_id() const {return latest_local_id_;}
  void PushBack(
    std::vector<sensor::LineData> line_data);
  void PushBack(
    int local_id, std::vector<sensor::LineData> line_data);
  void UpdateLineMap(
    int local_id, std::vector<sensor::LineData> line_data);
  void LineDataCulling();

  std::map<int, MapLine2D> GetActiveLineData() const
  {
    std::map<int, MapLine2D> res;
    for (auto it : line_map_data_list_) {
      int lose_time = it.second.lose_time();
      if (lose_time < 5) {
        res.insert(std::pair<int, MapLine2D>(it.first, it.second));
      }
    }

    for (auto it = res.begin(); it != res.end();) {
      if (res.size() < 20) {
        break;
      }
      res.erase(it++);
    }
    return res;
  }
  //   bool getNextestLine();

private:
  void AddToAngleIndex(int local_id, double angle);
  void RemoveFromAngleIndex(int local_id);
  void UpdateAngleIndex(int local_id, double new_angle);
};

}  // namespace mapping
}  // namespace cartographer


#endif  // CARTOGRAPHER__MAPPING__2D__LINE_MAP_2D_HPP_
