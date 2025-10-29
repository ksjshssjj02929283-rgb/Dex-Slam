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

#include "cartographer/mapping/internal/2d/scan_matching/line_matcher_2d.h"
#include <map>

namespace cartographer
{
namespace mapping
{
namespace scan_matching
{
static double normalize(double z) {return atan2(sin(z), cos(z));}

double angleDiff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a - b;
  d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {d2 *= -1.0;}
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

proto::LineMatcherOptions CreateLineMatcherOptions2D(
  common::LuaParameterDictionary * parameter_dictionary)
{
  proto::LineMatcherOptions options;
  options.set_line_delta_theta_threshold(
    parameter_dictionary->GetDouble("line_delta_theta_threshold"));
  options.set_line_delta_tra_elongation_threshold(
    parameter_dictionary->GetDouble("line_delta_tra_elongation_threshold"));
  options.set_min_delta_tra_parallel(
    parameter_dictionary->GetDouble("min_delta_tra_parallel"));
  options.set_max_match_distance(
    parameter_dictionary->GetDouble("max_match_distance"));
  return options;
}

Eigen::Vector2d LineMatcher2D::GetCross(
  const sensor::LineData & line_1,
  const sensor::LineData & line_2)
{
  Eigen::Vector2d res;
  res[0] = (line_1.b() - line_2.b()) / (line_2.k() - line_1.k());
  res[1] = line_1.k() * res[0] + line_1.b();
  return res;
}

Eigen::Vector2d LineMatcher2D::ComputeContactPoint(
  const sensor::LineData & line, const Eigen::Vector2d & point)
{
  Eigen::Vector2d res;
  if (line.style() == sensor::VerticalStyle::X) {
    res = Eigen::Vector2d{point[0], line.begin()[1]};
  } else if (line.style() == sensor::VerticalStyle::Y) {
    res = Eigen::Vector2d{line.begin()[0], point[1]};
  } else {
    double dx = line.begin()[0] - line.end()[0];
    double dy = line.begin()[1] - line.end()[1];
    double u =
      (point[0] - line.begin()[0]) * (line.begin()[0] - line.end()[0]) +
      (point[1] - line.begin()[1]) * (line.begin()[1] - line.end()[1]);
    u = u / ((dx * dx) + (dy * dy));
    res[0] = line.begin()[0] + u * dx;
    res[1] = line.begin()[1] + u * dy;
  }
  return res;
}
void LineMatcher2D::TrackLineFromMap(
                      const transform::Rigid2d &pose_prediction,
                      const std::vector<sensor::LineData> &line_data,
                      const LineMap2DList &line_map_2d,
                      std::map<int, std::vector<sensor::LineData>>& tracked_line_data)
{
  if (line_data.size() == 0) {
    return;
  }
  int new_id = -1;
  // 对于每一个检测到的线段，从active_submaps_的line_map_2d_list中找到最近的line
  for (size_t i = 0; i < line_data.size(); i++) {
    int local_id;
    sensor::LineData new_data(line_data[i]);
    if (new_data.begin().norm() > options_.max_match_distance() || 
        new_data.end().norm() > options_.max_match_distance()) {
      continue;
    }
    if (GetNearestLaneLine(pose_prediction, line_data[i],
                           line_map_2d, local_id, new_data))
    {
      // 如果找到，就使用匹配线段的ID
      tracked_line_data[local_id].push_back(new_data);
    } else {
      // 如果没找到，就创建新的ID
      tracked_line_data[new_id] = std::vector<sensor::LineData> {new_data};
      new_id--;
    }
  }
  // LOG(INFO) << "line data detected: " << line_data.size();
  // LOG(INFO) << "new_id size: " << new_id;
  // std::cout << "trakced line id:" << std::endl;
  // for (auto it = tracked_line_data.begin(); it != tracked_line_data.end(); it++) {
  //   std::cout << it->first << " ";
  // }
  // std::cout << std::endl;
}

void LineMatcher2D::TrackLineFromMap(
                      const transform::Rigid2d &pose_prediction,
                      const std::map<int, std::vector<sensor::LineData>>& map_line_data,
                      const LineMap2DList &line_map_2d,
                      std::map<int, std::vector<sensor::LineData>>& tracked_line_data)
{
  if (map_line_data.size() == 0) {
    return;
  }
  int new_id = -1;
  // 对于每一个检测到的线段，从active_submaps_的line_map_2d_list中找到最近的line
  for (const auto &it : map_line_data) {
    for (size_t i = 0; i < it.second.size(); i++) {
      int local_id;
      sensor::LineData new_data(it.second[i]);
      if (new_data.begin().norm() > options_.max_match_distance() || 
          new_data.end().norm() > options_.max_match_distance()) {
        continue;
      }
      if (GetNearestLaneLine(pose_prediction, it.second[i],
                             line_map_2d, local_id, new_data))
      {
        // 如果找到，就使用匹配线段的ID
        tracked_line_data[local_id].push_back(new_data);
      } else {
        // 如果没找到，就创建新的ID
        tracked_line_data[new_id] = std::vector<sensor::LineData> {new_data};
        new_id--;
      }
    }
  }

  // LOG(INFO) << "line data detected: " << line_data.size();
  // LOG(INFO) << "new_id size: " << new_id;
  // std::cout << "\033[32m trakced line id:" << std::endl;
  // for (auto it = tracked_line_data.begin(); it != tracked_line_data.end(); it++) {
  //   std::cout << it->first << " ";
  // }
  // std::cout << std::endl;
}

// bool LineMatcher2D::GetNearestLaneLine(const transform::Rigid2d
// &pose_prediction,
//                                        const sensor::LineData &data,
//                                        const GlobalLineMap &global_line_map,
//                                        int &global_id) {
//   double min_delta_tra_parallel = options_.min_delta_tra_parallel();
//   bool have_res = false;
//   const std::vector<MapLine2D>& line_map_list = global_line_map.GetLineMap();
//   sensor::LineData data_to_local = sensor::TransformLineData(pose_prediction,
//   data); for (int i = 0; i < line_map_list.size(); i++) {
//     double delta_theta, delta_tra_parallel, delta_tra_elongation;
//     delta_theta = std::min(fabs(angleDiff(data_to_local.angle(),
//     line_map_list[i].line_data().angle())),
//                            fabs(angleDiff(data_to_local.angle() + M_PI,
//                            line_map_list[i].line_data().angle())));
//     if (delta_theta > options_.line_delta_theta_threshold()) {
//       continue;
//     }

//     Eigen::Vector2d contact_point_begin =
//     ComputeContactPoint(line_map_list[i].line_data(), data_to_local.begin());
//     Eigen::Vector2d contact_point_end =
//     ComputeContactPoint(line_map_list[i].line_data(), data_to_local.end());
//     if ((contact_point_begin[0] >
//     std::max(line_map_list[i].line_data().begin()[0],
//     line_map_list[i].line_data().end()[0]) &&
//          contact_point_end[0] >
//          std::max(line_map_list[i].line_data().begin()[0],
//          line_map_list[i].line_data().end()[0])) ||
//         (contact_point_begin[0] <
//         std::min(line_map_list[i].line_data().begin()[0],
//         line_map_list[i].line_data().end()[0]) &&
//          contact_point_end[0] <
//          std::min(line_map_list[i].line_data().begin()[0],
//          line_map_list[i].line_data().end()[0]))) {
//       delta_tra_elongation = std::min(std::min((contact_point_begin -
//       line_map_list[i].line_data().begin()).norm(),
//                                                (contact_point_begin -
//                                                line_map_list[i].line_data().end()).norm()),
//                                       std::min((contact_point_end -
//                                       line_map_list[i].line_data().begin()).norm(),
//                                                (contact_point_end -
//                                                line_map_list[i].line_data().end()).norm()));
//     } else {
//       delta_tra_elongation = 0;
//     }

//     if (delta_tra_elongation >
//     options_.line_delta_tra_elongation_threshold()) {
//       continue;
//     }

//     delta_tra_parallel = ((data_to_local.begin() -
//     contact_point_begin).norm() +
//                           (data_to_local.end() - contact_point_end).norm()) /
//                           2;
//     if (delta_tra_parallel < min_delta_tra_parallel) {
//       min_delta_tra_parallel = delta_tra_parallel;
//       have_res = true;
//       global_id = line_map_list[i].global_id();
//     }
//   }
//   return have_res;
// }

bool LineMatcher2D::GetNearestLaneLine(
  const transform::Rigid2d & pose_prediction, const sensor::LineData & data,
  const LineMap2DList & line_map_2d, int & local_id)
{
  double min_delta_tra_parallel = options_.min_delta_tra_parallel();
  const std::map<int, MapLine2D> & line_map_list =
    line_map_2d.GetLineMapData2DList();
  bool have_res = false;
  sensor::LineData data_to_local =
    sensor::TransformLineData(pose_prediction, data);
  // std::cout << "\033[32m data_to_local begin: " << data_to_local.begin()[0]
  // << " " << data_to_local.begin()[1] << " end: "
  //           << data_to_local.end()[0] << " " << data_to_local.end()[1] <<
  //           "\033[0m" << std::endl;
  for (auto it : line_map_list) {
    // std::cout << "\033[36m begin: " << it.second.line_data().begin()[0] << "
    // " << it.second.line_data().begin()[1] << " end: "
    //           << it.second.line_data().end()[0] << " " <<
    //           it.second.line_data().end()[1] << "\033[0m" << std::endl;
    double delta_theta, delta_tra_parallel, delta_tra_elongation;
    delta_theta = std::min(
      fabs(angleDiff(data_to_local.angle(), it.second.line_data().angle())),
      fabs(
        angleDiff(
          data_to_local.angle() + M_PI,
          it.second.line_data().angle())));
    // std::cout << "delta_theta: " << delta_theta << std::endl;
    if (delta_theta > options_.line_delta_theta_threshold()) {
      continue;
    }
    // normal
    Eigen::Vector2d normal1 =
      data_to_local.normal(pose_prediction.translation());
    Eigen::Vector2d normal2 =
      it.second.line_data().normal(pose_prediction.translation());
    if (normal1.dot(normal2) < 0) {
      continue;
    }

    Eigen::Vector2d contact_point_begin =
      ComputeContactPoint(it.second.line_data(), data_to_local.begin());
    Eigen::Vector2d contact_point_end =
      ComputeContactPoint(it.second.line_data(), data_to_local.end());
    // std::cout << "contact_point_begin: " << contact_point_begin[0] << " " <<
    // contact_point_begin[1] << std::endl; std::cout << "contact_point_end: "
    // << contact_point_end[0] << " " << contact_point_end[1] << std::endl;
    if ((contact_point_begin[0] > std::max(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0]) &&
      contact_point_end[0] > std::max(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0])) ||
      (contact_point_begin[0] < std::min(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0]) &&
      contact_point_end[0] < std::min(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0])))
    {
      // delta_tra_elongation = 0;
      delta_tra_elongation = std::min(
        std::min(
          (contact_point_begin - it.second.line_data().begin()).norm(),
          (contact_point_begin - it.second.line_data().end()).norm()),
        std::min(
          (contact_point_end - it.second.line_data().begin()).norm(),
          (contact_point_end - it.second.line_data().end()).norm()));
    } else {
      delta_tra_elongation = 0;
    }

    // std::cout << "delta_tra_elongation: " << delta_tra_elongation <<
    // std::endl;
    if (delta_tra_elongation > options_.line_delta_tra_elongation_threshold()) {
      continue;
    }

    delta_tra_parallel = ((data_to_local.begin() - contact_point_begin) +
      (data_to_local.end() - contact_point_end)).norm() / 2;

    // std::cout << "delta_tra_parallel: " << delta_tra_parallel << std::endl;
    if (delta_tra_parallel < min_delta_tra_parallel) {
      min_delta_tra_parallel = delta_tra_parallel;
      have_res = true;
      local_id = it.first;
    }
  }
#ifdef TEST_DEBUG
  if (have_res == true) {
    const sensor::LineData target =
      line_map_list.find(local_id)->second.line_data();
    std::cout << "target begin: " << target.begin()[0] << " " <<
      target.begin()[1] << " end: " << target.end()[0] << " " <<
      target.end()[1] << std::endl;
    std::cout << "begin: " << data_to_local.begin()[0] << " " <<
      data_to_local.begin()[1] << " end: " << data_to_local.end()[0] <<
      " " << data_to_local.end()[1] << std::endl;
  }
#endif
  return have_res;
}

bool LineMatcher2D::GetNearestLaneLine(
  const transform::Rigid2d & pose_prediction, const sensor::LineData & data,
  const LineMap2DList & line_map_2d, int & local_id,
  sensor::LineData & new_data)
{
  new_data = data;
  double min_delta_tra_parallel = options_.min_delta_tra_parallel();
  const std::map<int, MapLine2D> & line_map_list =
    line_map_2d.GetLineMapData2DList();
  bool have_res = false;
  // 将LineData转换到局部坐标系下
  sensor::LineData data_to_local =
    sensor::TransformLineData(pose_prediction, data);
  // std::cout << "\033[32m data_to_local begin: " << data_to_local.begin()[0]
  // << " " << data_to_local.begin()[1] << " end: "
  //           << data_to_local.end()[0] << " " << data_to_local.end()[1] <<
  //           "\033[0m" << std::endl;
  // 遍历line map中的每一个线段
  for (auto it : line_map_list) {
    // std::cout << "\033[36m begin: " << it.second.line_data().begin()[0] << "
    // " << it.second.line_data().begin()[1] << " end: "
    //           << it.second.line_data().end()[0] << " " <<
    //           it.second.line_data().end()[1] << "\033[0m" << std::endl;
    double delta_theta, delta_tra_parallel, delta_tra_elongation;
    // 计算当前线段与line map线段的夹角
    delta_theta = std::min(
      fabs(angleDiff(data_to_local.angle(), it.second.line_data().angle())),
      fabs(
        angleDiff(
          data_to_local.angle() + M_PI,
          it.second.line_data().angle())));
    // std::cout << "delta_theta: " << delta_theta << std::endl;
    if (delta_theta > options_.line_delta_theta_threshold()) {
      continue;
    }
    // 两个线段的法向量方向相反，则跳过
    // normal
    Eigen::Vector2d normal1 =
      data_to_local.normal(pose_prediction.translation());
    Eigen::Vector2d normal2 =
      it.second.line_data().normal(pose_prediction.translation());
    if (normal1.dot(normal2) < 0) {
      continue;
    }

    // 当前线段在线地图中的起止点的投影, 就是contact point
    Eigen::Vector2d contact_point_begin =
      ComputeContactPoint(it.second.line_data(), data_to_local.begin());
    Eigen::Vector2d contact_point_end =
      ComputeContactPoint(it.second.line_data(), data_to_local.end());
    // std::cout << "contact_point_begin: " << contact_point_begin[0] << " " <<
    // contact_point_begin[1] << std::endl; std::cout << "contact_point_end: "
    // << contact_point_end[0] << " " << contact_point_end[1] << std::endl;
    if ((contact_point_begin[0] > std::max(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0]) &&
      contact_point_end[0] > std::max(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0])) ||
      (contact_point_begin[0] < std::min(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0]) &&
      contact_point_end[0] < std::min(
        it.second.line_data().begin()[0],
        it.second.line_data().end()[0])))
    {
      // delta_tra_elongation = 0;
      delta_tra_elongation = std::min(
        std::min(
          (contact_point_begin - it.second.line_data().begin()).norm(),
          (contact_point_begin - it.second.line_data().end()).norm()),
        std::min(
          (contact_point_end - it.second.line_data().begin()).norm(),
          (contact_point_end - it.second.line_data().end()).norm()));
    } else {
      delta_tra_elongation = 0;
    }

    // std::cout << "delta_tra_elongation: " << delta_tra_elongation <<
    // std::endl;
    if (delta_tra_elongation > options_.line_delta_tra_elongation_threshold()) {
      continue;
    }

    delta_tra_parallel = ((data_to_local.begin() - contact_point_begin) +
      (data_to_local.end() - contact_point_end)).norm() / 2;

    // std::cout << "delta_tra_parallel: " << delta_tra_parallel << std::endl;
    if (delta_tra_parallel < min_delta_tra_parallel) {
      min_delta_tra_parallel = delta_tra_parallel;
      Eigen::Vector3d matching_dis = Eigen::Vector3d{
        delta_tra_parallel, delta_tra_elongation, delta_theta};
      new_data.SetMatchingDis(matching_dis);
      have_res = true;
      local_id = it.first;
    }
  }
#ifdef TEST_DEBUG
  if (have_res == true) {
    const sensor::LineData target =
      line_map_list.find(local_id)->second.line_data();
    std::cout << "target begin: " << target.begin()[0] << " " <<
      target.begin()[1] << " end: " << target.end()[0] << " " <<
      target.end()[1] << std::endl;
    std::cout << "begin: " << data_to_local.begin()[0] << " " <<
      data_to_local.begin()[1] << " end: " << data_to_local.end()[0] <<
      " " << data_to_local.end()[1] << std::endl;
  }
#endif
  return have_res;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

