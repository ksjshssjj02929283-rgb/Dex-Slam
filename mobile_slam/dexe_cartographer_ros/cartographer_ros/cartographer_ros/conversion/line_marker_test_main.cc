/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// 线特征 Marker 发布的回归测试（standalone，运行：
// `ros2 run cartographer_ros line_marker_test`）。
//
// 起因：RViz 订阅 ~/matched_feature 时反复报
//   Duplicate Marker Check: Multiple Markers in the same MarkerArray message
//   had the same ns and id: (global_line, 14)
// 根因是 line_data_in_local 的类型是 map<跟踪 id, vector<线段>>——一个跟踪 id
// 在一帧里可能对应多条观测线段，而旧实现给每条线段单独发一个 Marker 且都把
// 跟踪 id 当作 marker.id，于是同一个 MarkerArray 内 (ns, id) 重复。

#include "cartographer_ros/conversion/msg_conversion.h"

#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/line_data.h"
#include "cartographer_ros/common/node_constants.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {

using Line = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

bool Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

// 复刻 localization_node / node 发布 matched_feature 时的摊平逻辑。
visualization_msgs::msg::MarkerArray BuildMatchedFeatureMsg(
    const std::map<int, std::vector<::cartographer::sensor::LineData>>&
        line_data_in_local) {
  std::vector<Line> lines;
  for (const auto& line_element : line_data_in_local) {
    for (const auto& line : line_element.second) {
      lines.emplace_back(line.begin(), line.end());
    }
  }
  visualization_msgs::msg::MarkerArray msg;
  msg.markers.push_back(cartographer_ros::CreateLineListMsg(
      rclcpp::Time(42, 0), "map", cartographer_ros::kMatchedLineMarkerNs,
      /*id=*/0, lines, cartographer_ros::COLOR::green));
  return msg;
}

bool TestNoDuplicateMarkerIds() {
  // 跟踪 id 14 带两条观测线段——正是现场报错的那种输入。
  std::map<int, std::vector<::cartographer::sensor::LineData>> line_data;
  line_data[14].emplace_back(Eigen::Vector2d(0., 0.), Eigen::Vector2d(1., 0.));
  line_data[14].emplace_back(Eigen::Vector2d(2., 0.), Eigen::Vector2d(3., 0.));
  line_data[2].emplace_back(Eigen::Vector2d(0., 1.), Eigen::Vector2d(1., 1.));

  const auto msg = BuildMatchedFeatureMsg(line_data);

  std::set<std::pair<std::string, int>> seen;
  for (const auto& marker : msg.markers) {
    if (!Expect(seen.insert({marker.ns, marker.id}).second,
                "duplicate (ns, id): (" + marker.ns + ", " +
                    std::to_string(marker.id) + ")")) {
      return false;
    }
  }
  // 3 条线段打包进 1 个 LINE_LIST，共 6 个端点。
  return Expect(msg.markers.size() == 1u,
                "expected a single LINE_LIST marker, got " +
                    std::to_string(msg.markers.size())) &&
         Expect(msg.markers.front().points.size() == 6u,
                "expected 6 endpoints, got " +
                    std::to_string(msg.markers.front().points.size())) &&
         Expect(msg.markers.front().type ==
                    visualization_msgs::msg::Marker::LINE_LIST,
                "marker type is not LINE_LIST") &&
         Expect(msg.markers.front().action ==
                    visualization_msgs::msg::Marker::ADD,
                "marker action is not ADD");
}

bool TestUsesProvidedStamp() {
  const rclcpp::Time stamp(1234, 5678);
  const auto marker = cartographer_ros::CreateLineListMsg(
      stamp, "map", cartographer_ros::kMatchedLineMarkerNs, /*id=*/7,
      {Line(Eigen::Vector2d(0., 0.), Eigen::Vector2d(1., 1.))});
  return Expect(marker.header.stamp.sec == 1234,
                "stamp.sec was not taken from the caller") &&
         Expect(marker.header.stamp.nanosec == 5678u,
                "stamp.nanosec was not taken from the caller") &&
         Expect(marker.header.frame_id == "map", "frame_id mismatch") &&
         Expect(marker.ns == cartographer_ros::kMatchedLineMarkerNs,
                "ns mismatch") &&
         Expect(marker.id == 7, "id mismatch");
}

// 空点集的 LINE_LIST 会让 RViz 报错，而发空 MarkerArray 又清不掉上一帧残留，
// 所以没有有效线段时必须发 DELETE。
bool TestDeletesWhenNoValidLines() {
  const auto empty = cartographer_ros::CreateLineListMsg(
      rclcpp::Time(), "map", cartographer_ros::kMatchedLineMarkerNs, /*id=*/0,
      {});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto all_nan = cartographer_ros::CreateLineListMsg(
      rclcpp::Time(), "map", cartographer_ros::kMatchedLineMarkerNs, /*id=*/0,
      {Line(Eigen::Vector2d(nan, 0.), Eigen::Vector2d(1., 1.))});
  return Expect(empty.action == visualization_msgs::msg::Marker::DELETE,
                "empty line set did not produce a DELETE marker") &&
         Expect(empty.points.empty(), "DELETE marker carries points") &&
         Expect(all_nan.action == visualization_msgs::msg::Marker::DELETE,
                "all-NaN line set did not produce a DELETE marker");
}

bool TestSkipsNonFiniteLines() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto marker = cartographer_ros::CreateLineListMsg(
      rclcpp::Time(), "map", cartographer_ros::kMatchedLineMarkerNs, /*id=*/0,
      {Line(Eigen::Vector2d(0., 0.), Eigen::Vector2d(1., 1.)),
       Line(Eigen::Vector2d(nan, 0.), Eigen::Vector2d(1., 1.)),
       Line(Eigen::Vector2d(0., 0.), Eigen::Vector2d(inf, 1.)),
       Line(Eigen::Vector2d(2., 2.), Eigen::Vector2d(3., 3.))});
  // 只保留 2 条有效线段；端点数必须成对，否则 RViz 丢弃整个 Marker。
  return Expect(marker.points.size() == 4u,
                "expected 4 endpoints after dropping non-finite lines, got " +
                    std::to_string(marker.points.size())) &&
         Expect(marker.points.size() % 2 == 0u,
                "LINE_LIST endpoint count must be even") &&
         Expect(marker.action == visualization_msgs::msg::Marker::ADD,
                "marker action is not ADD");
}

}  // namespace

int main() {
  const bool success = TestNoDuplicateMarkerIds() && TestUsesProvidedStamp() &&
                       TestDeletesWhenNoValidLines() &&
                       TestSkipsNonFiniteLines();
  if (!success) {
    return 1;
  }

  std::cout << "All line marker tests passed." << std::endl;
  return 0;
}
