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

// #include <sensor_msgs/PointCloud.h>
#include "cartographer/laser_perception/visualization.h"
double color_list[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // r, g , b

Visualization::Visualization()
{
//
//  lines_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
//  points_marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("points_marker", 10);
//  reflector_pub_ = n_.advertise<sensor_msgs::PointCloud>("visualization_points", 1);
}

void Visualization::linesAndTextVisual(LinesAndTextInfo lines_text_info)
{
//  visualization_msgs::MarkerArray lines_text_marker;
//
//  visualization_msgs::Marker line_markers, text_marker;
//  line_markers.header.frame_id = text_marker.header.frame_id = lines_text_info.frame;
//  line_markers.header.stamp = text_marker.header.stamp = ros::Time::now();
//  line_markers.ns = text_marker.ns = "lines_and_text";
//  line_markers.action = text_marker.action = visualization_msgs::Marker::ADD;
//  line_markers.pose.orientation.w = text_marker.pose.orientation.w = 1.0;
//
//  line_markers.id = 0;
//  text_marker.id = 1;
//  line_markers.type = visualization_msgs::Marker::LINE_LIST;
//  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//
//  line_markers.scale.x = 0.005;
//  text_marker.scale.z = 0.1;
//
//  line_markers.color.r = text_marker.color.r = color_list[lines_text_info.color][0];
//  line_markers.color.g = text_marker.color.g = color_list[lines_text_info.color][1];
//  line_markers.color.b = text_marker.color.b = color_list[lines_text_info.color][2];
//  line_markers.color.a = text_marker.color.a = 1.0;
//
//  text_marker.text = lines_text_info.text;
//  text_marker.pose.position.x = lines_text_info.text_position.x;
//  text_marker.pose.position.y = lines_text_info.text_position.y;
//
//  for(auto line: lines_text_info.lines) {
//    geometry_msgs::Point p;
//    p.x = line.first.x;
//    p.y = line.first.y;
//    line_markers.points.push_back(p);
//    p.x = line.second.x;
//    p.y = line.second.y;
//    line_markers.points.push_back(p);
//  }
//
//  lines_text_marker.markers.push_back(text_marker);
//  lines_text_marker.markers.push_back(line_markers);
//  lines_marker_pub_.publish(lines_text_marker);
}

void Visualization::pointsAndTextVisual(PointsAndTextInfo points_text_info)
{
//  visualization_msgs::MarkerArray lanmark_map_marker;
//  for(int i = 0; i < points_text_info.points.size(); i++){
//    visualization_msgs::Marker coordinate_markers, corrdinate_text;
//    coordinate_markers.header.frame_id = corrdinate_text.header.frame_id = points_text_info.frame;
//    coordinate_markers.header.stamp = corrdinate_text.header.stamp = points_text_info.time;
//    coordinate_markers.ns = corrdinate_text.ns = "";
//    coordinate_markers.action = corrdinate_text.action = visualization_msgs::Marker::ADD;
//    coordinate_markers.pose.orientation.w = coordinate_markers.pose.orientation.w =
//     corrdinate_text.pose.orientation.w = coordinate_markers.pose.orientation.w= 1.0;
//
//    coordinate_markers.id = i * 2;
//    corrdinate_text.id = i * 2 + 1;
//    coordinate_markers.type = visualization_msgs::Marker::POINTS;
//    corrdinate_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//
//    // POINTS markers use x and y scale for width/height respectively
//    coordinate_markers.scale.x = 0.1;
//    coordinate_markers.scale.y = 0.1;
//
//    // Points are red
//    corrdinate_text.scale.z = 0.1;
//    coordinate_markers.color.r = corrdinate_text.color.r = color_list[points_text_info.color][0];
//    coordinate_markers.color.g = corrdinate_text.color.g = color_list[points_text_info.color][1];
//    coordinate_markers.color.b = corrdinate_text.color.b = color_list[points_text_info.color][2];
//    coordinate_markers.color.a = corrdinate_text.color.a = 1.0;
//    std::ostringstream ss;
//    ss << "( ";
//    ss << points_text_info.points[i].x << ", " << points_text_info.points[i].y << " )";
//    corrdinate_text.text = ss.str();
//
//    corrdinate_text.pose.position.x = points_text_info.points[i].x  + 0.4;
//    corrdinate_text.pose.position.y = points_text_info.points[i].y  + 0.4;
//
//    coordinate_markers.points.clear();
//    geometry_msgs::Point landmark_pose;
//    landmark_pose.x = points_text_info.points[i].x;
//    landmark_pose.y = points_text_info.points[i].y;
//    coordinate_markers.points.push_back(landmark_pose);
//
//    lanmark_map_marker.markers.push_back(coordinate_markers);
//    lanmark_map_marker.markers.push_back(corrdinate_text);
//  }
//  points_marker_pub_.publish(lanmark_map_marker);
}

void Visualization::pointsVisual(PointsAndTextInfo points_info)
{
//  sensor_msgs::PointCloud cloud;
//  cloud.header.stamp = points_info.time;
//  cloud.header.frame_id = points_info.frame;
//  cloud.points.resize(points_info.points.size());
//  cloud.channels.resize(1);
//  cloud.channels[0].name = "intensities";
//  cloud.channels[0].values.resize(points_info.points.size());
//
//  for (int i = 0; i < points_info.points.size(); i++) {
//    cloud.points[i].x = points_info.points[i].x;
//    cloud.points[i].y = points_info.points[i].y;
//    cloud.points[i].z = 0.0;
//    cloud.channels[0].values[i] = 100;
//  }
//  reflector_pub_.publish(cloud);
}
