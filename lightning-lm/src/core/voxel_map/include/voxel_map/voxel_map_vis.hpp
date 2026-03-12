/**
 * @file voxel_map_vis.hpp
 * @brief VoxelMap ROS2 可视化工具
 *
 * 将 VoxelMapManager 导出的 PlaneVis 转换为 ROS2 MarkerArray 消息。
 * 这是唯一依赖 ROS2 的文件，核心 voxel_map 模块不依赖 ROS。
 */
#pragma once

#include "voxel_map/voxel_map_manager.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <string>

namespace lightning {
namespace voxel_map {

/**
 * @brief Jet 颜色映射 (v in [vmin, vmax] -> RGB)
 */
inline void mapJet(double v, double vmin, double vmax, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = g = b = 255;
    if (v < vmin) v = vmin;
    if (v > vmax) v = vmax;
    double dr, dg, db;
    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    } else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    } else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    } else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    } else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }
    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

/**
 * @brief 从 PlaneVis 生成单个平面的 Marker（CYLINDER 类型）
 */
inline visualization_msgs::msg::Marker createPlaneMarker(const PlaneVis& plane, const std::string& frame_id,
                                                         const std::string& ns, float alpha,
                                                         const Eigen::Vector3d& rgb) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = plane.id;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = plane.center[0];
    marker.pose.position.y = plane.center[1];
    marker.pose.position.z = plane.center[2];

    // 从法向量和轴向量构建旋转四元数
    Eigen::Matrix3d rot;
    rot << plane.x_normal(0), plane.x_normal(1), plane.x_normal(2), plane.y_normal(0), plane.y_normal(1),
        plane.y_normal(2), plane.normal(0), plane.normal(1), plane.normal(2);
    Eigen::Matrix3d rotation = rot.transpose();
    Eigen::Quaterniond eq(rotation);

    marker.pose.orientation.w = eq.w();
    marker.pose.orientation.x = eq.x();
    marker.pose.orientation.y = eq.y();
    marker.pose.orientation.z = eq.z();

    marker.scale.x = 3 * std::sqrt(plane.max_eigen_value);
    marker.scale.y = 3 * std::sqrt(plane.mid_eigen_value);
    marker.scale.z = 2 * std::sqrt(plane.min_eigen_value);

    marker.color.a = alpha;
    marker.color.r = rgb(0);
    marker.color.g = rgb(1);
    marker.color.b = rgb(2);

    return marker;
}

/**
 * @brief 将 PlaneVis 列表转换为 MarkerArray
 *
 * 用法示例:
 *   auto planes = voxel_map_manager->GetUpdatedPlanesForVis();
 *   auto marker_array = toMarkerArray(planes, "map", "voxel_plane");
 *   publisher->publish(marker_array);
 */
inline visualization_msgs::msg::MarkerArray toMarkerArray(const std::vector<PlaneVis>& planes,
                                                          const std::string& frame_id = "map",
                                                          const std::string& ns = "voxel_plane",
                                                          double max_trace = 0.25, double pow_num = 0.2,
                                                          float use_alpha = 0.8) {
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.reserve(planes.size());

    for (const auto& p : planes) {
        double trace = p.plane_cov_trace;
        if (trace >= max_trace) trace = max_trace;
        trace = trace * (1.0 / max_trace);
        trace = std::pow(trace, pow_num);

        uint8_t r, g, b;
        mapJet(trace, 0, 1, r, g, b);
        Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);

        float alpha = p.is_plane ? use_alpha : 0.0f;
        auto marker = createPlaneMarker(p, frame_id, ns, alpha, plane_rgb);
        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

}  // namespace voxel_map
}  // namespace lightning
