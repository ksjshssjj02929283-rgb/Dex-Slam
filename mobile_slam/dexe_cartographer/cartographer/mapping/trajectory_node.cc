/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/trajectory_node.h"

#include "Eigen/Core"
#include "cartographer/common/time.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping {

std::vector<sensor::RangeData> GetRangeDatas(const TrajectoryNode::Data& data) {
  if (!data.range_datas.empty()) return data.range_datas;
  std::vector<sensor::RangeData> out;
  out.reserve(data.compressed_range_datas.size());
  for (const auto& compressed : data.compressed_range_datas) {
    out.push_back(sensor::Decompress(compressed));
  }
  return out;
}

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data) {
  proto::TrajectoryNodeData proto;
  proto.set_timestamp(common::ToUniversal(constant_data.time));
  *proto.mutable_gravity_alignment() =
    transform::ToProto(constant_data.gravity_alignment);
  
  // 直接使用合并后的点云进行序列化
  *proto.mutable_filtered_gravity_aligned_point_cloud() =
    sensor::CompressedPointCloud(constant_data.filtered_gravity_aligned_point_cloud).ToProto();
  // 字段 10：高亮点云。FromProto 一直在读它，但此前这里从未写入，加载回来的
  // 节点高亮点云恒为空 —— 离线重建（对齐 / 替换 / 合并）的 highlight_cloud
  // 因此是空的。
  *proto.mutable_highlight_point_cloud() =
    sensor::CompressedPointCloud(constant_data.highlight_point_cloud).ToProto();
  for (const auto & lines : constant_data.lines) {
    for (int i = 0; i < static_cast<int>(lines.second.size()); i++) {
      auto * item = proto.add_lines();
      *item = sensor::ToProto(lines.second[i]);
      // proto.lines.push_back();
    }
  }
  for (const auto & reflector : constant_data.reflectors) {
    auto * item = proto.add_reflectors();
    *item = sensor::ToProto(reflector);
  }
  *proto.mutable_high_resolution_point_cloud() =
    sensor::CompressedPointCloud(constant_data.high_resolution_point_cloud)
      .ToProto();
  *proto.mutable_low_resolution_point_cloud() =
    sensor::CompressedPointCloud(constant_data.low_resolution_point_cloud)
      .ToProto();
  for (Eigen::VectorXf::Index i = 0;
     i != constant_data.rotational_scan_matcher_histogram.size(); ++i) {
  proto.add_rotational_scan_matcher_histogram(
    constant_data.rotational_scan_matcher_histogram(i));
  }
  *proto.mutable_local_pose() = transform::ToProto(constant_data.local_pose);

  // 字段 11：去畸变原始 range data。
  //   - 建图期：range_datas 由前端填入 → 压缩后写入；
  //   - 加载来的节点（定位 / 续建）：range_datas 为空，原样透传压缩数据，
  //     保证定位回写 pbstream（ResetMapOrigin / save_map）时不丢。
  if (!constant_data.range_datas.empty()) {
    for (const auto& range_data : constant_data.range_datas) {
      *proto.add_range_datas() = sensor::ToProto(sensor::Compress(range_data));
    }
  } else {
    for (const auto& compressed : constant_data.compressed_range_datas) {
      *proto.add_range_datas() = sensor::ToProto(compressed);
    }
  }
  return proto;
}

TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto) {
  Eigen::VectorXf rotational_scan_matcher_histogram(
    proto.rotational_scan_matcher_histogram_size());
  for (int i = 0; i != proto.rotational_scan_matcher_histogram_size(); ++i) {
  rotational_scan_matcher_histogram(i) =
    proto.rotational_scan_matcher_histogram(i);
  }
  std::map<int, std::vector<sensor::LineData>> lines;
  for (int i = 0; i != proto.lines_size(); ++i) {
    // 0
    lines[0].push_back(sensor::FromProto(proto.lines(i)));
  }
  std::vector<sensor::LandmarkPointData> reflectors;
  for (int i = 0; i != static_cast<int>(proto.reflectors_size()); ++i) {
    sensor::LandmarkPointData reflector = sensor::FromProto(proto.reflectors(i));
    reflectors.push_back(reflector);
  }

  // range_datas 保持为空：pbstream 里的 range data（字段 11）只以压缩态读入
  // compressed_range_datas，不在这里解压 —— 定位不需要它，解压要多占
  // ≈57 KB/节点。需要时用 GetRangeDatas() 按需解压。
  std::vector<sensor::RangeData> range_datas;
  std::vector<sensor::CompressedRangeData> compressed_range_datas;
  compressed_range_datas.reserve(proto.range_datas_size());
  for (const auto& compressed_proto : proto.range_datas()) {
    compressed_range_datas.push_back(sensor::FromProto(compressed_proto));
  }

  return TrajectoryNode::Data{
    common::FromUniversal(proto.timestamp()),
    transform::ToEigen(proto.gravity_alignment()),
    sensor::CompressedPointCloud(proto.filtered_gravity_aligned_point_cloud())
      .Decompress(),  // 直接使用单个点云
    sensor::CompressedPointCloud(proto.highlight_point_cloud())
      .Decompress(),  // 直接使用单个点云
    range_datas,
    lines,
    reflectors,
    sensor::CompressedPointCloud(proto.high_resolution_point_cloud())
      .Decompress(),
    sensor::CompressedPointCloud(proto.low_resolution_point_cloud())
      .Decompress(),
    rotational_scan_matcher_histogram,
    transform::ToRigid3(proto.local_pose()),
    std::move(compressed_range_datas)};
}

}  // namespace mapping
}  // namespace cartographer
