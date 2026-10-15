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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory_node_data.pb.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace mapping {

struct TrajectoryNodePose {
  struct ConstantPoseData {
    common::Time time;
    transform::Rigid3d local_pose;
  };
  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;

  absl::optional<ConstantPoseData> constant_pose_data;
};

struct TrajectoryNode {
  struct Data {
    common::Time time;

    // Transform to approximately gravity align the tracking frame as
    // determined by local SLAM.
    Eigen::Quaterniond gravity_alignment;

    // Used for loop closure in 2D: voxel filtered returns in the
    // 'gravity_alignment' frame.
    sensor::PointCloud filtered_gravity_aligned_point_cloud;
    sensor::PointCloud highlight_point_cloud;
    std::vector<sensor::RangeData> range_datas;

    std::map<int, std::vector<sensor::LineData>> lines;  // line data with id
    std::vector<sensor::LandmarkPointData> reflectors;

    // Used for loop closure in 3D.
    sensor::PointCloud high_resolution_point_cloud;
    sensor::PointCloud low_resolution_point_cloud;
    Eigen::VectorXf rotational_scan_matcher_histogram;

    // The node pose in the local SLAM frame.
    transform::Rigid3d local_pose;

    // 从 pbstream 加载来的 range data，保持压缩态、不解压。
    // range_datas（上面）只在建图期由前端填入；从 pbstream 加载的节点那一项为空，
    // 真数据留在这里。之所以不在加载时解压：定位根本不用它，解压后每节点常驻
    // ≈57 KB（2000 节点 ≈115 MB）。之所以又不能直接丢掉：定位侧会回写 pbstream
    // （ResetMapOrigin / save_map），丢了就永久没了 —— 所以原样保留、写回时透传。
    // 需要用时（重新生成全局子图）由调用方按需 sensor::Decompress。
    // 放在结构体末尾：现有按位置的 brace 初始化不需要改。
    std::vector<sensor::CompressedRangeData> compressed_range_datas;
  };

  common::Time time() const { return constant_data->time; }

  // This must be a shared_ptr. If the data is used for visualization while the
  // node is being trimmed, it must survive until all use finishes.
  std::shared_ptr<const Data> constant_data;

  // The node pose in the global SLAM frame.
  transform::Rigid3d global_pose;
};

proto::TrajectoryNodeData ToProto(const TrajectoryNode::Data& constant_data);
TrajectoryNode::Data FromProto(const proto::TrajectoryNodeData& proto);

// 取节点用于插栅格的 range data（节点 tracking 系，未做全局变换）：
// 优先建图期的 range_datas，其次解压加载来的 compressed_range_datas；
// 都没有返回空。
std::vector<sensor::RangeData> GetRangeDatas(const TrajectoryNode::Data& data);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_NODE_H_
