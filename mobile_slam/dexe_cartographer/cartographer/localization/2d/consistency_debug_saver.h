/*
 * Copyright 2026 The Cartographer Authors
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

#ifndef CARTOGRAPHER_LOCALIZATION_2D_CONSISTENCY_DEBUG_SAVER_H_
#define CARTOGRAPHER_LOCALIZATION_2D_CONSISTENCY_DEBUG_SAVER_H_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "cartographer/common/time.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace localization {

// 异步保存一致性检查失败时的调试数据（带彩色标注的地图图像和点云PCD）
class ConsistencyDebugSaver {
 public:
  ConsistencyDebugSaver();
  ~ConsistencyDebugSaver();

  // 异步保存调试数据（非阻塞，任务在后台线程执行）
  void SaveDebugDataAsync(
      common::Time time, const transform::Rigid2d& pose_estimate,
      std::shared_ptr<const mapping::Submap2D> local_submap,
      const sensor::PointCloud& current_cloud,
      const sensor::PointCloud& highlight_cloud,
      const std::map<int, std::vector<sensor::LineData>>& line_data);

 private:
  // 后台工作线程主循环
  void WorkerThread();

  // 同步保存实现（在后台线程中执行）
  void SaveDebugDataSync(
      common::Time time, const transform::Rigid2d& pose_estimate,
      std::shared_ptr<const mapping::Submap2D> local_submap,
      const sensor::PointCloud& current_cloud,
      const sensor::PointCloud& highlight_cloud,
      const std::map<int, std::vector<sensor::LineData>>& line_data);

  // 绘制带彩色标注的图像
  void DrawDebugImage(
      const mapping::Grid2D& grid, const transform::Rigid2d& pose,
      const sensor::PointCloud& current_cloud,
      const sensor::PointCloud& highlight_cloud,
      const std::map<int, std::vector<sensor::LineData>>& line_data,
      const std::string& filepath);

  // 保存带彩色标注的点云
  void SaveDebugCloud(
      const mapping::Submap2D& submap, const transform::Rigid2d& pose,
      const sensor::PointCloud& current_cloud,
      const sensor::PointCloud& highlight_cloud,
      const std::map<int, std::vector<sensor::LineData>>& line_data,
      const std::string& filepath);

  // 生成时间戳文件名
  std::string GenerateTimestampFilename(common::Time time,
                                        const std::string& extension);

  // 确保目录存在
  bool EnsureDirectoryExists(const std::string& dir);

  // 世界坐标转像素坐标
  Eigen::Array2i WorldToPixel(const mapping::Grid2D& grid,
                              const Eigen::Array2i& offset,
                              const Eigen::Vector2d& world_point) const;

  // 后台线程管理
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::queue<std::function<void()>> task_queue_;
  std::thread worker_thread_;
  std::atomic<bool> running_{true};
};

}  // namespace localization
}  // namespace cartographer

#endif  // CARTOGRAPHER_LOCALIZATION_2D_CONSISTENCY_DEBUG_SAVER_H_