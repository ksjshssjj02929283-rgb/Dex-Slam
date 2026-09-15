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

#include "cartographer/localization/2d/consistency_debug_saver.h"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "absl/memory/memory.h"
#include "boost/filesystem.hpp"
#include "cartographer/mapping/2d/probability_grid.h"
#include "glog/logging.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace cartographer {
namespace localization {

ConsistencyDebugSaver::ConsistencyDebugSaver() {
  worker_thread_ = std::thread(&ConsistencyDebugSaver::WorkerThread, this);
  LOG(INFO) << "ConsistencyDebugSaver created with background worker thread";
}

ConsistencyDebugSaver::~ConsistencyDebugSaver() {
  running_ = false;
  queue_cv_.notify_all();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
  LOG(INFO) << "ConsistencyDebugSaver destroyed, all tasks completed";
}

void ConsistencyDebugSaver::SaveDebugDataAsync(
    common::Time time, const transform::Rigid2d& pose_estimate,
    std::shared_ptr<const mapping::Submap2D> local_submap,
    const sensor::PointCloud& current_cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data) {
  // 深拷贝数据，避免后台线程访问时数据被修改
  auto pose_copy = pose_estimate;
  auto current_cloud_copy = current_cloud;
  auto highlight_cloud_copy = highlight_cloud;
  auto line_data_copy = line_data;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    task_queue_.push([this, time, pose_copy, local_submap, current_cloud_copy,
                      highlight_cloud_copy, line_data_copy]() {
      SaveDebugDataSync(time, pose_copy, local_submap, current_cloud_copy,
                        highlight_cloud_copy, line_data_copy);
    });
  }
  queue_cv_.notify_one();

  LOG(INFO) << "Queued consistency debug data save task for time: "
            << common::ToUniversal(time);
}

void ConsistencyDebugSaver::WorkerThread() {
  while (running_) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock,
                     [this]() { return !running_ || !task_queue_.empty(); });

      if (!running_ && task_queue_.empty()) {
        return;
      }

      task = std::move(task_queue_.front());
      task_queue_.pop();
    }

    if (task) {
      task();
    }
  }
}

void ConsistencyDebugSaver::SaveDebugDataSync(
    common::Time time, const transform::Rigid2d& pose_estimate,
    std::shared_ptr<const mapping::Submap2D> local_submap,
    const sensor::PointCloud& current_cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data) {
  const std::string dir =
      std::string(std::getenv("HOME") ? std::getenv("HOME") : "/tmp") +
      "/.dexe/data/debug_slam/";

  if (!EnsureDirectoryExists(dir)) {
    LOG(ERROR) << "Failed to create debug_slam directory: " << dir;
    return;
  }

  const std::string timestamp = GenerateTimestampFilename(time, "");
  const std::string image_path = dir + timestamp + ".png";
  const std::string cloud_path = dir + timestamp + ".pcd";

  LOG(INFO) << "Saving consistency debug data at " << timestamp;

  // 绘制调试图像
  const auto* grid = local_submap->grid();
  if (grid) {
    DrawDebugImage(*grid, pose_estimate, current_cloud, highlight_cloud,
                   line_data, image_path);
  }

  // 保存调试点云
  SaveDebugCloud(*local_submap, pose_estimate, current_cloud, highlight_cloud,
                 line_data, cloud_path);

  LOG(INFO) << "Consistency debug data saved: " << image_path << ", "
            << cloud_path;
}

void ConsistencyDebugSaver::DrawDebugImage(
    const mapping::Grid2D& grid, const transform::Rigid2d& pose,
    const sensor::PointCloud& current_cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data,
    const std::string& filepath) {
  const auto* prob_grid = dynamic_cast<const mapping::ProbabilityGrid*>(&grid);
  if (!prob_grid) {
    LOG(ERROR) << "DrawDebugImage: grid is not ProbabilityGrid.";
    return;
  }

  Eigen::Array2i offset;
  mapping::CellLimits cell_limits;
  grid.ComputeCroppedLimits(&offset, &cell_limits);

  // 创建 3 通道彩色图像，初始化为灰色（未知区域）
  cv::Mat img(cell_limits.num_y_cells, cell_limits.num_x_cells, CV_8UC3,
              cv::Scalar(127, 127, 127));

  // 绘制底图灰度
  for (const Eigen::Array2i& xy_index :
       mapping::XYIndexRangeIterator(cell_limits)) {
    const Eigen::Array2i global_index = xy_index + offset;
    if (!grid.IsKnown(global_index)) {
      continue;
    }
    const float probability = prob_grid->GetProbability(global_index);
    const uchar gray = static_cast<uchar>(probability * 255);
    img.at<cv::Vec3b>(xy_index.y(), xy_index.x()) = cv::Vec3b(gray, gray, gray);
  }

  // 绘制当前点云（红色）
  for (const auto& point : current_cloud.points()) {
    Eigen::Vector2d world_point =
        pose * point.position.head<2>().cast<double>();
    Eigen::Array2i pixel = WorldToPixel(grid, offset, world_point);
    if (pixel.y() >= 0 && pixel.y() < cell_limits.num_y_cells &&
        pixel.x() >= 0 && pixel.x() < cell_limits.num_x_cells) {
      img.at<cv::Vec3b>(pixel.y(), pixel.x()) = cv::Vec3b(0, 0, 255);
    }
  }

  // 绘制高亮点云（蓝色）
  for (const auto& point : highlight_cloud.points()) {
    Eigen::Vector2d world_point =
        pose * point.position.head<2>().cast<double>();
    Eigen::Array2i pixel = WorldToPixel(grid, offset, world_point);
    if (pixel.y() >= 0 && pixel.y() < cell_limits.num_y_cells &&
        pixel.x() >= 0 && pixel.x() < cell_limits.num_x_cells) {
      img.at<cv::Vec3b>(pixel.y(), pixel.x()) = cv::Vec3b(255, 0, 0);
    }
  }

  // 绘制线特征（绿色线段）
  for (const auto& data_vec : line_data) {
    for (const auto& line : data_vec.second) {
      Eigen::Array2i begin_pixel = WorldToPixel(grid, offset, line.begin());
      Eigen::Array2i end_pixel = WorldToPixel(grid, offset, line.end());

      // 检查线段是否在图像范围内
      bool begin_valid =
          (begin_pixel.y() >= 0 && begin_pixel.y() < cell_limits.num_y_cells &&
           begin_pixel.x() >= 0 && begin_pixel.x() < cell_limits.num_x_cells);
      bool end_valid =
          (end_pixel.y() >= 0 && end_pixel.y() < cell_limits.num_y_cells &&
           end_pixel.x() >= 0 && end_pixel.x() < cell_limits.num_x_cells);

      if (begin_valid && end_valid) {
        cv::line(img, cv::Point(begin_pixel.x(), begin_pixel.y()),
                 cv::Point(end_pixel.x(), end_pixel.y()), cv::Scalar(0, 255, 0),
                 2);
      }
    }
  }

  // 旋转 90 度（与原 DrawAsImage 保持一致）
  cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

  cv::imwrite(filepath, img);
}

void ConsistencyDebugSaver::SaveDebugCloud(
    const mapping::Submap2D& submap, const transform::Rigid2d& pose,
    const sensor::PointCloud& current_cloud,
    const sensor::PointCloud& highlight_cloud,
    const std::map<int, std::vector<sensor::LineData>>& line_data,
    const std::string& filepath) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // 底图点云（灰色）
  const auto* submap_cloud = submap.cloud();
  if (submap_cloud) {
    for (const auto& point : submap_cloud->points()) {
      pcl::PointXYZRGB p;
      p.x = point.position.x();
      p.y = point.position.y();
      p.z = 0.0;
      p.r = 200;
      p.g = 200;
      p.b = 200;
      colored_cloud->push_back(p);
    }
  }

  // 当前帧点云（红色）
  for (const auto& point : current_cloud.points()) {
    Eigen::Vector2d world_point =
        pose * point.position.head<2>().cast<double>();
    pcl::PointXYZRGB p;
    p.x = static_cast<float>(world_point.x());
    p.y = static_cast<float>(world_point.y());
    p.z = 0.0;
    p.r = 255;
    p.g = 0;
    p.b = 0;
    colored_cloud->push_back(p);
  }

  // 高亮点云（蓝色）
  for (const auto& point : highlight_cloud.points()) {
    Eigen::Vector2d world_point =
        pose * point.position.head<2>().cast<double>();
    pcl::PointXYZRGB p;
    p.x = static_cast<float>(world_point.x());
    p.y = static_cast<float>(world_point.y());
    p.z = 0.0;
    p.r = 0;
    p.g = 0;
    p.b = 255;
    colored_cloud->push_back(p);
  }

  // 线特征端点（绿色）
  for (const auto& data_vec : line_data) {
    for (const auto& line : data_vec.second) {
      // 起点
      pcl::PointXYZRGB p_begin;
      p_begin.x = static_cast<float>(line.begin().x());
      p_begin.y = static_cast<float>(line.begin().y());
      p_begin.z = 0.0;
      p_begin.r = 0;
      p_begin.g = 255;
      p_begin.b = 0;
      colored_cloud->push_back(p_begin);

      // 终点
      pcl::PointXYZRGB p_end;
      p_end.x = static_cast<float>(line.end().x());
      p_end.y = static_cast<float>(line.end().y());
      p_end.z = 0.0;
      p_end.r = 0;
      p_end.g = 255;
      p_end.b = 0;
      colored_cloud->push_back(p_end);
    }
  }

  pcl::io::savePCDFileBinary(filepath, *colored_cloud);

  LOG(INFO) << "Saved colored point cloud: " << colored_cloud->size()
            << " points to " << filepath;
}

std::string ConsistencyDebugSaver::GenerateTimestampFilename(
    common::Time time, const std::string& extension) {
  std::time_t t = common::ToUniversal(time) / 1000000000;
  int microseconds = (common::ToUniversal(time) % 1000000000) / 1000;

  std::tm tm_buf;
  localtime_r(&t, &tm_buf);

  char buf[64];
  std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_buf);

  std::stringstream ss;
  ss << buf << "_" << std::setfill('0') << std::setw(6) << microseconds
     << extension;
  return ss.str();
}

bool ConsistencyDebugSaver::EnsureDirectoryExists(const std::string& dir) {
  namespace fs = boost::filesystem;
  fs::path path(dir);

  if (fs::exists(path) && fs::is_directory(path)) {
    return true;
  }

  return fs::create_directories(path);
}

Eigen::Array2i ConsistencyDebugSaver::WorldToPixel(
    const mapping::Grid2D& grid, const Eigen::Array2i& offset,
    const Eigen::Vector2d& world_point) const {
  Eigen::Array2i cell_index =
      grid.limits().GetCellIndex(world_point.cast<float>());
  return cell_index - offset;
}

}  // namespace localization
}  // namespace cartographer