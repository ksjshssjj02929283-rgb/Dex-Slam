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

#include "cartographer/mapping/2d/submap_2d.h"

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "Eigen/Geometry"
#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
// #include "cartographer/mapping/2d/normal_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/internal/2d/tsdf_range_data_inserter_2d.h"
#include "cartographer/mapping/range_data_inserter_interface.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

proto::SubmapsOptions2D CreateSubmapsOptions2D(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::SubmapsOptions2D options;
  options.set_num_range_data(
      parameter_dictionary->GetNonNegativeInt("num_range_data"));
  options.set_cloud_culling_prob(
      parameter_dictionary->GetDouble("cloud_culling_prob"));
  options.set_min_line_length(
      parameter_dictionary->GetDouble("min_line_length"));
  options.set_max_line_length(
      parameter_dictionary->GetDouble("max_line_length"));
  options.set_line_culling_obs(
      parameter_dictionary->GetInt("line_culling_obs"));
  options.set_culling_interval(
      parameter_dictionary->GetInt("culling_interval"));
  *options.mutable_grid_options_2d() = CreateGridOptions2D(
      parameter_dictionary->GetDictionary("grid_options_2d").get());
  *options.mutable_range_data_inserter_options() =
      CreateRangeDataInserterOptions(
          parameter_dictionary->GetDictionary("range_data_inserter").get());

  bool valid_range_data_inserter_grid_combination = false;
  const proto::GridOptions2D_GridType& grid_type =
      options.grid_options_2d().grid_type();
  const proto::RangeDataInserterOptions_RangeDataInserterType&
      range_data_inserter_type =
          options.range_data_inserter_options().range_data_inserter_type();
  if (grid_type == proto::GridOptions2D::PROBABILITY_GRID &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  if (grid_type == proto::GridOptions2D::TSDF &&
      range_data_inserter_type ==
          proto::RangeDataInserterOptions::TSDF_INSERTER_2D) {
    valid_range_data_inserter_grid_combination = true;
  }
  CHECK(valid_range_data_inserter_grid_combination)
      << "Invalid combination grid_type " << grid_type
      << " with range_data_inserter_type " << range_data_inserter_type;
  CHECK_GT(options.num_range_data(), 0);
  CHECK_GE(options.cloud_culling_prob(), 0.1);
  CHECK_LE(options.cloud_culling_prob(), 0.9);
  return options;
}

Submap2D::Submap2D(const Eigen::Vector2f& origin,
                   std::unique_ptr<Grid2D> grid,
                   ValueConversionTables* conversion_tables,
                   std::unique_ptr<sensor::PointCloud> cloud,
                   std::unique_ptr<sensor::PointCloud> highlight_cloud,
                   int latest_line_id,
                   std::map<int, MapLine2D> active_line_data)
    : Submap(transform::Rigid3d::Translation(
          Eigen::Vector3d(origin.x(), origin.y(), 0.))),
      conversion_tables_(conversion_tables) {
  grid_ = std::move(grid);
  cloud_ = std::move(cloud);
  highlight_cloud_ = std::move(highlight_cloud);
  // note: IKD tree会导致内存占用增加，只在active submap中使用,，后续尽量避免使用（首先要避免highlight_cloud_ikdtree_的使用）
  cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();
  highlight_cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();
  line_map_2d_list_ = std::make_unique<LineMap2DList>(
      latest_line_id, std::move(active_line_data));
}

Submap2D::Submap2D(const proto::Submap2D& proto,
                   ValueConversionTables* conversion_tables)
    : Submap(transform::ToRigid3(proto.local_pose())),
      conversion_tables_(conversion_tables) {
  if (proto.has_grid()) {
    if (proto.grid().has_probability_grid_2d()) {
      // 占用栅格地图，对象初始化
      grid_ =
          std::make_unique<ProbabilityGrid>(proto.grid(), conversion_tables_);
    } else if (proto.grid().has_tsdf_2d()) {
      // TSDF地图，对象初始化
      grid_ = std::make_unique<TSDF2D>(proto.grid(), conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  cloud_ = std::make_unique<sensor::PointCloud>();
  highlight_cloud_ = std::make_unique<sensor::PointCloud>();

  // note: IKD tree会导致内存占用增加，从protobuf中创建不再使用
  // cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();
  // highlight_cloud_ikdtree_ = std::make_unique<ikd_tree::iKdTree<pcl::PointXYZ>>();
  
  // Task 4: 从 proto 反序列化 line_map_2d_list
  if (proto.has_line_map_2d_list()) {
    const auto& line_map_proto = proto.line_map_2d_list();
    int latest_local_id = line_map_proto.latest_local_id();
    std::map<int, MapLine2D> line_map_data;
    
    // 遍历所有的地图线段数据
    for (const auto& map_line_proto : line_map_proto.line_map_data()) {
      // 反序列化主线段数据
      const auto& line_data_proto = map_line_proto.line_data();
      Eigen::Vector2d begin(line_data_proto.begin_x(), line_data_proto.begin_y());
      Eigen::Vector2d end(line_data_proto.end_x(), line_data_proto.end_y());
      sensor::LineData line_data(begin, end, line_data_proto.weight());
      
      if (line_data_proto.has_normal()) {
        Eigen::Vector2d normal(line_data_proto.normal_x(), line_data_proto.normal_y());
        line_data.set_normal(normal);
      }
      
      // 创建 MapLine2D 对象（使用 global_id，local_id 在构造时设置）
      MapLine2D map_line(MapLine2D::static_line, map_line_proto.global_id(), line_data);
      map_line.SetLocalId(map_line_proto.local_id());  // 显式设置 local_id
      map_line.SetLoseTime(map_line_proto.lose_time());
      map_line.SetInliers(map_line_proto.inliers());  // 恢复 inliers
      
      // 反序列化线段数据列表
      for (const auto& line_list_proto : map_line_proto.line_data_list()) {
        Eigen::Vector2d list_begin(line_list_proto.begin_x(), line_list_proto.begin_y());
        Eigen::Vector2d list_end(line_list_proto.end_x(), line_list_proto.end_y());
        sensor::LineData list_line_data(list_begin, list_end, line_list_proto.weight());
        
        if (line_list_proto.has_normal()) {
          Eigen::Vector2d list_normal(line_list_proto.normal_x(), line_list_proto.normal_y());
          list_line_data.set_normal(list_normal);
        }
        
        // 更新到 MapLine2D 中（这里需要调用 UpdateLineData）
        map_line.UpdateLineData(list_line_data);
      }
      
      // 使用 local_id 作为 key（与序列化时一致）
      line_map_data.insert({map_line_proto.local_id(), std::move(map_line)});
    }
    
    line_map_2d_list_ = std::make_unique<LineMap2DList>(latest_local_id, std::move(line_map_data));
  } else {
    line_map_2d_list_ = std::make_unique<LineMap2DList>();
  }
  
  // 从 proto 反序列化点云数据
  // 无论 proto 中是否有点云数据，都要创建点云对象，确保指针不为 nullptr
  if (proto.has_cloud() && proto.cloud().points_size() > 0) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    
    // 预分配内存以减少重新分配
    points.reserve(proto.cloud().points_size());
    
    for (const auto& point_proto : proto.cloud().points()) {
      points.push_back(sensor::RangefinderPoint{
        Eigen::Vector3f(point_proto.x(), point_proto.y(), point_proto.z())
      });
    }
    
    // 反序列化强度信息（如果有）
    if (proto.cloud().intensities_size() > 0) {
      intensities.reserve(proto.cloud().intensities_size());
      for (float intensity : proto.cloud().intensities()) {
        intensities.push_back(intensity);
      }
      cloud_ = std::make_unique<sensor::PointCloud>(std::move(points), std::move(intensities));
    } else {
      cloud_ = std::make_unique<sensor::PointCloud>(std::move(points));
    }
  } else {
    // 如果没有点云数据，创建空的点云对象
    cloud_ = std::make_unique<sensor::PointCloud>();
  }
  
  // 从 proto 反序列化高亮点云数据
  // 无论 proto 中是否有高亮点云数据，都要创建点云对象，确保指针不为 nullptr
  if (proto.has_highlight_cloud() && proto.highlight_cloud().points_size() > 0) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    
    // 预分配内存以减少重新分配
    points.reserve(proto.highlight_cloud().points_size());
    
    for (const auto& point_proto : proto.highlight_cloud().points()) {
      points.push_back(sensor::RangefinderPoint{
        Eigen::Vector3f(point_proto.x(), point_proto.y(), point_proto.z())
      });
    }
    
    // 反序列化强度信息（如果有）
    if (proto.highlight_cloud().intensities_size() > 0) {
      intensities.reserve(proto.highlight_cloud().intensities_size());
      for (float intensity : proto.highlight_cloud().intensities()) {
        intensities.push_back(intensity);
      }
      highlight_cloud_ = std::make_unique<sensor::PointCloud>(std::move(points), std::move(intensities));
    } else {
      highlight_cloud_ = std::make_unique<sensor::PointCloud>(std::move(points));
    }
  } else {
    // 如果没有高亮点云数据，创建空的点云对象
    highlight_cloud_ = std::make_unique<sensor::PointCloud>();
  }
  
  set_num_range_data(proto.num_range_data());  // 设置激光点数量
  set_insertion_finished(proto.finished());
}

Submap2D::~Submap2D() {
  if (cloud_ikdtree_) {
    cloud_ikdtree_.reset();
  }
  if (highlight_cloud_ikdtree_) {
    highlight_cloud_ikdtree_.reset();
  }
  
  // 释放线段地图
  if (line_map_2d_list_) {
    line_map_2d_list_.reset();
  }
  
  // 释放点云数据
  if (cloud_) {
    cloud_.reset();
  }
  if (highlight_cloud_) {
    highlight_cloud_.reset();
  }
  
  // 释放栅格地图
  if (grid_) {
    grid_.reset();
  }
}

proto::Submap Submap2D::ToProto(const bool include_grid_data) const {
  proto::Submap proto;
  auto* const submap_2d = proto.mutable_submap_2d();
  *submap_2d->mutable_local_pose() = transform::ToProto(local_pose());
  submap_2d->set_num_range_data(num_range_data());
  submap_2d->set_finished(insertion_finished());
  if (include_grid_data) {
    CHECK(grid_);
    *submap_2d->mutable_grid() = grid_->ToProto();
  }
  
  // Task 3: 序列化 line_map_2d_list 到 proto
  if (line_map_2d_list_ && line_map_2d_list_->size() > 0) {
    auto* line_map_proto = submap_2d->mutable_line_map_2d_list();
    line_map_proto->set_latest_local_id(line_map_2d_list_->latest_local_id());
    
    // 遍历所有的地图线段数据
    const auto& line_map_data_list = line_map_2d_list_->GetLineMapData2DList();
    for (const auto& line_map_entry : line_map_data_list) {
      auto* map_line_proto = line_map_proto->add_line_map_data();
      const MapLine2D& map_line = line_map_entry.second;
      
      // 设置地图线段的基本信息
      map_line_proto->set_global_id(map_line.global_id());
      map_line_proto->set_local_id(map_line.local_id());
      map_line_proto->set_lose_time(map_line.lose_time());
      map_line_proto->set_inliers(map_line.inliers());
      
      // 序列化主线段数据
      const sensor::LineData& line_data = map_line.line_data();
      auto* line_data_proto = map_line_proto->mutable_line_data();
      line_data_proto->set_begin_x(line_data.begin().x());
      line_data_proto->set_begin_y(line_data.begin().y());
      line_data_proto->set_end_x(line_data.end().x());
      line_data_proto->set_end_y(line_data.end().y());
      line_data_proto->set_weight(line_data.weight());
      
      // 尝试获取法向量
      Eigen::Vector2d normal = Eigen::Vector2d::Zero();
      bool has_normal = line_data.GetNormal(normal);
      line_data_proto->set_has_normal(!has_normal);  // GetNormal返回false表示有法向量
      if (!has_normal) {
        line_data_proto->set_normal_x(normal.x());
        line_data_proto->set_normal_y(normal.y());
      }
      
      // 序列化线段数据列表
      for (const auto& line_in_list : map_line.line_data_list()) {
        auto* line_list_proto = map_line_proto->add_line_data_list();
        line_list_proto->set_begin_x(line_in_list.begin().x());
        line_list_proto->set_begin_y(line_in_list.begin().y());
        line_list_proto->set_end_x(line_in_list.end().x());
        line_list_proto->set_end_y(line_in_list.end().y());
        line_list_proto->set_weight(line_in_list.weight());
        
        // 尝试获取法向量
        Eigen::Vector2d list_normal = Eigen::Vector2d::Zero();
        bool has_list_normal = line_in_list.GetNormal(list_normal);
        line_list_proto->set_has_normal(!has_list_normal);  // GetNormal返回false表示有法向量
        if (!has_list_normal) {
          line_list_proto->set_normal_x(list_normal.x());
          line_list_proto->set_normal_y(list_normal.y());
        }
      }
    }
  }
  
  // 序列化点云数据 - 即使为空也要序列化，确保反序列化时指针不为nullptr
  auto* cloud_proto = submap_2d->mutable_cloud();
  if (cloud_ && !cloud_->empty()) {
    for (const auto& point : cloud_->points()) {
      auto* point_proto = cloud_proto->add_points();
      point_proto->set_x(point.position.x());
      point_proto->set_y(point.position.y());
      point_proto->set_z(point.position.z());
    }
    // 序列化强度信息（如果有）
    const auto& intensities = cloud_->intensities();
    if (!intensities.empty()) {
      for (float intensity : intensities) {
        cloud_proto->add_intensities(intensity);
      }
    }
  }
  // 如果 cloud_ 为空或nullptr，cloud_proto 仍然存在但没有 points，这样反序列化时会创建空点云
  
  // 序列化高亮点云数据 - 即使为空也要序列化，确保反序列化时指针不为nullptr
  auto* highlight_cloud_proto = submap_2d->mutable_highlight_cloud();
  if (highlight_cloud_ && !highlight_cloud_->empty()) {
    for (const auto& point : highlight_cloud_->points()) {
      auto* point_proto = highlight_cloud_proto->add_points();
      point_proto->set_x(point.position.x());
      point_proto->set_y(point.position.y());
      point_proto->set_z(point.position.z());
    }
    // 序列化强度信息（如果有）
    const auto& intensities = highlight_cloud_->intensities();
    if (!intensities.empty()) {
      for (float intensity : intensities) {
        highlight_cloud_proto->add_intensities(intensity);
      }
    }
  }
  // 如果 highlight_cloud_ 为空或nullptr，highlight_cloud_proto 仍然存在但没有 points，这样反序列化时会创建空点云
  
  return proto;
}

void Submap2D::UpdateFromProto(const proto::Submap& proto) {
  CHECK(proto.has_submap_2d());
  const auto& submap_2d = proto.submap_2d();
  set_num_range_data(submap_2d.num_range_data());
  set_insertion_finished(submap_2d.finished());
  if (proto.submap_2d().has_grid()) {
    if (proto.submap_2d().grid().has_probability_grid_2d()) {
      grid_ = std::make_unique<ProbabilityGrid>(proto.submap_2d().grid(),
                                                 conversion_tables_);
    } else if (proto.submap_2d().grid().has_tsdf_2d()) {
      grid_ = std::make_unique<TSDF2D>(proto.submap_2d().grid(),
                                        conversion_tables_);
    } else {
      LOG(FATAL) << "proto::Submap2D has grid with unknown type.";
    }
  }
  
  // Task 4: 从 proto 反序列化 line_map_2d_list (在 UpdateFromProto 中也需要处理)
  if (submap_2d.has_line_map_2d_list()) {
    const auto& line_map_proto = submap_2d.line_map_2d_list();
    int latest_local_id = line_map_proto.latest_local_id();
    std::map<int, MapLine2D> line_map_data;
    
    // 遍历所有的地图线段数据
    for (const auto& map_line_proto : line_map_proto.line_map_data()) {
      // 反序列化主线段数据
      const auto& line_data_proto = map_line_proto.line_data();
      Eigen::Vector2d begin(line_data_proto.begin_x(), line_data_proto.begin_y());
      Eigen::Vector2d end(line_data_proto.end_x(), line_data_proto.end_y());
      sensor::LineData line_data(begin, end, line_data_proto.weight());
      
      if (line_data_proto.has_normal()) {
        Eigen::Vector2d normal(line_data_proto.normal_x(), line_data_proto.normal_y());
        line_data.set_normal(normal);
      }
      
      // 创建 MapLine2D 对象
      MapLine2D map_line(MapLine2D::static_line, map_line_proto.global_id(), line_data);
      map_line.SetLoseTime(map_line_proto.lose_time());
      
      // 反序列化线段数据列表
      for (const auto& line_list_proto : map_line_proto.line_data_list()) {
        Eigen::Vector2d list_begin(line_list_proto.begin_x(), line_list_proto.begin_y());
        Eigen::Vector2d list_end(line_list_proto.end_x(), line_list_proto.end_y());
        sensor::LineData list_line_data(list_begin, list_end, line_list_proto.weight());
        
        if (line_list_proto.has_normal()) {
          Eigen::Vector2d list_normal(line_list_proto.normal_x(), line_list_proto.normal_y());
          list_line_data.set_normal(list_normal);
        }
        
        map_line.UpdateLineData(list_line_data);
      }
      
      line_map_data.insert({map_line_proto.global_id(), std::move(map_line)});
    }
    
    line_map_2d_list_ = std::make_unique<LineMap2DList>(latest_local_id, std::move(line_map_data));
  }
  
  // 从 proto 反序列化点云数据（在 UpdateFromProto 中也需要处理）
  if (submap_2d.has_cloud() && submap_2d.cloud().points_size() > 0) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    
    for (const auto& point_proto : submap_2d.cloud().points()) {
      points.push_back(sensor::RangefinderPoint{
        Eigen::Vector3f(point_proto.x(), point_proto.y(), point_proto.z())
      });
    }
    
    // 反序列化强度信息（如果有）
    if (submap_2d.cloud().intensities_size() > 0) {
      for (float intensity : submap_2d.cloud().intensities()) {
        intensities.push_back(intensity);
      }
      cloud_ = std::make_unique<sensor::PointCloud>(std::move(points), std::move(intensities));
    } else {
      cloud_ = std::make_unique<sensor::PointCloud>(std::move(points));
    }
  }
  
  // 从 proto 反序列化高亮点云数据
  if (submap_2d.has_highlight_cloud() && submap_2d.highlight_cloud().points_size() > 0) {
    std::vector<sensor::RangefinderPoint> points;
    std::vector<float> intensities;
    
    for (const auto& point_proto : submap_2d.highlight_cloud().points()) {
      points.push_back(sensor::RangefinderPoint{
        Eigen::Vector3f(point_proto.x(), point_proto.y(), point_proto.z())
      });
    }
    
    // 反序列化强度信息（如果有）
    if (submap_2d.highlight_cloud().intensities_size() > 0) {
      for (float intensity : submap_2d.highlight_cloud().intensities()) {
        intensities.push_back(intensity);
      }
      highlight_cloud_ = std::make_unique<sensor::PointCloud>(std::move(points), std::move(intensities));
    } else {
      highlight_cloud_ = std::make_unique<sensor::PointCloud>(std::move(points));
    }
  }
}

/**
 * @brief 将submap的信息转换为proto::SubmapQuery::Response
 * 
 * @param response 
 */
void Submap2D::ToResponseProto(
    const transform::Rigid3d&,
    proto::SubmapQuery::Response* const response) const {
  if (!grid_) return;
  // 设置submap的观测激光数量
  response->set_submap_version(num_range_data());
  // 获取texture
  proto::SubmapQuery::Response::SubmapTexture* const texture =
      response->add_textures();
  // 生成submap
  grid()->DrawToSubmapTexture(texture, local_pose());
}

bool Submap2D::SaveSubmapCloud(const std::string& folder,
                               const std::string& filename)
{
  if (!boost::filesystem::is_directory(folder)) {
    LOG(INFO) << "Creating folder: " << folder;
    if (!boost::filesystem::create_directories(folder)) {
      LOG(ERROR) << "Could not create directory: " << folder;
      return false;
    }
  }

  // save as pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& point : this->cloud()->points()) {
    pcl::PointXYZ p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = 0.0;
    cloud->push_back(p);
  }

  int res = pcl::io::savePCDFileBinary(folder + "/" + filename, *cloud);
  return res > 0;
}

void Submap2D::InsertRangeData(
    const sensor::RangeData& range_data,
    const RangeDataInserterInterface* range_data_inserter) {
  CHECK(grid_);
  CHECK(cloud_);
  CHECK(highlight_cloud_);
  CHECK(!insertion_finished());
  range_data_inserter->Insert(range_data, grid_.get());
}

void Submap2D::InsertCloudData(const sensor::PointCloud & cloud_data,
                               const sensor::PointCloud & highlight_cloud_data)
{
  is_dirty_cloud_ = true;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  PointCloudToPCL(cloud_data, cloud_pcl);

  if (cloud_ikdtree_->Root_Node == nullptr) {
    float resolution = grid_->limits().resolution();
    cloud_ikdtree_->set_downsample_param(resolution);
    cloud_ikdtree_->Build(cloud_pcl->points);
  } else {
    auto add_point_size = cloud_ikdtree_->Add_Points(cloud_pcl->points, true);
  }

  if (highlight_cloud_data.size() > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr highlight_cloud_pcl = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    PointCloudToPCL(highlight_cloud_data, highlight_cloud_pcl);

    if (highlight_cloud_ikdtree_->Root_Node == nullptr) {
      float resolution = grid_->limits().resolution();
      highlight_cloud_ikdtree_->set_downsample_param(resolution);
      highlight_cloud_ikdtree_->Build(highlight_cloud_pcl->points);
    } else {
      auto add_point_size = highlight_cloud_ikdtree_->Add_Points(highlight_cloud_pcl->points, true);
    }
  }

  // cloud_->InsertCloud(cloud_data);
  // highlight_cloud_->InsertCloud(highlight_cloud_data);

  cloud_culling_count_++;
  // 当cloud_culling_prob = 0.1时，不进行culling操作
  if ((cloud_culling_prob_ - 0.1) < std::numeric_limits<double>::epsilon()) {
    return;
  }
  
  if (cloud_culling_count_ % culling_interval_ != 0) {
    return;
  }

  CloudDataCulling();
}

void Submap2D::CloudDataCulling()
{
  const MapLimits& limits = grid_->limits();
  // 需要先调用flatten函数，更新PCL_Storage
  cloud_ikdtree_->flatten(cloud_ikdtree_->Root_Node,
                          cloud_ikdtree_->PCL_Storage,
                          ikd_tree::NOT_RECORD);

  ikd_tree::iKdTree<pcl::PointXYZ>::PointVector points_to_delete; // 用于保存需要删除的点  
  for (auto iter = cloud_ikdtree_->PCL_Storage.begin(); iter != cloud_ikdtree_->PCL_Storage.end(); iter++) {
    Eigen::Vector2f p_vec(iter->x, iter->y);
    const Eigen::Array2i p_ind = limits.GetCellIndex(p_vec);
    const float prob = CorrespondenceCostToProbability(grid_->GetCorrespondenceCost(p_ind));
    if (prob < cloud_culling_prob_) {
      points_to_delete.emplace_back(*iter); // 将需要删除的点保存到向量中  
    }
  }
  cloud_ikdtree_->Delete_Points(points_to_delete);

  points_to_delete.clear();

  highlight_cloud_ikdtree_->flatten(highlight_cloud_ikdtree_->Root_Node,
                                    highlight_cloud_ikdtree_->PCL_Storage,
                                    ikd_tree::NOT_RECORD);

  for (auto iter = highlight_cloud_ikdtree_->PCL_Storage.begin(); iter != highlight_cloud_ikdtree_->PCL_Storage.end(); iter++) {
    Eigen::Vector2f p_vec(iter->x, iter->y);
    const Eigen::Array2i p_ind = limits.GetCellIndex(p_vec);
    const float prob = CorrespondenceCostToProbability(grid_->GetCorrespondenceCost(p_ind));
    if (prob < cloud_culling_prob_) {
      points_to_delete.emplace_back(*iter); // 将需要删除的点保存到向量中  
    }
  }
  highlight_cloud_ikdtree_->Delete_Points(points_to_delete);

  // FreshCloudData();
}

void Submap2D::FreshCloudData() const
{
  ikd_tree::iKdTree<pcl::PointXYZ>::PointVector().swap(cloud_ikdtree_->PCL_Storage);
  cloud_ikdtree_->flatten(cloud_ikdtree_->Root_Node,
                          cloud_ikdtree_->PCL_Storage,
                          ikd_tree::NOT_RECORD);

  cloud_ = std::make_unique<sensor::PointCloud>();
  for (const auto& point : cloud_ikdtree_->PCL_Storage) {
    sensor::PointCloud::PointType p;
    p.position.x() = point.x;
    p.position.y() = point.y;
    p.position.z() = 0.0;
    cloud_->push_back(p);
  }

  ikd_tree::iKdTree<pcl::PointXYZ>::PointVector().swap(highlight_cloud_ikdtree_->PCL_Storage);
  highlight_cloud_ikdtree_->flatten(highlight_cloud_ikdtree_->Root_Node,
                                    highlight_cloud_ikdtree_->PCL_Storage,
                                    ikd_tree::NOT_RECORD);

  highlight_cloud_ = std::make_unique<sensor::PointCloud>();
  for (const auto& point : highlight_cloud_ikdtree_->PCL_Storage) {
    sensor::PointCloud::PointType p;
    p.position.x() = point.x;
    p.position.y() = point.y;
    p.position.z() = 0.0;
    highlight_cloud_->push_back(p);
  }
  is_dirty_cloud_ = false;
}

void Submap2D::PointCloudToPCL(const sensor::PointCloud & cloud_data,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud)
{
  if (pcl_cloud == nullptr) {
    pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  }

  for (const auto& p : cloud_data.points()) {
    pcl::PointXYZ p_pcl;
    p_pcl.x = p.position.x();
    p_pcl.y = p.position.y();
    p_pcl.z = p.position.z();
    pcl_cloud->push_back(p_pcl);
  }
}

void Submap2D::SetLineCullingParam(const double& min_line_length,
                          const double& max_line_length,
                          const int& culling_obs)
{
  line_map_2d_list_->SetCullingParam(min_line_length, max_line_length, culling_obs);
}

void Submap2D::InsertLineData(
  const std::map<int, std::vector<sensor::LineData>> & line_data)
{
  // std::cout << "line_data: " << line_data.size() << std::endl;
  line_map_2d_list_->UpdateLoseTime();
  for (auto iter = line_data.begin(); iter != line_data.end(); iter++) {
    if (iter->first < 0) {
      line_map_2d_list_->PushBack(iter->second);
    } else {
      line_map_2d_list_->UpdateLineMap(iter->first, iter->second);
    }
  }
  line_culling_count_++;
  if (line_culling_count_ % culling_interval_ == 0) {
    line_map_2d_list_->LineDataCulling();
  }
}

void Submap2D::InsertReflectorData(
  const std::vector<sensor::LandmarkPointData> & reflector_data)
{
  reflector_map_.insert(
    reflector_map_.end(), reflector_data.begin(), reflector_data.end());
}

void Submap2D::Finish() {
  CHECK(grid_);
  CHECK(cloud_);
  CHECK(highlight_cloud_);
  CHECK(!insertion_finished());
  grid_ = grid_->ComputeCroppedGrid();
  line_map_2d_list_->LineDataCulling();
  // LOG(WARNING) << "Finishing submap...";
  CloudDataCulling();

  // auto start_time = std::chrono::high_resolution_clock::now();
  // FreshCloudData();
  // auto end_time = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
  // LOG(INFO) << "FreshCloudData execution time: " << duration.count() << " microseconds";
  // static int submap_count = 0;
  // std::string home_dir = std::getenv("HOME") ? std::getenv("HOME") : "/tmp";
  // std::string folder = home_dir + "/.dexe/localization/active_submaps/";
  // std::string filename = "submap_" + std::to_string(submap_count++) + ".png";
  // grid_->DrawAsImage(folder, filename);

  set_insertion_finished(true);
}

ActiveSubmaps2D::ActiveSubmaps2D(const proto::SubmapsOptions2D& options)
    : options_(options), range_data_inserter_(CreateRangeDataInserter()) {}

std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::submaps() const {
  return std::vector<std::shared_ptr<const Submap2D>>(submaps_.begin(),
                                                      submaps_.end());
}


std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertData(
    const std::vector<sensor::RangeData>& range_data_vector,
    const sensor::PointCloud& cloud_data,
    const sensor::PointCloud& highlight_cloud_data,
    const std::map<int, std::vector<sensor::LineData>>& line_data,
    const std::vector<sensor::LandmarkPointData>& reflector_data) {
  
  // 如果所有数据都为空，直接返回当前submaps
  if (range_data_vector.empty() && cloud_data.empty() && 
      highlight_cloud_data.empty() && line_data.empty() && reflector_data.empty()) {
    return submaps();
  }
  CHECK(!range_data_vector.empty());

  // TODO: 增加角速度过大时候，不增加submap的操作(旋转中增加submap会增大建图的误差)
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() >= options_.num_range_data()) {
    const sensor::RangeData& first_range_data = range_data_vector.front();
    AddSubmap(first_range_data.origin.head<2>());
  }

  // 1. 插入range data
  InsertRangeData(range_data_vector);

  // 2. 插入cloud data（如果非空）
  InsertCloudData(cloud_data, highlight_cloud_data);
  
  // 3. 插入line data（如果非空）
  InsertLineData(line_data);
  
  // 4. 插入reflector data（如果非空）
  InsertReflectorData(reflector_data);
  // 检查是否需要完成前面的submap
  if (!submaps_.empty() && submaps_.front()->num_range_data() >= 2 * options_.num_range_data()) {
    submaps_.front()->Finish();
  }

  return submaps();
}

void ActiveSubmaps2D::InsertRangeData(
    const std::vector<sensor::RangeData>& range_data_vector) {
  if (range_data_vector.empty()) {
    return;
  }
  
  // 批量插入所有range data到现有submaps
  for (auto& submap : submaps_) {
    for (const auto& range_data : range_data_vector) {
      submap->InsertRangeData(range_data, range_data_inserter_.get());
    }
    submap->set_num_range_data(submap->num_range_data() + 1);
  }
}

void ActiveSubmaps2D::InsertReflectorData(
  const std::vector<sensor::LandmarkPointData> & reflector_data)
{
  if (submaps_.empty() || reflector_data.empty()) {
    return;
  }
  for (auto& submap : submaps_) {
    submap->InsertReflectorData(reflector_data);
  }
}

void ActiveSubmaps2D::InsertLineData(
  const std::map<int, std::vector<sensor::LineData>> & line_data)
{
  // std::cout << "\033[31m InsertLineData \033[0m" << std::endl;
  if (submaps_.empty() || line_data.empty()) {
    // std::cout << "return" << std::endl;
    return;
  }
  for (auto & submap : submaps_) {
    submap->InsertLineData(line_data);
  }
}

void ActiveSubmaps2D::InsertCloudData(const sensor::PointCloud & cloud_data,
                                      const sensor::PointCloud & highlight_cloud_data)
{
  if (submaps_.empty()) {
    return;
  }
  for (auto & submap : submaps_) {
    submap->InsertCloudData(cloud_data, highlight_cloud_data);
  }
}

std::unique_ptr<RangeDataInserterInterface>
ActiveSubmaps2D::CreateRangeDataInserter() {
  switch (options_.range_data_inserter_options().range_data_inserter_type()) {
    case proto::RangeDataInserterOptions::PROBABILITY_GRID_INSERTER_2D:
      return std::make_unique<ProbabilityGridRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .probability_grid_range_data_inserter_options_2d());
    case proto::RangeDataInserterOptions::TSDF_INSERTER_2D:
      return std::make_unique<TSDFRangeDataInserter2D>(
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d());
    default:
      LOG(FATAL) << "Unknown RangeDataInserterType.";
  }
}

std::unique_ptr<GridInterface> ActiveSubmaps2D::CreateGrid(
    const Eigen::Vector2f& origin) {
  constexpr int kInitialSubmapSize = 100;
  float resolution = options_.grid_options_2d().resolution();
  switch (options_.grid_options_2d().grid_type()) {
    case proto::GridOptions2D::PROBABILITY_GRID:
      return std::make_unique<ProbabilityGrid>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          &conversion_tables_);
    case proto::GridOptions2D::TSDF:
      return std::make_unique<TSDF2D>(
          MapLimits(resolution,
                    origin.cast<double>() + 0.5 * kInitialSubmapSize *
                                                resolution *
                                                Eigen::Vector2d::Ones(),
                    CellLimits(kInitialSubmapSize, kInitialSubmapSize)),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .truncation_distance(),
          options_.range_data_inserter_options()
              .tsdf_range_data_inserter_options_2d()
              .maximum_weight(),
          &conversion_tables_);
    default:
      LOG(FATAL) << "Unknown GridType.";
  }
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    // This will crop the finished Submap before inserting a new Submap to
    // reduce peak memory usage a bit.
    CHECK(submaps_.front()->insertion_finished());
    submaps_.erase(submaps_.begin());
  }

  int latest_line_id = 0;
  std::map<int, MapLine2D> active_line_data;
  if (submaps_.size() != 0) {
    latest_line_id = submaps_.front()->line_map_2d_list()->latest_local_id();
    active_line_data = submaps_.front()->line_map_2d_list()->GetActiveLineData();
  }

  submaps_.push_back(std::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_,
      std::make_unique<sensor::PointCloud>(),
      std::make_unique<sensor::PointCloud>(),
      latest_line_id, active_line_data));
  submaps_.back()->SetCloudCullingProb(options_.cloud_culling_prob());
  submaps_.back()->SetLineCullingParam(options_.min_line_length(),
                                       options_.max_line_length(),
                                       options_.line_culling_obs());
  submaps_.back()->SetCullingInterval(options_.culling_interval());
}



}  // namespace mapping
}  // namespace cartographer
