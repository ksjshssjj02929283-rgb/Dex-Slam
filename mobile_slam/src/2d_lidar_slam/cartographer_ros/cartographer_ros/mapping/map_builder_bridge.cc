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

#include "cartographer_ros/mapping/map_builder_bridge.h"

#include <fstream>

#include <cairo/cairo.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <jsoncpp/json/json.h>

#include "absl/memory/memory.h"
#include "cartographer/io/color.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/conversion/time_conversion.h"
#include "cartographer_ros/mapping/ros_map.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.2;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::msg::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::msg::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::msg::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id,
                                                  rclcpp::Time node_time) {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Trajectory " + std::to_string(trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.stamp = node_time;
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

visualization_msgs::msg::Marker CreateLandmarkMarker(int landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id,
                                                rclcpp::Time node_time) {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Landmarks";
  marker.id = landmark_index;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.header.stamp = node_time;
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::msg::Marker* marker,
                            std::vector<visualization_msgs::msg::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(
    const MappingNodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}
// 加载pbstream文件
void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  // 加载文件内容
  cartographer::io::ProtoStreamReader stream(state_filename);
  map_builder_->LoadState(&stream, load_frozen_state);
}
/**
 * @brief 根据传感器id和配置文件，创建位姿优化器
 * 
 * @param expected_sensor_ids 所有传感器的id
 * @param trajectory_options 轨迹的配置
 * @return int 轨迹id
 */
int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  // 1. 构建一个轨迹构建器，并将该id的map_builder与OnLocalSlamResult进行绑定
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      [this](const int trajectory_id, const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             std::vector<::cartographer::transform::Rigid2d> particle_poses,
             ::cartographer::sensor::RangeData range_data_in_local,
             std::map<int, std::vector<::cartographer::sensor::LineData>>  line_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult> insertion_result) {
        std::shared_ptr<::cartographer::sensor::PointCloud> cloud_map(
          new ::cartographer::sensor::PointCloud());
        std::shared_ptr<::cartographer::mapping::LineMap2DList> line_map_2d_list(
          new ::cartographer::mapping::LineMap2DList());
        // 因为map insertion 的频率和matching result的频率不一样，所以map_insertion可能会返回空值
        if (insertion_result) {
          cloud_map = insertion_result->local_map;
          line_map_2d_list = insertion_result->line_map_2d_list;
        }
        OnLocalSlamResult(trajectory_id, time, local_pose, particle_poses,
                          range_data_in_local, cloud_map,
                          line_data_in_local, line_map_2d_list);
      });
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet. 如果已经存在该轨迹id，则报错
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  // 2. 为该轨迹id创建一个ROS传感器数据桥接器
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.ignore_out_of_order_messages,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id));

  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

bool MapBuilderBridge::SaveMap(const std::string& map_filename, const double resolution)
{
  // SaveGridMap(map_filename, resolution);
  SaveGridMapFromRange(map_filename, resolution);
  SaveCloudMap(map_filename, resolution);
  SaveLineMap(map_filename);
  return true;
}

bool MapBuilderBridge::SaveLineMap(const std::string& map_filename)
{
  cartographer::mapping::MapById<
    cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNode>
  nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
  // 这里的每个节点中的线特征都是跟踪到的线特征
  cartographer::mapping::LineMap2DList line_map_2d_list;
  for (const auto& node : nodes) {
    // 获取每个节点的线特征
    const auto& lines = node.data.constant_data->lines;
    if (lines.empty()) {
      continue;
    }

    cartographer::transform::Rigid2d global_pose_2d =
            cartographer::transform::Project2D(node.data.global_pose);

    for (const auto& line_data_element : lines) {
      if (line_data_element.first < 0 ||
          line_data_element.second.empty())
      {
        continue;
      }
      // 获取每个节点的位姿
      std::map<int, std::vector<cartographer::sensor::LineData>> line_in_local;
      for (const auto& line: line_data_element.second) {
        line_map_2d_list.UpdateLineMap(
                            line_data_element.first,
                            {cartographer::sensor::TransformLineData(global_pose_2d, line)});
      }
    }
  }
  line_map_2d_list.SetCullingParam(0.3, 7.0, 1);
  line_map_2d_list.LineDataCulling();

  std::string line_map_filename = map_filename + "_line_map.json";

  std::ofstream line_map_file;
  line_map_file.open(line_map_filename, std::ios::out | std::ios::trunc);
  if (!line_map_file.is_open()) {
    LOG(ERROR) << "Failed to open file: " << line_map_filename;
    return false;
  }

  if (line_map_2d_list.size() == 0) {
    LOG(WARNING) << "No line data to save.";
    line_map_file.close();
    return false;
  }

  Json::Value root;
  for (const auto& line_map : line_map_2d_list.GetLineMapData2DList()) {
    Json::Value line_data;
    line_data["id"] = line_map.first;
    line_data["begin"]["x"] = line_map.second.line_data().begin().x();
    line_data["begin"]["y"] = line_map.second.line_data().begin().y();
    line_data["end"]["x"] = line_map.second.line_data().end().x();
    line_data["end"]["y"] = line_map.second.line_data().end().y();
    root.append(line_data);
  }

  Json::StyledWriter writer;
  line_map_file << writer.write(root);
  line_map_file.close();

  LOG(INFO) << "Save " << line_map_2d_list.size() <<
               "lines to line map file: " << line_map_filename;
  return true;
}

bool MapBuilderBridge::SaveCloudMap(
  const std::string& map_filename, const double resolution)
{
  // 1. 先获取node信息
  cartographer::mapping::MapById<
    cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNode>
  nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
  cartographer::mapping::ValueConversionTables conversion_tables;

  auto create_grid_function = [&](const Eigen::Vector2f& origin)  
    -> std::unique_ptr<cartographer::mapping::GridInterface>
  {
    constexpr int kInitialSubmapSize = 100;
    return absl::make_unique<cartographer::mapping::ProbabilityGrid>(
        cartographer::mapping::MapLimits(resolution,
                                         origin.cast<double>() + 0.5 * kInitialSubmapSize *  
                                         resolution *  
                                         Eigen::Vector2d::Ones(),  
                                         cartographer::mapping::CellLimits(kInitialSubmapSize, kInitialSubmapSize)),  
                                         &conversion_tables);
  };

  const Eigen::Vector2f origin = nodes.begin()->data.global_pose.translation().head<2>().cast<float>();
  std::unique_ptr<cartographer::mapping::Submap2D>
  submap = absl::make_unique<cartographer::mapping::Submap2D>(
                      origin,
                      std::unique_ptr<cartographer::mapping::Grid2D>(
                          static_cast<cartographer::mapping::Grid2D*>(create_grid_function(origin).release())),
                      &conversion_tables,
                      std::make_unique<cartographer::sensor::PointCloud>(),
                      std::make_unique<cartographer::sensor::PointCloud>(),
                      0, std::map<int, cartographer::mapping::MapLine2D>{});
  submap->SetCloudCullingProb(0.65);

  ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(0.65);
  options.set_miss_probability(0.48);
  options.set_insert_free_space(true);

  auto create_range_data_inserter_function = [](const cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D& options)  
      -> std::unique_ptr<cartographer::mapping::RangeDataInserterInterface> {  
      return absl::make_unique<cartographer::mapping::ProbabilityGridRangeDataInserter2D>(  
          options);  
  };  
    
  auto range_data_inserter = create_range_data_inserter_function(options);

  for (const auto& node : nodes) {
    for (const auto& range_data : node.data.constant_data->range_datas) {
      // 将range data转换到global坐标系下
      cartographer::sensor::RangeData global_range_data;
      
      // 转换origin到global坐标系
      const Eigen::Vector3f global_origin = (node.data.global_pose * 
                                             range_data.origin.cast<double>()).cast<float>();
      global_range_data.origin = global_origin;
      
      // 转换returns点云到global坐标系
      global_range_data.returns = cartographer::sensor::TransformPointCloud(
                                    range_data.returns,
                                    node.data.global_pose.cast<float>());
      
      // 转换misses点云到global坐标系（如果有的话）
      global_range_data.misses = cartographer::sensor::TransformPointCloud(
                                   range_data.misses,
                                   node.data.global_pose.cast<float>());

      // 插入range data到submap中进行概率栅格处理
      submap->InsertRangeData(global_range_data, range_data_inserter.get());
    }
    auto transformed_cloud = cartographer::sensor::TransformPointCloud(
                                node.data.constant_data->filtered_gravity_aligned_point_cloud,
                                node.data.global_pose.cast<float>());
    auto transformed_highlight_cloud = cartographer::sensor::TransformPointCloud(
                                        node.data.constant_data->highlight_point_cloud,
                                        node.data.global_pose.cast<float>());
    submap->InsertCloudData(transformed_cloud, transformed_highlight_cloud);
  }
  submap->Finish();

  // 3. 高效点云处理和保存
  LOG(INFO) << "Extracting and filtering point clouds...";
  
  // 使用移动语义避免拷贝
  cartographer::sensor::PointCloud cloud_map = std::move(*submap->cloud());
  
  // 早期过滤，减少后续处理的数据量
  cartographer::sensor::PointCloud filtered_cloud_map = 
      cartographer::sensor::VoxelFilter(cloud_map, resolution);
  
  // 优化的PCL点云转换
  const auto& points = filtered_cloud_map.points();
  const size_t num_points = points.size();
  
  LOG(INFO) << "Converting " << num_points << " points to PCL format...";
  
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_map;
  pcl_cloud_map.resize(num_points);
  
  // 使用指针访问提高效率
  pcl::PointXYZ* pcl_points = pcl_cloud_map.points.data();
  for (size_t i = 0; i < num_points; ++i) {
    const auto& pos = points[i].position;
    pcl_points[i].x = pos.x();
    pcl_points[i].y = pos.y();
    pcl_points[i].z = pos.z();
  }
  
  // 异步保存以提高I/O效率
  const std::string pcd_filename = map_filename + ".pcd";
  pcl::io::savePCDFileBinary(pcd_filename, pcl_cloud_map);
  LOG(INFO) << "Saved cloud map to " << pcd_filename << ", size: " << num_points;

  // 4. 高效处理highlight点云
  cartographer::sensor::PointCloud highlight_cloud_map = std::move(*submap->highlight_cloud());
  
  if (!highlight_cloud_map.empty()) {
    LOG(INFO) << "Processing highlight cloud with " << highlight_cloud_map.size() << " points...";
    
    cartographer::sensor::PointCloud filtered_highlight_cloud_map = 
        cartographer::sensor::VoxelFilter(highlight_cloud_map, resolution);

    const auto& highlight_points = filtered_highlight_cloud_map.points();
    const size_t num_highlight_points = highlight_points.size();
    
    pcl::PointCloud<pcl::PointXYZ> pcl_highlight_cloud_map;
    pcl_highlight_cloud_map.resize(num_highlight_points);
    
    // 使用指针访问优化性能
    pcl::PointXYZ* highlight_pcl_points = pcl_highlight_cloud_map.points.data();
    for (size_t i = 0; i < num_highlight_points; ++i) {
      const auto& pos = highlight_points[i].position;
      highlight_pcl_points[i].x = pos.x();
      highlight_pcl_points[i].y = pos.y();
      highlight_pcl_points[i].z = pos.z();
    }
    
    const std::string highlight_pcd_filename = map_filename + "_highlight.pcd";
    pcl::io::savePCDFileBinary(highlight_pcd_filename, pcl_highlight_cloud_map);
    LOG(INFO) << "Saved highlight cloud map to " << highlight_pcd_filename << ", size: " << num_highlight_points;
  } else {
    LOG(INFO) << "Highlight cloud map is empty, skipping save";
  }

  return true;
}

bool MapBuilderBridge::SaveGridMapFromRange(
  const std::string& map_filename, const double resolution)
{
  // 1. 先获取node信息
  cartographer::mapping::MapById<
    cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNode>
  nodes = map_builder_->pose_graph()->GetTrajectoryNodes();
  
  if (nodes.empty()) {
    LOG(WARNING) << "No trajectory nodes found, cannot save grid map from range.";
    return false;
  }

  cartographer::mapping::ValueConversionTables conversion_tables;

  // 获取第一个节点的位置作为原点
  const Eigen::Vector2f origin = nodes.begin()->data.global_pose.translation().head<2>().cast<float>();

  // 创建一个大的submap来积累所有的range data
  constexpr int kLargeSubmapSize = 2000;  // 创建一个大的地图
  std::unique_ptr<cartographer::mapping::Submap2D>
  accumulated_submap = absl::make_unique<cartographer::mapping::Submap2D>(
      origin,
      std::unique_ptr<cartographer::mapping::Grid2D>(
          static_cast<cartographer::mapping::Grid2D*>(
              absl::make_unique<cartographer::mapping::ProbabilityGrid>(
                  cartographer::mapping::MapLimits(resolution,
                                                   origin.cast<double>(),
                                                   cartographer::mapping::CellLimits(kLargeSubmapSize, kLargeSubmapSize)),
                  &conversion_tables).release())),
      &conversion_tables,
      std::make_unique<cartographer::sensor::PointCloud>(),
      std::make_unique<cartographer::sensor::PointCloud>(),
      0, std::map<int, cartographer::mapping::MapLine2D>{});

  // 设置概率栅格插入器选项
  ::cartographer::mapping::proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(0.65);
  options.set_miss_probability(0.48);
  options.set_insert_free_space(true);

  auto range_data_inserter = 
      absl::make_unique<cartographer::mapping::ProbabilityGridRangeDataInserter2D>(options);

  // 2. 逐帧处理激光数据并插入到栅格地图中
  for (const auto& node : nodes) {
    // 使用新的range_datas字段，包含了各个传感器的原始去畸变range data
    for (const auto& range_data : node.data.constant_data->range_datas) {
      // 将range data转换到global坐标系下
      cartographer::sensor::RangeData global_range_data;
      
      // 转换origin到global坐标系
      const Eigen::Vector3f global_origin = (node.data.global_pose * 
                                             range_data.origin.cast<double>()).cast<float>();
      global_range_data.origin = global_origin;
      
      // 转换returns点云到global坐标系
      global_range_data.returns = cartographer::sensor::TransformPointCloud(
                                    range_data.returns,
                                    node.data.global_pose.cast<float>());
      
      // 转换misses点云到global坐标系（如果有的话）
      global_range_data.misses = cartographer::sensor::TransformPointCloud(
                                   range_data.misses,
                                   node.data.global_pose.cast<float>());
        // LOG(INFO) << "node id: " << node.id << " range_data returns size: " 
        //           << range_data.returns.size() 
        //           << " misses size: " << range_data.misses.size() << std::endl;
      // 插入到概率栅格地图中
      accumulated_submap->InsertRangeData(global_range_data, range_data_inserter.get());
    }
  }

  // 完成submap构建
  accumulated_submap->Finish();

  // 3. 使用与SaveGridMap完全相同的流程 - 创建SubmapSlice并使用PaintSubmapSlices
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
  
  // 创建一个虚拟的submap ID
  const ::cartographer::mapping::SubmapId virtual_id{0, 0};
  ::cartographer::io::SubmapSlice& submap_slice = submap_slices[virtual_id];
  
  // 获取栅格地图
  const cartographer::mapping::Grid2D* grid = accumulated_submap->grid();
  const cartographer::mapping::ProbabilityGrid* probability_grid = 
      static_cast<const cartographer::mapping::ProbabilityGrid*>(grid);

  // 获取地图尺寸
  const cartographer::mapping::MapLimits& limits = probability_grid->limits();
  const int width = limits.cell_limits().num_x_cells;
  const int height = limits.cell_limits().num_y_cells;

  // 创建intensity和alpha数组，使用与DrawToSubmapTexture完全相同的逻辑
  std::vector<char> intensity_data;
  std::vector<char> alpha_data;
  intensity_data.reserve(width * height);
  alpha_data.reserve(width * height);
  
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      const Eigen::Array2i cell_index(j, i);  // 注意：这里j是x，i是y
      
      // 检查是否为已知区域（与DrawToSubmapTexture中的IsKnown逻辑相同）
      if (!probability_grid->IsKnown(cell_index)) {
        // 未知区域：intensity=0, alpha=0（与DrawToSubmapTexture完全相同）
        intensity_data.push_back(0);
        alpha_data.push_back(0);
      } else {
        // 使用与DrawToSubmapTexture完全相同的逻辑
        // 1. 获取概率并使用ProbabilityToLogOddsInteger转换
        const float probability = probability_grid->GetProbability(cell_index);
        const int log_odds_integer = ::cartographer::mapping::ProbabilityToLogOddsInteger(probability);
        
        // 2. 计算delta值（与DrawToSubmapTexture中相同）
        const int delta = 128 - log_odds_integer;
        
        // 3. 计算intensity(value)和alpha（与DrawToSubmapTexture中相同）
        const uint8_t alpha_value = delta > 0 ? 0 : static_cast<uint8_t>(-delta);
        const uint8_t value = delta > 0 ? static_cast<uint8_t>(delta) : 0;
        
        intensity_data.push_back(static_cast<char>(value));
        alpha_data.push_back(static_cast<char>((value || alpha_value) ? alpha_value : 1));
      }
    }
  }
  
  // 设置submap slice属性
  submap_slice.pose = cartographer::transform::Rigid3d::Identity();
  submap_slice.width = width;
  submap_slice.height = height;
  submap_slice.resolution = resolution;
  submap_slice.slice_pose = cartographer::transform::Rigid3d::Identity();
  submap_slice.cairo_data.clear();
  
  // 使用cartographer的DrawTexture函数生成surface
  submap_slice.surface = cartographer::io::DrawTexture(
      intensity_data, alpha_data, width, height, &submap_slice.cairo_data);

  // 使用与SaveGridMap相同的PaintSubmapSlices函数
  auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  
  // 转换为Image并保存
  ::cartographer::io::Image image(std::move(result.surface));
  
  // 保存PGM文件
  cartographer::io::StreamFileWriter pgm_writer(map_filename + ".pgm");
  WritePgm(image, resolution, &pgm_writer,
           node_options_.mapping_occ_thresh,
           node_options_.mapping_free_thresh);
  
  // 保存PNG文件
  WritePng(image, map_filename + ".png",
           node_options_.mapping_occ_thresh,
           node_options_.mapping_free_thresh);

  // 计算地图原点（使用与SaveGridMap相同的逻辑）
  const Eigen::Vector2d map_origin(
    -result.origin.x() * resolution,
    (result.origin.y() - image.height()) * resolution);

  // 保存YAML文件
  cartographer::io::StreamFileWriter yaml_writer(map_filename + ".yaml");
  WriteYaml(resolution, map_origin,
            map_filename, &yaml_writer,
            node_options_.mapping_occ_thresh,
            node_options_.mapping_free_thresh);

  LOG(INFO) << "Saving Grid Map From Range Success. Saved to " 
            << map_filename + ".pgm/.png/.yaml"
            << " with size " << image.width() << "x" << image.height();

  return true;
}


bool MapBuilderBridge::SaveGridMap(
  const std::string& map_filename, const double resolution)
{
  // 1. 获取所有的子图信息
  cartographer_ros_msgs::msg::SubmapList submap_list;
  submap_list.header.stamp = rclcpp::Time();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::msg::SubmapEntry submap_entry;
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }

  std::unique_ptr<
    std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>>
  submap_slices
    = absl::make_unique<
      std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice>>();

  for (const auto & submap_entry : submap_list.submap) {
    const ::cartographer::mapping::SubmapId
      id{submap_entry.trajectory_id, submap_entry.submap_index};

    ::cartographer::io::SubmapSlice& submap_slice = (*submap_slices)[id];
    submap_slice.pose = ToRigid3d(submap_entry.pose);
    submap_slice.metadata_version = submap_entry.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_entry.submap_version) {
      continue;
    }
    auto fetched_textures = FetchSubmapTextures(id);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();

    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // 2. 根据submap_slices生成PaintSubmapSlices
  auto result =
    ::cartographer::io::PaintSubmapSlices(*submap_slices, resolution);
  // 3. 将PaintSubmapSlices转换为Image，并保存为pgm文件
  ::cartographer::io::Image image(std::move(result.surface));
  ::cartographer::io::StreamFileWriter pgm_writer(map_filename + ".pgm");
  WritePgm(image, resolution, &pgm_writer,
           node_options_.mapping_occ_thresh,
           node_options_.mapping_free_thresh);
  WritePng(image, map_filename + ".png",
           node_options_.mapping_occ_thresh,
           node_options_.mapping_free_thresh);
  const Eigen::Vector2d origin(
    -result.origin.x() * resolution,
    (result.origin.y() - image.height()) * resolution);
  // 4. 保存yaml文件
  ::cartographer::io::StreamFileWriter yaml_writer(map_filename + ".yaml");
  WriteYaml(resolution, origin,
            map_filename, &yaml_writer,
            node_options_.mapping_occ_thresh,
            node_options_.mapping_free_thresh);
  LOG(INFO) << "Saving Grid Map Success." << std::endl;

  return true;
}

/**
 * @brief 获取指定id的submap的texture（内容信息）
 * 
 * @param submap_id 
 * @param srv 
 * @return std::unique_ptr<::cartographer::io::SubmapTextures> 
 */
std::unique_ptr<::cartographer::io::SubmapTextures>
MapBuilderBridge::FetchSubmapTextures(const ::cartographer::mapping::SubmapId& submap_id)
{
  using ServiceT = cartographer_ros_msgs::srv::SubmapQuery;
  auto req = std::make_shared<ServiceT::Request>();
  auto resp = std::make_shared<ServiceT::Response>();

  req->trajectory_id = submap_id.trajectory_id;
  req->submap_index = submap_id.submap_index;

  // 直接调用本节点内部处理函数
  this->HandleSubmapQuery(req, resp);

  if (resp->textures.empty() ||
      resp->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    // 失败则返回 nullptr
    return nullptr;
  }

  auto result = absl::make_unique<::cartographer::io::SubmapTextures>();
  result->version = resp->submap_version;

  for (const auto& texture : resp->textures) {
    const std::string compressed_cells(texture.cells.begin(), texture.cells.end());
    result->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(
            compressed_cells, texture.width, texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }

  return result;
}

bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
  return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                            filename);
}

/**
 * @brief 获取某个轨迹下某个id的子图信息
 * 
 * @param Request 
 * @param Response 
 */
void MapBuilderBridge::HandleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr Request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr Response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{Request->trajectory_id,
                                            Request->submap_index};
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    Response->status.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    Response->status.message = error;
    return;
  }
  Response->submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    // 向textures中添加一个texture
    Response->textures.emplace_back();
    auto& texture = Response->textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  Response->status.message = "Success.";
  Response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
}

std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  for (const auto& sensor_bridge : sensor_bridges_) {
    trajectory_states.insert(std::make_pair(
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE));
  }
  return trajectory_states;
}

/**
 * @brief 从map_builder_中获取所有的子图信息
 * 
 * @return cartographer_ros_msgs::SubmapList 
 */
cartographer_ros_msgs::msg::SubmapList MapBuilderBridge::GetSubmapList(rclcpp::Time node_time)
{
  cartographer_ros_msgs::msg::SubmapList submap_list;
  submap_list.header.stamp = node_time;
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::msg::SubmapEntry submap_entry;
    submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(
        submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}


visualization_msgs::msg::MarkerArray MapBuilderBridge::GetFeatureMap(){
  static visualization_msgs::msg::MarkerArray map_list;
  if(!map_list.markers.empty())
    return map_list;
  int marker_id = 0;
  for(auto& marker:GetLineMap(marker_id))
    map_list.markers.push_back(marker);
  // for(auto& marker:GetCodeMap(marker_id))
  //   map_list.markers.push_back(marker);
  for(auto& marker:GetReflectorMap(marker_id))
    map_list.markers.push_back(marker);
  return map_list;
}


std::vector<visualization_msgs::msg::Marker>
      MapBuilderBridge::GetLineMap(int& marker_id){
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> line_map = 
      CreateLineMapMsg(rclcpp::Time(), node_options_.map_frame, 
                      marker_id, map_builder_->GetLineMap());
  if(line_map == nullptr)
    return {};
  return *line_map;
}


// std::vector<visualization_msgs::Marker> MapBuilderBridge::GetCodeMap(int& marker_id){
//   std::unique_ptr<std::vector<visualization_msgs::Marker>> code_map
//     = CreateCodeMapMsg(ros::Time::now(), node_options_.map_frame, 
//           marker_id, map_builder_->GetCodeMap());
//   if(code_map == nullptr){
//     return std::vector<visualization_msgs::Marker>();
//   }
//   return *code_map;
// }


std::vector<visualization_msgs::msg::Marker> MapBuilderBridge::GetReflectorMap(int& marker_id)
{
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> reflector_map = 
      CreateReflectorMapMsg(rclcpp::Time(), node_options_.map_frame, 
                        marker_id, map_builder_->GetReflectorMap());
  if(reflector_map == nullptr)
    return {};
  return *reflector_map;
}


// std::vector<Eigen::Vector2d> MapBuilderBridge::GetAllReflectors(){  
//   return map_builder_->pose_graph()->GetAllReflectors();
// }

/**
 * @brief 获取当前时间局部轨迹数据
 * 
 * @return std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData> 轨迹id以及对应的局部轨迹数据
 */
std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    // 获取当前轨迹的local_slam_data数据
    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      absl::MutexLock lock(&mutex_);
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    // 获取局部坐标系轨迹数据
    local_trajectory_data[trajectory_id] = {
        local_slam_data,
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id), // local to map // noted by hz: 获取global->local的转换，实现在pose_graph_2d
        sensor_bridge.tf_bridge().LookupToTracking(  // tracking to tracking
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};

    // LOG(INFO) << map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
  }

  return local_trajectory_data;
}

void MapBuilderBridge::HandleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr Request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr Response) {
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  for (const auto& node_id_data :
       node_poses.trajectory(Request->trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = node_options_.map_frame;
    pose_stamped.header.stamp =
        ToRos(node_id_data.data.constant_pose_data.value().time);
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    Response->trajectory.push_back(pose_stamped);
  }
  Response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  Response->status.message =
      "Retrieved " + std::to_string(Response->trajectory.size()) +
      " trajectory nodes from trajectory " + std::to_string(Request->trajectory_id) + ".";
}

cartographer::mapping::MapById<
  cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNodePose>
MapBuilderBridge::GetTrajectoryPoses() const
{
  return map_builder_->pose_graph()->GetTrajectoryNodePoses();
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetTrajectoryNodeList(
    rclcpp::Time node_time)
{
  visualization_msgs::msg::MarkerArray trajectory_node_list;
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }
  const auto constraints = map_builder_->pose_graph()->constraints();
  for (const auto& constraint : constraints) {
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::msg::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame, node_time);
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }

    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::msg::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetLandmarkPosesList(
    rclcpp::Time node_time)
{
  visualization_msgs::msg::MarkerArray landmark_poses_list;
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame, node_time));
  }
  return landmark_poses_list;
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetConstraintList(rclcpp::Time node_time) {
  visualization_msgs::msg::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::msg::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = node_time;
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::msg::Marker *constraint_marker, *residual_marker;
    std_msgs::msg::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

/**
 * @brief 获取对应轨迹id的SensorBridge指针
 * 
 * @param[in] trajectory_id 
 * @return SensorBridge* 
 */
SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

/**
 * @brief 局部slam结果回调函数（获取局部坐标系中的local_slam_data）
 * 
 * @param trajectory_id 轨迹id
 * @param time          时间
 * @param local_pose    局部位姿
 * @param range_data_in_local 局部地图中的激光数据
 */
void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    std::vector<::cartographer::transform::Rigid2d> particle_poses,
    ::cartographer::sensor::RangeData range_data_in_local,
    std::shared_ptr<::cartographer::sensor::PointCloud> local_map,
    std::map<int, std::vector<::cartographer::sensor::LineData>>  line_data_in_local,
    std::shared_ptr<::cartographer::mapping::LineMap2DList> line_map_2d_list) {
  // 构造当前轨迹的local_slam_data
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time, local_pose, particle_poses,
                                             std::move(range_data_in_local),
                                             local_map,
                                             std::move(line_data_in_local),
                                             line_map_2d_list
                                             });
  absl::MutexLock lock(&mutex_);
  // 将local_slam_data保存到local_slam_data_中
  local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

}  // namespace cartographer_ros
