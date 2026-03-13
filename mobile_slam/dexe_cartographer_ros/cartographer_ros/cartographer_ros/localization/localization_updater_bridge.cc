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

#include "cartographer_ros/localization/localization_updater_bridge.h"
#include <fstream>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <jsoncpp/json/json.h>

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/image.h"
#include "cartographer/io/file_writer.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/2d/submap_2d.h"
#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"
#include "cartographer/mapping/2d/line_map_2d.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/localization/2d/local_map_updater_2d.h"
#include "cartographer/localization/internal/2d/pose_estimator_2d.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/conversion/time_conversion.h"
#include "cartographer_ros/mapping/ros_map.h"

#include "cartographer_ros_msgs/msg/status_code.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.2;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::msg::ColorRGBA ToMessage(const cartographer::io::FloatColor& color)
{
  ::std_msgs::msg::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::msg::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                       const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.stamp = rclcpp::Clock().now();
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index)
{
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = static_cast<int>(landmark_id_to_index->size());
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

visualization_msgs::msg::Marker CreateLandmarkMarker(const int landmark_index,
                                                     const Rigid3d& landmark_pose,
                                                     const std::string& frame_id)
{
  visualization_msgs::msg::Marker marker;
  marker.ns = "Landmarks";
  marker.id = landmark_index;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.header.stamp = rclcpp::Clock().now();
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(landmark_index));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::msg::Marker* marker,
                            std::vector<visualization_msgs::msg::Marker>* markers)
{
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace  

LocalizationUpdaterBridge::LocalizationUpdaterBridge(
    const LocalizationNodeOptions& node_options,
    std::unique_ptr<cartographer::localization::LocalizationUpdater> localization_updater,
    tf2_ros::Buffer* const tf_buffer)  
    : node_options_(node_options),
      localization_updater_(std::move(localization_updater)),
      tf_buffer_(tf_buffer) {}

void LocalizationUpdaterBridge::LoadState(const std::string& state_filename,
                                          bool load_frozen_state)
{
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - static_cast<int>(suffix.size()), 0)),
           suffix)  
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  cartographer::io::ProtoStreamReader stream(state_filename);
  localization_updater_->LoadState(&stream, load_frozen_state);
}

bool LocalizationUpdaterBridge::SetInitialPose(const carto::transform::Rigid3d &initial_pose)
{
  return localization_updater_->SetInitialPose(initial_pose);
}

std::optional<cartographer::localization::LocalizationState>
LocalizationUpdaterBridge::GetLocalizationState() const
{
  return localization_updater_->GetLocalizationState();
}

/**
 * @brief 根据传感器id和配置文件，创建位姿优化器
 * 
 * @param expected_sensor_ids 所有传感器的id
 * @param trajectory_options 轨迹的配置
 * @return int 轨迹id
 */
int LocalizationUpdaterBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  // 1. 构建一个轨迹构建器，并将该id的map_builder与OnLocalSlamResult进行绑定
  const int trajectory_id = localization_updater_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      [this](const int trajectory_id, const ::cartographer::common::Time time,
             const Rigid3d local_pose,
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
        OnLocalSlamResult(trajectory_id, time, local_pose,
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
      localization_updater_->GetTrajectoryBuilder(trajectory_id));

  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void LocalizationUpdaterBridge::FinishTrajectory(const int trajectory_id)
{
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  localization_updater_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

/**
 * @brief 获取指定id的submap的texture（内容信息）
 * 
 * @param submap_id 
 * @param srv 
 * @return std::unique_ptr<::cartographer::io::SubmapTextures> 
 */
std::unique_ptr<::cartographer::io::SubmapTextures>
LocalizationUpdaterBridge::FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    cartographer_ros_msgs::srv::SubmapQuery::Response & response)
{
  cartographer_ros_msgs::srv::SubmapQuery::Request request;
  request.trajectory_id = submap_id.trajectory_id;
  request.submap_index = submap_id.submap_index;

  this->HandleSubmapQuery(request, response);
  if (response.textures.empty()) {
    return nullptr;
  }
  auto out = absl::make_unique<::cartographer::io::SubmapTextures>();
  out->version = response.submap_version;
  for (const auto& texture : response.textures) {
    const std::string compressed_cells(texture.cells.begin(), texture.cells.end());
    out->textures.emplace_back(::cartographer::io::SubmapTexture{
        ::cartographer::io::UnpackTextureData(compressed_cells, texture.width, texture.height),
        texture.width, texture.height, texture.resolution,
        ToRigid3d(texture.slice_pose)});
  }
  return out;
}

bool LocalizationUpdaterBridge::SerializeState(const std::string& filename,
                                               const bool include_unfinished_submaps) {
  return localization_updater_->SerializeStateToFile(include_unfinished_submaps, filename);
}

bool LocalizationUpdaterBridge::SaveMap(const std::string& map_filename,
                                        const float occ_thresh,
                                        const float free_thresh) {
  LOG(INFO) << "LocalizationUpdaterBridge::SaveMap - saving to: " << map_filename;

  // 通过 PoseEstimator2D 获取当前全局 submap
  // PERSISTENT 策略下返回 global_dynamic_submap_（含运行时更新）
  // NONE 策略下返回 global_static_submap_（原始地图）
  auto* pose_estimator = localization_updater_->GetPoseEstimator2D();
  if (!pose_estimator) {
    LOG(ERROR) << "SaveMap: PoseEstimator2D is null";
    return false;
  }

  auto* local_map_updater = pose_estimator->GetLocalMapUpdater();
  if (!local_map_updater) {
    LOG(ERROR) << "SaveMap: LocalMapUpdater2D is null";
    return false;
  }

  auto submap = local_map_updater->GetGlobalSubmap();
  if (!submap) {
    // 如果 GetGlobalSubmap 返回 nullptr（如 NONE 策略但未初始化），尝试静态 submap
    submap = pose_estimator->GetGlobalStaticSubmap();
    if (!submap) {
      LOG(ERROR) << "SaveMap: Global submap is null, cannot generate map image";
      return false;
    }
  }

  const cartographer::mapping::Grid2D* grid = submap->grid();
  if (!grid) {
    LOG(ERROR) << "SaveMap: Grid is null";
    return false;
  }

  const cartographer::mapping::ProbabilityGrid* probability_grid =
      static_cast<const cartographer::mapping::ProbabilityGrid*>(grid);

  // 裁剪空白边缘，与 SaveGridMapFromRange / DrawToSubmapTexture 保持一致
  Eigen::Array2i offset;
  cartographer::mapping::CellLimits cropped_cell_limits;
  probability_grid->ComputeCroppedLimits(&offset, &cropped_cell_limits);

  const cartographer::mapping::MapLimits& limits = probability_grid->limits();
  const double resolution = limits.resolution();
  const int width = cropped_cell_limits.num_x_cells;
  const int height = cropped_cell_limits.num_y_cells;

  LOG(INFO) << "SaveMap: cropped grid dimensions " << width << "x" << height
            << " (original "
            << limits.cell_limits().num_x_cells << "x"
            << limits.cell_limits().num_y_cells << ")"
            << ", resolution=" << resolution;

  // 构建强度/透明度数组（与 MapBuilderBridge::SaveGridMapFromRange 相同逻辑）
  std::vector<char> intensity_data;
  std::vector<char> alpha_data;
  intensity_data.reserve(width * height);
  alpha_data.reserve(width * height);

  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      // 使用裁剪偏移量访问正确的 cell
      const Eigen::Array2i cell_index(j + offset.x(), i + offset.y());
      if (!probability_grid->IsKnown(cell_index)) {
        intensity_data.push_back(0);
        alpha_data.push_back(0);
      } else {
        const float probability = probability_grid->GetProbability(cell_index);
        const int log_odds_integer =
            ::cartographer::mapping::ProbabilityToLogOddsInteger(probability);
        const int delta = 128 - log_odds_integer;
        const uint8_t alpha_value = delta > 0 ? 0 : static_cast<uint8_t>(-delta);
        const uint8_t value = delta > 0 ? static_cast<uint8_t>(delta) : 0;
        intensity_data.push_back(static_cast<char>(value));
        alpha_data.push_back(static_cast<char>((value || alpha_value) ? alpha_value : 1));
      }
    }
  }

  // 计算 slice_pose（与 DrawToSubmapTexture / SaveGridMapFromRange 一致，考虑裁剪偏移）
  const double max_x = limits.max().x() - resolution * offset.y();
  const double max_y = limits.max().y() - resolution * offset.x();

  const auto local_pose = submap->local_pose();
  const auto slice_pose = local_pose.inverse() *
      cartographer::transform::Rigid3d::Translation(Eigen::Vector3d(max_x, max_y, 0.));

  // 填充 SubmapSlice
  std::map<::cartographer::mapping::SubmapId, ::cartographer::io::SubmapSlice> submap_slices;
  const ::cartographer::mapping::SubmapId virtual_id{0, 0};
  ::cartographer::io::SubmapSlice& submap_slice = submap_slices[virtual_id];
  submap_slice.pose = local_pose;
  submap_slice.width = width;
  submap_slice.height = height;
  submap_slice.resolution = resolution;
  submap_slice.slice_pose = slice_pose;
  submap_slice.cairo_data.clear();
  submap_slice.surface = cartographer::io::DrawTexture(
      intensity_data, alpha_data, width, height, &submap_slice.cairo_data);

  // 生成图像
  auto result = ::cartographer::io::PaintSubmapSlices(submap_slices, resolution);
  ::cartographer::io::Image image(std::move(result.surface));

  // 保存 PNG
  WritePng(image, map_filename + ".png", occ_thresh, free_thresh);

  // 计算原点并保存 YAML
  const Eigen::Vector2d map_origin(
      -result.origin.x() * resolution,
      (result.origin.y() - image.height()) * resolution);

  cartographer::io::StreamFileWriter yaml_writer(map_filename + ".yaml");
  WriteYaml(resolution, map_origin, map_filename, &yaml_writer, occ_thresh, free_thresh);

  LOG(INFO) << "SaveMap: successfully saved " << map_filename << ".png/.yaml"
            << " (" << image.width() << "x" << image.height() << " pixels)";

  // 生成 navigation map（对 map.png 进行噪声过滤，生成适合导航的地图）
  if (node_options_.save_navigation_map) {
    // 从 map_filename 推导 map 目录
    std::string map_dir;
    size_t last_slash = map_filename.find_last_of("/");
    if (last_slash != std::string::npos) {
      map_dir = map_filename.substr(0, last_slash + 1);
    } else {
      map_dir = "./";
    }
    WriteNavigationMap(map_filename + ".png",
                       map_filename + "_navigation.png",
                       node_options_.navigation_map_filter_mode,
                       node_options_.save_debug_map,
                       map_dir);
    LOG(INFO) << "SaveMap: navigation map saved to " << map_filename << "_navigation.png";
  }

  return true;
}

/**
 * @brief 获取某个轨迹下某个id的子图信息
 * 
 * @param request 
 * @param response 
 */
void LocalizationUpdaterBridge::HandleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request& request,
    cartographer_ros_msgs::srv::SubmapQuery::Response& response)
{
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request.trajectory_id, request.submap_index};
  const std::string error = localization_updater_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(ERROR) << error;
    response.status.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    response.status.message = error;
    return;
  }
  response.submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    // 向textures中添加一个texture
    response.textures.emplace_back();
    // 取刚刚加入的texture
    auto& texture = response.textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response.status.message = "Success.";
  response.status.code = cartographer_ros_msgs::msg::StatusCode::OK;
}

std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
LocalizationUpdaterBridge::GetTrajectoryStates()
{
  auto trajectory_states = localization_updater_->pose_graph()->GetTrajectoryStates();
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
 * @return cartographer_ros_msgs::msg::SubmapList
 */
cartographer_ros_msgs::msg::SubmapList LocalizationUpdaterBridge::GetSubmapList() {
  cartographer_ros_msgs::msg::SubmapList submap_list;
  submap_list.header.stamp = rclcpp::Clock().now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :  
       localization_updater_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::msg::SubmapEntry submap_entry;
    submap_entry.is_frozen =  
        localization_updater_->pose_graph()->IsTrajectoryFrozen(submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}


visualization_msgs::msg::MarkerArray LocalizationUpdaterBridge::GetFeatureMap()
{
  static visualization_msgs::msg::MarkerArray map_list;
  if (!map_list.markers.empty()) {
    return map_list;
  }
  int marker_id = 0;
  for (auto& marker : GetLineMap(marker_id)) {
    map_list.markers.push_back(marker);
  }
  // for(auto& marker:GetCodeMap(marker_id))
  //   map_list.markers.push_back(marker);
  for (auto& marker : GetReflectorMap(marker_id)) {
    map_list.markers.push_back(marker);
  }
  return map_list;
}

sensor_msgs::msg::PointCloud2 LocalizationUpdaterBridge::GetCloudMap()
{
  CHECK(localization_updater_ != nullptr);
  const auto g_cloud_map = localization_updater_->GetGlobalCloudMap();
  const auto cloud_map_msg = ToPointCloud2Message(ToUniversal(FromRos(rclcpp::Clock().now())),
                                                 node_options_.map_frame,
                                                 *g_cloud_map->cloud());
  return cloud_map_msg;
}

std::vector<visualization_msgs::msg::Marker>
LocalizationUpdaterBridge::GetLineMap(int& marker_id)
{
  const auto g_line_map = localization_updater_->GetGlobalLineMap();
  if (g_line_map == nullptr) {
    return {};
  }
  const auto line_map_msg = CreateLineMapMsg(rclcpp::Clock().now(),
                                             node_options_.map_frame,
                                             marker_id, g_line_map, COLOR::black);

  return *line_map_msg;
}


// std::vector<visualization_msgs::Marker> LocalizationUpdaterBridge::GetCodeMap(int& marker_id){
//   std::unique_ptr<std::vector<visualization_msgs::Marker>> code_map
//     = CreateCodeMapMsg(ros::Time::now(), node_options_.map_frame, 
//           marker_id, localization_updater_->GetCodeMap());
//   if(code_map == nullptr){
//     return std::vector<visualization_msgs::Marker>();
//   }
//   return *code_map;
// }


std::vector<visualization_msgs::msg::Marker>
LocalizationUpdaterBridge::GetReflectorMap(int& marker_id)
{
  std::unique_ptr<std::vector<visualization_msgs::msg::Marker>> reflector_map =  
      CreateReflectorMapMsg(rclcpp::Clock().now(), node_options_.map_frame,
                            marker_id, localization_updater_->GetReflectorMap());
  if (reflector_map == nullptr) {
    return {};
  }
  return *reflector_map;
}


// std::vector<Eigen::Vector2d> LocalizationUpdaterBridge::GetAllReflectors(){  
//   return localization_updater_->pose_graph()->GetAllReflectors();
// }

/**
 * @brief 获取当前时间局部轨迹数据
 * 
 * @return std::unordered_map<int, LocalizationUpdaterBridge::LocalTrajectoryData> 轨迹id以及对应的局部轨迹数据
 */
std::unordered_map<int, LocalizationUpdaterBridge::LocalTrajectoryData>
LocalizationUpdaterBridge::GetLocalTrajectoryData()
{
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
        localization_updater_->pose_graph()->GetLocalToGlobalTransform(trajectory_id), // local to map
        sensor_bridge.tf_bridge().LookupToTracking(  // tracking to tracking
            local_slam_data->time,
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};

    // LOG(INFO) << localization_updater_->pose_graph()->GetLocalToGlobalTransform(trajectory_id);
  }
  return local_trajectory_data;
}

void LocalizationUpdaterBridge::HandleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request& request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response& response)
{
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
  const auto node_poses = localization_updater_->pose_graph()->GetTrajectoryNodePoses();
  for (const auto& node_id_data : node_poses.trajectory(request.trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = node_options_.map_frame;
    pose_stamped.header.stamp =
        ToRos(node_id_data.data.constant_pose_data.value().time);
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    response.trajectory.push_back(pose_stamped);
  }
  response.status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response.status.message = absl::StrCat(
      "Retrieved ", response.trajectory.size(),
      " trajectory nodes from trajectory ", request.trajectory_id, ".");
}

cartographer::mapping::MapById<
    cartographer::mapping::NodeId, cartographer::mapping::TrajectoryNodePose>
LocalizationUpdaterBridge::GetTrajectoryPoses() const
{
  return localization_updater_->pose_graph()->GetTrajectoryNodePoses();
}

visualization_msgs::msg::MarkerArray LocalizationUpdaterBridge::GetTrajectoryNodeList() {
  visualization_msgs::msg::MarkerArray trajectory_node_list;
  const auto node_poses = localization_updater_->pose_graph()->GetTrajectoryNodePoses();
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
  const auto constraints = localization_updater_->pose_graph()->constraints();
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
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame);
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    if (localization_updater_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
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

      if (node_id_data.id.node_index == last_inter_trajectory_constrained_node) {
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

visualization_msgs::msg::MarkerArray LocalizationUpdaterBridge::GetLandmarkPosesList()
{
  visualization_msgs::msg::MarkerArray landmark_poses_list;
  const std::map<std::string, Rigid3d> landmark_poses =
      localization_updater_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        GetLandmarkIndex(id_to_pose.first, &landmark_to_index_),
        id_to_pose.second, node_options_.map_frame));
  }
  return landmark_poses_list;
}

visualization_msgs::msg::MarkerArray LocalizationUpdaterBridge::GetConstraintList()
{
  visualization_msgs::msg::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::msg::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = rclcpp::Clock().now();
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
      localization_updater_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = localization_updater_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = localization_updater_->pose_graph()->constraints();

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
      if (constraint.node_id.trajectory_id == constraint.submap_id.trajectory_id) {
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

SensorBridge* LocalizationUpdaterBridge::sensor_bridge(const int trajectory_id) {
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
void LocalizationUpdaterBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local,
    std::shared_ptr<::cartographer::sensor::PointCloud> local_map,
    std::map<int, std::vector<::cartographer::sensor::LineData>>  line_data_in_local,
    std::shared_ptr<::cartographer::mapping::LineMap2DList> line_map_2d_list) {
  // 构造当前轨迹的local_slam_data
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time, local_pose,
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
