#include "cartographer/preprocess/data_preprocessor.h"

#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/internal/voxel_filter.h"
#include "cartographer/sensor/sync_point_cloud.h"
#include "cartographer/transform/transform.h"
#include <future>
#include <thread>

namespace cartographer {
namespace preprocess {

DataPreprocessor::DataPreprocessor(const proto::DataPreprocessorOptions& options)
: options_(options)
{
}

/**
 * @brief 处理多个传感器的同步数据
 */
MergedData DataPreprocessor::ProcessMultipleSensorData(
    const std::map<std::string, sensor::TimedPointCloudData>& sensor_data_map,
    const common::Time& sync_time) {
  
  std::vector<ProcessedSensorData> processed_sensors;
  
  const size_t num_sensors = sensor_data_map.size();
  // std::cout << "Processing data from " << num_sensors << " sensors at time: " << sync_time << std::endl;
  if (num_sensors == 1) {
    // 单个传感器时直接处理，避免线程开销
    for (const auto& sensor_pair : sensor_data_map) {
      const std::string& sensor_id = sensor_pair.first;
      const sensor::TimedPointCloudData& laser_data = sensor_pair.second;    
      auto processed_data = ProcessSingleSensorData(sensor_id, laser_data, sync_time);
      if (processed_data.has_value()) {
        processed_sensors.push_back(processed_data.value());
      }
    }
  } else if (options_.enable_multi_sensor_fusion()) {
    // 多传感器时使用C++标准库多线程并行处理
    std::vector<std::future<absl::optional<ProcessedSensorData>>> futures;
    futures.reserve(num_sensors);
    
    // 为每个传感器启动异步任务
    for (const auto& sensor_pair : sensor_data_map) {
      const std::string& sensor_id = sensor_pair.first;
      const sensor::TimedPointCloudData& laser_data = sensor_pair.second;
      
      // 使用std::async启动异步任务
      auto future = std::async(std::launch::async, [this, sensor_id, laser_data, sync_time]() {
        return ProcessSingleSensorData(sensor_id, laser_data, sync_time);
      });
      
      futures.push_back(std::move(future));
    }
    
    // 等待所有任务完成并收集结果
    for (auto& future : futures) {
      auto processed_data = future.get();  // 等待任务完成
      if (processed_data.has_value()) {
        processed_sensors.push_back(processed_data.value());
      }
    }
  } else {
    // 不启用多传感器融合时，顺序处理
    for (const auto& sensor_pair : sensor_data_map) {
      const std::string& sensor_id = sensor_pair.first;
      const sensor::TimedPointCloudData& laser_data = sensor_pair.second;    
      auto processed_data = ProcessSingleSensorData(sensor_id, laser_data, sync_time);
      if (processed_data.has_value()) {
        processed_sensors.push_back(processed_data.value());
      }
    }
  }
  
  return GetMergedData(processed_sensors);
}

/**
 * @brief 处理单个传感器数据
 */
absl::optional<ProcessedSensorData> DataPreprocessor::ProcessSingleSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& laser_data,
    const common::Time& sync_time) {
  
  // 获取或创建对应的激光感知器
  auto* laser_perception = GetLaserPerception(sensor_id);
  if (!laser_perception && options_.enable_feature_detection()) {
    CreateLaserPerception(sensor_id);
    laser_perception = GetLaserPerception(sensor_id);
  }
  
  // 将时间点云数据转换为同步格式
  sensor::TimedPointCloudOriginData sync_laser_data;
  sync_laser_data.time = laser_data.time;
  sync_laser_data.origins.push_back(laser_data.origin);
  sync_laser_data.sensor_to_trackings.push_back(laser_data.sensor_to_tracking);
  
  assert(laser_data.ranges.size() == laser_data.intensities.size() || 
         laser_data.intensities.empty());

  // 转换点云数据格式，并保留强度信息
  for (size_t i = 0; i < laser_data.ranges.size(); ++i) {
    sensor::TimedPointCloudOriginData::RangeMeasurement range_measurement;
    range_measurement.point_time = laser_data.ranges[i];
    range_measurement.intensity = laser_data.intensities.empty() ? 0.0f : laser_data.intensities[i];
    range_measurement.origin_index = 0;
    sync_laser_data.ranges.push_back(range_measurement);
  }
  
  // 数据预处理（去畸变）
  auto undistort_range = UndistortPointCloud(sync_laser_data);
  if (!undistort_range.has_value()) {
    LOG(WARNING) << "DataPreprocessor: Failed to get processed range data for sensor: " << sensor_id
                 << ", odom_buff_size: " << odom_data_buff_.size();
    if (!odom_data_buff_.empty()) {
      LOG(WARNING) << "Odom buffer range: " << odom_data_buff_.front().time 
                   << " to " << odom_data_buff_.back().time;
    }
    LOG(WARNING) << "Input laser data time: " << laser_data.time
                 << ", sync laser data time: " << sync_laser_data.time
                 << ", ranges count: " << sync_laser_data.ranges.size();
    if (!sync_laser_data.ranges.empty()) {
      const common::Time first_point_time = sync_laser_data.time + 
          common::FromSeconds(sync_laser_data.ranges.front().point_time.time);
      LOG(WARNING) << "First point time: " << first_point_time;
    }
    return absl::nullopt;
  }
  
  ProcessedSensorData processed_data;
  processed_data.sensor_id = sensor_id;
  processed_data.undistorted_range_data = undistort_range->data;
  
  if (options_.enable_feature_detection() && laser_perception != nullptr) {
    // 特征检测
    sensor::SyncPointCloudWithOdom sync_point_cloud;
    sync_point_cloud.stamp = undistort_range->time;
    // sync_point_cloud.odom = curr_odom_data_;  // 注意：这里需要从外部获取当前里程计数据
    
    sensor::PointCloudWithOrigin cloud;
    cloud.origin = undistort_range->data.origin.cast<double>();
    for (size_t i = 0; i < undistort_range->data.returns.points().size(); i++) {
      cloud.sync_points_base_link.push_back(undistort_range->data.returns.points()[i].position);
    }
    cloud.intensities = undistort_range->data.returns.intensities();
    sync_point_cloud.sync_point_clouds.push_back(cloud);
    
    // 单独进行特征检测
    std::vector<laser_perception::FeatureMsgWithOrigin> feature_data_origin = 
        laser_perception->laserUpdate(sync_point_cloud);
    processed_data.feature_data = GetFeatureData(sync_time, feature_data_origin);
    
    // 获取处理后的点云数据
    processed_data.highlight_point_cloud = *laser_perception->getHighlightCloud();
    processed_data.less_flat_cloud = *laser_perception->getLessFlatCloud();
  }
  
  // std::cout << "Processed sensor: " << sensor_id 
  //           << ", original points: " << undistort_range->data.returns.size()
  //           << ", features - lines: " << processed_data.feature_data.line_data.size()
  //           << ", reflectors: " << processed_data.feature_data.reflector_data.size()
  //           << std::endl;
  return processed_data;
}

/**
 * @brief 获取融合后的数据
 */
MergedData DataPreprocessor::GetMergedData(const std::vector<ProcessedSensorData>& processed_sensors) const {
  MergedData merged_data;
  
  if (processed_sensors.empty()) {
    LOG(WARNING) << "DataPreprocessor: No processed sensor data to merge";
    // 即使没有处理的数据，也要确保返回一个有效的空数据结构
    // 避免在滤波阶段出现问题
    merged_data.merged_filtered_cloud = sensor::PointCloud();
    return merged_data;
  }
  
  // LOG(INFO) << "DataPreprocessor: Merging data from " << processed_sensors.size() << " sensors";
  
  // 初始化合并的range data origin为第一个传感器的origin
  merged_data.merged_range_data.origin = processed_sensors[0].undistorted_range_data.origin;
  
  // 遍历所有传感器数据进行融合
  for (const auto& sensor_data : processed_sensors) {
    const auto& range_data = sensor_data.undistorted_range_data;
    
    // 合并returns点云
    for (const auto& point : range_data.returns) {
      merged_data.merged_range_data.returns.push_back(point);
    }
    
    // 合并misses点云
    for (const auto& point : range_data.misses) {
      merged_data.merged_range_data.misses.push_back(point);
    }
    
    // 合并highlight点云
    for (const auto& point : sensor_data.highlight_point_cloud) {
      merged_data.merged_highlight_cloud.push_back(point);
    }
    
    // 合并less flat点云
    for (const auto& point : sensor_data.less_flat_cloud) {
      merged_data.merged_less_flat_cloud.push_back(point);
    }
    
    // 合并特征数据
    merged_data.merged_line_data.insert(merged_data.merged_line_data.end(),
                                       sensor_data.feature_data.line_data.begin(),
                                       sensor_data.feature_data.line_data.end());
    merged_data.merged_reflector_data.insert(merged_data.merged_reflector_data.end(),
                                            sensor_data.feature_data.reflector_data.begin(),
                                            sensor_data.feature_data.reflector_data.end());
    
    // 保存原始去畸变数据
    merged_data.original_undistorted_range_datas.push_back(range_data);
  }
  
  // 对合并后的返回点云进行自适应体素滤波，生成filtered_gravity_aligned_point_cloud
  if (!merged_data.merged_range_data.returns.empty()) {
    merged_data.merged_filtered_cloud = 
        sensor::AdaptiveVoxelFilter(merged_data.merged_range_data.returns, options_.adaptive_voxel_filter_options());
  } else {
    merged_data.merged_filtered_cloud = sensor::PointCloud();
  }
  
  // 对其他点云数据进行体素滤波处理
  if (options_.voxel_filter_size() > 0.0f) {
    merged_data.merged_filtered_cloud = sensor::VoxelFilter(merged_data.merged_filtered_cloud, options_.voxel_filter_size());
    merged_data.merged_highlight_cloud = sensor::VoxelFilter(merged_data.merged_highlight_cloud, options_.voxel_filter_size());
    merged_data.merged_less_flat_cloud = sensor::VoxelFilter(merged_data.merged_less_flat_cloud, options_.voxel_filter_size());
  }
  
  // LOG(INFO) << "DataPreprocessor: Merged data summary (after filtering):";
  // LOG(INFO) << "  - Total points: " << merged_data.merged_range_data.returns.size();
  // LOG(INFO) << "  - Filtered cloud: " << merged_data.merged_filtered_cloud.size();
  // LOG(INFO) << "  - Highlight cloud: " << merged_data.merged_highlight_cloud.size();
  // LOG(INFO) << "  - Less flat cloud: " << merged_data.merged_less_flat_cloud.size();
  // LOG(INFO) << "  - Lines: " << merged_data.merged_line_data.size();
  // LOG(INFO) << "  - Reflectors: " << merged_data.merged_reflector_data.size();
  // LOG(INFO) << "  - Original undistorted data: " << merged_data.original_undistorted_range_datas.size();
  
  return merged_data;
}

/**
 * @brief 获取激光感知器
 */
laser_perception::LaserPerception* DataPreprocessor::GetLaserPerception(const std::string& sensor_id) {
  auto it = laser_perceptions_.find(sensor_id);
  if (it == laser_perceptions_.end()) {
    return nullptr;
  }
  return it->second.get();
}

/**
 * @brief 创建激光感知器
 */
void DataPreprocessor::CreateLaserPerception(const std::string& sensor_id) {
  if (laser_perceptions_.find(sensor_id) != laser_perceptions_.end()) {
    return; // 已存在
  }
  
  // 从配置选项创建LaserPerceptionOptions
  laser_perception::proto::LaserPerceptionOptions laser_options;
  laser_options.set_predict_rec(options_.laser_perception_options().predict_rec());
  laser_options.set_reflect_rec(options_.laser_perception_options().reflect_rec());
  laser_options.set_shape_rec(options_.laser_perception_options().shape_rec());
  laser_options.set_line_rec(options_.laser_perception_options().line_rec());
  laser_options.set_angle_rec(options_.laser_perception_options().angle_rec());
  laser_options.set_intensity_threshold(options_.laser_perception_options().intensity_threshold());
  laser_options.set_connect_threshold(options_.laser_perception_options().connect_threshold());
  laser_options.set_min_cluster_points(options_.laser_perception_options().min_cluster_points());
  laser_options.set_search_distance(options_.laser_perception_options().search_distance());
  laser_options.set_max_dis_error(options_.laser_perception_options().max_dis_error());
  laser_options.set_max_ang_error(options_.laser_perception_options().max_ang_error());
  laser_options.set_start_rec_dis(options_.laser_perception_options().start_rec_dis());
  laser_options.set_target_file(options_.laser_perception_options().target_file());
  laser_options.set_model_file(options_.laser_perception_options().model_file());
  laser_options.set_min_select_points(options_.laser_perception_options().min_select_points());
  laser_options.set_reflector_error(options_.laser_perception_options().reflector_error());
  laser_options.set_reflector_radius(options_.laser_perception_options().reflector_radius());
  laser_options.set_min_line_length(options_.laser_perception_options().min_line_length());
  laser_options.set_shape_match_score(options_.laser_perception_options().shape_match_score());
  laser_options.set_shape_match_close_degree(options_.laser_perception_options().shape_match_close_degree());
  laser_options.set_shape_dis_to_robot_center(options_.laser_perception_options().shape_dis_to_robot_center());
  laser_options.set_intensity_max_threshold(options_.laser_perception_options().intensity_max_threshold());
  laser_options.set_laser_intensity_type(options_.laser_perception_options().laser_intensity_type());
  laser_options.set_intensity_max_range_threshold(options_.laser_perception_options().intensity_max_range_threshold());
  laser_options.set_intensity_min_range_threshold(options_.laser_perception_options().intensity_min_range_threshold());
  laser_options.set_reflector_error_range(options_.laser_perception_options().reflector_error_range());
  laser_options.set_angle_increment(options_.laser_perception_options().angle_increment());
  
  laser_perceptions_[sensor_id] = std::make_unique<laser_perception::LaserPerception>(laser_options);
  LOG(INFO) << "DataPreprocessor: Created laser perception for sensor: " << sensor_id;
}

/**
 * @brief 获取特征数据
 */
sensor::FeatureData DataPreprocessor::GetFeatureData(
    const common::Time& time,
    const std::vector<laser_perception::FeatureMsgWithOrigin>& feature_data_origin) {
  
  sensor::FeatureData res;
  for (size_t i = 0; i < feature_data_origin.size(); i++) {
    transform::Rigid2d origin = transform::Rigid2d::FromVector(feature_data_origin[i].origin);
    for (const auto& it : feature_data_origin[i].MsgVec) {
      if (it.target_name == "line") {
        Eigen::Vector2d begin = origin * it.line.first.cast<double>().head<2>();
        Eigen::Vector2d end = origin * it.line.second.cast<double>().head<2>();
        res.line_data.push_back(sensor::LineData(begin, end));
      } else if (it.target_name == "reflector") {
        Eigen::Vector2d reflector = origin * Eigen::Vector2d{it.x, it.y};
        res.reflector_data.push_back(
          sensor::LandmarkPointData{time, transform::Rigid3d(), reflector});
      }
    }
  }
  return res;
}

// 外插的精度不够，全部使用内插进行去畸变计算

absl::optional<sensor::TimedRangeData> DataPreprocessor::UndistortPointCloud(const sensor::TimedPointCloudOriginData& cloud_in)
{
  if (!options_.use_odometry()) {
    // 不使用里程计时，直接返回未去畸变的数据
    auto undistort_range_data = sensor::TimedRangeData{cloud_in.time, {{}, {}, {}}};
    undistort_range_data.data.origin = cloud_in.origins.front();
    
    for (const auto& range : cloud_in.ranges) {
      const sensor::RangefinderPoint point = sensor::ToRangefinderPoint(range.point_time);
      undistort_range_data.data.returns.push_back(point, range.intensity);
    }
    return undistort_range_data;
  }

  // 1. 快速检查数据可用性（持锁时间短）
  std::vector<sensor::OdometryData> odom_copy;
  {
    std::unique_lock<std::mutex> lock(odometry_buffer_mutex_);
    
    // 检查里程计数据是否足够
    if (odom_data_buff_.empty() || odom_data_buff_.front().time > cloud_in.time) {
      LOG(WARNING) << "UndistortPointCloud: Odom data insufficient. odom_empty: " << odom_data_buff_.empty()
                   << ", cloud_time: " << cloud_in.time;
      if (!odom_data_buff_.empty()) {
        LOG(WARNING) << "odom_front_time: " << odom_data_buff_.front().time;
      }
      return absl::nullopt;
    }

    const common::Time cloud_time = cloud_in.time;
    const common::Time latest_odom_time = odom_data_buff_.back().time;
    const double time_diff_seconds = common::ToSeconds(cloud_time - latest_odom_time);
    
    // 检查时间差是否过大
    if (time_diff_seconds > 0.1) {
      LOG(WARNING) << "UndistortPointCloud: Odom data too old. odom_back_time: " << latest_odom_time
                   << ", cloud_time: " << cloud_time 
                   << ", time_diff: " << time_diff_seconds << "s";
      return absl::nullopt;
    }

    // 复制需要的里程计数据，释放锁后进行计算
    odom_copy.assign(odom_data_buff_.begin(), odom_data_buff_.end());
  } // 释放锁
  
  LOG(INFO) << "UndistortPointCloud: Processing cloud with time: " << cloud_in.time
            << ", odom_buff_size: " << odom_copy.size();
  
  // 2. 获取第一个点的时间戳进行检查（使用复制的数据）
  const common::Time& time = cloud_in.time;
  const common::Time time_first_point =
      time + common::FromSeconds(cloud_in.ranges.front().point_time.time);
  
  DLOG(INFO) << "UndistortPointCloud: Cloud time: " << time
            << ", first_point_time: " << time_first_point
            << ", odom_front_time: " << odom_copy.front().time;

  // 检查第一个点是否太早（早于最早的里程计数据）
  if (time_first_point < odom_copy.front().time) {
    LOG(WARNING) << "UndistortPointCloud: First point too early. time_first_point: " << time_first_point
                 << ", odom_front_time: " << odom_copy.front().time;
    return absl::nullopt;
  }
  
  // 检查第一个点是否需要过度外插
  const double first_point_time_diff = common::ToSeconds(time_first_point - odom_copy.back().time);
  if (first_point_time_diff > 0.1) {
    LOG(WARNING) << "UndistortPointCloud: First point requires excessive extrapolation. time_first_point: " << time_first_point
                 << ", odom_back_time: " << odom_copy.back().time
                 << ", time_diff: " << first_point_time_diff << "s";
    return absl::nullopt;
  }

  // 3. 去畸变处理
  // 3.1 计算每个点采集时雷达的位姿（在odom坐标系下）
  std::vector<transform::Rigid3f> range_data_poses;
  range_data_poses.reserve(cloud_in.ranges.size());

  for (size_t i = 0; i < cloud_in.ranges.size(); i++) {
    common::Time time_point = time + common::FromSeconds(cloud_in.ranges[i].point_time.time);
    range_data_poses.push_back(GetOdometryPoseFromBuffer(time_point, odom_copy).cast<float>());
  }

  auto undistort_range_data = sensor::TimedRangeData{{}, {{}, {}, {}}};

  // 3.2 在odom坐标系下去畸变
  for (size_t i = 0; i < cloud_in.ranges.size(); ++i) {
    // a 获取该点观测数据
    const sensor::TimedRangefinderPoint& hit =
        cloud_in.ranges[i].point_time;
    if (hit.position.x() > -0.3 && hit.position.x() < 0.3 &&
        hit.position.y() > -0.3 && hit.position.y() < 0.3)
    {
      continue;
    }
    bool is_miss = false;
    const float range = hit.position.head<2>().norm();
    if (range > options_.max_range()) {
      is_miss = true;
    }
    // b 获取该点在odom坐标系下的位姿
    const Eigen::Vector3f origin_in_odom =
        range_data_poses[i] *
        cloud_in.origins.at(cloud_in.ranges[i].origin_index);
    // c 去畸变后的点在odom系下的坐标
    sensor::RangefinderPoint hit_in_odom;
    if (options_.use_deskew()) {
      hit_in_odom = range_data_poses[i] * sensor::ToRangefinderPoint(hit);
    } else {
      hit_in_odom = range_data_poses.back() * sensor::ToRangefinderPoint(hit);
    }

    if (is_miss) {
      const Eigen::Vector3f delta = hit_in_odom.position - origin_in_odom;
      hit_in_odom.position =
          origin_in_odom +
          options_.missing_data_ray_length() / delta.norm() * delta;
      undistort_range_data.data.misses.push_back(hit_in_odom);
      continue;
    }
    // d 保留在min_range和max_range之间的点，并计算器位姿，添加到undistort_range_data中

    undistort_range_data.data.returns.push_back(hit_in_odom, cloud_in.ranges[i].intensity);
  }
  
  // 4. 将去畸变后的点云从odom坐标系转换回雷达坐标系
  // 使用最后一个点的位姿作为参考坐标系（即点云结束时刻雷达的位姿）
  const transform::Rigid3f range_in_odom = range_data_poses.back();
  sensor::TimedRangeData final_range_data;
  final_range_data.time = time;
  final_range_data.data.origin = Eigen::Vector3f::Zero(); // 在雷达坐标系下，origin为零
  
  // 转换returns点云到雷达坐标系
  for (size_t i = 0; i < undistort_range_data.data.returns.size(); i++) {
    auto hit_in_sensor = range_in_odom.inverse() * undistort_range_data.data.returns[i].position;
    final_range_data.data.returns.push_back(
        sensor::RangefinderPoint{hit_in_sensor}, 
        undistort_range_data.data.returns.intensities()[i]);
  }
  
  // 转换misses点云到雷达坐标系
  for (const auto& miss : undistort_range_data.data.misses) {
    auto miss_in_sensor = range_in_odom.inverse() * miss.position;
    final_range_data.data.misses.push_back(sensor::RangefinderPoint{miss_in_sensor});
  }

  return final_range_data;
}

/**
 * @brief 获得该点在odom下的位姿插值（专门负责位姿插值计算）
 * 
 * @param time 时间戳
 * @return transform::Rigid3d 
 */
transform::Rigid3d DataPreprocessor::GetOdometryPose(const common::Time& time) const
{
  std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
  
  if (odom_data_buff_.empty()) {
    return transform::Rigid3d::Identity();
  }
  
  // 如果查询时间早于最早的数据
  if (time < odom_data_buff_.front().time) {
    return odom_data_buff_.front().pose;
  }
  
  // 如果查询时间晚于最新的数据，进行外插
  if (time > odom_data_buff_.back().time) {
    // 外插逻辑：使用最后一个数据点的速度信息
    const auto& last = odom_data_buff_.back();
    const double dt = common::ToSeconds(time - last.time);

    const Eigen::Vector3d w = last.twist.angular; // 角速度
    Eigen::Vector3d v = last.twist.linear;
    v = last.pose.rotation() * v;  // 将线速度转换到世界坐标系

    const Eigen::Vector3d t = last.pose.translation() + dt * v;
    const Eigen::Quaterniond dq =
        transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(w * dt));
    Eigen::Quaterniond q = last.pose.rotation() * dq;
    q.normalize();

    return transform::Rigid3d(t, q);
  }
  
  // 从后往前查找合适的插值区间（优化：大多数查询都是最近时间）
  auto it_lower = odom_data_buff_.end() - 1;
  auto it_upper = odom_data_buff_.end();
  
  // 从后往前找到第一个时间小于等于查询时间的数据点
  for (auto it = odom_data_buff_.rbegin(); it != odom_data_buff_.rend(); ++it) {
    if (it->time <= time) {
      it_lower = it.base() - 1;  // 转换反向迭代器到正向迭代器
      it_upper = it_lower + 1;
      break;
    }
  }
  
  // 如果没找到小于等于查询时间的点，说明查询时间早于所有数据
  if (it_lower == odom_data_buff_.end() - 1 && odom_data_buff_.back().time > time) {
    return odom_data_buff_.front().pose;
  }
  
  // 如果找到的是最后一个点，说明需要外插（但这种情况前面已经处理过了）
  if (it_upper == odom_data_buff_.end()) {
    return it_lower->pose;
  }
  
  // 如果时间完全匹配
  if (it_lower->time == time) {
    return it_lower->pose;
  }
  if (it_upper->time == time) {
    return it_upper->pose;
  }
  
  // 线性插值
  const double duration_to_lower = common::ToSeconds(time - it_lower->time);
  const double duration_between = common::ToSeconds(it_upper->time - it_lower->time);
  
  if (duration_between <= 0.0) {
    return it_lower->pose;
  }
  
  const double interpolation_ratio = duration_to_lower / duration_between;
  
  // 插值位置
  const Eigen::Vector3d translation = 
      it_lower->pose.translation() + 
      interpolation_ratio * (it_upper->pose.translation() - it_lower->pose.translation());
  
  // 插值旋转（球面线性插值）
  const Eigen::Quaterniond rotation = 
      it_lower->pose.rotation().slerp(interpolation_ratio, it_upper->pose.rotation());
  
  return transform::Rigid3d(translation, rotation);
}

/**
 * @brief 从给定缓冲区获得该点在odom下的位姿插值（无锁版本）
 * 
 * @param time 时间戳
 * @param odom_buffer 里程计数据缓冲区
 * @return transform::Rigid3d 
 */
transform::Rigid3d DataPreprocessor::GetOdometryPoseFromBuffer(const common::Time& time, 
                                                               const std::vector<sensor::OdometryData>& odom_buffer) const
{
  if (odom_buffer.empty()) {
    return transform::Rigid3d::Identity();
  }
  
  // 如果查询时间早于最早的数据
  if (time < odom_buffer.front().time) {
    return odom_buffer.front().pose;
  }
  
  // 如果查询时间晚于最新的数据，进行外插
  if (time > odom_buffer.back().time) {
    // 外插逻辑：使用最后一个数据点的速度信息
    const auto& last = odom_buffer.back();
    const double dt = common::ToSeconds(time - last.time);

    const Eigen::Vector3d w = last.twist.angular; // 角速度
    Eigen::Vector3d v = last.twist.linear;
    v = last.pose.rotation() * v;  // 将线速度转换到世界坐标系

    const Eigen::Vector3d t = last.pose.translation() + dt * v;
    const Eigen::Quaterniond dq =
        transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(w * dt));
    Eigen::Quaterniond q = last.pose.rotation() * dq;
    q.normalize();

    return transform::Rigid3d(t, q);
  }
  
  // 从后往前查找合适的插值区间
  auto it_lower = odom_buffer.end() - 1;
  auto it_upper = odom_buffer.end();
  
  for (auto it = odom_buffer.rbegin(); it != odom_buffer.rend(); ++it) {
    if (it->time <= time) {
      it_lower = it.base() - 1;
      it_upper = it_lower + 1;
      break;
    }
  }
  
  if (it_lower == odom_buffer.end() - 1 && odom_buffer.back().time > time) {
    return odom_buffer.front().pose;
  }
  
  if (it_upper == odom_buffer.end()) {
    return it_lower->pose;
  }
  
  // 线性插值
  const double duration_between = common::ToSeconds(it_upper->time - it_lower->time);
  const double duration_to_lower = common::ToSeconds(time - it_lower->time);
  
  if (duration_between == 0) {
    return it_lower->pose;
  }
  
  const double interpolation_ratio = duration_to_lower / duration_between;
  
  // 插值位置
  const Eigen::Vector3d translation = 
      it_lower->pose.translation() + 
      interpolation_ratio * (it_upper->pose.translation() - it_lower->pose.translation());
  
  // 插值旋转（球面线性插值）
  const Eigen::Quaterniond rotation = 
      it_lower->pose.rotation().slerp(interpolation_ratio, it_upper->pose.rotation());
  
  return transform::Rigid3d(translation, rotation);
}

void DataPreprocessor::AddOdometryData(const sensor::OdometryData& odometry_data)
{
  std::unique_lock<std::mutex> lock(odometry_buffer_mutex_);
  if (!odom_data_buff_.empty() &&
    odom_data_buff_.back().time > odometry_data.time)
  {
    return;
  }
  odom_data_buff_.push_back(odometry_data);
  
  // 每添加100个数据点进行一次清理，避免频繁清理（约每秒一次）
  static int add_count = 0;
  if (++add_count >= 100) {
    TrimOdomData();
    add_count = 0;
  }
  
  std::cout << "DataPreprocessor: Added odometry data at time: " << odometry_data.time
            << ", odom_buff_size: " << odom_data_buff_.size() << std::endl;
}

void DataPreprocessor::TrimOdomData()
{
  // 保留最近30秒的里程计数据用于插值
  if (odom_data_buff_.empty()) {
    return;
  }
  
  const common::Time current_time = odom_data_buff_.back().time;
  const common::Time cutoff_time = current_time - common::FromSeconds(10.0);
  
  // 优化：使用binary search快速定位需要删除的位置
  auto it = std::lower_bound(odom_data_buff_.begin(), odom_data_buff_.end(), 
                            cutoff_time, 
                            [](const sensor::OdometryData& data, const common::Time& time) {
                              return data.time < time;
                            });
  
  // 批量删除过期数据，比逐个pop_front效率更高
  if (it != odom_data_buff_.begin()) {
    size_t removed_count = std::distance(odom_data_buff_.begin(), it);
    odom_data_buff_.erase(odom_data_buff_.begin(), it);
    
    // 只在实际删除数据时打印日志
    if (removed_count > 0) {
      DLOG(INFO) << "TrimOdomData: Removed " << removed_count 
                 << " old odometry entries, remaining: " << odom_data_buff_.size();
    }
  }
}

absl::optional<sensor::OdometryData> DataPreprocessor::GetOdometryAtTime(
    const common::Time& time) const {
  std::lock_guard<std::mutex> lock(odometry_buffer_mutex_);
  
  if (odom_data_buff_.empty()) {
    return absl::nullopt;
  }
  
  // 如果查询时间早于最早的数据
  if (time < odom_data_buff_.front().time) {
    return odom_data_buff_.front();
  }
  
  // 如果查询时间晚于最新的数据，进行外插
  if (time > odom_data_buff_.back().time) {
    // 外插逻辑：使用最后一个数据点的速度信息
    const auto& last = odom_data_buff_.back();
    const double dt = common::ToSeconds(time - last.time);

    const Eigen::Vector3d w = last.twist.angular; // 角速度
    Eigen::Vector3d v = last.twist.linear;
    v = last.pose.rotation() * v;  // 将线速度转换到世界坐标系

    const Eigen::Vector3d t = last.pose.translation() + dt * v;
    const Eigen::Quaterniond dq =
        transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(w * dt));
    Eigen::Quaterniond q = last.pose.rotation() * dq;
    q.normalize();

    return sensor::OdometryData{time, transform::Rigid3d(t, q), last.twist};
  }
  
  // 从后往前查找合适的插值区间（优化：大多数查询都是最近时间）
  auto it_lower = odom_data_buff_.end() - 1;
  auto it_upper = odom_data_buff_.end();
  
  // 从后往前找到第一个时间小于等于查询时间的数据点
  for (auto it = odom_data_buff_.rbegin(); it != odom_data_buff_.rend(); ++it) {
    if (it->time <= time) {
      it_lower = it.base() - 1;  // 转换反向迭代器到正向迭代器
      it_upper = it_lower + 1;
      break;
    }
  }
  
  // 如果没找到小于等于查询时间的点，说明查询时间早于所有数据
  if (it_lower == odom_data_buff_.end() - 1 && odom_data_buff_.back().time > time) {
    return odom_data_buff_.front();
  }
  
  // 如果找到的是最后一个点，说明需要外插（但这种情况前面已经处理过了）
  if (it_upper == odom_data_buff_.end()) {
    return *it_lower;
  }
  
  // 如果时间完全匹配
  if (it_lower->time == time) {
    return *it_lower;
  }
  if (it_upper->time == time) {
    return *it_upper;
  }
  
  // 线性插值
  const double duration_to_lower = common::ToSeconds(time - it_lower->time);
  const double duration_between = common::ToSeconds(it_upper->time - it_lower->time);
  
  if (duration_between <= 0.0) {
    return *it_lower;
  }
  
  const double interpolation_ratio = duration_to_lower / duration_between;
  
  // 插值位置
  const Eigen::Vector3d translation = 
      it_lower->pose.translation() + 
      interpolation_ratio * (it_upper->pose.translation() - it_lower->pose.translation());
  
  // 插值旋转（球面线性插值）
  const Eigen::Quaterniond rotation = 
      it_lower->pose.rotation().slerp(interpolation_ratio, it_upper->pose.rotation());
  
  // 插值速度
  const Eigen::Vector3d linear_velocity = 
      it_lower->twist.linear + 
      interpolation_ratio * (it_upper->twist.linear - it_lower->twist.linear);
  const Eigen::Vector3d angular_velocity = 
      it_lower->twist.angular + 
      interpolation_ratio * (it_upper->twist.angular - it_lower->twist.angular);
  
  return sensor::OdometryData{time, transform::Rigid3d(translation, rotation), 
                             sensor::Twist{linear_velocity, angular_velocity}};
}
 
} // namespace preprocess
} // namespace cartographer
