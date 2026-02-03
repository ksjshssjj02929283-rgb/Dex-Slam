#ifndef DATA_PREPROCESSOR_H_
#define DATA_PREPROCESSOR_H_

#include <string>
#include <memory>
#include <deque>
#include <thread>
#include <mutex>
#include <map>
#include <vector>

#include "absl/types/optional.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/line_data.h"
#include "cartographer/sensor/landmark_data.h"
#include "cartographer/sensor/feature_data.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/preprocess/proto/data_preprocessor_options.pb.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"
#include "cartographer/laser_perception/laser_perception.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace preprocess {

/**
 * @brief 处理后的传感器数据结构
 */
struct ProcessedSensorData {
  std::string sensor_id;
  sensor::RangeData undistorted_range_data;
  sensor::FeatureData feature_data;
  sensor::PointCloud highlight_point_cloud;
  sensor::PointCloud less_flat_cloud;
};

/**
 * @brief 融合后的数据结构
 */
struct MergedData {
  sensor::RangeData merged_range_data;
  sensor::PointCloud merged_filtered_cloud;
  sensor::PointCloud merged_highlight_cloud;
  sensor::PointCloud merged_less_flat_cloud;
  std::vector<sensor::LineData> merged_line_data;
  std::vector<sensor::LandmarkPointData> merged_reflector_data;
  std::vector<sensor::RangeData> original_undistorted_range_datas;

  bool IsEmpty() const {
    return merged_range_data.returns.empty() && 
           merged_filtered_cloud.empty() && 
           merged_highlight_cloud.empty();
  }
};

/**
 * @brief 数据预处理器，包含的功能有：数据同步，雷达数据去畸变，特征检测，多传感器融合等
 */
class DataPreprocessor
{
public:
  typedef std::shared_ptr<DataPreprocessor> SharedPtr;
  typedef std::unique_ptr<DataPreprocessor> UniquePtr;
  
  explicit DataPreprocessor(const proto::DataPreprocessorOptions& options);
  ~DataPreprocessor() = default;

public:
  absl::optional<sensor::TimedRangeData> UndistortPointCloud(const sensor::TimedPointCloudOriginData& cloud_in);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  
  // 新增接口
  /**
   * @brief 处理多个传感器的同步数据
   * @param sensor_data_map 传感器ID到激光数据的映射
   * @param sync_time 同步时间
   * @return 融合后的数据
   */
  MergedData ProcessMultipleSensorData(
      const std::map<std::string, sensor::TimedPointCloudData>& sensor_data_map,
      const common::Time& sync_time);

  /**
   * @brief 处理单个传感器数据
   * @param sensor_id 传感器ID
   * @param laser_data 激光数据
   * @param sync_time 同步时间
   * @return 处理后的数据，如果处理失败返回nullopt
   */
  absl::optional<ProcessedSensorData> ProcessSingleSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& laser_data,
      const common::Time& sync_time);

  /**
   * @brief 获取融合后的数据
   * @param processed_sensors 处理后的传感器数据列表
   * @return 融合后的数据结构
   */
  MergedData GetMergedData(const std::vector<ProcessedSensorData>& processed_sensors) const;

  // 获取/设置激光感知器
  laser_perception::LaserPerception* GetLaserPerception(const std::string& sensor_id);
  void CreateLaserPerception(const std::string& sensor_id);

  /**
   * @brief 根据时间获取里程计数据（线性插值）
   * @param time 查询时间
   * @return 插值后的里程计数据
   */
  absl::optional<sensor::OdometryData> GetOdometryAtTime(const common::Time& time) const;

  /**
   * @brief 添加里程计数据到缓冲区
   * @param odometry_data 里程计数据
   */

private:
  // 里程计插值方法
  transform::Rigid3d GetOdometryPose(const common::Time& time) const;
  transform::Rigid3d GetOdometryPoseFromBuffer(const common::Time& time, 
                                               const std::vector<sensor::OdometryData>& odom_buffer) const;
  void TrimOdomData();

  // 新增私有方法
  /**
   * @brief 获取特征数据
   * @param time 时间
   * @param feature_data_origin 原始特征数据
   * @return 转换后的特征数据
   */
  sensor::FeatureData GetFeatureData(
    const common::Time& time,
    const std::vector<laser_perception::FeatureMsgWithOrigin>& feature_data_origin);

private:
  proto::DataPreprocessorOptions options_;

  std::deque<sensor::TimedRangeData> accumlated_range_buff_;

  // 多个激光感知器，每个sensor_id对应一个LaserPerception实例
  std::map<std::string, std::unique_ptr<laser_perception::LaserPerception>> laser_perceptions_;

  std::deque<sensor::OdometryData> odom_data_buff_;
  mutable std::mutex odometry_buffer_mutex_;
};

}  // namespace preprocess
}  // namespace cartographer

#endif  // DATA_PREPROCESSOR_H_
