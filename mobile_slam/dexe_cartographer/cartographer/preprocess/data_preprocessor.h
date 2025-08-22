#ifndef DATA_PREPROCESSOR_H_
#define DATA_PREPROCESSOR_H_

#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <thread>
#include <mutex>

#include "absl/types/optional.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/sensor/range_data.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer/preprocess/data_preprocessor_options.h"


namespace cartographer {
namespace preprocess {

/**
 * @brief 数据预处理器，包含的功能有：数据同步，雷达数据去畸变等
 * 
 */
class DataPreprocessor
{
public:
  typedef std::shared_ptr<DataPreprocessor> SharedPtr;
  typedef std::unique_ptr<DataPreprocessor> UniquePtr;
  explicit DataPreprocessor(const proto::DataPreprocessorOptions& options);
  ~DataPreprocessor() = default;

public:
  absl::optional<sensor::TimedRangeData> GetProcessedRangeData();

  void AddSyncedPointCloud(const sensor::TimedPointCloudOriginData& cloud_in);
  void AddOdometryData(const sensor::OdometryData& odometry_data);
  const transform::Rigid3f& GetOdomDeltaPose() const {
    return odom_delta_pose_;
  }
  // sensor::RangeDataWithPose RemoveDistortion(const sensor::TimedPointCloudOriginData& cloud_in);
  // sensor::RangeDataWithPose AccumulateData(const sensor::RangeDataWithPose& range_data_in);
  // transform::Rigid3f GetRangeDataPose() const {
  //   return range_data_pose_;
  // }
private:
  transform::Rigid3f GetPointPose(
    const common::Time& time,
    const std::deque<sensor::OdometryData>& odom_data_buff) const;

  void TrimOdomData();

  // 积累range data
  void AccumulateRangeData(
    const sensor::TimedRangeData& range_data_in,
    const transform::Rigid3f& range_in_odom);

private:
  std::mutex data_mutex_;
  std::deque<sensor::TimedPointCloudOriginData> cloud_data_buff_;
  std::deque<sensor::OdometryData> odom_data_buff_;

  proto::DataPreprocessorOptions options_;

  sensor::TimedRangeData accumulated_range_data_;
  std::deque<sensor::TimedRangeData> accumlated_range_buff_;
  int num_accumulated_ = 0;
  common::Time last_laser_time_ = common::Time::min() + common::FromSeconds(0.1);

  transform::Rigid3f last_odom_pose_ = transform::Rigid3f::Identity();
  transform::Rigid3f odom_delta_pose_ = transform::Rigid3f::Identity();
};

}  // namespace preprocess
}  // namespace cartographer

#endif  // DATA_PREPROCESSOR_H_
