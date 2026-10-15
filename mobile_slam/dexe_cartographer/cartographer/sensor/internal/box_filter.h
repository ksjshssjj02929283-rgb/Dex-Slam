/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Guo Linfeng
 * @Date: 2025-01-03
 */
#ifndef CARTOGRAPHER_SENSOR_INTERNAL_BOX_FILTER_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_BOX_FILTER_H_

#include <vector>
#include <memory>

#include <Eigen/Core>

#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace sensor {

class CropBox {
public:
  struct Box {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
  };

  CropBox(const Box& box) : box_(box) {}

  PointCloud Filter(const PointCloud& input_cloud) const {
    return input_cloud.copy_if([this](const PointCloud::PointType & point) {
      return isInside(point);
    });
  }

  void SetBox(const Box& box) {
    box_ = box;
  }

private:
  bool isInside(const PointCloud::PointType& point) const {
    return (point.position.x() >= box_.min_x && point.position.x() <= box_.max_x) &&
           (point.position.y() >= box_.min_y && point.position.y() <= box_.max_y) &&
           (point.position.z() >= box_.min_z && point.position.z() <= box_.max_z);
    }

private:
  Box box_;
};

class BoxFilter
{
public:
  BoxFilter(const CropBox::Box& box);
  ~BoxFilter() = default;
  // 裁剪 cloud_in 到 box_（含 SetOrigin 平移）并写入 cloud_out。
  // 返回裁剪后 cloud_out 是否非空：false 表示裁剪窗口与点云完全不重叠。
  bool Filter(const PointCloud& cloud_in,
              PointCloud& cloud_out);

  void SetSize(std::vector<float> size);
  void SetOrigin(const Eigen::Vector3f& origin);
  std::vector<float> GetEdge();

private:
  void CalculateEdge();

private:
  std::unique_ptr<CropBox> crop_box_;

  Eigen::Vector3f origin_ = Eigen::Vector3f::Zero();
  std::vector<float> size_;
  std::vector<float> edge_;
};

} // namespace sensor
} // namespace cartographer

#endif // CARTOGRAPHER_SENSOR_INTERNAL_BOX_FILTER_H_