/*
 * @Description: 从点云中截取一个立方体部分
 * @Author: Guo Linfeng
 * @Date: 2025-01-03
 */
#include <vector>
#include <iostream>
#include "glog/logging.h"

#include "cartographer/sensor/internal/box_filter.h"

namespace cartographer {
namespace sensor {

BoxFilter::BoxFilter(const CropBox::Box& box)
{
  size_.resize(6);
  edge_.resize(6);

  size_.at(0) = box.min_x;
  size_.at(1) = box.max_x;
  size_.at(2) = box.min_y;
  size_.at(3) = box.max_y;
  size_.at(4) = box.min_z;
  size_.at(5) = box.max_z;

  crop_box_ = std::make_unique<CropBox>(box);

  SetSize(size_);
}

// pcl::PointCloud<pcl::PointXYZ> BoxFilter::Filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
bool BoxFilter::Filter(const PointCloud& cloud_in,
                       PointCloud& cloud_out)
{
  crop_box_->SetBox({edge_.at(0), edge_.at(2), edge_.at(4),
                     edge_.at(1), edge_.at(3), edge_.at(5)});

  cloud_out = crop_box_->Filter(cloud_in);
  return true;
}

void BoxFilter::SetSize(std::vector<float> size) {
  size_ = size;
  LOG(INFO) << "Box Filter Size: " << std::endl
            << "min_x: " << size.at(0) << ", "
            << "max_x: " << size.at(1) << ", "
            << "min_y: " << size.at(2) << ", "
            << "max_y: " << size.at(3) << ", "
            << "min_z: " << size.at(4) << ", "
            << "max_z: " << size.at(5);

  CalculateEdge();
}

void BoxFilter::SetOrigin(const Eigen::Vector3f& origin) {
  origin_ = origin;
  CalculateEdge();
}

void BoxFilter::CalculateEdge() {
  for (size_t i = 0; i < origin_.size(); ++i) {
    edge_.at(2 * i) = size_.at(2 * i) + origin_(i);
    edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_(i);
  }
}

std::vector<float> BoxFilter::GetEdge() {
  return edge_;
}

} // sensor
} // namespace cartographer
