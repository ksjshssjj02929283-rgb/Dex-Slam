#ifndef PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_CLOUD_GENERATOR_HPP_
#define PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_CLOUD_GENERATOR_HPP_
#include <Eigen/Core>
#include <glog/logging.h>

#include "perception_2d/sensor/point_cloud.hpp"

namespace perception_2d
{

class ModelCloudGenerator
{
public:
  typedef std::shared_ptr<ModelCloudGenerator> Ptr;

  ModelCloudGenerator() = default;

  // 设置模板的顶点
  void SetVertices(const std::vector<Eigen::Vector2d>& vertices)
  {
    if(vertices.empty())
    {
      LOG(WARNING) << "empty vertex input." << std::endl;
      return;
    }

    vertices_ = vertices;
  }

  // 设置模板点云的分辨率，default resolution : 0.01
  void SetResolution(const double& resolution)
  {
    if(resolution < 0.001)
    {
      resolution_ = 0.001;
      LOG(WARNING) << "too small resolution input, set resolution to 0.005m." << std::endl;
      return;
    }

    resolution_ = resolution;
  }

  // // 获取模板点云
  // bool GetModelCloud(sensor::PointCloud & cloud_out)
  // {
  //   cloud_out.clear();

  //   GeneratePointsVector(vertices_, points_vec_, resolution_);
    
  //   if (points_vec_.empty())
  //   {
  //     LOG(ERROR) << "fatal : Empty cloud vector generatered!" << std::endl;
  //     return false;
  //   }

  //   for (auto iter = points_vec_.begin(); iter != points_vec_.end(); ++iter) {
  //     sensor::RangefinderPoint p;
  //     p.x = iter->x();
  //     p.y = iter->y();

  //     cloud_out.push_back(p);
  //   }
  //   return true;
  // }

  static void GenerateModelCloudCircle(
                const float radius,
                const double& resolution,
                sensor::PointCloud & cloud_out)
  {
    cloud_out.clear();
    // 周长是多少
    float perimeter = 2 * M_PI * radius;
    int num_points = std::ceil(perimeter / resolution);
    for (int i = 0; i < num_points; i++) {
      float theta = i * 2 * M_PI / num_points;
      sensor::RangefinderPoint p;
      p.position.x() = radius * cos(theta);
      p.position.y() = radius * sin(theta);
      cloud_out.push_back(p);
    }
  }

  static void GenerateModelCloudPolygon(
                const std::vector<Eigen::Vector2d>& vertices, 
                const double& resolution,
                sensor::PointCloud & cloud_out)
  {
    cloud_out.clear();

    std::vector<Eigen::Vector2d> points_vec;
    GeneratePointsVector(vertices, points_vec, resolution);

    if (points_vec.empty()) {
      LOG(ERROR) << "fatal : Empty cloud vector generatered!" << std::endl;
      return;
    }

    for (auto iter = points_vec.begin(); iter != points_vec.end(); ++iter) {
      sensor::RangefinderPoint p;
      p.position.x() = iter->x();
      p.position.y() = iter->y();

      cloud_out.push_back(p);
    }
  }

private:
  static void GeneratePointsVector(
    const std::vector<Eigen::Vector2d>& vertices, 
    std::vector<Eigen::Vector2d>& points_vec, 
    const double& resolution)
  {
    if(vertices.empty())
    {
      LOG(ERROR) << "Empty vertex input, cannot generate points." << std::endl;
      return;
    }

    points_vec.clear();
    for(size_t i = 0; i < vertices.size() - 1; ++i)
    {
      const Eigen::Vector2d& p1 = vertices[i];
      const Eigen::Vector2d& p2 = vertices[i+1];
      // std::cout << p1.transpose() << std::endl;
      // std::cout << p2.transpose() << std::endl << std::endl;

      double dist_p1p2 = (p2 - p1).norm();

      int points_cnt = static_cast<int>(dist_p1p2 / resolution);

      double step_x = - ( p1(0) - p2(0) ) / points_cnt;
      double step_y = - ( p1(1) - p2(1) ) / points_cnt;

      for(int j = 0; j < points_cnt; ++j)
      {
        Eigen::Vector2d point_temp;
        point_temp(0) = p1(0) + j*step_x;
        point_temp(1) = p1(1) + j*step_y;
        points_vec.emplace_back(point_temp);
      }
    }

    // LOG(INFO) << "Generate model cloud with: " << points_vec.size() << " points." << std::endl;
  }

private:
  std::vector<Eigen::Vector2d> vertices_;
  std::vector<Eigen::Vector2d> points_vec_;

  double resolution_ = 0.01;
};

}  // namespace perception_2d

#endif  // PERCEPTION_2D__LASER_PERCEPTION_2D__MODEL_CLOUD_GENERATOR_HPP_