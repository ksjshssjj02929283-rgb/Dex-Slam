#ifndef CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_
#define CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_

#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

namespace cartographer
{
namespace laser_perception
{

struct Pose
{
  double x;
  double y;
};

struct Element
{
  std::string type;
  std::vector<Pose> poses;
};

struct Component
{
  int element_id;
  int type;
  double x;
  double y;
  double theta;
};

struct Model
{
  std::string name;
  std::map<int, Element> elements;
  std::vector<Component> components;
};

struct RackInfo
{
  double type;
  double rack_length;
  double rack_width;
  double rack_leg_length;
  double rack_leg_width;
};

struct ReflectorInfo
{
  double radian;
  double radius;
};

struct Line
{
  std::string type;
  std::pair<cv::Point2d, cv::Point2d> line;
};

struct FeatureInfo
{
  std::string target_name;
  cv::Point3d pose;
};

struct FeatureMsg
{
  int id;
  std::string target_name;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> line;
  double x;
  double y;
  double theta;
  double corner_struct_angle;
};

}  // namespace laser_perception
}  // namespace cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_
