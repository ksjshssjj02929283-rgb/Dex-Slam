#ifndef CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_
#define CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_

#include <string>
#include <utility>

#include <Eigen/Core>

namespace cartographer
{
namespace laser_perception
{

struct FeatureMsg
{
  std::string target_name;
  std::pair<Eigen::Vector3f, Eigen::Vector3f> line;
};

}  // namespace laser_perception
}  // namespace cartographer

#endif  // CARTOGRAPHER__LASER_PERCEPTION__DATA_TYPE_H_