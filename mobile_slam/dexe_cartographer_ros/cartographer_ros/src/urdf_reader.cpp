/**
 * @file urdf_reader.cpp
 * @author ZheHe
 * @brief 从urdf读取静态tf
 * @version 0.1
 * @date 2023-05-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "cartographer_ros/urdf_reader.h"

#include <string>
#include <vector>

#include "cartographer_ros/msg_conversion.h"
#include "urdf/model.h"

namespace cartographer_ros {

std::vector<geometry_msgs::msg::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, std::shared_ptr<tf2_ros::Buffer> tf_buffer) {
  urdf::Model model;
  CHECK(model.initFile(urdf_filename));
  std::vector<urdf::LinkSharedPtr> links;
  model.getLinks(links);
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }

    const urdf::Pose& pose =
        link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::msg::TransformStamped transform;
    transform.transform =
        ToGeometryMsgTransform(cartographer::transform::Rigid3d(
            Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
            Eigen::Quaterniond(pose.rotation.w, pose.rotation.x,
                               pose.rotation.y, pose.rotation.z)));
    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    tf_buffer->setTransform(transform, "urdf", true /* is_static */);
    transforms.push_back(transform);
  }
  return transforms;
}

}  // namespace cartographer_ros
