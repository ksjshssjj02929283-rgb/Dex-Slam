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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/conversion/msg_conversion.h"
#include "cartographer_ros/common/node_constants.h"
#include "cartographer_ros/mapping/ros_log_sink.h"
#include "cartographer_ros/mapping/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  bool StartMappingCallback(std_srvs::Trigger::Request &req,
                            std_srvs::Trigger::Response &res);
  bool FinishMappingCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res);

  ::ros::NodeHandle node_handle_;
  ::ros::NodeHandle node_handle_private_;

  const double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;

  ros::ServiceServer start_mapping_srv_;
  ros::ServiceServer finish_mapping_srv_;
  bool is_activating_ = false;
};

/**
 * @brief Construct a new Node:: Node object
 * 
 * @param resolution 地图分辨率
 * @param publish_period_sec 多久发布一次（单位 秒）
 */
Node::Node(const double resolution, const double publish_period_sec)
    : node_handle_private_("~"),
      resolution_(resolution)
{
  client_ = node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(kSubmapQueryServiceName);  // 请求submap_query的service
  submap_list_subscriber_ = node_handle_.subscribe( // 订阅submap_list的topic
                                kSubmapListTopic, kLatestOnlyPublisherQueueSize,
                                boost::function<void(
                                    const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
                                    [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                                      HandleSubmapList(msg);
                                    }));

  occupancy_grid_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                                            ComputeRepeatedTopicNames(FLAGS_occupancy_grid_topic, 1)[0],
                                            kLatestOnlyPublisherQueueSize, true);

  occupancy_grid_publisher_timer_ = node_handle_.createWallTimer(
                                                  ::ros::WallDuration(FLAGS_publish_period_sec),
                                                  &Node::DrawAndPublish, this);

  start_mapping_srv_ = node_handle_private_.advertiseService("start_mapping", &Node::StartMappingCallback, this);
  finish_mapping_srv_ = node_handle_private_.advertiseService("finish_mapping", &Node::FinishMappingCallback, this);
}

bool Node::StartMappingCallback(std_srvs::Trigger::Request &req,
                                std_srvs::Trigger::Response &res)
{
  occupancy_grid_publisher_timer_.stop();
  is_activating_ = true;
  res.success = true;
  return true;
}

bool Node::FinishMappingCallback(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res)
{
  occupancy_grid_publisher_timer_.start();
  is_activating_ = false;
  res.success = true;
  return true;
}

/**
 * @brief 订阅submap_list的topic，并画出submap，用于发布
 * 
 * @param msg 
 */
void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // 1. 检查submap_list话题是否被定于
  // We do not do any work if nobody listens.
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // 2. 获取所有的submap id，存入submap_ids_to_delete，嘉定所有的都不需要展示
  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  // 3. 遍历submap_list中的所有submap
  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    // 3.1 处理过的submap，从submap_ids_to_delete中删除，即该id需要展示
    submap_ids_to_delete.erase(id);
    // 3.2 （当submap是frozen，且不需要展示frozen_submaps） ||（当submap不是frozen，且不需要展示unfrozen_submaps时），跳过
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    // 3.3 从submap_slices_中获取要处理的submap_slice
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }
    // 3.4 获取submap的texture
    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    // 3.5 对submap_slice进行赋值
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    // 3.6 将submap的texture画到submap_slice上
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // 4. 删除已经不需要展示的submap
  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

/**
 * @brief 画出submap，并发布occupancy_grid的topic
 * 
 * @param unused_timer_event 
 */
void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  // 调用cartographer::io::PaintSubmapSlices接口进行画图
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occupancy_grid_publisher_.publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
