add_executable(perception_2d_node node_main.cpp)

ament_target_dependencies(perception_2d_node
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  nav_msgs
  visualization_msgs
  std_srvs
  nav_interfaces
)

target_link_libraries(perception_2d_node
  ${PROJECT_NAME}
  perception_2d
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS perception_2d_node
  DESTINATION lib/${PROJECT_NAME}
)
