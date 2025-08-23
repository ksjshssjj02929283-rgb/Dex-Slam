cmake_minimum_required(VERSION 3.10)
project(dcm_srvs)

# 编译标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) 基础依赖：ament 与 rosidl
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2) 依赖的消息包（ROS 2）
# 注意：确保 dcm_msgs 也已迁移到 ROS 2 并使用 rosidl 生成
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(dcm_msgs REQUIRED)

# 3) 声明本包要生成的 srv 接口（对应 ROS 1 的 add_service_files）
# 目录结构保持 srv/*.srv
set(srv_files
  "srv/AddTask.srv"
  "srv/AddTaskGroup.srv"
  "srv/AddNavPoint.srv"
  "srv/SampleNavPoint.srv"
  "srv/RemoveNavPoint.srv"
  "srv/GetNavPoint.srv"
  "srv/GetAllNavPoints.srv"
  "srv/NavToPoint.srv"
  "srv/GetCurrentTaskInfo.srv"
  "srv/SetPose.srv"
  "srv/GetStatus.srv"
  "srv/SetCommandBoard.srv"
  "srv/SetMapName.srv"
  "srv/GetMapList.srv"
  "srv/LoadMap.srv"
  "srv/DeleteMap.srv"
  "srv/GetMapInfo.srv"
  "srv/GetString.srv"
  "srv/AddAreaInfo.srv"
  "srv/GetAreaInfoList.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
    std_msgs
    dcm_msgs
    nav_msgs
    geometry_msgs
    actionlib_msgs
)

# 4) 导出依赖，供其他包使用这些接口
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  dcm_msgs
)

# 5) 安装 srv 源文件（可选，但通常保留源以便复用/查看）
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# 6) 可选：添加可执行/库目标（如果有）
# add_executable(example_node src/example_node.cpp)
# ament_target_dependencies(example_node
#   rclcpp
#   ${PROJECT_NAME}          # 若需要使用本包生成的类型，可依赖 rosidl_default_runtime
# )
# install(TARGETS example_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# 7) 用于接口包的宏，确保 rosidl 的生成产物正确导出
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(introspection_target ${PROJECT_NAME} "rosidl_typesupport_introspection_cpp")
rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")
rosidl_get_typesupport_target(cppfastrtps_target ${PROJECT_NAME} "rosidl_typesupport_fastrtps_cpp")
rosidl_get_typesupport_target(cfastrtps_target ${PROJECT_NAME} "rosidl_typesupport_fastrtps_c")

ament_package()