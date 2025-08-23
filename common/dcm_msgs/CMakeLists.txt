cmake_minimum_required(VERSION 3.10)
project(dcm_msgs)

# 编译配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1) 基础依赖（ROS 2）
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 2) 依赖的已迁移消息/动作包
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# ROS 2 的 action 基础包
find_package(action_msgs REQUIRED)

# 3) 声明要生成的消息
set(MSG_FILES
  "msg/Path.msg"
  "msg/PathPose.msg"
  "msg/AreaInfo.msg"
  "msg/DTLane.msg"
  "msg/Lane.msg"
  "msg/LaneArray.msg"
  "msg/Waypoint.msg"
  "msg/WaypointState.msg"
  "msg/NavPoint.msg"
  "msg/DockingInfo.msg"
  "msg/Task.msg"
  "msg/TaskGroup.msg"
  "msg/BatteryInfo.msg"
  "msg/Line.msg"
  "msg/LineArray.msg"
  "msg/LineMapData.msg"
  "msg/LineMapDataInfo.msg"
  "msg/TargetInfo.msg"
  "msg/TargetMapInfo.msg"
  "msg/LocalizationState.msg"
  "msg/RobotStatus.msg"
)

# 5) 生成接口（取代 ROS 1 的 add_message_files / add_action_files / generate_messages）
#    注意：如果 .msg 中引用了本包类型，不用显式把 dcm_msgs 写到 DEPENDENCIES（rosidl 会处理本包），
#    但外部依赖必须列出。
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    action_msgs
)

# 6) 导出运行期依赖
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  geometry_msgs
  nav_msgs
  action_msgs
)

# 7) 安装接口源文件（可选，便于查看/复用）
install(DIRECTORY msg action
  DESTINATION share/${PROJECT_NAME}
)

# 8) 如果有参数 YAML 或其它共享资源，可一起安装
# install(DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
