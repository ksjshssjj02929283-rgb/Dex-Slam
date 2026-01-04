cmake_minimum_required(VERSION 3.8)
project(amcl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -std=c++17 -march=native")

include(CheckIncludeFile)
include(CheckSymbolExists)
include(FindPackageHandleStandardArgs)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

# config files
set(AMCL_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/amcl/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/amcl/)

install(DIRECTORY cmake DESTINATION share/amcl/)

configure_file(
  ${PROJECT_SOURCE_DIR}/include/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/include/common/config.h)

include_directories(
  ../utility/include
  /apps
  include
  ${PROJECT_BINARY_DIR}
  ${PROJECT_BINARY_DIR}/include
)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system filesystem)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(glog REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(absl REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)

set(dependencies
  diagnostic_updater
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  nav_interfaces
  pluginlib
  nav2_util
  pcl_conversions
)

file(GLOB_RECURSE ALL_PROTOS "include/proto/*.proto")
file(GLOB_RECURSE ALL_GRPC_SERVICES "proto/*_service.proto")
list(REMOVE_ITEM ALL_PROTOS ALL_GRPC_SERVICES)

set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR}/include ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  # message("${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR}/include ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})
set(INSTALL_GENERATED_HDRS ${ALL_PROTO_HDRS})

include_directories(include)
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

check_include_file(unistd.h HAVE_UNISTD_H)
if (HAVE_UNISTD_H)
  add_definitions(-DHAVE_UNISTD_H)
endif (HAVE_UNISTD_H)

check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)
if (HAVE_DRAND48)
  add_definitions(-DHAVE_DRAND48)
endif (HAVE_DRAND48)

add_library(amcl_pf
    src/amcl/pf/pf.c
    src/amcl/pf/pf_kdtree.c
    src/amcl/pf/pf_pdf.c
    src/amcl/pf/pf_vector.c
    src/amcl/pf/eig3.c
    src/amcl/pf/pf_draw.c)

add_library(amcl_map
    src/amcl/map/map.c
    src/amcl/map/map_cspace.cpp
    src/amcl/map/map_range.c
    src/amcl/map/map_store.c
    src/amcl/map/map_draw.c)

add_library(amcl_sensors
    src/amcl/sensors/amcl_sensor.cpp
    src/amcl/sensors/amcl_odom.cpp
    src/amcl/sensors/amcl_laser.cpp
    src/amcl/sensors/amcl_pose.cpp
    src/amcl/sensors/amcl_cloud.cpp)

file(GLOB_RECURSE SCAN_MATCH_SRC_FILES
    src/common/*.cc
    src/scan_match/*.cc
    src/sensor/*.cc
    src/transform/*.cc)
add_library(scan_match
  ${SCAN_MATCH_SRC_FILES}
  ${ALL_LIBRARY_HDRS}
  ${ALL_LIBRARY_SRCS})

add_library(pose_estimator_2d
  src/amcl/core/pose_estimator_2d.cpp
)

ament_target_dependencies(pose_estimator_2d ${dependencies})

message($<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>)
target_include_directories(scan_match PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(amcl_sensors amcl_map amcl_pf)
target_link_libraries(scan_match PUBLIC
  ${PROTOBUF_LIBRARY}
  ${LUA_LIBRARIES}
  ${CERES_LIBRARIES}
  absl::algorithm
  absl::base
  absl::debugging
  absl::flat_hash_map
  absl::memory
  absl::meta
  absl::numeric
  absl::str_format
  absl::strings
  absl::synchronization
  absl::time
  absl::utility
)
target_link_libraries(pose_estimator_2d
  amcl_sensors amcl_map amcl_pf scan_match
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam)

add_library(utility SHARED
  ../utility/src/converter.cpp
  ../utility/src/file_manager.cpp
  ../utility/src/pose_save_tool.cpp)
target_link_libraries(utility glog::glog)

add_executable(amcl
               apps/node_main.cpp
               apps/amcl_node.cpp)

ament_target_dependencies(amcl ${dependencies})

target_link_libraries(amcl
    amcl_sensors amcl_map amcl_pf pose_estimator_2d
    ${Boost_LIBRARIES}
    ${GLOG_LIBRARIES}
    utility
)

install(TARGETS
    amcl
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    amcl_sensors amcl_map amcl_pf pose_estimator_2d
    utility scan_match
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/amcl/ include/scan_match include/common include/sensor include/transform
  DESTINATION include
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  amcl_pf
  amcl_map
  amcl_sensors
  scan_match
  pose_estimator_2d
  utility
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(HDR ${INSTALL_GENERATED_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  message(STATUS "Installing ${HDR} to include/${DIR}")
  install(
    FILES ${HDR}
    DESTINATION include/${DIR}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(amcl_pf amcl_map amcl_sensors scan_match pose_estimator_2d utility)
ament_export_dependencies(${dependencies})
ament_export_targets(${PROJECT_NAME})
ament_package()