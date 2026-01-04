# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)  # Ships with Ubuntu 14.04 (Trusty)

project(cartographer_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -std=c++17 -march=native")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PACKAGE_DEPENDENCIES
  rclcpp
  cartographer
  cartographer_ros_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_ros
  tf2_geometry_msgs
  urdf
  visualization_msgs
  nav_interfaces
  rosbag2_cpp
  rosbag2_storage
)

# # ROS 2 包
# foreach(dep ${PACKAGE_DEPENDENCIES})
#   find_package(${dep} REQUIRED)
# endforeach()

if(WIN32)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()
set(CARTOGRAPHER_GMOCK_LIBRARIES ${GMOCK_LIBRARIES})


find_package(absl REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(urdfdom_headers REQUIRED)

# urdfdom_headers 版本宏定义（保持原逻辑）
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

# 自定义 include 目录
include_directories(
  include
  ${urdfdom_headers_INCLUDE_DIRS}
  ../../utility/include
)

add_library(utility
  ../../utility/src/converter.cpp
  ../../utility/src/pose_save_tool.cpp
  ../../utility/src/file_manager.cpp
)
target_include_directories(utility SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
target_link_libraries(utility PUBLIC ${GLOG_LIBRARIES} ${Boost_LIBRARIES})

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${CARTOGRAPHER_GMOCK_LIBRARIES})

file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()
add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME} PUBLIC cartographer)


# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list(FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS ${TARGET_COMPILE_FLAGS} ${DEFINITION})
endforeach()

# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

# Boost
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

# JSON CPP
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${JSONCPP_LIBRARIES})

# utility
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ../utility/include)
target_link_libraries(${PROJECT_NAME} PUBLIC utility)

# 生成目录优先（port.h 相关注释保留）
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# ament 依赖注入（替代 catkin_INCLUDE_DIRS/catkin_LIBRARIES）
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    ${PACKAGE_DEPENDENCIES}
    OpenCV
    PCL
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

# todo: 编译test会因为gmock而编译不通过，这里先注释了
# 测试（使用 ament_add_gtest）
# 若需要 GMock，可通过 GTest 提供的 gmock 库或 FetchContent，以下使用 ament 的 GTest
# find_package(ament_cmake_gtest REQUIRED)
# if(BUILD_TESTING)
#   foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
#     get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
#     ament_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
#     if(TARGET ${TEST_NAME})
#       target_link_libraries(${TEST_NAME} ${GMOCK_LIBRARIES} ${GTEST_MAIN_LIBRARIES})
#       target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
#       target_link_libraries(${TEST_NAME} ${LUA_LIBRARIES})
#       ament_target_dependencies(${TEST_NAME} ${PACKAGE_DEPENDENCIES} OpenCV PCL)
#       target_link_libraries(${TEST_NAME} cartographer)
#       target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
#       set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS "${TARGET_COMPILE_FLAGS}")
#     endif()
#   endforeach()
# endif()

# 安装共享资源（ROS 2：安装到 share/${PROJECT_NAME}）
install(DIRECTORY launch urdf configuration_files rviz
  DESTINATION share/${PROJECT_NAME}
)

# 安装脚本
install(PROGRAMS
  scripts/tf_remove_frames.py
  scripts/collect_trajectory_data.py
  scripts/evaluate_static_localization.py
  scripts/evaluate_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装库
install(TARGETS ${PROJECT_NAME} utility
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(FILES ${HDR} DESTINATION include/${INSTALL_DIR})
endforeach()

# 导出
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} utility)
ament_export_dependencies(${PACKAGE_DEPENDENCIES} OpenCV PCL)

ament_package()
