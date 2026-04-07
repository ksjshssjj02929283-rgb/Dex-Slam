cmake_minimum_required(VERSION 3.5)
project(perception_2d_ros2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -std=c++17 -march=native")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif ()
message(STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_interfaces REQUIRED)
find_package(perception_2d REQUIRED)

# Find additional dependencies
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")
google_initialize_cartographer_project()
google_enable_testing()
include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED COMPONENTS common)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
  ${urdfdom_headers_INCLUDE_DIRS}
)

set(dependencies
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

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${perception_2d_INCLUDE_DIRS}
)

set(PERCEPTION_2D_HEADERS
  perception_2d_ros2/ros_log_sink.hpp
  perception_2d_ros2/node_constants.hpp
  perception_2d_ros2/node.hpp
  perception_2d_ros2/node_option.hpp
  perception_2d_ros2/time_conversion.hpp
  perception_2d_ros2/msg_conversion.hpp
  perception_2d_ros2/tf_bridge.hpp
  perception_2d_ros2/sensor_bridge.hpp
  perception_2d_ros2/perception_processor_bridge.hpp
  perception_2d_ros2/feature_map_bridge.hpp
)

set(PERCEPTION_2D_SOURCES
  perception_2d_ros2/ros_log_sink.cpp
  perception_2d_ros2/node_constants.cpp
  perception_2d_ros2/node.cpp
  perception_2d_ros2/node_option.cpp
  perception_2d_ros2/time_conversion.cpp
  perception_2d_ros2/msg_conversion.cpp
  perception_2d_ros2/tf_bridge.cpp
  perception_2d_ros2/sensor_bridge.cpp
  perception_2d_ros2/perception_processor_bridge.cpp
  perception_2d_ros2/feature_map_bridge.cpp
)

# Create library
add_library(${PROJECT_NAME} ${PERCEPTION_2D_HEADERS} ${PERCEPTION_2D_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})

target_link_libraries(${PROJECT_NAME} perception_2d)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list(FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if(${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} -fPIC ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Add executable node
add_subdirectory("perception_2d_ros2")

# Install
install(DIRECTORY configuration_files
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
