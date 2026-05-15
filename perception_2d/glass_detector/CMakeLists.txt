cmake_minimum_required(VERSION 3.8)
project(glass_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# ---- 算法库（纯函数，无 ROS 依赖，便于单测） ----
add_library(${PROJECT_NAME}_algorithms SHARED
  src/algorithms/e1_geometric.cpp
  src/algorithms/e2_intensity.cpp
)
ament_target_dependencies(${PROJECT_NAME}_algorithms
  sensor_msgs
)
target_include_directories(${PROJECT_NAME}_algorithms PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- 检测器库（ROS 节点逻辑） ----
add_library(${PROJECT_NAME}_lib SHARED
  src/single_laser_detector.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_algorithms)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---- 可执行节点 ----
add_executable(glass_detector_node src/glass_detector_node.cpp)
ament_target_dependencies(glass_detector_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(glass_detector_node
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_algorithms
)

# ---- 安装 ----
install(
  TARGETS
    ${PROJECT_NAME}_algorithms
    ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS glass_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---- 测试 ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # 一期跳过纯 style 检查（不影响功能验证；后期统一 uncrustify 修一次）
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_e1_geometric test/test_e1_geometric.cpp)
  if(TARGET test_e1_geometric)
    target_link_libraries(test_e1_geometric ${PROJECT_NAME}_algorithms)
  endif()

  ament_add_gtest(test_e2_intensity test/test_e2_intensity.cpp)
  if(TARGET test_e2_intensity)
    target_link_libraries(test_e2_intensity ${PROJECT_NAME}_algorithms)
  endif()
endif()

# ---- ament 导出 ----
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_algorithms)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs std_msgs)

ament_package()
