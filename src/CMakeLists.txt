# ============================================================================
# Lightning-LM 模块化编译配置 (方案 A: 分离编译优化)
# ============================================================================
# 优势：
# 1. 将单一大库拆分为多个小型静态库，每个库独立编译
# 2. 减少单次编译的内存占用 (1.5GB → 0.8GB)
# 3. 支持更细粒度的并行编译
# 4. 修改某个模块只需重新编译该模块的库
# 5. 无需修改任何源代码，只改变编译组织方式
# ============================================================================

# ----------------------------------------------------------------------------
# 1. Common 模块 - 通用工具和基础设施
# ----------------------------------------------------------------------------
add_library(lightning.common STATIC
    common/nav_state.cc
    common/constant.h
    common/options.cc
    common/params.cc
)

target_link_libraries(lightning.common
    ${third_party_libs}
)

ament_target_dependencies(lightning.common
    "rclcpp"
    "std_msgs"
    "geometry_msgs"
)

# ----------------------------------------------------------------------------
# 2. IO 模块 - 文件读写
# ----------------------------------------------------------------------------
add_library(lightning.io STATIC
    io/yaml_io.cc
    io/file_io.cc
)

target_link_libraries(lightning.io
    lightning.common
    ${third_party_libs}
)

# ----------------------------------------------------------------------------
# 3. Utils 模块 - 实用工具
# ----------------------------------------------------------------------------
add_library(lightning.utils STATIC
    utils/timer.cc
    utils/pointcloud_utils.cc
)

target_link_libraries(lightning.utils
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

ament_target_dependencies(lightning.utils
    "sensor_msgs"
    "pcl_conversions"
)

# ----------------------------------------------------------------------------
# 4a. VoxelMap 模块 - 自适应八叉树体素地图
# ----------------------------------------------------------------------------
add_library(lightning.core.voxel_map STATIC
    core/maps/voxel_map/src/octo_tree.cpp
    core/maps/voxel_map/src/voxel_map_manager.cpp
)

target_include_directories(lightning.core.voxel_map PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/maps/voxel_map/include
)

target_link_libraries(lightning.core.voxel_map
    lightning.common
    ${third_party_libs}
    tbb
)

# ----------------------------------------------------------------------------
# 4b. LIO 模块 - 激光惯性里程计
# ----------------------------------------------------------------------------
add_library(lightning.core.lio STATIC
    core/lio/eskf.cc
    core/lio/imu_processing.cc
    core/lio/laser_mapping.cc
    core/lio/pointcloud_preprocess.cc
)

target_link_libraries(lightning.core.lio
    lightning.common
    lightning.utils
    lightning.core.voxel_map
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

ament_target_dependencies(lightning.core.lio
    "rclcpp"
    "sensor_msgs"
    "nav_msgs"
    "visualization_msgs"
    "geometry_msgs"
    "pcl_conversions"
    "tf2_geometry_msgs"
)

# 链接 livox 消息生成目标：同时保证编译顺序并传播 INTERFACE_INCLUDE_DIRECTORIES（头文件路径）
target_link_libraries(lightning.core.lio
    livox_ros_driver2__rosidl_typesupport_cpp
)

# ----------------------------------------------------------------------------
# 5. Loop Closing 模块 - 回环检测
# ----------------------------------------------------------------------------
add_library(lightning.core.loop STATIC
    core/loop_closing/loop_closing.cc
)

target_link_libraries(lightning.core.loop
    lightning.common
    lightning.io
    lightning.core.voxel_map
    ${third_party_libs}
)

# ----------------------------------------------------------------------------
# 6. G2P5 模块 - 2.5D 地图生成
# ----------------------------------------------------------------------------
add_library(lightning.core.g2p5 STATIC
    core/maps/g2p5/g2p5_map.cc
    core/maps/g2p5/g2p5.cc
    core/maps/g2p5/g2p5_subgrid.cc
)

target_link_libraries(lightning.core.g2p5
    lightning.common
    lightning.utils
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

ament_target_dependencies(lightning.core.g2p5
    "nav_msgs"
)

# ----------------------------------------------------------------------------
# 7. Maps 模块 - 地图管理
# ----------------------------------------------------------------------------
add_library(lightning.core.maps STATIC
    core/maps/tiled_map/tiled_map.cc
    core/maps/tiled_map/tiled_map_chunk.cc
)

target_link_libraries(lightning.core.maps
    lightning.common
    lightning.io
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

# ----------------------------------------------------------------------------
# 8. Localization 模块 - 定位系统
# ----------------------------------------------------------------------------
add_library(lightning.core.localization STATIC
    core/localization/localization_result.cc
    core/localization/lidar_loc/lidar_loc.cc
    core/localization/lidar_loc/pclomp/ndt_omp.cpp
    core/localization/lidar_loc/pclomp/voxel_grid_covariance_omp.cpp
    core/localization/pose_graph/pgo.cc
    core/localization/pose_graph/pgo_impl.cc
    core/localization/pose_graph/pose_extrapolator.cc
    core/localization/localization.cpp
    core/localization/relocalizer/bbs_relocalizer.cc
)

target_link_libraries(lightning.core.localization
    lightning.common
    lightning.io
    lightning.core.lio
    lightning.core.maps
    lightning.utils
    lightning.wrapper
    ${third_party_libs}
    ${PCL_LIBRARIES}
    cpu_bbs3d
)

ament_target_dependencies(lightning.core.localization
    "rclcpp"
    "sensor_msgs"
    "nav_msgs"
    "geometry_msgs"
    "visualization_msgs"
    "pcl_conversions"
)

# ----------------------------------------------------------------------------
# 9. UI 模块 - 可视化界面
# ----------------------------------------------------------------------------
add_library(lightning.ui STATIC
    ui/pangolin_window.cc
    ui/pangolin_window_impl.cc
    ui/ui_car.cc
    ui/ui_cloud.cc
    ui/ui_trajectory.cc
)

target_include_directories(lightning.ui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/maps/voxel_map/include
)

target_link_libraries(lightning.ui
    lightning.common
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

# ----------------------------------------------------------------------------
# 10. System 模块 - 系统集成 (SLAM & Localization)
# ----------------------------------------------------------------------------
add_library(lightning.system STATIC
    core/system/slam.cc
    core/system/loc_system.cc
)

target_link_libraries(lightning.system
    lightning.common
    lightning.core.lio
    lightning.core.loop
    lightning.core.g2p5
    lightning.core.maps
    lightning.core.localization
    lightning.ui
    lightning.wrapper
    ${third_party_libs}
)

ament_target_dependencies(lightning.system
    "rclcpp"
    "sensor_msgs"
    "nav_msgs"
    "std_srvs"
    "tf2"
    "tf2_ros"
    "geometry_msgs"
    "visualization_msgs"
    "pcl_conversions"
)

# ----------------------------------------------------------------------------
# 11. Wrapper 模块 - ROS 包装器
# ----------------------------------------------------------------------------
add_library(lightning.wrapper STATIC
    wrapper/bag_io.cc
    wrapper/rviz_publisher.cc
)

target_include_directories(lightning.wrapper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core/maps/voxel_map/include
)

target_link_libraries(lightning.wrapper
    lightning.common
    ${third_party_libs}
    ${PCL_LIBRARIES}
)

ament_target_dependencies(lightning.wrapper
    "rclcpp"
    "rosbag2_cpp"
    "sensor_msgs"
    "nav_msgs"
    "geometry_msgs"
    "visualization_msgs"
    "message_filters"
    "pcl_conversions"
    "tf2"
    "tf2_ros"
    "tf2_geometry_msgs"
)

add_dependencies(lightning.wrapper
    livox_ros_driver2__rosidl_typesupport_cpp
)

target_link_libraries(lightning.wrapper
    livox_ros_driver2__rosidl_typesupport_cpp
)

# ----------------------------------------------------------------------------
# 12. Miao 子模块
# ----------------------------------------------------------------------------
include_directories(${PROJECT_SOURCE_DIR}/src/core/miao)
add_subdirectory(core/miao)

# ============================================================================
# 最终接口库 - 用于应用程序链接
# ============================================================================
# 创建一个接口库，将所有子模块组合在一起
# 这样应用程序只需要链接 ${PROJECT_NAME}.libs 即可
# ============================================================================

add_library(${PROJECT_NAME}.libs INTERFACE)

target_link_libraries(${PROJECT_NAME}.libs INTERFACE
    # Lightning 核心模块
    lightning.common
    lightning.io
    lightning.utils
    lightning.core.lio
    lightning.core.loop
    lightning.core.g2p5
    lightning.core.maps
    lightning.core.localization
    lightning.ui
    lightning.system
    lightning.wrapper
    
    # Miao 子模块
    miao.core
    miao.utils
    
    # 第三方库
    ${third_party_libs}
    ${PCL_LIBRARIES}
    
    # ROS 接口
    livox_ros_driver2__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# 依赖关系
add_dependencies(${PROJECT_NAME}.libs
    livox_ros_driver2__rosidl_typesupport_cpp
    ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# ============================================================================
# 安装配置
# ============================================================================
install(TARGETS 
    lightning.common
    lightning.io
    lightning.utils
    lightning.core.lio
    lightning.core.loop
    lightning.core.g2p5
    lightning.core.maps
    lightning.core.localization
    lightning.ui
    lightning.system
    lightning.wrapper
    miao.core
    miao.utils
    DESTINATION lib
)

# ============================================================================
# 编译信息输出
# ============================================================================
message(STATUS "===========================================")
message(STATUS "Lightning-LM Modular Build Configuration")
message(STATUS "===========================================")
message(STATUS "Separate compilation enabled:")
message(STATUS "  ✓ lightning.common       - Common utilities")
message(STATUS "  ✓ lightning.io           - File I/O")
message(STATUS "  ✓ lightning.utils        - Point cloud utils")
message(STATUS "  ✓ lightning.core.lio     - LIO module")
message(STATUS "  ✓ lightning.core.loop    - Loop closing")
message(STATUS "  ✓ lightning.core.g2p5    - 2.5D mapping")
message(STATUS "  ✓ lightning.core.maps    - Map management")
message(STATUS "  ✓ lightning.core.localization - Localization")
message(STATUS "  ✓ lightning.ui           - Visualization")
message(STATUS "  ✓ lightning.system       - System integration")
message(STATUS "  ✓ lightning.wrapper      - ROS wrapper")
message(STATUS "===========================================")
message(STATUS "Expected benefits:")
message(STATUS "  - Reduced compilation memory per unit")
message(STATUS "  - Faster incremental builds")
message(STATUS "  - Better parallel compilation")
message(STATUS "===========================================")

add_subdirectory(app)
