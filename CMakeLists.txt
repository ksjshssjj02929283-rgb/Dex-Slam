cmake_minimum_required(VERSION 3.16)

project(lightning)

option(BUILD_CUDA "Build with CUDA GPU BBS3D support" OFF)
option(ENABLE_NVTX_PROFILING "Enable NVIDIA Nsight Systems NVTX ranges" OFF)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# Default to Release, while still allowing callers to request RelWithDebInfo.
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif ()

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

# 编译器内存优化选项
# --param ggc-min-expand=20: 减少GC触发阈值，更频繁释放内存
# --param ggc-min-heapsize=32768: 限制GC堆大小
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --param ggc-min-expand=20 --param ggc-min-heapsize=32768")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O2 -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb -g -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
    "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fno-omit-frame-pointer -fopenmp -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")

# NVTX v3 is header-only. Nsight Systems injects its collector only while a
# profile is active, so these ranges are effectively no-ops during normal use.
if (ENABLE_NVTX_PROFILING)
    file(GLOB NVTX3_INCLUDE_HINTS
        "/opt/nvidia/nsight-systems/*/target-linux-*/nvtx/include"
        "/usr/local/cuda-*/include"
    )
    find_path(NVTX3_INCLUDE_DIR nvtx3/nvToolsExt.h
        HINTS ${NVTX3_INCLUDE_HINTS} /usr/local/cuda/include
    )
    if (NVTX3_INCLUDE_DIR)
        message(STATUS "NVTX profiling enabled: ${NVTX3_INCLUDE_DIR}")
        add_compile_definitions(LIGHTNING_ENABLE_NVTX_PROFILING)
        include_directories(${NVTX3_INCLUDE_DIR})
    else ()
        message(WARNING "NVTX3 headers not found; profiling ranges will be disabled")
    endif ()
endif ()


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

# 设置 RPATH，确保在 Jetson 等平台运行时能找到 /usr/local/lib 下的 Pangolin 等第三方库
set(CMAKE_INSTALL_RPATH "/usr/local/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

# msg 和 srv
set(MY_SRV_FILES "")
set(SRC_DIR ${PROJECT_SOURCE_DIR}/srv)
file(GLOB_RECURSE SRC_FILES ${SRC_DIR}/*.srv)
foreach (SRC_FILE ${SRC_FILES})
    file(RELATIVE_PATH REL_PATH ${SRC_DIR} ${SRC_FILE})
    list(APPEND MY_SRV_FILES ${PROJECT_SOURCE_DIR}:srv/${REL_PATH})
endforeach ()

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MY_SRV_FILES}
        ${MY_MSG_FILES}
        ${MY_ACTION_FILES}
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
        # livox_ros_driver2 的消息头文件在其子目录的构建目录中生成
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/livox_ros_driver/rosidl_generator_cpp
        # nav_interfaces 的消息头文件在其子目录的构建目录中生成
        ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/nav_interfaces/rosidl_generator_cpp
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
        DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
        scripts/debug/monitor_lightning_resources.sh
        scripts/profile_lightning_threads.sh
        scripts/debug/visualize_lightning_resources.py
        DESTINATION lib/${PROJECT_NAME}
)

add_subdirectory(thirdparty/livox_ros_driver)
add_subdirectory(thirdparty/3d_bbs)
add_subdirectory(thirdparty/nav_interfaces)

add_subdirectory(src)

ament_package()
