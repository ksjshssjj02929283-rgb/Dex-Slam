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

google_binary(cartographer_assets_writer
  SRCS
    assets_writer/assets_writer_main.cc
    assets_writer/ros_map_writing_points_processor.h
    assets_writer/ros_map_writing_points_processor.cc
)

install(TARGETS cartographer_assets_writer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}

)

# Standalone regression test for line-feature Marker publishing
# (RViz Duplicate Marker Check on ~/matched_feature).
google_binary(line_marker_test
  SRCS
    conversion/line_marker_test_main.cc
)

install(TARGETS line_marker_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(localization_node
  SRCS
    localization/loc_node_main.cc
)

install(TARGETS localization_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Phase 4 注入测试 binary（standalone，运行：`ros2 run cartographer_ros atomic_dir_swap_test`）
google_binary(atomic_dir_swap_test
  SRCS
    localization/atomic_dir_swap_test_main.cc
)

install(TARGETS atomic_dir_swap_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(loc_offline_node
  SRCS
    localization/loc_offline_node_main.cc
)

install(TARGETS loc_offline_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_node
  SRCS
    mapping/node_main.cc
)

install(TARGETS cartographer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Standalone regression test for the destructive start-mapping directory reset.
google_binary(map_directory_reset_test
  SRCS
    mapping/map_directory_reset_test_main.cc
)

install(TARGETS map_directory_reset_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_offline_node
  SRCS
    mapping/offline_node_main.cc
)

install(TARGETS cartographer_offline_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_occupancy_grid_node
  SRCS
    mapping/occupancy_grid_node_main.cc
)

install(TARGETS cartographer_occupancy_grid_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Standalone regression test for mapping_node submaps producing /map.
google_binary(occupancy_grid_interface_test
  SRCS
    mapping/occupancy_grid_interface_test_main.cc
)

install(TARGETS occupancy_grid_interface_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_rosbag_validate
  SRCS
    rosbag_validate/rosbag_validate_main.cc
)

install(TARGETS cartographer_rosbag_validate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_pbstream_to_ros_map
  SRCS
    mapping/pbstream_to_ros_map_main.cc
)

install(TARGETS cartographer_pbstream_to_ros_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(cartographer_pbstream_map_publisher
  SRCS
    mapping/pbstream_map_publisher_main.cc
)

install(TARGETS cartographer_pbstream_map_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# TODO(cschuet): Add support for shared library case.
if (${BUILD_GRPC})
  google_binary(cartographer_grpc_node
    SRCS
      cartographer_grpc/node_grpc_main.cc
  )

  install(TARGETS cartographer_grpc_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  google_binary(cartographer_grpc_offline_node
    SRCS
      cartographer_grpc/offline_node_grpc_main.cc
  )

  install(TARGETS cartographer_grpc_offline_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

  install(PROGRAMS
    ../scripts/cartographer_grpc_server.sh
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

google_binary(reset_map_origin_tool
  SRCS
    tools/reset_map_origin_main.cc
)

install(TARGETS reset_map_origin_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(merge_maps_tool
  SRCS
    tools/merge_maps_main.cc
)

install(TARGETS merge_maps_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(bbs_benchmark_tool
  SRCS
    tools/bbs_benchmark_main.cc
    tools/bbs_benchmark/benchmark_common.h
    tools/bbs_benchmark/benchmark_common.cc
    tools/bbs_benchmark/instrumented_fast_correlative_scan_matcher_2d.h
    tools/bbs_benchmark/instrumented_fast_correlative_scan_matcher_2d.cc
)

install(TARGETS bbs_benchmark_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

google_binary(pose_initializer_benchmark_tool
  SRCS
    tools/pose_initializer_benchmark_main.cc
    tools/bbs_benchmark/benchmark_common.h
    tools/bbs_benchmark/benchmark_common.cc
)

install(TARGETS pose_initializer_benchmark_tool
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
