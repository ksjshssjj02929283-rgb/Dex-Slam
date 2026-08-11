add_executable(run_frontend_offline
        run_frontend_offline.cc
)

target_link_libraries(run_frontend_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loop_offline
        run_loop_offline.cc
)

target_link_libraries(run_loop_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_offline
        run_slam_offline.cc
)

target_link_libraries(run_slam_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_slam_online
        run_slam_online.cc
)

target_link_libraries(run_slam_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

ament_target_dependencies(run_slam_online
    "rclcpp"
    "std_msgs"
    "std_srvs"
)
add_dependencies(run_slam_online nav_interfaces__rosidl_typesupport_cpp)
target_link_libraries(run_slam_online nav_interfaces__rosidl_typesupport_cpp)

add_executable(run_loc_offline
        run_loc_offline.cc
)

target_link_libraries(run_loc_offline
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

ament_target_dependencies(run_loc_online
    "rclcpp"
    "std_msgs"
    "std_srvs"
)
add_dependencies(run_loc_online nav_interfaces__rosidl_typesupport_cpp)
target_link_libraries(run_loc_online nav_interfaces__rosidl_typesupport_cpp)

add_executable(test_ui
        test_ui.cc
)

target_link_libraries(test_ui
        ${PROJECT_NAME}.libs
        ${third_party_libs}
)

add_executable(map_odom_tf_broadcaster
        ../wrapper/map_odom_tf_broadcaster.cc
)

ament_target_dependencies(map_odom_tf_broadcaster
    "rclcpp"
    "nav_msgs"
    "geometry_msgs"
    "tf2_ros"
    "tf2_geometry_msgs"
    "tf2"
)

install(TARGETS run_slam_offline run_slam_online run_loc_offline run_loc_online map_odom_tf_broadcaster
        DESTINATION lib/${PROJECT_NAME})