#!/usr/bin/env python3

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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value='/home/fengbro/dexforce/0_slam/data/maps/',
        description='Path to the folder containing maps'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map_2',
        description='Name of the map to load'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.04',
        description='Resolution for occupancy grid'
    )

    use_rosbag_arg = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Whether to play rosbag'
    )

    bag_filename_arg = DeclareLaunchArgument(
        'bag_filename',
        default_value='',
        description='Path to the bag file to play'
    )

    start_time_arg = DeclareLaunchArgument(
        'start_time',
        default_value='0',
        description='Start time for bag playback'
    )

    # Get launch configurations
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_filename = LaunchConfiguration('bag_filename')
    start_time = LaunchConfiguration('start_time')

    # Get package directory for configuration files
    cartographer_ros_share = FindPackageShare('cartographer_ros')
    configuration_directory = PathJoinSubstitution([
        cartographer_ros_share,
        'configuration_files'
    ])

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        cartographer_ros_share,
        'rviz',
        'B1-2d_mapping_indoor.rviz'
    ])

    # Localization node
    localization_node = Node(
        package='cartographer_ros',
        executable='localization_node',
        name='localization_node',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}  # 与dcm_localization.launch.py保持一致
        ],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', 'dexe_localization_2d.lua',
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-collect_metrics', 'false'
        ],
        remappings=[
            ('scan_1', 'scan_1_refined'),
            ('scan_2', 'scan_2_refined'),
            ('imu', 'imu'),
            ('odom', 'feedback/odom')
        ]
    )

    # Occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}  # 与dcm_localization.launch.py保持一致
        ],
        arguments=[
            '-resolution', resolution
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',  # ROS2中使用rviz2
        executable='rviz2',
        name='rviz',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}  # 与dcm_localization.launch.py保持一致
        ],
        arguments=[
            '-d', rviz_config_file
        ]
    )

    # Rosbag playback - 使用ros2 bag play代替rosbag play
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            bag_filename,
            '--clock',  # 对应ROS1的--clock
            '--start-offset', start_time  # 对应ROS1的-s参数
        ],
        name='playbag',
        output='screen',
        condition=IfCondition(PythonExpression(['"', use_rosbag, '" == "true"']))
    )

    return LaunchDescription([
        # Launch arguments
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        use_rosbag_arg,
        bag_filename_arg,
        start_time_arg,
        
        # Nodes
        localization_node,
        cartographer_occupancy_grid_node,
        rviz_node,
        rosbag_play,
    ])
