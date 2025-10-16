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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get user home directory
    home_dir = os.path.expanduser('~')
    default_data_dir = os.path.join(home_dir, '.dexe', 'data')
    
    # Declare launch arguments
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
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

    # Get launch configurations
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')

    # Get package directory for configuration files
    cartographer_ros_share = FindPackageShare('cartographer_ros')
    configuration_directory = PathJoinSubstitution([
        cartographer_ros_share,
        'configuration_files'
    ])

    # Localization node
    localization_node = Node(
        package='cartographer_ros',
        executable='localization_node',
        name='localization_node',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', 'dexe_localization_2d.lua',
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-collect_metrics', 'false'
        ],
        remappings=[
            ('scan_1', 'laser_scan/front'),
            ('scan_2', 'laser_scan/rear'),
            ('imu', 'imu/chassis'),
            ('odom', 'feedback/odom')
        ]
    )

    # Occupancy grid node (currently commented out)
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=False,
        arguments=[
            '-resolution', resolution
        ]
    )

    return LaunchDescription([
        # Launch arguments
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        
        # Nodes
        localization_node,
        # cartographer_occupancy_grid_node,  # 取消注释如需要
    ])
