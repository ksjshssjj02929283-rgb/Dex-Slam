#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lightning_launch_utils import default_data_dir, resolve_config_file


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('lightning')
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    map_folder = LaunchConfiguration('map_folder').perform(context)
    config_file = resolve_config_file(pkg_dir, lidar_type)

    return [
        Node(
            package='lightning',
            executable='run_slam_online',
            name='mapping_3d_manager',
            output='screen',
            arguments=[
                '--config', config_file,
                '--map_folder', map_folder,
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type',
            default_value='airy',
            description='LiDAR type used to choose config yaml: airy, livox, vbr',
        ),
        DeclareLaunchArgument(
            'map_folder',
            default_value=os.path.join(default_data_dir(), 'maps'),
            description='Directory to save/load maps',
        ),
        OpaqueFunction(function=launch_setup),
    ])