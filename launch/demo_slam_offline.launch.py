#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lightning_launch_utils import default_data_dir, resolve_config_file


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('lightning')
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    input_bag = LaunchConfiguration('input_bag').perform(context)
    map_folder = LaunchConfiguration('map_folder').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    config_file = resolve_config_file(pkg_dir, lidar_type)

    if not input_bag:
        raise RuntimeError('input_bag is required for offline SLAM. Pass input_bag:=<path>')

    executable = os.path.join(get_package_prefix('lightning'), 'lib', 'lightning', 'run_slam_offline')
    return [
        ExecuteProcess(
            cmd=[
                executable,
                '--input_bag', input_bag,
                '--config', config_file,
                '--map_folder', map_folder,
                '--map_name', map_name,
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory('lightning')
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type',
            default_value='airy',
            description='LiDAR type used to choose config yaml: airy, livox, vbr',
        ),
        DeclareLaunchArgument(
            'input_bag',
            default_value='',
            description='Path to the ROS2 bag file directory',
        ),
        DeclareLaunchArgument(
            'map_folder',
            default_value=os.path.join(default_data_dir(), 'maps'),
            description='Directory to save offline maps',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='offline_map',
            description='Name for the offline map',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_dir, 'rviz', 'lightning.rviz'),
            description='RViz configuration file path',
        ),
        OpaqueFunction(function=launch_setup),
    ])