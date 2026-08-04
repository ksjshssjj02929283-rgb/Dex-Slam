#!/usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from lightning_launch_utils import default_data_dir, resolve_config_file


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('lightning')
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    map_folder = LaunchConfiguration('map_folder').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    config_file = resolve_config_file(pkg_dir, lidar_type)

    map_yaml = ''
    for p in [os.path.join(map_folder, map_name, 'map.yaml'),
              os.path.join(map_folder, map_name, 'slam', 'map.yaml')]:
        if os.path.exists(p):
            map_yaml = p
            break
    if not map_yaml:
        map_yaml = os.path.join(map_folder, map_name, 'map.yaml')

    map_pub_script = os.path.join(pkg_dir, 'scripts', 'map_publisher.py')
    bridge_script = os.path.join(pkg_dir, 'scripts', 'initialpose_bridge.py')
    map_tf_script = os.path.join(pkg_dir, 'scripts', 'map_tf_pub.py')

    return [
        Node(
            package='lightning',
            executable='run_loc_online',
            name='localization_3d_manager',
            output='screen',
            arguments=[
                '--config', config_file,
                '--map_folder', map_folder,
                '--map_name', map_name,
                '--ros-args', '-r', '/tf:=/tf_loc',
            ],
        ),
        ExecuteProcess(
            cmd=['python3', map_pub_script, '--ros-args', '-p', f'map_yaml:={map_yaml}'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', bridge_script],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', map_tf_script],
            output='screen'),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan', output='screen',
            remappings=[('cloud_in', '/lightning/cloud/current_scan'), ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_footprint',
                'transform_tolerance': 0.5,
                'min_height': -0.3, 'max_height': 1.5,
                'angle_min': -3.14159, 'angle_max': 3.14159,
                'angle_increment': 0.0087,
                'range_min': 0.5, 'range_max': 70.0,
                'scan_time': 0.1, 'use_inf': True, 'inf_epsilon': 1.0,
            }]),
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
            'map_folder',
            default_value=os.path.join(default_data_dir(), 'maps'),
            description='Directory to load maps from',
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='default_map',
            description='Name of the map to load',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_dir, 'rviz', 'lightning.rviz'),
            description='RViz configuration file path',
        ),
        OpaqueFunction(function=launch_setup),
    ])
