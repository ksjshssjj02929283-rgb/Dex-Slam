#!/usr/bin/env python3
"""
Lightning-LM SLAM Online Launch File - Demo (Livox)

启动在线 SLAM 建图系统（Demo 模式，自动打开 RViz）。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('lightning')
    default_config_file = os.path.join(pkg_dir, 'config', 'default_livox.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'lightning.rviz')

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the SLAM configuration YAML file'
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file'
    )

    config_file = LaunchConfiguration('config_file')
    rviz_config = LaunchConfiguration('rviz_config')

    slam_node = ExecuteProcess(
        cmd=['ros2', 'run', 'lightning', 'run_slam_online', '--config', config_file],
        output='screen',
        shell=False
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        shell=False
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    return ld
