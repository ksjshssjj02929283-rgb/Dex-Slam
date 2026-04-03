#!/usr/bin/env python3
"""
Lightning-LM Localization Online Launch File (VBR)

启动在线定位系统，并可选择自动打开 RViz。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('lightning')
    default_config_file = os.path.join(pkg_dir, 'config', 'default_vbr.yaml')
    declare_config_file_cmd = DeclareLaunchArgument('config_file', default_value=default_config_file,
                                                   description='Full path to the localization configuration YAML file')

    config_file = LaunchConfiguration('config_file')

    loc_node = ExecuteProcess(cmd=['ros2', 'run', 'lightning', 'run_loc_online', '--config', config_file],
                              output='screen', shell=False)

    ld = LaunchDescription()
    ld.add_action(declare_config_file_cmd)
    ld.add_action(loc_node)
    return ld
