#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get user home directory
    home_dir = os.path.expanduser('~')
    default_data_dir = os.path.join(home_dir, '.dexe', 'data')
    
    # Launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=os.path.join(home_dir,'datasets/dongguan/bags/mapping_big_filtered'),
        description='Path to the ROS2 bag file for offline localization testing')
    
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Directory containing the map')
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='offline_map',
        description='Map name to load for localization')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dexe_localization_2d_offline.lua',
        description='Configuration file name for localization')
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Optional URDF file for static transforms')
    
    pose_output_file_arg = DeclareLaunchArgument(
        'pose_output_file',
        default_value='',
        description='Optional file to save localization poses')

    # gdb 调试开关（默认关）。gdb:=true 时用 gdb 包装节点运行，崩溃时停在 gdb 提示符（输入 bt 看栈）。
    gdb_arg = DeclareLaunchArgument(
        'gdb',
        default_value='false',
        description='Run the localization node under gdb (true/false). Default false.')
    # gdb 包装命令，可改：'xterm -e gdb -ex run --args'（独立窗口）/
    # 'gdb -batch -ex run -ex bt --args'（崩溃自动打印栈后退出，适合离线回放抓崩溃）。
    gdb_prefix_arg = DeclareLaunchArgument(
        'gdb_prefix',
        default_value='gdb -ex run --args',
        description='Command prefix used to wrap the node when gdb:=true.')

    # LaunchConfiguration handles
    bag_path = LaunchConfiguration('bag_path')
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    config_file = LaunchConfiguration('config_file')
    urdf_file = LaunchConfiguration('urdf_file')
    pose_output_file = LaunchConfiguration('pose_output_file')
    gdb = LaunchConfiguration('gdb')
    gdb_prefix = LaunchConfiguration('gdb_prefix')

    # Locate cartographer_ros share directory
    carto_share = get_package_share_directory('cartographer_ros')
    configuration_directory = PathJoinSubstitution([carto_share, 'configuration_files'])
    
    # Construct map filestream path
    map_filestream = PathJoinSubstitution([map_folder, map_name, 'map.pbstream'])

    # 话题重映射 - 将bag中的实际话题映射到cartographer期望的传感器ID
    topic_remappings = [
        ('scan_1', 'laser_scan/front'),
        ('scan_2', 'laser_scan/rear'),
        ('imu', 'imu/chassis/refined'),
        ('odom', 'feedback/odom')
    ]

    # loc_offline_node 公共参数（gdb 开/关两个实例共用，仅 prefix + condition 不同）
    loc_offline_node_kwargs = dict(
        package='cartographer_ros',
        executable='loc_offline_node',
        name='loc_offline_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', config_file,
            '-bag_filenames', bag_path,
            '-map_filestream', map_filestream,
            '-urdf_filenames', urdf_file,
            '-load_frozen_state', 'true',
            '-use_bag_transforms', 'true',
            '-pose_output_file', pose_output_file,
        ],
        remappings=topic_remappings,
    )
    # gdb:=false（默认）→ 正常运行；gdb:=true → 用 gdb_prefix 包装节点。
    loc_offline_node = Node(condition=UnlessCondition(gdb), **loc_offline_node_kwargs)
    loc_offline_node_gdb = Node(condition=IfCondition(gdb), prefix=[gdb_prefix], **loc_offline_node_kwargs)

    return LaunchDescription([
        bag_path_arg,
        map_folder_arg,
        map_name_arg,
        config_file_arg,
        urdf_file_arg,
        pose_output_file_arg,
        gdb_arg,
        gdb_prefix_arg,
        loc_offline_node,
        loc_offline_node_gdb,
    ])
