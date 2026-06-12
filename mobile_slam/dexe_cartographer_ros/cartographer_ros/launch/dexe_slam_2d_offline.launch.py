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
        description='Path to the ROS2 bag file for offline mapping')
    
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Directory to save maps')
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='offline_map_2',
        description='Map name (used for output files)')
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Occupancy grid resolution in meters')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dexe_slam_2d_offline.lua',
        description='Configuration file name')
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',
        description='Optional URDF file for static transforms')

    # gdb 调试开关（默认关）。gdb:=true 时用 gdb 包装节点运行，崩溃时停在 gdb 提示符（输入 bt 看栈）。
    gdb_arg = DeclareLaunchArgument(
        'gdb',
        default_value='false',
        description='Run the cartographer node under gdb (true/false). Default false.')
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
    resolution = LaunchConfiguration('resolution')
    config_file = LaunchConfiguration('config_file')
    urdf_file = LaunchConfiguration('urdf_file')
    gdb = LaunchConfiguration('gdb')
    gdb_prefix = LaunchConfiguration('gdb_prefix')

    # Locate cartographer_ros share directory
    carto_share = get_package_share_directory('cartographer_ros')
    configuration_directory = PathJoinSubstitution([carto_share, 'configuration_files'])

    # 话题重映射 - 将bag中的实际话题映射到cartographer期望的传感器ID
    topic_remappings = [
        ('scan_1', 'laser_scan/front'),
        ('scan_2', 'laser_scan/rear'),
        ('imu', 'imu/chassis/refined'),
        ('odom', 'feedback/odom')
    ]

    # cartographer_offline_node 公共参数（gdb 开/关两个实例共用，仅 prefix + condition 不同）
    offline_node_kwargs = dict(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        name='cartographer_offline_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basenames', config_file,
            '-bag_filenames', bag_path,
            '-urdf_filenames', urdf_file,
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-resolution', resolution,
            '-use_bag_transforms', 'true',
        ],
        remappings=topic_remappings,
    )
    # gdb:=false（默认）→ 正常运行；gdb:=true → 用 gdb_prefix 包装节点。
    offline_node = Node(condition=UnlessCondition(gdb), **offline_node_kwargs)
    offline_node_gdb = Node(condition=IfCondition(gdb), prefix=[gdb_prefix], **offline_node_kwargs)

    return LaunchDescription([
        bag_path_arg,
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        config_file_arg,
        urdf_file_arg,
        gdb_arg,
        gdb_prefix_arg,
        offline_node,
        offline_node_gdb,
    ])
