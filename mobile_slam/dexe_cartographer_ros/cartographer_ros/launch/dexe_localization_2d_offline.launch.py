#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        default_value=os.path.join(home_dir,'datasets/mapping_new'),
        description='Path to the ROS2 bag file for offline localization testing')
    
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Directory containing the map')
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='offline_map_2',
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

    # LaunchConfiguration handles
    bag_path = LaunchConfiguration('bag_path')
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    config_file = LaunchConfiguration('config_file')
    urdf_file = LaunchConfiguration('urdf_file')
    pose_output_file = LaunchConfiguration('pose_output_file')

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

    # loc_offline_node
    loc_offline_node = Node(
        package='cartographer_ros',
        executable='loc_offline_node',
        name='localization_node',  # 统一域名到 /localization_node/（与在线定位一致）；executable 二进制名不变
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

    return LaunchDescription([
        bag_path_arg,
        map_folder_arg,
        map_name_arg,
        config_file_arg,
        urdf_file_arg,
        pose_output_file_arg,
        loc_offline_node,
    ])
