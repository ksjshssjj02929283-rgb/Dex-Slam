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

    # LaunchConfiguration handles
    bag_path = LaunchConfiguration('bag_path')
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')
    config_file = LaunchConfiguration('config_file')
    urdf_file = LaunchConfiguration('urdf_file')

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

    # cartographer_offline_node
    offline_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        name='mapping_node',
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

    return LaunchDescription([
        bag_path_arg,
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        config_file_arg,
        urdf_file_arg,
        offline_node,
    ])
