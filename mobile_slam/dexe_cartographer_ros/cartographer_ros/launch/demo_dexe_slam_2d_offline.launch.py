#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        default_value='offline_map',
        description='Map name (used for output files)')
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Occupancy grid resolution in meters')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dexe_slam_2d.lua',
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

    # Include the offline slam launch
    include_offline_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([carto_share, 'launch', 'dexe_slam_2d_offline.launch.py'])
        ),
        launch_arguments={
            'bag_path': bag_path,
            'map_folder': map_folder,
            'map_name': map_name,
            'resolution': resolution,
            'config_file': config_file,
            'urdf_file': urdf_file,
        }.items()
    )

    # RViz2 node with a config file path
    rviz_config = PathJoinSubstitution([carto_share, 'rviz', '2d_mapping_indoor.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        bag_path_arg,
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        config_file_arg,
        urdf_file_arg,
        include_offline_slam,
        rviz_node,
    ])
