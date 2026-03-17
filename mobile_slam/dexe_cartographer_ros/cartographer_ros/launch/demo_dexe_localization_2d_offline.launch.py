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

    # Include the offline localization launch
    include_offline_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([carto_share, 'launch', 'dexe_localization_2d_offline.launch.py'])
        ),
        launch_arguments={
            'bag_path': bag_path,
            'map_folder': map_folder,
            'map_name': map_name,
            'config_file': config_file,
            'urdf_file': urdf_file,
            'pose_output_file': pose_output_file,
        }.items()
    )

    # RViz2 node with a config file path
    rviz_config = PathJoinSubstitution([carto_share, 'rviz', '2d_localization_indoor.rviz'])
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
        config_file_arg,
        urdf_file_arg,
        pose_output_file_arg,
        include_offline_localization,
        rviz_node,
    ])
