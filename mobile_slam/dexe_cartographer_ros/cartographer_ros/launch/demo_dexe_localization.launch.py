#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_rosbag = LaunchConfiguration('use_rosbag')
    bag_filename = LaunchConfiguration('bag_filename')
    start_time = LaunchConfiguration('start_time')

    # Declare launch arguments
    declare_use_rosbag = DeclareLaunchArgument(
        'use_rosbag',
        default_value='false',
        description='Whether to play rosbag'
    )

    declare_bag_filename = DeclareLaunchArgument(
        'bag_filename',
        default_value='',
        description='Path to the bag file to play'
    )

    declare_start_time = DeclareLaunchArgument(
        'start_time',
        default_value='0',
        description='Start time for bag playback'
    )

    # Locate cartographer_ros shared directory
    carto_share = get_package_share_directory('cartographer_ros')

    # Get robot_localization package directory
    robot_localization_dir = get_package_share_directory('robot_localization')
    robot_localization_launch_file = os.path.join(
        robot_localization_dir,
        'launch',
        'dexe_localization.launch.py'
    )

    # Include robot_localization launch file
    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_localization_launch_file),
        launch_arguments={
            'filter_type': 'ukf',
        }.items()
    )

    # Include dexe_localization.launch.py
    include_cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([carto_share, 'launch', 'dexe_localization.launch.py'])
        )
    )

    # RViz configuration file
    rviz_config_file = PathJoinSubstitution([
        carto_share,
        'rviz',
        '2d_localization_indoor.rviz'
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=[
            '-d', rviz_config_file
        ]
    )

    # Rosbag playback
    rosbag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', 
            bag_filename,
            '--clock',
            '--start-offset', start_time
        ],
        name='playbag',
        output='screen',
        condition=IfCondition(PythonExpression(['"', use_rosbag, '" == "true"']))
    )

    return LaunchDescription([
        # Launch arguments
        declare_use_rosbag,
        declare_bag_filename,
        declare_start_time,
        
        # Robot localization
        robot_localization_launch,
        
        # Include localization
        include_cartographer,
        
        # RViz
        rviz_node,
        
        # Rosbag playback
        rosbag_play,
    ])
