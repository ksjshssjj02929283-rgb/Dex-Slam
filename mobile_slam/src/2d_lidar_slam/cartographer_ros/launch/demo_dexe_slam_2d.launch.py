#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  # Launch arguments
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_rosbag = LaunchConfiguration('use_rosbag')
  bag_filename = LaunchConfiguration('bag_filename')

  # Declare launch arguments
  declare_use_sim_time = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation time'
  )

  declare_use_rosbag = DeclareLaunchArgument(
    'use_rosbag',
    default_value='false',
    description='If true, play a rosbag2 bag'
  )

  declare_bag_filename = DeclareLaunchArgument(
    'bag_filename',
    default_value='',
    description='Path to rosbag2 bag (directory for rosbag2)'
  )

  # Locate cartographer_ros shared directory
  carto_share = get_package_share_directory('cartographer_ros')

  # Include another launch file (assumed ROS2 .launch.py)
  # Adjust the filename if your ROS2 package provides a different launcher.
  include_cartographer = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      PathJoinSubstitution([carto_share, 'launch', 'dexe_slam_2d.launch.py'])
    )
  )

  # RViz2 node with a config file path
  rviz_config = PathJoinSubstitution([carto_share, 'configuration_files', 'B1-2d_mapping_indoor.rviz'])
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=['-d', rviz_config],
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}]
  )

  # rosbag2 play (conditional)
  # rosbag2 expects a directory containing metadata.yaml
  rosbag2_play = Node(
    package='ros2bag',
    executable='play',  # on some distros it is provided by package 'rosbag2'
    name='playbag',
    output='screen',
    arguments=['--clock', bag_filename],
    condition=IfCondition(use_rosbag)
  )

  # Alternatively, depending on your distro, the package/executable might be:
  # package='rosbag2', executable='play'
  # If you see "package not found", change to the correct pair for your ROS2 distribution.

  # Build the launch description
  ld = LaunchDescription()

  # Parameters at process level
  ld.add_action(declare_use_sim_time)
  ld.add_action(declare_use_rosbag)
  ld.add_action(declare_bag_filename)

  # Include cartographer
  ld.add_action(include_cartographer)

  # Start rviz2
  ld.add_action(rviz_node)

  # Conditionally play bag
  ld.add_action(rosbag2_play)

  return ld