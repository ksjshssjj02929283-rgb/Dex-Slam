"""
    launch file for B1 2D localization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    ## ***** Launch arguments *****
    # bag_filename_arg = DeclareLaunchArgument('bag_filename',
    #                                          default_value='/home/hz/hz/Dataset/rslidar-outdoor-gps.bag',
    #                                          description='Load bag file')
    load_state_filename_arg = DeclareLaunchArgument('load_state_filename',
                                                    default_value='/home/hz/hz/Work/Grabotics/B1-project/B1-SLAM/src/B1-2dLidar-SLAM/SLAM_ros/B1-cartographer_ros/cartographer_ros/results/map/rslidar-outdoor-gps.pbstream',
                                                    description='Load pbstream file')

    ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'test_mapping_hz.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}],
        output = 'screen'
        )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'test_localization_hz.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename')],
        remappings = [
            ('scan', 'front_scan'),
            ('odom', 'odom_scout'),
            ('imu', 'imu')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': True},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/test_localization_hz.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    # ros2_bag_play_cmd = ExecuteProcess(
    #     cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
    #     name = 'rosbag_play',
    # )

    return LaunchDescription([
        # Launch arguments
        # bag_filename_arg,
        load_state_filename_arg,
        # Nodes
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
        # ros2_bag_play_cmd
    ])
