"""
    launch file for B1 2D mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time = LaunchConfiguration('use_sim_time')
    ## 如果不是仿真，这里需要设置为false
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    ## ***** urdf File paths ******
    ## ***** 通过urdf发布静态tf *****

    
    ## ***** 发布静态tf,如果有urdf，则不需要 *****
    # static_tf_publisher_1_node = Node(
    #     parameters=[
    #       {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_1',
    #     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'footprint', 'base_link'],
    #     output='screen') 
    
    # static_tf_publisher_2_node = Node(
    #     parameters=[
    #       {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_2',
    #     arguments=['0.382', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'base_laser'],
    #     output='screen') 
    
    ## ***** 启动SLAM *****
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': use_sim_time}],
        arguments = [
            '-configuration_directory', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'W1-2d_mapping_indoor.lua'],
        remappings = [
            ('scan_1', 'laser_scan/front'),
            ('scan_2', 'laser_scan/rear'),
            ('odom', 'odom'),
            ('imu', 'imu/chassis')],
        output = 'screen'
        )
    ## ***** 构建2D栅格地图 *****
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
        )
    ## ***** 启动rviz *****
    rviz_node = Node(
        parameters=[
          {'use_sim_time': use_sim_time}
        ],
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/B1-2d_mapping_indoor.rviz'],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        # robot_state_publisher_node,
        # 发布静态tf
        # static_tf_publisher_1_node,
        # static_tf_publisher_2_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        #rviz_node,
    ])
