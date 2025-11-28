"""
    launch file for B1 2D localization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory
import json
import os
import subprocess

# added by hz for launching detect_apriltags.py
# def is_node_running(context, *args, **kwargs):
#     node_name = 'detect_apriltags'
#     try:
#         result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
#         nodes = result.stdout.split('\n')
#         if node_name in nodes:
#             print(f"Node '{node_name}' is already running.")
#             return []
#         else:
#             print(f"Node '{node_name}' is not running. Starting node...")
#             return [Node(
#                 package='mobile_perception',
#                 executable='detect_apriltags.py',
#                 name='detect_apriltags',
#                 output='screen'
#             )]
#     except Exception as e:
#         print(f"Error checking node status: {e}")
#         return []
# end


json_file = os.path.join(
    '/home/dexforce/w1/dexe_mobile_application/mobile_simulation/w1_nav2/',
    'config',
    'config.json'
)

with open(json_file, 'r') as f:
    config_data = json.load(f)

# 从 json 读取 Time
map_folder = config_data.get("map_filename", "default")
print(map_folder)
load_state_filename = os.path.join(
    '/home/dexforce/w1/dexe_mobile_application/mobile_simulation/w1_nav2',
    'maps',
    map_folder,
    'map.pbstream'
)

initial_poses_filepath = os.path.join(
    get_package_share_directory('cartographer_ros'),
    'configuration_files',
    'initial_poses.yaml'
)

apriltag_poses_filepath = os.path.join(
    get_package_share_directory('cartographer_ros'),
    'configuration_files',
    'apriltag_poses.yaml'
)

def generate_launch_description():
    ## ***** Launch arguments *****
    use_sim_time = LaunchConfiguration('use_sim_time')
    ## 如果不是仿真，这里需要设置为false
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    load_state_filename_arg = DeclareLaunchArgument('load_state_filename',
                                                    default_value=load_state_filename,
                                                    description='Load pbstream file')
    initial_poses_filename_arg = DeclareLaunchArgument('initial_poses_filename',
                                                    default_value=initial_poses_filepath,
                                                    description='Load pbstream file')
    apriltag_poses_filename_arg = DeclareLaunchArgument('apriltag_poses_filename',
                                                    default_value=apriltag_poses_filepath,
                                                    description='Load pbstream file')
    location_name_arg = DeclareLaunchArgument('location_name',
                                        default_value='shenzhen',
                                        description='Location of the robot')
    ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = os.path.join(pkg_share, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'b1_startup_v1.1.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    ## ***** Nodes *****
    ### urdf从b1_tf_manager中发布
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}],
        output = 'screen'
        )

    ## ***** robot_localization融合定位节点 *****
    robot_localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_localization'),
                'launch',
                'dexe_localization.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

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
            '-configuration_basename', 'W1-2d_localization_indoor.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename'),
            '-initial_poses_filename', LaunchConfiguration('initial_poses_filename'),
            '-location_name', LaunchConfiguration('location_name'),
            '-apriltag_poses_filename', LaunchConfiguration('apriltag_poses_filename')],
        remappings = [
            ('scan_1', 'laser_scan/front'),
            ('scan_2', 'laser_scan/rear'),
            ('odom', 'odom'),
            ('imu', 'imu/chassis')], # 实际未起作用，需要在node_constants.h中修改topic名
        output = 'screen'
        )
    ### 此节点用于启动栅格地图更新
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05}],
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/B1-2d_mapping_indoor.rviz'],
        parameters = [{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Launch arguments
        # OpaqueFunction(function=is_node_running), # added by hz
        use_sim_time_arg,
        load_state_filename_arg,
        initial_poses_filename_arg,
        apriltag_poses_filename_arg,
        location_name_arg,
        # Nodes
        # robot_state_publisher_node,
        robot_localization_node,
        cartographer_node,
        # cartographer_occupancy_grid_node,
        # rviz_node,
    ])
