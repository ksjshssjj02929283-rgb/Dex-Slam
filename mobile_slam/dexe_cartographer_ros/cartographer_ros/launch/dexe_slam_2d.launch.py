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
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Directory to save/load maps')
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='dex_map_test',
        description='Map name (used for output files)')
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.04',
        description='Occupancy grid resolution in meters')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dexe_slam_2d.lua',
        description='Configuration file name')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    # 话题重映射
    topic_remappings = [
        ('scan_1', 'laser_scan/front'),
        ('scan_2', 'laser_scan/rear'),
        ('imu', 'imu/chassis'),
        ('odom', 'feedback/odom')
    ]

    # LaunchConfiguration handles
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Locate cartographer_ros share directory to replace $(find cartographer_ros)
    carto_share = get_package_share_directory('cartographer_ros')
    configuration_directory = PathJoinSubstitution([carto_share, 'configuration_files'])

    # mapping_node: cartographer_node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],  # 修复: 使用LaunchConfiguration
        respawn=True,
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', config_file,  # 修复: 使用LaunchConfiguration而不是DeclareLaunchArgument
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-resolution', resolution,
            '-collect_metrics', 'false',
        ],
        remappings=topic_remappings,
        # prefix=['gdb -ex run --args'],
    )

    # cartographer_occupancy_grid_node
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=True,
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],  # 修复: 使用LaunchConfiguration
        arguments=[
            '-resolution', resolution,   # 修复: 使用LaunchConfiguration而不是硬编码
        ],
    )

    # Optionally enable robot_state_publisher with URDF (commented out in ROS 1)
    # If you want to activate it in ROS 2, you can do something like:
    #
    # from launch_ros.parameter_descriptions import ParameterValue
    # from launch.substitutions import Command, TextSubstitution
    # urdf_path = PathJoinSubstitution([carto_share, 'urdf', 'backpack_2d.urdf'])
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': ParameterValue(
    #         Command(['xacro ', urdf_path]), value_type=str)}]
    # )

    # If you need to use simulated time in ROS 2:
    # use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    # and pass parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] to nodes

    return LaunchDescription([
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        config_file_arg,  # 修复: 添加缺失的config_file_arg
        use_sim_time_arg,
        cartographer_node,
        occupancy_grid_node,
        # robot_state_publisher,  # uncomment if enabled above
    ])
