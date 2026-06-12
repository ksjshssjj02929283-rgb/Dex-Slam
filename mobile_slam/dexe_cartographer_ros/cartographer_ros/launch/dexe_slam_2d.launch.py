from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
        default_value='0.05',
        description='Occupancy grid resolution in meters')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='dexe_slam_2d.lua',
        description='Configuration file name')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    # gdb 调试开关（默认关）。gdb:=true 时用 gdb 包装 cartographer_node（并禁用 respawn，
    # 否则崩溃退出后会被反复重启）。崩溃时停在 gdb 提示符，输入 bt 看栈。
    gdb_arg = DeclareLaunchArgument(
        'gdb',
        default_value='false',
        description='Run the cartographer node under gdb (true/false). Default false.')
    gdb_prefix_arg = DeclareLaunchArgument(
        'gdb_prefix',
        default_value='gdb -ex run --args',
        description='Command prefix used to wrap the node when gdb:=true '
                    "(e.g. 'xterm -e gdb -ex run --args' or 'gdb -batch -ex run -ex bt --args').")

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
    gdb = LaunchConfiguration('gdb')
    gdb_prefix = LaunchConfiguration('gdb_prefix')

    # Locate cartographer_ros share directory to replace $(find cartographer_ros)
    carto_share = get_package_share_directory('cartographer_ros')
    configuration_directory = PathJoinSubstitution([carto_share, 'configuration_files'])

    # mapping_node: cartographer_node 公共参数（gdb 开/关两实例共用，仅 respawn/prefix/condition 不同）
    cartographer_node_kwargs = dict(
        package='cartographer_ros',
        executable='cartographer_node',
        name='mapping_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', config_file,
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-resolution', resolution,
            '-collect_metrics', 'false',
        ],
        remappings=topic_remappings,
    )
    # gdb:=false（默认）→ 正常运行（respawn=True）；gdb:=true → gdb 包装且 respawn=False。
    cartographer_node = Node(
        condition=UnlessCondition(gdb), respawn=True, **cartographer_node_kwargs)
    cartographer_node_gdb = Node(
        condition=IfCondition(gdb), respawn=False, prefix=[gdb_prefix], **cartographer_node_kwargs)

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
        gdb_arg,
        gdb_prefix_arg,
        cartographer_node,
        cartographer_node_gdb,
        occupancy_grid_node,
        # robot_state_publisher,  # uncomment if enabled above
    ])
