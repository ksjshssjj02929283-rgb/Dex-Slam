#!/usr/bin/env python3

# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get user home directory
    home_dir = os.path.expanduser('~')
    default_data_dir = os.path.join(home_dir, '.dexe', 'data')
    
    # Declare launch arguments
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Path to the folder containing maps'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='offline_map',
        description='Name of the map to load'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Resolution for occupancy grid'
    )

    # gdb 调试开关。gdb:=true 时用 gdb 包装 localization_node。
    gdb_arg = DeclareLaunchArgument(
        'gdb',
        default_value='true',
        description='Run the localization node under gdb (true/false).'
    )
    # 默认用 batch 模式：崩溃时自动打印完整栈（bt full + 所有线程 bt）再退出，
    # 不会卡在交互式 (gdb) 提示符（ros2 launch 不转发 stdin，交互式会挂住不输出）。
    # 双引号包住多词 gdb 命令，launch 的 shlex 拆分会保留它们为单个 -ex 参数。
    # 要交互式调试改成：gdb_prefix:='gdb -ex run --args'（或 'xterm -e gdb -ex run --args'）。
    gdb_prefix_arg = DeclareLaunchArgument(
        'gdb_prefix',
        default_value='gdb -batch -ex run -ex "bt full" -ex "thread apply all bt" --args',
        description="Command prefix used to wrap the node when gdb:=true."
    )

    # Get launch configurations
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')
    gdb = LaunchConfiguration('gdb')
    gdb_prefix = LaunchConfiguration('gdb_prefix')

    # Get package directory for configuration files
    cartographer_ros_share = FindPackageShare('cartographer_ros')
    configuration_directory = PathJoinSubstitution([
        cartographer_ros_share,
        'configuration_files'
    ])

    # Get robot_localization package directory
    robot_localization_dir = get_package_share_directory('robot_localization')
    robot_localization_launch_file = os.path.join(
        robot_localization_dir,
        'launch',
        'dexe_localization.launch.py'
    )

    # Include robot_localization launch file
    # 用 scoped GroupAction 隔离：EKF/UKF 是依赖节点非调试目标，这里把它的 gdb 钉死为 false。
    # scoped=True 防止该 'gdb'='false' 泄漏到父作用域——否则 IncludeLaunchDescription 的
    # launch_arguments 会污染父 scope，把本 launch 的 localization_node 也判成 gdb=false（实测过这个坑）。
    robot_localization_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_localization_launch_file),
                launch_arguments={
                    'filter_type': 'ukf',
                    'gdb': 'false',
                }.items()
            )
        ]
    )

    # Localization node 公共参数（gdb 开/关两实例共用，仅 prefix + condition 不同）
    localization_node_kwargs = dict(
        package='cartographer_ros',
        executable='localization_node',
        name='localization_node',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', 'dexe_localization_2d.lua',
            '-map_folder', map_folder,
            '-map_name', map_name,
            '-collect_metrics', 'false',
            '-auto_load_map', 'true',
        ],
        remappings=[
            ('scan_1', 'laser_scan/front'),
            ('scan_2', 'laser_scan/rear'),
            ('imu', 'imu/chassis/refined'),
            ('odom', 'feedback/odom')
        ]
    )
    # gdb:=false（默认）→ 正常运行；gdb:=true → 用 gdb_prefix 包装节点。
    localization_node = Node(condition=UnlessCondition(gdb), **localization_node_kwargs)
    localization_node_gdb = Node(
        condition=IfCondition(gdb), prefix=[gdb_prefix], **localization_node_kwargs)

    # Occupancy grid node
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        respawn=False,
        parameters=[
            {'use_sim_time': False}
        ],
        arguments=[
            '-resolution', resolution
        ]
    )

    return LaunchDescription([
        # Launch arguments
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        gdb_arg,
        gdb_prefix_arg,

        # Robot localization
        robot_localization_launch,

        # Nodes
        localization_node,
        localization_node_gdb,
        # cartographer_occupancy_grid_node,
    ])
