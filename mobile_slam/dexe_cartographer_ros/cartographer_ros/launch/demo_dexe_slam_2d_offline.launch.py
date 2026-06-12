#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 本 demo = dexe_slam_2d_offline.launch.py + RViz。
    # 地图/数据集目录、config、gdb 等所有参数的**唯一来源**就是 dexe_slam_2d_offline.launch.py：
    # 改这些只需编辑那一个文件。这里不再重复声明，也不通过 launch_arguments 覆盖，
    # 被包含 launch 直接用它自己的默认值；CLI 传参（如 bag_path:=... / gdb:=true）会透传下去。
    carto_share = get_package_share_directory('cartographer_ros')

    include_offline_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([carto_share, 'launch', 'dexe_slam_2d_offline.launch.py'])
        )
    )

    # RViz2（离线回放用 sim time）
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
        include_offline_slam,
        rviz_node,
    ])
