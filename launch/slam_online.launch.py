#!/usr/bin/env python3
"""
Lightning-LM SLAM Online Launch File

启动在线 SLAM 建图系统并自动打开 RViz 可视化

使用方法:
    ros2 launch lightning slam_online.launch.py
    ros2 launch lightning slam_online.launch.py config_file:=/path/to/config.yaml
    ros2 launch lightning slam_online.launch.py config_file:=/path/to/config.yaml use_rviz:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的共享目录
    pkg_dir = get_package_share_directory('lightning')
    
    # 默认配置文件路径（从 share 目录读取）
    default_config_file = os.path.join(pkg_dir, 'config', 'default_vbr.yaml')
    
    # 默认 RViz 配置文件路径（从 share 目录读取）
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'lightning.rviz')
    
    # 声明启动参数
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the SLAM configuration YAML file'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Full path to the RViz configuration file'
    )
    
    # 获取启动参数
    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # SLAM 节点
    slam_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'lightning', 'run_slam_online',
            '--config', config_file
        ],
        output='screen',
        shell=False
    )
    
    # RViz2 节点 (条件启动)
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen',
        shell=False,
        condition=IfCondition(use_rviz)
    )
    
    # 创建并返回 LaunchDescription
    ld = LaunchDescription()
    
    # 添加启动参数声明
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    
    # 添加节点
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    
    return ld
