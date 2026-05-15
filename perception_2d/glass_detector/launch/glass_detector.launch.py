"""glass_detector 单节点启动 launch（一期 A 路径）。

用法：
  ros2 launch glass_detector glass_detector.launch.py
  ros2 launch glass_detector glass_detector.launch.py use_glass_detection:=false

参数：
  use_glass_detection (bool, default=true)：
    true  → 玻璃检测 + scan 修正发布到 /laser_scan/{front,rear}_corrected
    false → pass-through（corrected 话题完全等同于原 scan，方便 A/B 对比测试）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_glass_arg = DeclareLaunchArgument(
        'use_glass_detection',
        default_value='true',
        description='Enable glass detection scan correction (false = pass-through).',
    )

    params_file = os.path.join(
        get_package_share_directory('glass_detector'),
        'config',
        'glass_detector_params.yaml',
    )

    glass_node = Node(
        package='glass_detector',
        executable='glass_detector_node',
        name='glass_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {'use_glass_detection': LaunchConfiguration('use_glass_detection')},
        ],
    )

    return LaunchDescription([use_glass_arg, glass_node])
