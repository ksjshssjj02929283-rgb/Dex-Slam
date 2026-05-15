"""glass_detector + Cartographer 全链路 launch（一期 A 路径占位）。

⚠️ 注意：本文件提供 glass_detector 部分 + 在终端打印 Cartographer 应做的 remap，
但**不直接启动 Cartographer**（避免假定 dexforce/0_slam/slam_2d 的具体 launch
结构）。实测时按打印提示在你的 Cartographer launch 文件里加 remap：

    remappings=[
        ('scan_front', LaunchConfiguration_of_front_topic),
        ('scan_rear',  LaunchConfiguration_of_rear_topic),
    ]

或者更简单：直接在 Cartographer launch 里把订阅的话题改成 `/laser_scan/*_corrected`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_glass_arg = DeclareLaunchArgument(
        'use_glass_detection',
        default_value='true',
        description='Enable glass detection scan correction.',
    )

    # 根据 use_glass_detection 选用 corrected / 原始话题
    front_topic = PythonExpression(
        [
            "'/laser_scan/front_corrected' if '",
            LaunchConfiguration('use_glass_detection'),
            "' == 'true' else '/laser_scan/front'",
        ]
    )
    rear_topic = PythonExpression(
        [
            "'/laser_scan/rear_corrected' if '",
            LaunchConfiguration('use_glass_detection'),
            "' == 'true' else '/laser_scan/rear'",
        ]
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

    info_front = LogInfo(
        msg=[
            '[glass_detector] Cartographer should subscribe FRONT scan topic: ',
            front_topic,
        ]
    )
    info_rear = LogInfo(
        msg=[
            '[glass_detector] Cartographer should subscribe REAR scan topic:  ',
            rear_topic,
        ]
    )

    return LaunchDescription([use_glass_arg, glass_node, info_front, info_rear])
