"""
Copyright 2026 Dexforce Inc

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    perception_2d_ros2_dir = get_package_share_directory('perception_2d_ros2')
    perception_2d_dir = get_package_share_directory('perception_2d')
    
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Configuration and model directories
    configuration_directory = os.path.join(perception_2d_ros2_dir, 'configuration_files')
    configuration_basename = 'perception_2d.lua'
    model_directory = os.path.join(perception_2d_dir, 'model')
    model_basename = 'model.yaml'
    
    # Parameters file
    params_file = os.path.join(perception_2d_ros2_dir, 'params', 'params_perception_2d.yaml')
    
    # Perception node
    perception_node = Node(
        package='perception_2d_ros2',
        executable='perception_2d_node',
        name='perception_2d_node',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'target_file_path': os.path.join(model_directory, model_basename)}
        ],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-model_directory', model_directory,
            '-model_basename', model_basename
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        perception_node
    ])
