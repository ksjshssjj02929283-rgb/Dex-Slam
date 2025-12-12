# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
DCM localization launch file with configurable filter type.
Supports EKF, UKF, and ESKF filters.
Includes Cartographer 2D localization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional logic based on filter type."""
    
    # Get the robot_localization package directory
    robot_localization_dir = get_package_share_directory('robot_localization')
    
    # Get launch configuration values
    filter_type = LaunchConfiguration('filter_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_cartographer = LaunchConfiguration('enable_cartographer').perform(context)
    map_folder = LaunchConfiguration('map_folder')
    map_name = LaunchConfiguration('map_name')
    resolution = LaunchConfiguration('resolution')
    
    # Determine executable and params file based on filter type
    if filter_type.lower() == 'ekf':
        executable = 'ekf_node'
        node_name = 'ekf_filter_node'
        params_file = os.path.join(robot_localization_dir, 'params', 'dexe_ekf.yaml')
    elif filter_type.lower() == 'ukf':
        executable = 'ukf_node'
        node_name = 'ukf_filter_node'
        params_file = os.path.join(robot_localization_dir, 'params', 'dexe_ukf.yaml')
    elif filter_type.lower() == 'eskf':
        executable = 'eskf_node'
        node_name = 'eskf_filter_node'
        params_file = os.path.join(robot_localization_dir, 'params', 'dexe_eskf.yaml')
    else:
        raise ValueError(f"Invalid filter_type: {filter_type}. Must be 'ekf', 'ukf', or 'eskf'")
    
    # Localization node
    localization_node = Node(
        package='robot_localization',
        executable=executable,
        name='robot_localization',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            # Add any topic remappings here if needed
            # ('odometry/filtered', '/odometry/local'),
        ]
    )
    
    nodes_to_launch = [localization_node]
    
    # Add Cartographer launch file if enabled
    if enable_cartographer.lower() == 'true':
        # Get cartographer_ros package share directory
        cartographer_ros_dir = get_package_share_directory('cartographer_ros')
        cartographer_launch_file = os.path.join(
            cartographer_ros_dir,
            'launch',
            'dexe_localization.launch.py'
        )
        
        # Include Cartographer localization launch file
        cartographer_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_launch_file),
            launch_arguments={
                'map_folder': map_folder,
                'map_name': map_name,
                'resolution': resolution,
            }.items()
        )
        
        nodes_to_launch.append(cartographer_launch)
    
    return nodes_to_launch


def generate_launch_description():
    """Generate launch description with configurable filter type."""
    
    # Get user home directory for default map location
    home_dir = os.path.expanduser('~')
    default_data_dir = os.path.join(home_dir, '.dexe', 'data')
    
    # Declare launch arguments
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='ukf',
        description='Type of filter to use: ekf, ukf, or eskf',
        choices=['ekf', 'ukf', 'eskf']
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_cartographer_arg = DeclareLaunchArgument(
        'enable_cartographer',
        default_value='true',
        description='Enable Cartographer 2D localization'
    )
    
    map_folder_arg = DeclareLaunchArgument(
        'map_folder',
        default_value=os.path.join(default_data_dir, 'maps'),
        description='Path to the folder containing maps (for Cartographer)'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map_2_standard',
        description='Name of the map to load (for Cartographer)'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.04',
        description='Resolution for occupancy grid (for Cartographer)'
    )
    
    return LaunchDescription([
        filter_type_arg,
        use_sim_time_arg,
        enable_cartographer_arg,
        map_folder_arg,
        map_name_arg,
        resolution_arg,
        OpaqueFunction(function=launch_setup)
    ])
