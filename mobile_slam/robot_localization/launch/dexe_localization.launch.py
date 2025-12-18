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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    """Setup function to handle conditional logic based on filter type."""
    
    # Get the robot_localization package directory
    robot_localization_dir = get_package_share_directory('robot_localization')
    
    # Get launch configuration values
    filter_type = LaunchConfiguration('filter_type').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
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
    
    return [localization_node]


def generate_launch_description():
    """Generate launch description with configurable filter type."""
    
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
    
    return LaunchDescription([
        filter_type_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup)
    ])
