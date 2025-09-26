#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')

    # AMCL node
    amcl_node = Node(
        package='amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_model_type': 'diff',
            'odom_alpha5': 0.1,
            'gui_publish_rate': 10.0,
            'laser_max_beams': 30,
            'min_particles': 200,
            'max_particles': 2000,
            'kld_err': 0.05,
            'kld_z': 0.99,
            'odom_alpha1': 0.2,
            'odom_alpha2': 0.2,
            'odom_alpha3': 0.3,  # translation std dev, m
            'odom_alpha4': 0.2,
            'laser_z_hit': 0.95,
            'laser_z_short': 0.05,
            'laser_z_max': 0.05,
            'laser_z_rand': 0.50,
            'laser_sigma_hit': 0.2,
            'laser_lambda_short': 0.1,
            'laser_model_type': 'likelihood_field_prob',
            'laser_likelihood_max_dist': 2.0,
            'update_min_d': 0.2,
            'update_min_a': 0.5,
            'odom_frame_id': 'odom',
            'resample_interval': 1,
            'transform_tolerance': 0.1,
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_a': 0.0,
            'initial_cov_xx': 0.25,
            'initial_cov_yy': 0.25,
            'initial_cov_aa': 0.068539,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        amcl_node
    ])
