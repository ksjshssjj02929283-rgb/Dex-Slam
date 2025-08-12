from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ## ***** Launch arguments *****
    bag_filenames_arg = DeclareLaunchArgument('bag_filenames')
    no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value='false')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/B1-2d_mapping_indoor.rviz')
    configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files')
    configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames', default_value = 'B1-2d_mapping_indoor.lua')
    urdf_filenames_arg = DeclareLaunchArgument('urdf_filenames', default_value = FindPackageShare('cartographer_ros').find('cartographer_ros') + '/urdf/B1-2d_mapping_indoor.urdf')

    ## ***** Nodes *****
    offline_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(FindPackageShare('cartographer_ros').find('cartographer_ros') + '/launch/B1-2d_offline_mapping_indoor_node.launch.py'),
        launch_arguments = {
            'bag_filenames': LaunchConfiguration('bag_filenames'),
            'no_rviz': LaunchConfiguration('no_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'configuration_directory': LaunchConfiguration('configuration_directory'),
            'configuration_basenames': LaunchConfiguration('configuration_basenames'),
            'urdf_filenames': LaunchConfiguration('urdf_filenames')}.items(),

        )
    set_remap = SetRemap('scan', 'scan')

    return LaunchDescription([
        # Launch arguments
        bag_filenames_arg,
        no_rviz_arg,
        rviz_config_arg,
        configuration_directory_arg,
        configuration_basenames_arg,
        urdf_filenames_arg,

        # Nodes
        set_remap,
        offline_node_launch,
    ])
