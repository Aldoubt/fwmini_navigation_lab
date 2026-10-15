from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = str(Path(get_package_share_directory('tag_follow_controller')) /
                 'config/tag_follow.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('extrinsics_verified', default_value='false'),
        DeclareLaunchArgument('target_id', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='tag_follow_controller', executable='tag_follow_controller_node',
             name='tag_follow_controller',
             parameters=[LaunchConfiguration('params_file'),
                         {'extrinsics_verified': ParameterValue(
                              LaunchConfiguration('extrinsics_verified'), value_type=bool),
                          'target_id': ParameterValue(LaunchConfiguration('target_id'), value_type=int),
                          'use_sim_time': ParameterValue(
                              LaunchConfiguration('use_sim_time'), value_type=bool)}])])
