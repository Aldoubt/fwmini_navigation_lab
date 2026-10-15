"""Static robot preview. No camera, odometry, localization or motor drivers."""
import math
import yaml
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def start(context):
    config = LaunchConfiguration('geometry_file').perform(context)
    preview = LaunchConfiguration('preview').perform(context).lower() == 'true'
    with open(config, encoding='utf-8') as stream:
        geometry = yaml.safe_load(stream)
    for section in ('body', 'camera_mount', 'color_optical'):
        if not preview and geometry[section]['measured'] is not True:
            raise RuntimeError(f'{section} must be measured before non-preview use')
    for key in ('length_m', 'width_m', 'height_m'):
        value = geometry['body'][key]
        if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid body {key}')
    for section in ('camera_mount', 'color_optical'):
        for key in ('xyz_m', 'rpy_rad'):
            values = geometry[section][key]
            if len(values) != 3 or any(not isinstance(v, (float, int)) or not math.isfinite(v) for v in values):
                raise ValueError(f'Invalid {section}.{key}')
    model = Path(get_package_share_directory('fwmini_description')) / 'urdf/fwmini.urdf.xacro'
    robot = xacro.process_file(str(model), mappings={'geometry_file': config}).toxml()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': robot, 'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'}])]


def generate_launch_description():
    default = str(Path(get_package_share_directory('fwmini_bringup')) / 'config/geometry.preview.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('geometry_file', default_value=default),
        DeclareLaunchArgument('preview', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start)])
