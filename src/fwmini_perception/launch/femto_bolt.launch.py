"""P1: include the official driver only. No external camera mounting TF."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    defaults = {
        'camera_name': 'camera', 'serial_number': '',
        'enable_color': 'true', 'enable_depth': 'true',
        'enable_point_cloud': 'false', 'depth_registration': 'false',
        # No internal driver TF by default: resolve ownership before enabling.
        'publish_tf': 'false', 'color_width': '1280', 'color_height': '720',
        'color_fps': '30', 'depth_width': '640', 'depth_height': '576',
        'depth_fps': '30'}
    driver = Path(get_package_share_directory('orbbec_camera')) / 'launch/femto_bolt.launch.py'
    return LaunchDescription(
        [DeclareLaunchArgument('namespace', default_value='fwmini')]
        + [DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()]
        + [GroupAction([PushRosNamespace(LaunchConfiguration('namespace')),
            IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver)),
            launch_arguments={key: LaunchConfiguration(key) for key in defaults}.items())])])
