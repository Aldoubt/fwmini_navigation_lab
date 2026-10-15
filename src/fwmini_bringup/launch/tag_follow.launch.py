"""Camera -> real detector -> adapter -> follower -> chassis command adapter.
No external mount TF is fabricated and no CAN driver is started automatically.
"""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def include(package, filename, arguments=None, condition=None):
    path = Path(get_package_share_directory(package)) / 'launch' / filename
    kwargs = {'launch_arguments': (arguments or {}).items()}
    if condition is not None:
        kwargs['condition'] = condition
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), **kwargs)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('tag_size_m', default_value='0.0'),
        DeclareLaunchArgument('target_id', default_value='0'),
        DeclareLaunchArgument('extrinsics_verified', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        include('fwmini_perception', 'femto_bolt.launch.py',
                condition=IfCondition(LaunchConfiguration('start_camera'))),
        include('apriltag_detector_adapter', 'detector.launch.py',
                {'tag_size_m': LaunchConfiguration('tag_size_m'),
                 'target_id': LaunchConfiguration('target_id'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}),
        include('tag_follow_controller', 'controller.launch.py',
                {'extrinsics_verified': LaunchConfiguration('extrinsics_verified'),
                 'target_id': LaunchConfiguration('target_id'),
                 'use_sim_time': LaunchConfiguration('use_sim_time')}),
        include('fwmini_chassis_adapter', 'adapter.launch.py',
                {'use_sim_time': LaunchConfiguration('use_sim_time')})])
