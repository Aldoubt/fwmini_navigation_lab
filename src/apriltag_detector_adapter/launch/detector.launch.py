"""Rectify RGB, launch community detector, adapt detection + timed TF."""
import math
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start(context):
    size = float(LaunchConfiguration('tag_size_m').perform(context))
    target = int(LaunchConfiguration('target_id').perform(context))
    if not math.isfinite(size) or size <= 0 or target < 0:
        raise ValueError('Set measured positive tag_size_m and nonnegative target_id')
    raw = LaunchConfiguration('image_raw_topic').perform(context)
    info = LaunchConfiguration('camera_info_topic').perform(context)
    rect = LaunchConfiguration('image_rect_topic').perform(context)
    config = str(Path(get_package_share_directory('apriltag_detector_adapter')) /
                 'config/detector_adapter.yaml')
    sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [
        Node(package='image_proc', executable='rectify_node', name='fwmini_color_rectify',
             parameters=[{'use_sim_time': sim}],
             remappings=[('image', raw), ('camera_info', info), ('image_rect', rect)]),
        Node(package='apriltag_ros', executable='apriltag_node', name='fwmini_apriltag',
             parameters=[{'family': '36h11', 'size': size, 'max_hamming': 0,
                          'pose_estimation_method': 'pnp', 'image_transport': 'raw',
                          'qos_profile': 'sensor_data', 'detector.decimate': 1.0,
                          'tag.ids': [target], 'tag.frames': ['fwmini_observed_tag'],
                          'tag.sizes': [size], 'use_sim_time': sim}],
             remappings=[('image_rect', rect), ('camera_info', info),
                         ('detections', '/fwmini/tags/raw_detections')]),
        Node(package='apriltag_detector_adapter', executable='apriltag_detector_adapter_node',
             name='apriltag_detector_adapter',
             parameters=[config, {'target_id': target, 'use_sim_time': sim}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tag_size_m', default_value='0.0'),
        DeclareLaunchArgument('target_id', default_value='0'),
        DeclareLaunchArgument('image_raw_topic', default_value='/fwmini/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/fwmini/camera/color/camera_info'),
        DeclareLaunchArgument('image_rect_topic', default_value='/fwmini/camera/color/image_rect'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start)])
