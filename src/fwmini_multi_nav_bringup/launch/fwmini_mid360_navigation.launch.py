"""FW-mini MID360 + AGT global relocalization + Nav2 field integration.

External owners: Livox and YHS CAN drivers. This launch owns FAST-LIO2,
map->odom localization, map server, scan projection, Nav2, and optional final
FW-mini command adapter. The adapter is opt-in and motion remains disabled
until /fwmini/enable_motion is explicitly enabled.
"""

import hashlib
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from agt_navigation_runtime.lio_config import freeze_fastlio_config
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _file_sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _tree_sha256(path):
    digest = hashlib.sha256()
    for file_path in sorted(item for item in path.rglob('*') if item.is_file()):
        relative = file_path.relative_to(path).as_posix().encode('utf-8')
        digest.update(len(relative).to_bytes(4, 'big'))
        digest.update(relative)
        digest.update(_file_sha256(file_path).encode('ascii'))
    return digest.hexdigest()


def _validate_explicit_map(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    nav_yaml = Path(value('map_yaml')).expanduser().resolve()
    global_pcd = Path(value('global_map_pcd')).expanduser().resolve()
    relocalization = Path(value('relocalization_assets')).expanduser().resolve()
    metadata_path = next(
        (parent / 'metadata.yaml' for parent in nav_yaml.parents
         if (parent / 'metadata.yaml').is_file()),
        None,
    )
    if metadata_path is None:
        raise RuntimeError(f'FW-mini field launch requires map metadata above: {nav_yaml}')
    map_root = metadata_path.parent
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    expected_id, expected_version = value('map_id'), value('map_version')
    if (metadata.get('map_id'), metadata.get('map_version')) != (expected_id, expected_version):
        raise RuntimeError('Explicit map ID/version does not match map metadata')
    for key, selected_path in (
        ('navigation_map', nav_yaml),
        ('localization_map', global_pcd),
        ('relocalization_assets', relocalization),
    ):
        asset = metadata.get('assets', {}).get(key)
        if not asset:
            raise RuntimeError(f'Map metadata is missing asset entry: {key}')
        asset_path = (map_root / asset['path']).resolve()
        if asset_path != selected_path or not asset_path.exists():
            raise RuntimeError(f'Map asset path mismatch or missing: {key}: {selected_path}')
        actual_hash = (_tree_sha256(asset_path) if asset_path.is_dir()
                       else _file_sha256(asset_path))
        if actual_hash != asset.get('sha256'):
            raise RuntimeError(f'Map asset checksum mismatch: {key}')
    nav = yaml.safe_load(nav_yaml.read_text(encoding='utf-8'))
    image = (nav_yaml.parent / nav['image']).resolve()
    if not image.is_file():
        raise RuntimeError(f'Nav2 map image is missing: {image}')
    return []


def _validate_nav2_parameter_types(context):
    params_path = Path(LaunchConfiguration('nav2_params_file').perform(context)).expanduser()
    config = yaml.safe_load(params_path.read_text(encoding='utf-8'))
    local = config.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters', {})
    if not isinstance(local.get('width'), int):
        raise RuntimeError(f'local_costmap.width must be YAML integer in {params_path}')
    if not isinstance(local.get('height'), int):
        raise RuntimeError(f'local_costmap.height must be YAML integer in {params_path}')
    geometry_file = Path(get_package_share_directory('fwmini_bringup')) / 'config/geometry.field_trial.yaml'
    geometry = yaml.safe_load(geometry_file.read_text(encoding='utf-8'))
    length = float(geometry['body']['length_m'])
    width = float(geometry['body']['width_m'])
    expected = [[length/2, width/2], [length/2, -width/2],
                [-length/2, -width/2], [-length/2, width/2]]
    for section in ('local_costmap', 'global_costmap'):
        params = config.get(section, {}).get(section, {}).get('ros__parameters', {})
        footprint = yaml.safe_load(params.get('footprint', 'null'))
        if footprint is None or len(footprint) != len(expected) or any(
                len(point) != 2 or any(abs(float(a)-float(b)) > 1e-6
                for a,b in zip(point, reference))
                for point,reference in zip(footprint,expected)):
            raise RuntimeError(f'Nav2 {section} footprint differs from {geometry_file}')
    return []


def _include(package, launch_file, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch' / launch_file)),
        launch_arguments=arguments.items(),
    )


def _localization_nodes(context):
    """Compose the currently installed localization components directly.

    The installed agt_system_bringup localization launch still references the
    retired agt_navigation_runtime/lio.launch.py entry point.  Build the same
    component graph here using the current FAST-LIO navigation launch.
    """
    def value(name):
        return LaunchConfiguration(name).perform(context)

    automatic = value('auto_relocalize').strip().lower() in ('1', 'true', 'yes', 'on')
    # Freeze once so FAST-LIO's adapter and the global relocalizer consume the
    # same calibration and topic settings for this run.
    calibration = freeze_fastlio_config(
        value('fastlio_config'), value('lidar_topic'), value('imu_topic'))
    common = {'use_sim_time': value('use_sim_time')}
    return [
        _include('agt_navigation_runtime', 'fastlio_navigation_lio.launch.py', {
            'fastlio_config': calibration,
            'lidar_topic': value('lidar_topic'),
            'imu_topic': value('imu_topic'),
            'body_to_base_calibration_file': calibration,
            **common,
        }),
        _include('agt_livox_tools', 'livox_format_bridge.launch.py', {}),
        _include('agt_global_relocalization', 'global_relocalization.launch.py', {
            'global_map': value('global_map_pcd'),
            'relocalization_assets': value('relocalization_assets'),
            'query_capture_dir': value('query_capture_dir'),
            **({
                'cloud_contract_capture_dir': value('cloud_contract_capture_dir'),
                'gicp_constraint_mode': value('gicp_constraint_mode'),
            } if not automatic else {}),
            'selected_map_id': value('map_id'),
            'selected_map_version': value('map_version'),
            'body_to_base_calibration_file': calibration,
            'batch_lio_config_file': '',
            'follow_map_manager': 'false',
            'scan_topic': '/agt/livox/points',
            'auto_request': value('auto_relocalize'),
            # Automatic mode runs the full-map BBS + GICP backend after five
            # stationary frames. Manual mode uses RViz /initialpose and local GICP.
            'initialization_mode': 'auto' if automatic else 'manual',
            'relocalization_executable': (
                'global_relocalization' if automatic else 'initialization_relocalization'),
            **common,
        }),
        _include('agt_localization_manager', 'localization_manager.launch.py', {
            'map_id': value('map_id'),
            'map_version': value('map_version'),
            **common,
        }),
    ]


def generate_launch_description():
    map_root = Path(
        '/home/yangxuan/ros2_ws/maps/bunker_mid360/'
        '20260924-trav-integration-v1'
    )
    agt_runtime = Path(get_package_share_directory('agt_navigation_runtime'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    fwmini_share = Path(get_package_share_directory('fwmini_multi_nav_bringup'))
    # Single-source FW-mini field geometry. Used by the URDF preview and this
    # production trial launch; legacy static extrinsic file is provenance only.
    geometry_file = Path(get_package_share_directory('fwmini_bringup')) / 'config/geometry.field_trial.yaml'
    geometry = yaml.safe_load(geometry_file.read_text(encoding='utf-8'))
    lidar_xyz = geometry['lidar_mount']['xyz_m']
    lidar_rpy = geometry['lidar_mount']['rpy_rad']

    args = [
        DeclareLaunchArgument(
            'map_yaml', default_value=str(map_root / 'navigation/bunker_v1/map.yaml')),
        DeclareLaunchArgument('global_map_pcd', default_value=str(map_root / 'localization/global_map.pcd')),
        DeclareLaunchArgument('relocalization_assets', default_value=str(map_root / 'localization/relocalization')),
        DeclareLaunchArgument('map_id', default_value='bunker_mid360'),
        DeclareLaunchArgument('map_version', default_value='20260924-trav-integration-v1'),
        DeclareLaunchArgument('query_capture_dir', default_value='/home/yangxuan/.ros/fwmini_relocalization'),
        DeclareLaunchArgument(
            'cloud_contract_capture_dir',
            default_value='/home/yangxuan/.ros/fwmini_relocalization/cloud_contracts',
            description='Save the exact multi-frame query and transform contract for manual GICP.'),
        DeclareLaunchArgument(
            'gicp_constraint_mode', default_value='full_se3',
            description='Manual seed registration: full_se3, gravity_constrained, or gravity_prior.'),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value=str(agt_runtime / 'config/fastlio2_mid360_navigation.yaml')),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=str(fwmini_share / 'config/nav2/fwmini_mid360_nav2.yaml')),
        DeclareLaunchArgument('auto_relocalize', default_value='false'),
        DeclareLaunchArgument('lidar_topic', default_value='/livox/lidar'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu'),
        # MID360 uses the synchronizer's clock domain. Keep FAST-LIO,
        # localization, TF, costmaps, Nav2, and RViz on that same clock.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_chassis_adapter', default_value='false'),
        DeclareLaunchArgument('initial_motion_enabled', default_value='false'),
        DeclareLaunchArgument('allow_local_search', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('base_footprint_to_base_z', default_value=str(geometry['base_footprint_to_base_z'])),
        DeclareLaunchArgument('lidar_x', default_value=str(lidar_xyz[0])),
        DeclareLaunchArgument('lidar_y', default_value=str(lidar_xyz[1])),
        DeclareLaunchArgument('lidar_z', default_value=str(lidar_xyz[2])),
        DeclareLaunchArgument('lidar_roll', default_value=str(lidar_rpy[0])),
        DeclareLaunchArgument('lidar_pitch', default_value=str(lidar_rpy[1])),
        DeclareLaunchArgument('lidar_yaw', default_value=str(lidar_rpy[2])),
    ]

    return LaunchDescription(args + [
        OpaqueFunction(function=_validate_explicit_map),
        OpaqueFunction(function=_validate_nav2_parameter_types),
        Node(
            package='fwmini_multi_nav_bringup', executable='fwmini_livox_clock',
            name='fwmini_livox_clock', output='screen',
            parameters=[{'imu_topic': LaunchConfiguration('imu_topic'),
                         'clock_topic': '/clock', 'use_sim_time': False}],
        ),
        # CAD base_link is 70 mm above the ground plane at the wheel radius.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='fwmini_base_footprint_to_base_link', output='screen',
            arguments=[
                '0', '0', LaunchConfiguration('base_footprint_to_base_z'),
                '0', '0', '0', 'base_footprint', 'base_link',
            ],
        ),
        # Candidate from the in-place spin fit + CAD height; override launch args
        # after measuring turn-center-to-base_link offset.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='fwmini_base_link_to_livox', output='screen',
            arguments=[
                LaunchConfiguration('lidar_x'), LaunchConfiguration('lidar_y'),
                LaunchConfiguration('lidar_z'), LaunchConfiguration('lidar_yaw'),
                LaunchConfiguration('lidar_pitch'), LaunchConfiguration('lidar_roll'),
                'base_link', 'livox_frame',
            ],
        ),
        # Current installed launch graph: FAST-LIO2 + adapter, Livox bridge,
        # relocalizer and the sole map->odom correction owner.
        OpaqueFunction(function=_localization_nodes),
        # Nav2 consumes the matching 2D occupancy map. Localization above
        # publishes map->odom; AMCL is intentionally not launched.
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{
                'yaml_filename': LaunchConfiguration('map_yaml'),
                'topic_name': 'map', 'frame_id': 'map',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True, 'node_names': ['map_server'],
            }],
        ),
        # MID360 cloud -> planar obstacle scan for Nav2 costmaps.
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            name='fwmini_mid360_to_scan', output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'target_frame': 'base_footprint', 'transform_tolerance': 0.20,
                'min_height': 0.05, 'max_height': 1.60,
                'angle_min': -3.14159265, 'angle_max': 3.14159265,
                'angle_increment': 0.0087, 'scan_time': 0.1,
                'range_min': 0.25, 'range_max': 8.0,
                'use_inf': True, 'queue_size': 10,
            }],
            remappings=[('cloud_in', '/agt/livox/points'), ('scan', '/scan')],
        ),
        _include('nav2_bringup', 'navigation_launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False',
        }),
        Node(
            package='fwmini_multi_nav_bringup', executable='fwmini_navigation_guard',
            name='fwmini_navigation_guard', output='screen',
            parameters=[{
                'map_id': LaunchConfiguration('map_id'),
                'map_version': ParameterValue(LaunchConfiguration('map_version'), value_type=str),
                'max_linear_x': 0.15, 'max_linear_y': 0.15,
                'max_angular_z': 0.30,
                'max_linear_accel': 0.20, 'max_angular_accel': 0.30,
                'command_timeout': 0.20, 'status_timeout': 1.0,
                'publish_rate': 50.0,
                'allow_local_search': ParameterValue(LaunchConfiguration('allow_local_search'), value_type=bool),
                'manual_search_topic': '/fwmini/search_cmd_vel',
                'search_max_linear_x': 0.10,
                'search_max_linear_y': 0.0,
                'search_max_angular_z': 0.20,
            }],
        ),
        # Final actuator boundary. It clamps x/y/yaw, has a 0.2 s watchdog and
        # starts motion-disabled. It is optional to keep software preview no-CAN.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory('fwmini_chassis_adapter')) /
                'launch/agt_guarded_adapter.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_chassis_adapter')),
            launch_arguments={
                'params_file': str(
                    Path(get_package_share_directory('fwmini_chassis_adapter')) /
                    'config/agt_guarded_cmd_adapter.yaml'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_motion_enabled': LaunchConfiguration('initial_motion_enabled'),
            }.items(),
        ),
        Node(
            package='rviz2', executable='rviz2', name='fwmini_nav_rviz',
            output='screen', arguments=['-d', str(nav2_share / 'rviz/nav2_default_view.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ])
