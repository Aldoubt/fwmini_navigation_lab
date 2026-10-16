"""Backend-selectable SLAM + optional gated Nav2 for ROS 2 Humble FW-mini.

Only map->odom is published by the selected backend; wheel odom->base_link,
camera/MID360 drivers and all sensor TFs remain owned by the existing robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fwmini_multi_nav_bringup.contracts import (
    ContractError,
    validate_hardware_contract,
    validate_selection,
)


def _cfg(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    value = _cfg(context, name).lower()
    if value not in ("true", "false"):
        raise ContractError(name + " must be true or false")
    return value == "true"


def _spawn(context):
    backend = _cfg(context, "backend")
    mode = _cfg(context, "mode")
    database = os.path.expanduser(_cfg(context, "database_path"))
    posegraph = os.path.expanduser(_cfg(context, "posegraph_file"))
    source = validate_selection(
        backend, mode, _cfg(context, "scan_source"), database, posegraph
    )
    use_sim = _bool(context, "use_sim_time")
    actions = []
    share = get_package_share_directory("fwmini_multi_nav_bringup")

    # No SLAM backend in this launch creates odom->base_link. The original
    # fwmini_odom_adapter owns it. External bringup is started independently.
    if source == "depth":
        actions.append(Node(
            package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
            name="fwmini_depth_to_scan", output="screen",
            parameters=[{
                "use_sim_time": use_sim, "scan_height": 12,
                "scan_time": 0.1, "range_min": 0.25, "range_max": 8.0,
            }],
            remappings=[
                ("depth", _cfg(context, "depth_topic")),
                ("depth_camera_info", _cfg(context, "depth_camera_info_topic")),
                ("scan", "/scan"),
            ],
        ))
    elif source == "cloud":
        actions.append(Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="fwmini_cloud_to_scan", output="screen",
            parameters=[{
                "use_sim_time": use_sim,
                "target_frame": "base_link", "transform_tolerance": 0.15,
                "min_height": -0.10, "max_height": 0.40,
                "angle_min": -3.14159265, "angle_max": 3.14159265,
                "angle_increment": 0.0087, "scan_time": 0.1,
                "range_min": 0.25, "range_max": 8.0, "use_inf": True,
                "queue_size": 10,
            }],
            remappings=[("cloud_in", _cfg(context, "cloud_topic")), ("scan", "/scan")],
        ))

    if backend == "rtabmap":
        # Deliberately do NOT start rtabmap_odom/rgbd_odometry.
        # The robot's own /odom supplies continuous tracking and owns odom TF.
        params = {
            "use_sim_time": use_sim,
            "frame_id": "base_link",
            "map_frame_id": "map",
            "odom_frame_id": "",
            "publish_tf": True,
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_rgbd": False,
            "subscribe_scan": False,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "qos_image": 2,
            "qos_camera_info": 2,
            "qos_odom": 2,
            "database_path": database,
            "Mem/IncrementalMemory": "true" if mode == "mapping" else "false",
            "Mem/InitWMWithAllNodes": "false" if mode == "mapping" else "true",
            "Grid/Sensor": "1",
            "Grid/3D": "false",
            "Grid/RangeMax": "8.0",
            "Grid/CellSize": "0.05",
        }
        actions.append(Node(
            package="rtabmap_slam", executable="rtabmap",
            name="rtabmap", namespace="rtabmap", output="screen",
            parameters=[params],
            remappings=[
                ("rgb/image", _cfg(context, "rgb_topic")),
                ("depth/image", _cfg(context, "depth_topic")),
                ("rgb/camera_info", _cfg(context, "camera_info_topic")),
                ("odom", _cfg(context, "odom_topic")),
                # RTAB-Map Humble actually publishes the relative topic 'map'
                # under /rtabmap. 'grid_map' is NOT its OccupancyGrid name.
                ("map", "/map"),
            ],
        ))
    elif backend == "slam_toolbox":
        # Use the same ordinary Node launch model as the official Humble
        # online_async/localization launch files. Custom lifecycle events
        # caused shutdown callback exceptions on SIGINT in Humble.
        config = os.path.join(
            share, "config", "slam_toolbox", mode + ".yaml"
        )
        overrides = {"use_sim_time": use_sim}
        if mode == "localization":
            overrides["map_file_name"] = posegraph
        actions.append(Node(
            package="slam_toolbox",
            executable=(
                "async_slam_toolbox_node" if mode == "mapping"
                else "localization_slam_toolbox_node"
            ),
            name="slam_toolbox", namespace="", output="screen",
            parameters=[config, overrides],
            remappings=[("scan", "/scan")],
        ))
    elif backend == "cartographer_2d":
        lua_dir = os.path.join(share, "config", "cartographer")
        actions.extend([
            Node(
                package="cartographer_ros", executable="cartographer_node",
                name="cartographer_node", output="screen",
                parameters=[{"use_sim_time": use_sim}],
                arguments=[
                    "-configuration_directory", lua_dir,
                    "-configuration_basename", "fwmini_2d.lua",
                ],
                remappings=[("scan", "/scan"), ("odom", _cfg(context, "odom_topic"))],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="cartographer_occupancy_grid_node", output="screen",
                parameters=[{"use_sim_time": use_sim}],
                arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
                remappings=[("map", "/map")],
            ),
        ])

    if _bool(context, "navigation"):
        contract = _cfg(context, "hardware_contract")
        validate_hardware_contract(contract, backend, source)
        nav2_file = os.path.expanduser(_cfg(context, "nav2_params_file"))
        if not nav2_file or not os.path.isfile(nav2_file):
            raise ContractError("Nav2 requires a full verified nav2_params_file")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch", "navigation_launch.py",
            )),
            launch_arguments={
                "use_sim_time": "true" if use_sim else "false",
                "params_file": nav2_file,
                "use_composition": "False",
            }.items(),
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("fwmini_multi_nav_bringup")
    defaults = [
        ("backend", "rtabmap", "rtabmap | slam_toolbox | cartographer_2d"),
        ("mode", "mapping", "mapping or native-map localization"),
        ("scan_source", "auto", "auto | none | depth | cloud"),
        ("use_sim_time", "false", "true for rosbag2 --clock replay"),
        ("navigation", "false", "Nav2 is gated until physical calibration"),
        ("database_path", "~/.ros/fwmini_rtabmap.db", "RTAB-Map native .db"),
        ("posegraph_file", "", "SLAM Toolbox serialized pose graph"),
        ("odom_topic", "/wheel/odom", "Existing fwmini_odom_adapter output"),
        ("rgb_topic", "/fwmini/camera/color/image_raw", "Rectified RGB Image"),
        ("depth_topic", "/fwmini/camera/depth/image_raw", "Registered depth Image"),
        ("camera_info_topic", "/fwmini/camera/color/camera_info", "RGB CameraInfo"),
        ("depth_camera_info_topic", "/fwmini/camera/depth/camera_info", "Depth CameraInfo"),
        ("cloud_topic", "/livox/lidar", "MID360 PointCloud2 topic"),
        ("hardware_contract", os.path.join(share, "config", "hardware_contract.json"),
         "Confirmed real extrinsics/chassis status JSON"),
        ("nav2_params_file", "", "Complete verified Nav2 omni parameters"),
    ]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, desc in defaults],
        OpaqueFunction(function=_spawn),
    ])
