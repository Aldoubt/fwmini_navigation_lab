"""FW-mini S2 route workbench addon for an already running MID360 Nav2 stack.

Does not launch drivers, localization, TF, Nav2 or a velocity publisher.
Execution stays fail-closed until NavigationHealth is READY, a path passes
Nav2 /is_path_valid, and the operator explicitly confirms Execute.
"""

from pathlib import Path

import hashlib
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_map(context):
    map_package = Path(LaunchConfiguration("map_package").perform(context)).expanduser().resolve()
    meta = map_package / "metadata.yaml"
    if not meta.is_file():
        raise RuntimeError(f"Missing immutable map metadata: {meta}")
    record = yaml.safe_load(meta.read_text(encoding="utf-8"))
    expected = (LaunchConfiguration("map_id").perform(context),
                LaunchConfiguration("map_version").perform(context))
    if (record.get("map_id"), record.get("map_version")) != expected:
        raise RuntimeError(f"Route workbench map identity differs: {record!r} vs {expected}")
    assets = record.get("assets", {})
    for name in ("navigation_map", "localization_map", "relocalization_assets"):
        expected_asset = assets.get(name)
        if not isinstance(expected_asset, dict):
            raise RuntimeError(f"Missing immutable map asset contract: {name}")
        asset = (map_package / str(expected_asset.get("path", ""))).resolve()
        if not asset.is_relative_to(map_package):
            raise RuntimeError(f"Map asset escapes package: {name}")
        if not asset.exists():
            raise RuntimeError(f"Missing map asset: {name}: {asset}")
        digest = hashlib.sha256()
        if asset.is_dir():
            for file_path in sorted(p for p in asset.rglob("*") if p.is_file()):
                relative = file_path.relative_to(asset).as_posix().encode("utf-8")
                digest.update(len(relative).to_bytes(4, "big"))
                digest.update(relative)
                digest.update(hashlib.sha256(file_path.read_bytes()).hexdigest().encode("ascii"))
        else:
            with asset.open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b""):
                    digest.update(chunk)
        if digest.hexdigest() != expected_asset.get("sha256"):
            raise RuntimeError(f"Map asset hash mismatch: {name}")
    return []


def generate_launch_description():
    rviz = Path(get_package_share_directory("agt_rviz_patrol")) / "config/agt_rviz_costmap_diagnostics.rviz"
    return LaunchDescription([
        DeclareLaunchArgument("map_package"),
        DeclareLaunchArgument("map_id", default_value="fwmini_mid360"),
        DeclareLaunchArgument("map_version"),
        DeclareLaunchArgument("robot_profile", default_value="fwmini_20261021_field_trial"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        OpaqueFunction(function=_validate_map),
        # Sole read-only navigation-health authority for this addon.
        Node(package="agt_navigation_supervisor", executable="navigation_supervisor",
             name="agt_navigation_supervisor", output="screen", parameters=[{
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                 "robot_profile": LaunchConfiguration("robot_profile"),
                 "map_id": LaunchConfiguration("map_id"),
                 "map_version": ParameterValue(LaunchConfiguration("map_version"), value_type=str),
                 "map_valid": True,
                 "nav2_health_source": "manager",
                 "platform_lidar_frame": "livox_frame",
                 "lidar_topic": "/agt/livox/points",
                 "lidar_message_type": "pointcloud2",
                 "imu_topic": "/livox/imu",
                 "local_odom_topic": "/agt/odometry/local",
                 "localization_status_topic": "/agt/localization/status",
                 "wheel_odom_topic": "/odom",
             }]),
        # Adapter of the existing V3 FollowRoute action to native Nav2 FollowPath.
        Node(package="agt_navigation_capability", executable="navigation_capability",
             name="agt_navigation_capability", output="screen", parameters=[{
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                 "robot_profile": LaunchConfiguration("robot_profile"),
                 "map_id": LaunchConfiguration("map_id"),
                 "map_version": ParameterValue(LaunchConfiguration("map_version"), value_type=str),
                 "require_payload_drive_permission": False,
             }]),
        # rviz_path_tool already imports and runs the revisioned RouteWorkbench.
        Node(package="agt_rviz_patrol", executable="rviz_path_tool",
             name="agt_rviz_path_tool", output="screen", parameters=[{
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                 "map_id": LaunchConfiguration("map_id"),
                 "map_version": ParameterValue(LaunchConfiguration("map_version"), value_type=str),
                 "preview_only": False,
                 # FW-mini precision test: never silently splice a diagonal
                 # path to the operator-drawn line.
                 "start_connection_policy": "drawn_only",
                 "max_start_connection": 0.35,
                 "max_start_lateral": 0.12,
                 "max_start_heading_deg": 25.0,
                 "max_start_behind": 0.05,
                 "follow_route_action": "/navigation/follow_route",
                 "global_frame": "map", "base_frame": "base_link",
                 "lio_odom_topic": "/agt/odometry/local",
                 "wheel_odom_topic": "/odom",
             }]),
        Node(package="rviz2", executable="rviz2", name="fwmini_route_workbench_rviz",
             arguments=["-d", str(rviz)], output="screen",
             parameters=[{"use_sim_time": ParameterValue(
                 LaunchConfiguration("use_sim_time"), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration("start_rviz"))),
    ])
