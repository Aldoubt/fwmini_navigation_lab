"""Environment and optional live TF checks; no motion commands."""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROS_PACKAGES = (
    "nav2_bringup", "slam_toolbox", "rtabmap_slam", "cartographer_ros",
    "depthimage_to_laserscan", "pointcloud_to_laserscan",
)


def probe_environment():
    findings = []
    os_release = Path("/etc/os-release").read_text(encoding="utf8")
    findings.append(("os_ubuntu_22", 'VERSION_ID="22.04"' in os_release))
    findings.append(("ros_humble", os.getenv("ROS_DISTRO") == "humble"))
    findings.append(("ros2_cli", shutil.which("ros2") is not None))
    if shutil.which("ros2"):
        for package in ROS_PACKAGES:
            result = subprocess.run(
                ["ros2", "pkg", "prefix", package],
                capture_output=True, text=True, timeout=10, check=False,
            )
            findings.append(("pkg:" + package, result.returncode == 0))
    return findings


def probe_live(sensor_frame):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("fwmini_preflight")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        # Allows /tf_static and /tf to arrive; does not drive the base.
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        findings = [
            ("exactly_one_topic:/wheel/odom_publisher",
             len(node.get_publishers_info_by_topic("/wheel/odom")) == 1),
            ("tf:odom->base_link", buffer.can_transform("odom", "base_link", Time())),
        ]
        if sensor_frame:
            findings.append((
                "tf:base_link->" + sensor_frame,
                buffer.can_transform("base_link", sensor_frame, Time())
            ))
        return findings
    finally:
        # Keep listener referenced while spin is running.
        del listener
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--strict", action="store_true", help="Exit nonzero if a check fails")
    parser.add_argument("--live", action="store_true", help="Also check live ROS graph and TF")
    parser.add_argument("--sensor-frame", default="", help="Optional calibrated frame, e.g. camera_link")
    args = parser.parse_args(argv)
    results = probe_environment()
    if args.live:
        try:
            results.extend(probe_live(args.sensor_frame))
        except Exception as exc:
            results.append(("live_check:" + str(exc), False))
    for name, ok in results:
        print(("[PASS] " if ok else "[FAIL] ") + name)
    failures = [name for name, ok in results if not ok]
    if failures:
        print("UNVERIFIED: " + ", ".join(failures), file=sys.stderr)
    return 1 if args.strict and failures else 0


if __name__ == "__main__":
    sys.exit(main())
