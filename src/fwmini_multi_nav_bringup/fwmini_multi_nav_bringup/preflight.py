1: """Environment and optional live TF checks; no motion commands."""
2: 
3: import argparse
4: import os
5: import shutil
6: import subprocess
7: import sys
8: from pathlib import Path
9: 
10: ROS_PACKAGES = (
11:     "nav2_bringup", "slam_toolbox", "rtabmap_slam", "cartographer_ros",
12:     "depthimage_to_laserscan", "pointcloud_to_laserscan",
13: )
14: 
15: 
16: def probe_environment():
17:     findings = []
18:     os_release = Path("/etc/os-release").read_text(encoding="utf8")
19:     findings.append(("os_ubuntu_22", 'VERSION_ID="22.04"' in os_release))
20:     findings.append(("ros_humble", os.getenv("ROS_DISTRO") == "humble"))
21:     findings.append(("ros2_cli", shutil.which("ros2") is not None))
22:     if shutil.which("ros2"):
23:         for package in ROS_PACKAGES:
24:             result = subprocess.run(
25:                 ["ros2", "pkg", "prefix", package],
26:                 capture_output=True, text=True, timeout=10, check=False,
27:             )
28:             findings.append(("pkg:" + package, result.returncode == 0))
29:     return findings
30: 
31: 
32: def probe_live(sensor_frame):
33:     import rclpy
34:     from rclpy.node import Node
35:     from rclpy.time import Time
36:     from tf2_ros import Buffer, TransformListener
37: 
38:     rclpy.init()
39:     node = Node("fwmini_preflight")
40:     buffer = Buffer()
41:     listener = TransformListener(buffer, node)
42:     try:
43:         # Allows /tf_static and /tf to arrive; does not drive the base.
44:         for _ in range(30):
45:             rclpy.spin_once(node, timeout_sec=0.1)
46:         findings = [
47:             ("exactly_one_topic:/wheel/odom_publisher",
48:              len(node.get_publishers_info_by_topic("/wheel/odom")) == 1),
49:             ("tf:odom->base_link", buffer.can_transform("odom", "base_link", Time())),
50:         ]
51:         if sensor_frame:
52:             findings.append((
53:                 "tf:base_link->" + sensor_frame,
54:                 buffer.can_transform("base_link", sensor_frame, Time())
55:             ))
56:         return findings
57:     finally:
58:         # Keep listener referenced while spin is running.
59:         del listener
60:         node.destroy_node()
61:         rclpy.shutdown()
62: 
63: 
64: def main(argv=None):
65:     parser = argparse.ArgumentParser(description=__doc__)
66:     parser.add_argument("--strict", action="store_true", help="Exit nonzero if a check fails")
67:     parser.add_argument("--live", action="store_true", help="Also check live ROS graph and TF")
68:     parser.add_argument("--sensor-frame", default="", help="Optional calibrated frame, e.g. camera_link")
69:     args = parser.parse_args(argv)
70:     results = probe_environment()
71:     if args.live:
72:         try:
73:             results.extend(probe_live(args.sensor_frame))
74:         except Exception as exc:
75:             results.append(("live_check:" + str(exc), False))
76:     for name, ok in results:
77:         print(("[PASS] " if ok else "[FAIL] ") + name)
78:     failures = [name for name, ok in results if not ok]
79:     if failures:
80:         print("UNVERIFIED: " + ", ".join(failures), file=sys.stderr)
81:     return 1 if args.strict and failures else 0
82: 
83: 
84: if __name__ == "__main__":
85:     sys.exit(main())