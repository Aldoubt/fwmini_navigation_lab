# FW-mini multi-SLAM P0 — Humble machine acceptance evidence

Date: 2026-10-16. Target machine: Ubuntu 22.04 / ROS 2 Humble reachable
over SSH; staging worktree path:
`/home/yangxuan/fwmini_ws/worktrees/multinav-humble-p0`.
Original FW-mini checkout is preserved on `~/fwmini_ws/project` main.

## What PASS means in this report

All P0 tests were **no-motion software tests**. The three algorithms used a
completely synthetic RGB-D source, a static simulated TF, and stationary wheel
odometry in ROS_DOMAIN_ID=91, ROS_LOCALHOST_ONLY=1. The synthetic generator
**never publishes /cmd_vel or any vendor CAN control topic**.

| Test | Result | Evidence |
| --- | --- | --- |
| ROS 2 Humble / Ubuntu 22.04 | PASS | ROS_DISTRO=humble, ROS 2 executable available |
| ROS apt dependencies | PASS | Installed RTAB-Map, SLAM Toolbox, Cartographer, Nav2 and projection executables |
| Colcon package | PASS | 1 package built, ROS index and --show-args passed |
| Pure Python offline tests | PASS | 10/10 passing |
| RTAB-Map RGB-D + wheel odom | PASS (synthetic) | Processed 9+ frames, wrote native DB with Node/Data/Link records |
| RTAB-Map OccupancyGrid | PASS (synthetic) | One publisher on /map, width=62 cells |
| RTAB-Map map->odom | PASS (synthetic) | tf2_echo returned zero translation, identity rotation |
| SLAM Toolbox Depth->scan->map | PASS (synthetic) | /scan received in camera_depth_frame, /map had one publisher, width=42 |
| Cartographer 2D Depth->scan->map | PASS (synthetic) | /scan frame resolved, /map one publisher, width=54 |
| Nav2 before calibration | PASS (refusal) | Navigation BLOCKED until real contract is verified |
| FAST-LIO2 unsupported adapter | PASS (refusal) | Fail-closed before launch |
| Real Femto Bolt/odometry/CAN | NOT TESTED | Production device drivers not running during these tests |
| Physical TF, footprints, Nav2 motion | BLOCKED | Awaiting measured extrinsics and supervised field acceptance |

**A synthetic OccupancyGrid is evidence of software interconnection, not an
independent SLAM accuracy benchmark.** All test poses, intrinsics and depths
are invented; the 42/54/62 cell widths must not be compared for accuracy.

## Bugs observed and repaired on the real ROS 2 Humble machine

1. The original project publishes `/wheel/odom`, not `/odom`. RTAB-Map and
   Cartographer now subscribe to the real adapter topic.
2. The Orbbec wrapper uses default `/fwmini/camera/` names.
3. Humble's LifecycleNode requires an explicit namespace; we now launch
   SLAM Toolbox via a regular Node, as in the installed official launch.
4. RTAB-Map `Grid/FromDepth` is deprecated; changed to `Grid/Sensor=1`.
5. RTAB-Map occupies `/rtabmap/map` before remapping; correct remap is
   `map -> /map` (not `grid_map -> /map`).
6. The installed depthimage_to_laserscan publishes a LaserScan with header
   frame `camera_depth_frame`. It MUST have valid measured robot TF.
7. Fast DDS rejected ROS_DOMAIN_ID=233 due to UDP port range; isolated tests
   subsequently used ROS_DOMAIN_ID=91 successfully.

The built-in `timeout --signal=INT` test harness can report process exit -2
when the entire process group receives SIGINT. It is not evidence of
spontaneous SLAM crashes; check logs before the intentional timeout.

## Reproduction without hardware, on the Humble machine

In both terminals:

```bash
source /opt/ros/humble/setup.bash
source ~/fwmini_ws/worktrees/multinav-humble-p0/install/multinav/setup.bash
export ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
```

Terminal A (synthetic test only, NEVER for real navigation):

```bash
python3 ~/fwmini_ws/worktrees/multinav-humble-p0/tools/multi_nav_synthetic_smoke.py
```

Terminal B — run exactly one of:

```bash
ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=rtabmap
ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=slam_toolbox
ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=cartographer_2d
```

Verify `ros2 topic info /map`, `ros2 topic echo /map --once --field
info.width`, `ros2 topic info /scan` (2D), and `ros2 run tf2_ros tf2_echo
map odom`. Stop current algorithm before selecting the next.

## Real-hardware handover

Do not use this synthetic publisher in the robot's active ROS domain.
For the next stage, the user will supply calibrated
`base_link->femto_bolt_link`, valid depth scanner TF, MID360 extrinsics
(when present), body footprint and verified omni-motion limits.
Until then, `navigation:=true` remains blocked by the JSON contract.

The GitHub Actions run initially failed with **zero jobs**, so GitHub CI does
not count as a valid Humble build result. Direct SSH onto Ubuntu 22.04 is the
source of all runtime evidence above.
