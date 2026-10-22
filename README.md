# FW-mini Navigation Lab

ROS 2 Humble / Ubuntu 22.04 research platform for **FW-mini four-steer/four-drive chassis + Livox MID360 + Femto Bolt RGB-D + AprilTag**.

## Status (2026-10-21)

**Source recovered from Windows and migrated to GitHub.** The source-only migration commit is `60411aa` (Git tree `f916099cea0033ef5a9087cca869af21db5a69a8`, identical to the previous local Windows `318e2db` tree). Original local three-commit history remains on Windows.

The source now includes an integrated MID360 navigation launch candidate. It composes AGT FAST-LIO2/global relocalization, the selected occupancy map, Nav2 with holonomic DWB limits, point-cloud obstacle projection, RViz, a localization-aware FW-mini velocity gate, and the FW-mini command adapter. This is a software integration candidate; ROS runtime and physical navigation acceptance are still pending.

Existing eight ROS 2 packages:

| Package | Function |
|---|---|
| `fwmini_description` | Preliminary box envelope, URDF/Xacro and static frames |
| `fwmini_interfaces` | Project messages and services |
| `fwmini_bringup` | Description and Tag-follow launch integration |
| `fwmini_chassis_adapter` | Guarded Twist → YHS CtrlCmd adapter |
| `fwmini_odom_adapter` | YHS feedback → planar Odometry + TF |
| `fwmini_perception` | Femto Bolt official driver launch wrapper |
| `apriltag_detector_adapter` | Tag observation adapter |
| `tag_follow_controller` | Target-following control core |

ROS 2 Humble source now builds for the integrated navigation packages. Live localization, CAN motion, accuracy, and RGB-D runtime remain unverified.

## Field deployment — start here

**[Ubuntu Humble deployment & first-drive checklist](docs/field_deployment_humble.md)**

**[Femto Bolt handheld RGB-D mapping without a chassis](docs/femto_bolt_rtabmap_handheld.md)** — camera-only RTAB-Map visual odometry and 3D map database experiment.

The first-day acceptance sequence is: compile → verify real YHS CAN feedback → confirm FAST-LIO localization and TF → supervised low-speed navigation trial → measure path and relocalization accuracy.

The original manufacturer ROS 2 driver and message source files are now included under [`vendor/FW-mini-ros2/`](vendor/FW-mini-ros2/), so cloning the repository provides `yhs_can_control` and `yhs_can_interfaces`. No binary vendor archives, ROS 1 packages, CAD or office documents were added in this update. The vendor materials were supplied separately; no license granting public redistribution was found in the imported ROS2 directory. **The repository is currently public; change it to Private immediately if redistribution is not permitted.** The original vendor files should remain unchanged; use a separate maintained copy for fixes.

## Core checks on Windows (no ROS)

    python tools/check_scaffold.py
    python tools/test_adapter_cores.py --cxx D:/msys/ucrt64/bin/g++.exe
    python tools/test_tag_follow_cores.py --cxx D:/msys/ucrt64/bin/g++.exe

Core PASS is not evidence of ROS runtime, DDS/TF or real hardware acceptance.

## MID360 navigation trial

The current field entry point is `ros2 launch fwmini_multi_nav_bringup fwmini_mid360_navigation.launch.py` after building this worktree and sourcing its overlay. Start the MID360 driver and YHS CAN feedback owner first. The launch starts FAST-LIO2/localization, map server, point-cloud scan projection, Nav2, RViz, and a FW-mini velocity gate. With `start_chassis_adapter:=true` it also starts the final command adapter. The velocity gate passes holonomic commands only when fresh local odometry and a valid global correction match the selected map ID/version. The chain is `/cmd_vel` → localization-aware gate → `/agt/base/cmd_vel` → `/ctrl_cmd`. The final adapter starts motion-disabled; enable it only after localization and TF are ready by calling `/fwmini/enable_motion`.

Default map arguments point directly to `bunker_mid360/20260924-trav-integration-v1`, using `navigation/bunker_v1/map.yaml`; this version is not in the validated map registry. The launch passes its explicit ID/version and files to localization and Nav2 without promoting it in the registry. Review the map identity and file hashes before using it. The LiDAR mount transform is the spin-fit/CAD candidate in `config/field_trial` (x/y may be shifted if the physical turn center differs from `base_link`). Nav2 motion limits are capped at 0.15 m/s x/y and 0.30 rad/s for this supervised trial. The synchronized-trigger profile preserves source timestamps, disables the adapter's absolute-age filter, and uses the MID360 IMU timestamp as `/clock` across navigation; the navigation guard and chassis watchdog remain on steady wall time.

The previous multi-backend SLAM launch remains available for mapping experiments; it is not the MID360 field navigation entry point.

## Architecture

The RGB-D experimental path remains **YHS CAN → chassis adapters → RTAB-Map → Nav2**, with AprilTag and depth-based obstacle experiments. The new MID360 field candidate uses **Livox → AGT FAST-LIO2/global relocalization → Nav2 → FW-mini localization-aware velocity gate → YHS CAN adapter**. The two bringup paths have separate launch entries; do not run both odometry/localization owners at once.

More details: [vendor review](docs/vendor_middleware_review.md), [adapter contract](docs/contracts.md), [adapter validation](docs/adapter_validation.md), [Tag pipeline](docs/tag_follow_pipeline.md).
