# FW-mini Navigation Lab

ROS 2 Humble / Ubuntu 22.04 research platform for **FW-mini steerable chassis + Femto Bolt RGB-D + AprilTag**, with RTAB-Map + Nav2 as the next integration milestone.

## Status (2026-10-15)

**Source recovered from Windows and migrated to GitHub.** The source-only migration commit is `60411aa` (Git tree `f916099cea0033ef5a9087cca869af21db5a69a8`, identical to the previous local Windows `318e2db` tree). Original local three-commit history remains on Windows.

Eight ROS 2 packages are present:

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

Windows static checks and C++ core tests passed, but **ROS 2 Humble node builds, CAN, hardware motion and RGB-D runtime are not yet verified**. RTAB-Map/ Nav2 launch/config are **not yet implemented** in this repository.

## Field deployment — start here

**[Ubuntu Humble deployment & first-drive checklist](docs/field_deployment_humble.md)**

The first-day acceptance sequence is: compile → verify real YHS CAN feedback → wheel odometry → guarded 0.05 m/s motion → start Femto Bolt. Only after that, integrate RTAB-Map mapping/localization and Nav2.

The original manufacturer ROS 2 driver and message source files are now included under [`vendor/FW-mini-ros2/`](vendor/FW-mini-ros2/), so cloning the repository provides `yhs_can_control` and `yhs_can_interfaces`. No binary vendor archives, ROS 1 packages, CAD or office documents were added in this update. The vendor materials were supplied separately; no license granting public redistribution was found in the imported ROS2 directory. **The repository is currently public; change it to Private immediately if redistribution is not permitted.** The original vendor files should remain unchanged; use a separate maintained copy for fixes.

## Core checks on Windows (no ROS)

    python tools/check_scaffold.py
    python tools/test_adapter_cores.py --cxx D:/msys/ucrt64/bin/g++.exe
    python tools/test_tag_follow_cores.py --cxx D:/msys/ucrt64/bin/g++.exe

Core PASS is not evidence of ROS runtime, DDS/TF or real hardware acceptance.

## Architecture

The experimental path is **YHS CAN → chassis/odom adapters → RGB-D SLAM (RTAB-Map) → Nav2**, with AprilTag for initialization/relocalization and a separate depth-based local obstacle pipeline. This repository stays separate from AGT Navigation's MID360 + LIO + GNSS outdoor-agriculture integration stack.

More details: [vendor review](docs/vendor_middleware_review.md), [adapter contract](docs/contracts.md), [adapter validation](docs/adapter_validation.md), [Tag pipeline](docs/tag_follow_pipeline.md).
