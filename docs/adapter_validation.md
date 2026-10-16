# Adapter validation record — 2026-10-12

Base commit: f7577f8. Existing three ROS packages unchanged. No remote push.

## Executed
- Windows (MSYS UCRT64 g++): python tools/test_adapter_cores.py --cxx D:/msys/ucrt64/bin/g++.exe: PASS.
- Linux cloud scratch (g++ C++17, -Wall -Wextra -Werror -pedantic): both actual C++ core test programs compiled and executed: PASS.
- Package XML and new Launch/integration-test Python syntax: PASS.
- Core cases: startup disabled, enable clears prior command, fresh command, signed clamping, disable,
  watchdog and clock regression, NaN/Inf rejection, SI/vendor units, YHS lateral correction,
  vx/vy/wz integration, rotated lateral displacement, duplicate/reversed stamps and feedback gap rebaseline.
- Changes scoped to new adapter packages, test tools and new docs. Original vendor packages were not edited.

## Not executed
- ROS node compilation / colcon: BLOCKED, no ROS2/colcon on connected Windows or cloud runtime.
- wsl.exe --list --quiet returned no installed distributions.
- ROS launch tests (fake commands/feedback, DDS topics, odom/TF, diagnostics): authored, NOT RUN.
- Real hardware CAN, gear mapping, VCU watchdog, sign/reference point and covariance calibration: NOT RUN.
- Standalone C++ PASS does not imply ROS API compilation or TF graph acceptance.

## Vendor source integrity
SHA256 over sorted relative paths + NUL + file bytes:
- yhs_can_control: 6 files, 05edea8b6fda3fac76a832d4f5d0d5613d96570a5c400a201192a34cbd6b9582
- yhs_can_interfaces: 18 files, 11e5f484e61894b2c45cfeb3bf04fac9310021208ede3b794bfdfcc6bd20b05d

The reviewed_yhs_ros2 feedback profile is for the unmodified vendor implementation only.
Choose mps_degps for a corrected decoder or SI-valued fake feedback.

## Next reproducible acceptance
On Ubuntu 22.04 / Humble install dependencies, then run bash tools/test_ros_adapters.sh.
The script builds original vendor interfaces in a separate overlay and runs both adapter tests.
It never launches yhs_can_control or opens a CAN device.

**2026-10-16 corrected vendor driver:** choose `mps_degps` for `/chassis_info_fb`.
The `reviewed_yhs_ros2` profile remains available for the **old unmodified** vendor decoder ONLY. Never multiply lateral feedback by 0.1 after vendor decoding is fixed.
