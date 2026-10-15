#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble; build only vendor interfaces, never launch CAN.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
source /opt/ros/humble/setup.bash
# Prefer the versioned vendor ROS 2 source bundled in this repository.
# FWM_VENDOR_ROS2_DIR may override it for another tested vendor revision.
task_vendor="${FWM_VENDOR_ROS2_DIR:-$task_root/vendor/FW-mini-ros2}"
if [[ ! -f "$task_vendor/yhs_can_interfaces/package.xml" ]]; then
  echo "Missing YHS ROS2 interfaces at $task_vendor/yhs_can_interfaces" >&2
  echo "Set FWM_VENDOR_ROS2_DIR to the FW-mini-ros2 directory or restore vendor/FW-mini-ros2." >&2
  exit 2
fi
# Original vendor sources remain untouched. Their own lint/test configuration
# is excluded from this dependency-only build.
colcon build --base-paths "$task_vendor/yhs_can_interfaces" \
  --build-base build/vendor --install-base install/vendor \
  --cmake-args -DBUILD_TESTING=OFF
source install/vendor/setup.bash
colcon build --base-paths src --build-base build/project --install-base install/project \
  --cmake-args -DBUILD_TESTING=ON
source install/project/setup.bash
colcon test --base-paths src --build-base build/project --install-base install/project \
  --packages-select fwmini_chassis_adapter fwmini_odom_adapter \
  --event-handlers console_direct+
colcon test-result --test-result-base build/project --verbose
