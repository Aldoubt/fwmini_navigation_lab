#!/usr/bin/env bash
# Ubuntu 22.04 / ROS 2 Humble; build only vendor interfaces, never launch CAN.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
source /opt/ros/humble/setup.bash
task_vendor="$task_root/Fw-mini导航中间件/01_ros通讯协议解析包/FW-mini-ros2"
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
