#!/usr/bin/env bash
# No camera hardware or CAN driver is started by these tests.
set -eo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$task_root"
source /opt/ros/humble/setup.bash
task_vendor="$task_root/Fw-mini导航中间件/01_ros通讯协议解析包/FW-mini-ros2"
colcon build --base-paths "$task_vendor/yhs_can_interfaces" \
  --build-base build/vendor --install-base install/vendor --cmake-args -DBUILD_TESTING=OFF
source install/vendor/setup.bash
colcon build --base-paths src --build-base build/project --install-base install/project \
  --cmake-args -DBUILD_TESTING=ON
source install/project/setup.bash
colcon test --base-paths src --build-base build/project --install-base install/project \
  --packages-select apriltag_detector_adapter tag_follow_controller \
  --event-handlers console_direct+
colcon test-result --test-result-base build/project --verbose
