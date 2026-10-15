# P1–P4 validation (2026-10-12)

Base commit c5ca482. No vendor source modification; no remote push.

Executed:
- Linux C++17 -Wall -Wextra -Werror -pedantic: detector quality and follow controller core tests PASS.
- Windows UCRT64 g++: same two core tests PASS.
- New package XML and Python launch/test syntax PASS.
- Official Orbbec launch args, AprilTag message/TF contract, and Humble image_proc executable reviewed from primary source.
- Combined quality -> body point -> follower -> chassis gate -> vendor command core test: Windows PASS; Linux equivalent chain PASS.
- Existing scaffold checks and new shell-script syntax PASS.

Not executed:
- ROS node compilation/colcon, ROS image/TF integration: BLOCKED (connected environment has no ROS2).
- Real Femto Bolt capture and FW-mini CAN motion: NOT RUN.
- Camera external mount TF, printed tag size, gear mode and hardware sign checks remain physical inputs.

P4 real detector test exists in tag_follow_controller/test/test_pipeline.py.
It uses official tag36h11 ID0 image raster, synthetic matching CameraInfo and a TEST-ONLY static camera mount.
It exercises real image_proc/apriltag_ros, adapter/follower/chassis adapter, white image loss and camera timeout.
No synthetic TF or size is used by the production launch.
