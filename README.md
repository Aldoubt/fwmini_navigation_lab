# fwmini_navigation_lab

ROS 2 Humble / Ubuntu 22.04. First implementation: static model, typed contracts and independent description launch. Camera placement is pending ruler/SolidWorks measurement.

## Packages

| Package | Implemented responsibility |
|---|---|
| fwmini_description | Box envelope and fixed housing/optical TF |
| fwmini_interfaces | Chassis status, localization status, anchor service |
| fwmini_bringup | Description launch and external geometry config validation |

## Build and preview

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch fwmini_bringup description.launch.py
```

Use RViz Fixed Frame `base_link`, add RobotModel and TF. This preview publishes only static body/camera transforms. No map or odom is fabricated.

## Measurement workflow

Copy `src/fwmini_bringup/config/geometry.preview.yaml` to your own geometry YAML. All distances use metres; roll/pitch/yaw use radians. Base axes: x forward, y left, z up. Choose and document the chassis motion reference before measuring. Body box starts at z=0 in this preliminary model; set the base origin consistently with the actual geometry.

1. Measure body envelope in SW/ruler and record uncertainty.
2. Measure housing pose relative to base_link; enter camera_mount xyz/rpy.
3. Obtain color optical origin relative to housing from vendor model or measurement. Optical axes are x right, y down, z forward; the preview rotation assumes housing x forward/y left/z up.
4. Set measured=true separately for each verified section. Store evidence and date with the config.
5. Run `ros2 launch fwmini_bringup description.launch.py geometry_file:=/absolute/path/geometry.yaml preview:=false`.

The future Orbbec driver must have one TF owner per edge. Disable/omit overlapping driver TF or replace this optical joint with the verified vendor tree before integration. Depth optical extrinsics are not defined yet.

## Interface and next stages

See docs/contracts.md. No CAN protocol, camera driver, AprilTag detector, continuous odometry, localization runtime or Nav2 has been implemented in this first batch. Complete these as independently launchable modules after static TF verification.

Windows can edit and run static checks. Ubuntu ROS build/TF verification and hardware acceptance are separate gates.
