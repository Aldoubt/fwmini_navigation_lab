# FW-mini multi-backend ROS 2 Humble integration

Original repository: Aldoubt/fwmini_navigation_lab (private).
Actual Humble checkout previously recorded at: ~/fwmini_ws/project.
This is an opt-in new package under src/fwmini_multi_nav_bringup. It does
NOT change the existing chassis, odom, camera driver or vendor ROS sources.

## Actual code reconciliation

- fwmini_odom_adapter/config/odom_adapter.yaml outputs /wheel/odom and
  publishes odom -> base_link. Do NOT assume the topic is /odom.
- fwmini_perception/femto_bolt.launch.py wraps Orbbec in /fwmini namespace,
  with default RGB/depth topics /fwmini/camera/color/image_raw and
  /fwmini/camera/depth/image_raw. Verify actual topics before launch.
- fwmini_bringup geometry.preview.yaml is UNMEASURED. Do not treat preview
  camera/body values as real TF. The existing URDF's camera link is
  femto_bolt_link and its color optical link is
  femto_bolt_color_optical_frame. Confirm the official driver's frame IDs
  and avoid duplicate TF broadcasters.
- Previous camera-only RTAB-Map handheld experiments used rgbd_odometry.
  These new mobile-base experiments use existing /wheel/odom instead;
  stop the visual odometry node before running this overlay.

## Humble build, no motor control

Run ONLY on Ubuntu 22.04 with ROS 2 Humble:

    source /opt/ros/humble/setup.bash
    sudo apt update
    sudo apt install ros-humble-rtabmap-ros ros-humble-slam-toolbox \
      ros-humble-cartographer-ros ros-humble-depthimage-to-laserscan \
      ros-humble-pointcloud-to-laserscan ros-humble-nav2-bringup

    cd ~/fwmini_ws/project
    git status -sb
    # Preserve all uncommitted changes; do not blindly git pull.
    colcon build --base-paths src --packages-select fwmini_multi_nav_bringup \
      --build-base build/multinav --install-base install/multinav \
      --symlink-install
    source install/multinav/setup.bash
    ros2 run fwmini_multi_nav_bringup fwmini_preflight --strict
    python3 -m unittest discover -s tools -p test_multi_nav_contracts.py -v

## Static and no-motion validation

1. Existing chassis feedback /wheel/odom has exactly one publisher and the
   odom->base_link edge only one TF publisher. No second odometry node.
2. Registered Femto RGB/depth topic and CameraInfo timestamps / optical
   frames are aligned. Verify /fwmini/camera/color/camera_info and
   /fwmini/camera/depth/camera_info topics exist and match depth registration.
3. Validate the measured static camera TF or fail; do not run with preview
   extrinsics. No automatic drivetrain commands from this package.
4. Start ONE of these experiments and check /map and TF:

       ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=rtabmap
       ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=slam_toolbox
       ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=cartographer_2d

5. For SLAM Toolbox/Cartographer, inspect /scan projection, camera horizontal
   FOV and floor clipping in RViz; do not regard a single RGB-D camera as a
   360-degree safety scanner.
6. Save RTAB-Map .db or SLAM Toolbox serialized graph, restart in localization
   mode only after verifying the native map and TF continuity.

Default navigation:=false. A complete human-verified Nav2 omni config and
hardware_contract JSON are required to enable Nav2. No measured footprint,
exterior camera pose, chassis gains or motion limits are supplied here.
FAST-LIO2, Cartographer 3D and pure-localization .pbstream are NOT enabled.

## Current status

Ubuntu 24.04 ShunCode environment: offline software tests PASS; no Humble,
colcon or runtime ROS tests here. Original Ubuntu 22.04 SSH host/address is
not configured on this machine. To run SSH tests, supply a reachable
Ubuntu 22.04 SSH alias or host/IP, username and optional port (never share
a private key or password). Identical hostname strings on two machines do
not establish identity.
