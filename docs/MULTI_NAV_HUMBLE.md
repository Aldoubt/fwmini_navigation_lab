1: # FW-mini multi-backend ROS 2 Humble integration
2: 
3: Original repository: Aldoubt/fwmini_navigation_lab (private).
4: Actual Humble checkout previously recorded at: ~/fwmini_ws/project.
5: This is an opt-in new package under src/fwmini_multi_nav_bringup. It does
6: NOT change the existing chassis, odom, camera driver or vendor ROS sources.
7: 
8: ## Actual code reconciliation
9: 
10: - fwmini_odom_adapter/config/odom_adapter.yaml outputs /wheel/odom and
11:   publishes odom -> base_link. Do NOT assume the topic is /odom.
12: - fwmini_perception/femto_bolt.launch.py wraps Orbbec in /fwmini namespace,
13:   with default RGB/depth topics /fwmini/camera/color/image_raw and
14:   /fwmini/camera/depth/image_raw. Verify actual topics before launch.
15: - fwmini_bringup geometry.preview.yaml is UNMEASURED. Do not treat preview
16:   camera/body values as real TF. The existing URDF's camera link is
17:   femto_bolt_link and its color optical link is
18:   femto_bolt_color_optical_frame. Confirm the official driver's frame IDs
19:   and avoid duplicate TF broadcasters.
20: - Previous camera-only RTAB-Map handheld experiments used rgbd_odometry.
21:   These new mobile-base experiments use existing /wheel/odom instead;
22:   stop the visual odometry node before running this overlay.
23: 
24: ## Humble build, no motor control
25: 
26: Run ONLY on Ubuntu 22.04 with ROS 2 Humble:
27: 
28:     source /opt/ros/humble/setup.bash
29:     sudo apt update
30:     sudo apt install ros-humble-rtabmap-ros ros-humble-slam-toolbox \
31:       ros-humble-cartographer-ros ros-humble-depthimage-to-laserscan \
32:       ros-humble-pointcloud-to-laserscan ros-humble-nav2-bringup
33: 
34:     cd ~/fwmini_ws/project
35:     git status -sb
36:     # Preserve all uncommitted changes; do not blindly git pull.
37:     colcon build --base-paths src --packages-select fwmini_multi_nav_bringup \
38:       --build-base build/multinav --install-base install/multinav \
39:       --symlink-install
40:     source install/multinav/setup.bash
41:     ros2 run fwmini_multi_nav_bringup fwmini_preflight --strict
42:     python3 -m unittest discover -s tools -p test_multi_nav_contracts.py -v
43: 
44: ## Static and no-motion validation
45: 
46: 1. Existing chassis feedback /wheel/odom has exactly one publisher and the
47:    odom->base_link edge only one TF publisher. No second odometry node.
48: 2. Registered Femto RGB/depth topic and CameraInfo timestamps / optical
49:    frames are aligned. Verify /fwmini/camera/color/camera_info and
50:    /fwmini/camera/depth/camera_info topics exist and match depth registration.
51: 3. Validate the measured static camera TF or fail; do not run with preview
52:    extrinsics. No automatic drivetrain commands from this package.
53: 4. Start ONE of these experiments and check /map and TF:
54: 
55:        ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=rtabmap
56:        ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=slam_toolbox
57:        ros2 launch fwmini_multi_nav_bringup bringup.launch.py backend:=cartographer_2d
58: 
59: 5. For SLAM Toolbox/Cartographer, inspect /scan projection, camera horizontal
60:    FOV and floor clipping in RViz; do not regard a single RGB-D camera as a
61:    360-degree safety scanner.
62: 6. Save RTAB-Map .db or SLAM Toolbox serialized graph, restart in localization
63:    mode only after verifying the native map and TF continuity.
64: 
65: Default navigation:=false. A complete human-verified Nav2 omni config and
66: hardware_contract JSON are required to enable Nav2. No measured footprint,
67: exterior camera pose, chassis gains or motion limits are supplied here.
68: FAST-LIO2, Cartographer 3D and pure-localization .pbstream are NOT enabled.
69: 
70: ## Current status
71: 
72: Ubuntu 24.04 ShunCode environment: offline software tests PASS; no Humble,
73: colcon or runtime ROS tests here. Original Ubuntu 22.04 SSH host/address is
74: not configured on this machine. To run SSH tests, supply a reachable
75: Ubuntu 22.04 SSH alias or host/IP, username and optional port (never share
76: a private key or password). Identical hostname strings on two machines do
77: not establish identity.