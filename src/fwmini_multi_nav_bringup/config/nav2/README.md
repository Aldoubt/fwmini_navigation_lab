1: # Nav2 parameter handover
2: 
3: The camera/LiDAR extrinsics and FW-mini chassis footprint are not yet measured.
4: For this reason, no fabricated ready-to-drive Nav2 YAML is shipped.
5: 
6: On the actual Ubuntu22/Humble workstation, start with the **installed** Humble
7: Nav2 baseline from:
8: 
9:     $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml
10: 
11: Copy the baseline to a new, independent file in the FW-mini workspace. Change
12: at least the following based on measured FW-mini and verified IO:
13: 
14: 1. footprint (or robot_radius) and footprint_padding;
15: 2. global/local costmap robot_base_frame, obstacle topics, max/min height,
16:    inflation radius, observation raytracing and RGB-D limited FOV;
17: 3. controller plugin kinematics. Use a correctly configured holonomic DWB,
18:    MPPI or another omni-compatible controller; do not deploy a differential
19:    configuration if lateral motion is expected;
20: 4. x/y/angular velocity and acceleration limits *from the actual controller*;
21: 5. frame IDs map / odom / base_link, /cmd_vel, /odom topic, robot progress and
22:    collision monitoring (especially for camera-only navigation);
23: 6. chosen global map source: the one selected SLAM /map and map->odom
24:    broadcaster. Do not launch Nav2's own AMCL/localization_launch.py while
25:    SLAM is already publishing map->odom.
26: 
27: Only after physical tests mark the deployed hardware_contract.json flags as
28: true and explicitly invoke navigation:=true nav2_params_file:=/absolute/path.
29: This does not itself prove that full navigation is safe or performant.