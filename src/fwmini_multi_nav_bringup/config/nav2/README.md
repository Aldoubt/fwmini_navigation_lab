# Nav2 parameter handover

The camera/LiDAR extrinsics and FW-mini chassis footprint are not yet measured.
For this reason, no fabricated ready-to-drive Nav2 YAML is shipped.

On the actual Ubuntu22/Humble workstation, start with the **installed** Humble
Nav2 baseline from:

    $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml

Copy the baseline to a new, independent file in the FW-mini workspace. Change
at least the following based on measured FW-mini and verified IO:

1. footprint (or robot_radius) and footprint_padding;
2. global/local costmap robot_base_frame, obstacle topics, max/min height,
   inflation radius, observation raytracing and RGB-D limited FOV;
3. controller plugin kinematics. Use a correctly configured holonomic DWB,
   MPPI or another omni-compatible controller; do not deploy a differential
   configuration if lateral motion is expected;
4. x/y/angular velocity and acceleration limits *from the actual controller*;
5. frame IDs map / odom / base_link, /cmd_vel, /odom topic, robot progress and
   collision monitoring (especially for camera-only navigation);
6. chosen global map source: the one selected SLAM /map and map->odom
   broadcaster. Do not launch Nav2's own AMCL/localization_launch.py while
   SLAM is already publishing map->odom.

Only after physical tests mark the deployed hardware_contract.json flags as
true and explicitly invoke navigation:=true nav2_params_file:=/absolute/path.
This does not itself prove that full navigation is safe or performant.
