# FW-mini CAN and multi-SLAM Humble integration (2026-10-16)

## Confirmed
- SSH to Ubuntu 22.04.5, ROS 2 Humble works. This is an independent worktree from existing multi-nav branch; `main` remains untouched.
- Vendor ROS2 decoder originally used /100 for lateral feedback despite ×0.001m/s CAN layout; corrected vendor source to /1000.
- Vendor `publish_odom` now integrates x and y body velocities with midpoint heading and reports nonzero twist.y. **Vendor /odom is diagnostic only**; the external FW-mini odom adapter owns `/wheel/odom` plus `odom->base_link` TF.
- `fwmini_odom_adapter` now defaults to `mps_degps` for the corrected vendor decoder, rather than the old 0.1 lateral compensation. The legacy `reviewed_yhs_ros2` profile remains available ONLY with the unpatched driver.
- Source C++ adapter cores and ROS2 Humble build tested without live motor commands. No live CAN movement has been authorized or run.

## Startup contracts (do not enable autonomous motion yet)
- The manufacturer's drive gear enumeration is **unverified**; keep default `control_gear: 0` / gate disabled until physically verified. No fabricated gear=6.
- The chassis command adapter already publishes `/ctrl_cmd` at 50 Hz and applies a 0.2-second ROS command expiry. This is not proof of VCU hardware failsafe; it must be tested with lifted wheels.
- Avoid duplicate `/cmd_vel` publishers or bypass of Nav2 velocity smoother. Set one owner of `odom->base_link` (odom adapter), one owner of `map->odom` (chosen SLAM/localization backend).
- Require real camera extrinsics, 3D footprint, RGB-D registration and FPS/CameraInfo sync. Single Femto Bolt has large rear/lateral blind zones; disable holonomic movements and rearward navigation until obstacle coverage is verified.
- All extra navigation gates start false: `can_protocol_verified`, `odom_omni_verified`, `command_watchdog_verified`, and `tf_ownership_verified`.

## Algorithm gaps
- RTAB-Map: current bringup uses RGB-D + external wheel odom, not internal visual odometry; verify registered depth topic, optical frames, timestamps, map database persistence and cold-start relocalization. Do not run `rgbd_odometry` as second odom TF source.
- Cartographer 2D: `depthimage_to_laserscan` or `pointcloud_to_laserscan` source required; front-camera limited FOV and clipping around floor/ceiling, tune based on measured camera TF. Save `.pbstream`; pure localization launch still blocked (not tested).
- SLAM Toolbox 2D: verify narrow-FOV scan efficacy and serialize native posegraph, not just 2D `.pgm`.
- FAST-LIO2 + 3D-BBS/GICP: blocked until MID360+IMU is installed, point timing and 15-degree lidar tilt TF calibrated, LIO odom is bridged without competing TF publishers, global-map alignment provides `map->odom`, PCD-derived occupancy map is validated. Pure FAST-LIO2 does not provide global cold-start relocalization.
- Nav2: requires the first measured vehicle footprint and controller/costmap YAML, validated x/y/yaw speed and acceleration, obstacle clearing and Nav2 command mux. A 20 Hz DWB + 50 Hz smoother is a proposed experiment, not validated hardware control.

## Repeatable commands (read-only/runtime test; no CAN transmission)
```
source /opt/ros/humble/setup.bash
cd ~/fwmini_ws/worktrees/fwmini-can-protocol-humble
python3 tools/test_fwmini_can_protocol.py
python3 tools/test_adapter_cores.py
python3 -m unittest discover -s tools -p test_multi_nav_contracts.py -v
source install/can_fix/setup.bash
colcon --log-base log/can_fix_test test --build-base build/can_fix --install-base install/can_fix --packages-select fwmini_odom_adapter fwmini_chassis_adapter --event-handlers console_direct+
colcon test-result --test-result-base build/can_fix --verbose
```

On actual hardware, first examine `candump -L can0` read-only, check frame 0x18C4D1EF XOR and counter continuity vs `/chassis_info_fb`; only then separately authorize wheel-raised command tests. Real firmware gear enum and acceleration/braking dynamics cannot be inferred from the DBC alone.
