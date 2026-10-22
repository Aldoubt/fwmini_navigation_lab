# FW-mini MID360 navigation trial

This is the isolated field-candidate entry point. It uses the installed AGT
FAST-LIO2/global relocalization stack for `odom -> base_footprint` and
`map -> odom`, Nav2 for planning/control, and the FW-mini adapter as the final
CAN command boundary. It does not start either physical driver.

## Build this worktree as an overlay

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
cd /home/yangxuan/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge
colcon --log-base log/nav2_trial build \
  --packages-up-to agt_navigation_interfaces agt_fastlio_adapter \
    fwmini_multi_nav_bringup fwmini_chassis_adapter \
  --base-paths src vendor/FW-mini-ros2/yhs_can_interfaces vendor/FW-mini-ros2/yhs_can_control \
  --build-base build/nav2_trial \
  --install-base install/nav2_trial --symlink-install \
  --allow-overriding yhs_can_interfaces agt_navigation_interfaces agt_fastlio_adapter
```

In each test terminal, source the same overlays:

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
source /home/yangxuan/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/install/nav2_trial/setup.bash
```

## Start the external hardware owners

Start MID360 CustomMsg and the YHS CAN driver in separate terminals. For the
MID360, use the generated host IP in its config and omit empty ROS parameters:

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
python3 /home/yangxuan/ros2_ws/src/agt_navigation_v3/scripts/generate_mid360_config.py \
  --lidar-ip 192.168.1.145 --host-ip auto \
  --output /home/yangxuan/.ros/agt_mid360/MID360_config.json
ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 -p multi_topic:=0 -p data_src:=0 \
  -p publish_freq:=10.0 -p output_data_type:=0 -p frame_id:=livox_frame \
  -p user_config_path:=/home/yangxuan/.ros/agt_mid360/MID360_config.json
```

Start the already-built YHS CAN driver in its own terminal with the FW-mini
vendor/project overlays sourced:

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/fwmini_ws/project/install/vendor/setup.bash
source /home/yangxuan/fwmini_ws/project/install/yhs_driver/setup.bash
source /home/yangxuan/fwmini_ws/project/install/project/setup.bash
ros2 launch yhs_can_control yhs_can_control.launch.py
```

Do not start a second FAST-LIO2 process; the navigation launch owns FAST-LIO2. Do not
start `fwmini_odom_adapter` in this composition; FAST-LIO's calibrated adapter
is the sole `odom -> base_footprint` TF owner. The vendor driver may continue
publishing its diagnostic `/odom` message.

Before navigation, confirm `/livox/lidar`, `/livox/imu`, and
`/chassis_info_fb` are publishing, and check that no old FAST-LIO/localization/
Nav2/adapter process remains in the ROS domain.

## Start localization, map, Nav2, RViz

```bash
ros2 launch fwmini_multi_nav_bringup fwmini_mid360_navigation.launch.py \
  start_chassis_adapter:=true \
  auto_relocalize:=false
```

The defaults select map ID `bunker_mid360`, version
`20260924-trav-integration-v1`, with Nav2 YAML at
`navigation/bunker_v1/map.yaml`. The launch checks that the supplied paths,
metadata identity, and asset hashes agree. This map version is not in the
validated map registry; explicit use here does not promote or register it.

By default, this bringup uses the manual-seed initialization node. To try the
automatic full-map BBS + GICP search instead, set `auto_relocalize:=true` and
keep the chassis adapter disabled for this stationary localization check. The
automatic search starts after five fresh stationary cloud frames; it does not
use RViz's **2D Pose Estimate**. Its result remains a candidate until the
localization manager accepts it and publishes `map -> odom`.

For manual initialization, once `/scan` and local odometry are fresh and the
robot is stationary, choose **2D Pose Estimate** in RViz and click-drag the
robot's approximate position and heading on the map. That pose is only a GICP
starting estimate. Monitor:

```bash
ros2 topic echo /agt/global_relocalization/status
ros2 topic echo /agt/localization/initialization_status
ros2 topic echo /agt/localization/status
ros2 topic echo /fwmini/navigation_guard/status
```

Wait for a valid correction and check RViz TF, map, scan, and robot pose. Nav2
uses the omni-capable DWB controller; both x/y velocity ranges are set to
0.15 m/s, angular speed to 0.30 rad/s. The final adapter additionally clamps
x/y to 0.30 m/s and angular speed to 0.50 rad/s, times out commands after
0.20 s, and starts motion-disabled.

This launch uses the synchronized MID360 IMU stamp as `/clock` and defaults
`use_sim_time` to `true`. Start the Livox driver first and confirm both
`/livox/imu` and `/clock` are advancing. FAST-LIO, relocalization, localization,
Nav2, TF consumers, and RViz then share the synchronizer time domain. The
navigation guard and chassis command watchdog use steady wall time, so a frozen
sensor clock still expires commands and stops the command stream.

Wait for `/fwmini/navigation_guard/status` to report `READY`. Confirm
`/agt/odometry/adapter_status` is `FRESH`, `/agt/odometry/local` is advancing
with strictly increasing source timestamps, and the chassis emergency-stop and
remote feedback indicate the actual disabled/enabled state you expect. If the
FAST-LIO adapter reports stale input or no `odom -> base_footprint` TF, do not
enable chassis motion. In this synchronized-trigger profile, the adapter's
absolute source-stamp age rejection is disabled; transport freshness uses the
host monotonic clock and source stamps are preserved. The `/scan` costmap
subscriptions use best-effort QoS to match the projected LiDAR scan publisher.
Do not enable motion until localization is actually `LOCALIZED`, Nav2 nodes are
active, and the map, scan, robot pose, and TF chain are coherent in RViz.

Only after the localization/map/TF display is coherent, enable the adapter:

```bash
ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: true}"
```

Then set a short 2D Goal Pose in RViz. Disable the adapter after the run:

```bash
ros2 service call /fwmini/enable_motion std_srvs/srv/SetBool "{data: false}"
```

## Evidence boundary

The LiDAR transform in `config/field_trial/fwmini_mid360_extrinsic.yaml` is a
spin-fit/CAD candidate, not a field-accepted calibration. Its x offset assumes
the spin center equals `base_link`; confirm this against the chassis geometry.
The launch composes software but does not establish CAN gear semantics,
stop-distance behavior, relocalization accuracy, or real path accuracy. Verify
the live TF chain and command/feedback topics before treating a goal result as
an accuracy measurement.
