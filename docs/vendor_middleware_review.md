# FW-mini vendor middleware review (2026-10-12)

Reviewed the local ROS2 C++ driver, message definitions, launch/config, URDF, and GB18030-decoded DBC. ROS1 and XLS/DOC manuals were not fully reviewed. No hardware execution or ROS compilation performed.

## Reusable assets
- yhs_can_interfaces: four wheel velocity/pulse, front/rear steering, battery, emergency stop, remote takeover, impact/drop feedback.
- yhs_can_control: Linux SocketCAN transport, signed packed fields, alive counter and XOR BCC.
- DBC: protocol source for units, offsets, CAN identifiers. Includes free-wheel commands absent from reviewed driver.
- STL: visual asset. Vendor URDF has one base_link visual only, no wheel joints, collision, inertial or camera frames.

## Confirmed issues and gaps
| Finding | Evidence | Integration action |
|---|---|---|
| Lateral feedback factor wrong | C++ line 243 divides by 100; DBC ctrl_fb_y_linear scale 0.001 m/s | Decode /1000 and test signed known frames |
| Angular command uses degrees/sec | DBC ctrl_cmd_z_angular unit degrees/sec; C++ scales by 100 | Adapter converts ROS rad/s to degrees/s |
| Odom ignores lateral motion | publish_odom takes vx,wz; twist.linear.y=0 | Integrate vx,vy,wz in SE(2) |
| No odom TF broadcaster | Only Odometry publication in full reviewed driver | Assign one odom->base_link publisher |
| Odom covariance left zero | pose/twist default initialized without covariance values | Specify measured uncertainty |
| No host command watchdog/rate limiter | Commands transmit only in subscription callbacks | Add periodic guarded output and stop timeout; confirm VCU timeout |
| No frame-size check | read>=0 then decode; no DLC/complete-frame validation | Reject partial, RTR/error and unexpected DLC frames |
| Ultrasonic unchecked indexing | config default empty; callback indexes eight entries | Validate length and indices before use |
| Aggregate state can contain stale subfields | accumulator updated per frame; publish only ctrl_fb | Add per-subfield age/validity, especially estop and remote takeover |
| Shutdown join can hang | stop closes socket then joins; receive loop depends only on rclcpp::ok; shutdown called after stop | Add stop flag and bounded wake/join |
| Missing direct build dependencies | Uses nav_msgs and tf2, not explicitly declared in CMake/package.xml | Declare direct dependencies |
| Interface CMake typos | CMAKECOMPILER_IS_GNUCXX, CMAKECXXCOMPILER_ID, addCompile_options, amentCmakeCopyrightFOUND | Correct variables/command in maintained copy |
| Failed driver init returns success | main returns 0 on run failure | Return nonzero |
| Preview geometry insufficient for navigation | Vendor URDF mesh offset -0.34,0.275,0 / rotation 1.57,0,0 is visual placement | Measure motion center/envelope; do not infer camera extrinsics |

## Planned adapter boundary
Guarded geometry_msgs/Twist -> fwmini chassis adapter (unit/mode conversion) -> yhs CtrlCmd -> CAN.
Vendor chassis_info_fb -> timestamp/validity-aware state adapter -> project state.
Wheel or chassis velocity -> continuous odometry/fusion -> sole odom TF publisher.
Select ctrl_cmd or steering_ctrl_cmd explicitly; gear semantics and firmware compatibility still require vendor manual confirmation. Do not send both modes concurrently.

## Build separation
Keep this original reference directory unchanged except COLCON_IGNORE to prevent recursive discovery of ROS1 and duplicate ROS2 packages.
Use colcon --base-paths src for current project packages. Later place a maintained ROS2-only vendor copy in a dedicated driver directory, with provenance and separate patch history.
