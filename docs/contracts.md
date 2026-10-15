# Runtime contracts v1

| Edge | Sole owner |
|---|---|
| map -> odom | Future localization manager |
| odom -> base_link | Future continuous odometry/fusion |
| base_link -> femto_bolt_link | robot_state_publisher |
| housing -> color optical | robot_state_publisher in preview; resolve vendor ownership before driver use |

| Topic | Type | Meaning |
|---|---|---|
| /fwmini/chassis/odom | nav_msgs/Odometry | continuous odom-frame body pose |
| /fwmini/chassis/state | fwmini_interfaces/ChassisState | feedback only |
| /fwmini/chassis/cmd_vel_guarded | geometry_msgs/Twist | future guard output to chassis adapter |
| /fwmini/camera/color/image_rect | sensor_msgs/Image | rectified RGB image |
| /fwmini/camera/color/camera_info | sensor_msgs/CameraInfo | matching rectified intrinsics/stamp |
| /fwmini/camera/depth/points | sensor_msgs/PointCloud2 | frame must identify actual depth geometry |
| /fwmini/localization/pose | geometry_msgs/PoseWithCovarianceStamped | map-frame base pose |
| /fwmini/localization/status | fwmini_interfaces/LocalizationStatus | quality state |

SetLocalizationAnchor is a service contract only; no server exists yet. Reject unmeasured extrinsics, unknown map/tag, stale observations and invalid covariance in its future implementation. AprilTag array type will be selected with the actual ROS detector version; it is intentionally not invented here.

Full SE(3) is required for tilted camera/tag poses:
T_map_base = T_map_tag * inverse(T_camera_tag) * inverse(T_base_camera).
At the same observation timestamp: T_map_odom = T_map_base * inverse(T_odom_base).
Continuous TF publication and loss state need an explicit runtime policy; a tag observation alone does not provide ongoing odometry.
