1: include "map_builder.lua"
2: include "trajectory_builder.lua"
3: options = {
4:   map_builder = MAP_BUILDER,
5:   trajectory_builder = TRAJECTORY_BUILDER,
6:   map_frame = "map",
7:   tracking_frame = "base_link",
8:   published_frame = "odom",
9:   odom_frame = "odom",
10:   provide_odom_frame = false,
11:   publish_frame_projected_to_2d = true,
12:   use_odometry = true,
13:   use_nav_sat = false,
14:   use_landmarks = false,
15:   num_laser_scans = 1,
16:   num_multi_echo_laser_scans = 0,
17:   num_subdivisions_per_laser_scan = 1,
18:   num_point_clouds = 0,
19:   lookup_transform_timeout_sec = 0.2,
20:   submap_publish_period_sec = 0.5,
21:   pose_publish_period_sec = 0.01,
22:   trajectory_publish_period_sec = 0.05,
23:   rangefinder_sampling_ratio = 1.0,
24:   odometry_sampling_ratio = 1.0,
25:   fixed_frame_pose_sampling_ratio = 1.0,
26:   imu_sampling_ratio = 1.0,
27:   landmarks_sampling_ratio = 1.0,
28: }
29: MAP_BUILDER.use_trajectory_builder_2d = true
30: TRAJECTORY_BUILDER_2D.use_imu_data = false
31: TRAJECTORY_BUILDER_2D.min_range = 0.25
32: TRAJECTORY_BUILDER_2D.max_range = 8.0
33: TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
34: TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
35: return options