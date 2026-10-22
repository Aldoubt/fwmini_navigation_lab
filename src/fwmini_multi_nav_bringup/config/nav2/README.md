# FW-mini MID360 Nav2 candidate

`fwmini_mid360_nav2.yaml` is the Humble Nav2 baseline used by
`fwmini_mid360_navigation.launch.py`. It enables holonomic DWB x/y sampling,
uses the 495 x 360 mm vendor footprint, consumes the projected MID360 scan,
and limits the trial controller to 0.15 m/s in x/y and 0.30 rad/s in yaw.

The launcher pairs this file with the FW-mini localization-aware velocity gate
and the final chassis adapter. The gate requires fresh FAST-LIO odometry and a
valid global relocalization correction whose map ID/version match the selected
map. The adapter remains disabled until `/fwmini/enable_motion` is enabled.

These are software trial values, not verified physical motion limits. The
LiDAR transform is a rotation-fit/CAD candidate in
`config/field_trial/fwmini_mid360_extrinsic.yaml`. Check real footprint,
sensor timing, TF, CAN gear semantics, stop response, and obstacle projection
before using navigation results as physical accuracy evidence.
