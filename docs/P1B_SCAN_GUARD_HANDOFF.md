# P1b: front-obstacle fail-closed software gate (FW-mini)

## What changed

The ROS2 \`fwmini_navigation_guard\` is the last software publisher of
\`/agt/base/cmd_vel\` before the chassis adapter's watchdog and CAN interface.
Its \`/scan\` subscriber checks a **base_footprint-frame** laser projection.
Any obstacle in a speed-dependent forward corridor or in-place-turn footprint
causes **immediate zero velocity**, even during manual local search.

The verified route is still Nav2 -> VelocitySmoother -> /cmd_vel ->
FwminiNavigationGuard -> /agt/base/cmd_vel -> FW-mini chassis adapter -> CAN.
The scan safety decision happens after regular route / local-search selection,
so neither command path bypasses it. Stale or never-seen scans, incorrect
frame, non-increasing/zero timestamp or invalid geometric data stop movement.

The fixed FW-mini body dimensions used are 0.495 x 0.360m. Candidate forward
protection boundary is
\`0.2475m + clearance + speed*reaction + speed²/(2*brake)\`.
With clearance=0.30m, reaction=0.30s and brake=0.50m/s²:
- At 0.15m/s: stop corridor extends ~0.615m from robot center.
- At 1.00m/s: stop corridor extends ~1.848m from robot center.
These are NOT measured chassis braking data or a safety certification.

For no-CAN pure-Python tests:

\`\`\`bash
PYTHONPATH=src/fwmini_multi_nav_bringup \
 python3 -m unittest discover -s src/fwmini_multi_nav_bringup/test -p test_safety_scan.py -v
\`\`\`

## Important limits

- This is a software emergency gate, not an ISO-certified protective device.
- It cannot tell humans from plants; all near-field obstacles stop.
- It cannot detect something hidden behind a LiDAR blind spot, projected out
  by pointcloud_to_laserscan, or too close for the scan min range (0.25m).
- Never declare 1.0m/s safe based solely on unit tests or an assumed 0.50m/s²
  deceleration.
- High-speed motion, bypass of physical E-stop, and unsupervised autonomy
  remain BLOCKED pending cloud Humble, bag replay, integrated no-CAN ROS
  tests, and **physical measured stop distance**.
- Reverse motion is rejected by this guard until rear sensing is validated.
- The callback requires advancing scan header stamps; clock resets fail
  closed and need deliberate diagnostics.
- Inspect /fwmini/navigation_guard/status for WAIT_OBSTACLE_SCAN,
  STALE_OBSTACLE_SCAN, OBSTACLE_STOP, INVALID_SCAN_GEOMETRY,
  REVERSE_UNVERIFIED, and other pre-existing localization errors.

## RPP comparison

\`config/nav2/fwmini_mid360_nav2_rpp_candidate.yaml\` is an **isolated
candidate**, not a production default, used through the existing
\`nav2_params_file\` launch argument during a separate simulation.
It holds speed limits at the conservative trial baseline (0.12m/s RPP).
Compare lateral error P95, heading error, angular reversals, stopping delay,
command-vs-wheel response and no-unplanned-rotation events to DWB.
Do not apply the RPP file without a complete Nav2 parameter check.

## Cloud acceptance before field use

1. ros2_humble: install nav2 collision/controller plugins and build packages.
2. Check /scan frame and frequency (>=10 Hz recommended), monotonic stamps,
   active TF and whole command topic ownership.
3. Inject timed synthetic /scan obstacles in a **NO-CAN test domain**.
4. Under each route and manual search, verify immediate stop and stale-scan stop,
   no other publisher directly uses /agt/base/cmd_vel or /ctrl_cmd.
5. Prove legacy YHS three-field CtrlCmd isn't in the runtime.
6. Verify effective field launch params; do not increase default motion limits.
