"""Localization-aware FW-mini Twist gate for the Nav2-to-CAN command path."""

from math import isfinite
import time

import rclpy
from agt_robot_interfaces.msg import AdapterStatus, LocalizationStatus
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from .safety_scan import evaluate_scan
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String


class FwminiNavigationGuard(Node):
    """Pass holonomic Nav2 commands only while the selected map pose is valid."""

    def __init__(self):
        super().__init__('fwmini_navigation_guard')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/agt/base/cmd_vel')
        self.declare_parameter('status_topic', '/fwmini/navigation_guard/status')
        self.declare_parameter('localization_topic', '/agt/localization/status')
        self.declare_parameter('odom_status_topic', '/agt/odometry/adapter_status')
        self.declare_parameter('map_id', 'bunker_mid360')
        self.declare_parameter('map_version', '20260922_143427-fixed-replay-v1')
        self.declare_parameter('max_linear_x', 0.15)
        self.declare_parameter('max_linear_y', 0.15)
        self.declare_parameter('max_angular_z', 0.30)
        self.declare_parameter('max_linear_accel', 0.20)
        self.declare_parameter('max_angular_accel', 0.30)
        self.declare_parameter('command_timeout', 0.20)
        self.declare_parameter('status_timeout', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('allow_local_search', False)
        self.declare_parameter('manual_search_topic', '/fwmini/search_cmd_vel')
        self.declare_parameter('search_max_linear_x', 0.10)
        self.declare_parameter('search_max_linear_y', 0.0)
        self.declare_parameter('search_max_angular_z', 0.20)
        # Always fail closed for field profiles: any stale/missing scan blocks
        # BOTH Nav2 route commands and localization search commands.
        self.declare_parameter('require_fresh_obstacle_scan', True)
        self.declare_parameter('obstacle_scan_topic', '/scan')
        self.declare_parameter('obstacle_scan_frame', 'base_footprint')
        self.declare_parameter('obstacle_scan_timeout_s', 0.40)
        self.declare_parameter('reaction_time_s', 0.30)
        self.declare_parameter('brake_decel_mps2', 0.50)
        self.declare_parameter('clearance_m', 0.30)
        self.declare_parameter('side_margin_m', 0.18)
        self.declare_parameter('scan_min_hits', 2)
        self.declare_parameter('rotation_clearance_m', 0.55)

        self._localization = None
        self._localization_rx = None
        self._odom_status = None
        self._odom_status_rx = None
        self._command = (0.0, 0.0, 0.0)
        self._command_rx = None
        self._manual_command = (0.0, 0.0, 0.0)
        self._manual_rx = None
        self._current = (0.0, 0.0, 0.0)
        self._last_tick = time.monotonic()
        self._scan = None
        self._scan_rx = None
        self._scan_stamp_ns = None
        self._scan_error = 'WAIT_OBSTACLE_SCAN'
        for name in ('obstacle_scan_timeout_s', 'reaction_time_s',
                     'brake_decel_mps2', 'clearance_m', 'side_margin_m',
                     'rotation_clearance_m'):
            value = float(self._p(name))
            if not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not 1 <= int(self._p('scan_min_hits')) <= 10:
            raise ValueError('scan_min_hits must be 1..10')
        self._state = 'WAIT_LOCALIZATION'

        self._cmd_pub = self.create_publisher(Twist, self._p('output_topic'), 10)
        self._status_pub = self.create_publisher(String, self._p('status_topic'), 10)
        self.create_subscription(Twist, self._p('input_topic'), self._on_command, 10)
        self.create_subscription(Twist, self._p('manual_search_topic'), self._on_manual_command, 10)
        self.create_subscription(
            LaserScan, str(self._p('obstacle_scan_topic')), self._on_scan, 10)
        self.create_subscription(
            LocalizationStatus, self._p('localization_topic'), self._on_localization, 10)
        self.create_subscription(
            AdapterStatus, self._p('odom_status_topic'), self._on_odom_status, 10)
        rate = float(self._p('publish_rate'))
        if not isfinite(rate) or rate < 20.0:
            raise ValueError('publish_rate must be finite and at least 20 Hz')
        # The stop gate must keep running if synchronized sensor /clock stalls.
        self.create_timer(
            1.0 / rate, self._tick,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _p(self, name):
        return self.get_parameter(name).value

    def _on_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(isfinite(value) for value in values):
            self._command = (0.0, 0.0, 0.0)
            self._command_rx = None
            return
        self._command = values
        self._command_rx = time.monotonic()

    def _on_manual_command(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(isfinite(value) for value in values):
            self._manual_command = (0.0, 0.0, 0.0)
            self._manual_rx = None
            return
        self._manual_command = values
        self._manual_rx = time.monotonic()

    def _on_scan(self, msg):
        # Evaluate in the base_footprint plane. A wrong frame is not clear.
        self._scan_rx = time.monotonic()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if msg.header.frame_id != str(self._p('obstacle_scan_frame')):
            self._scan, self._scan_error = None, 'OBSTACLE_SCAN_FRAME_MISMATCH'
        elif stamp_ns <= 0 or (
                self._scan_stamp_ns is not None and stamp_ns <= self._scan_stamp_ns):
            self._scan, self._scan_error = None, 'OBSTACLE_SCAN_STAMP_INVALID'
        else:
            self._scan, self._scan_error = msg, ''
        self._scan_stamp_ns = stamp_ns

    def _scan_gate(self, command, now):
        if not bool(self._p('require_fresh_obstacle_scan')):
            return None  # only for explicit offline diagnostics, not field mode
        if self._scan_rx is None:
            return 'WAIT_OBSTACLE_SCAN'
        if now - self._scan_rx > float(self._p('obstacle_scan_timeout_s')):
            return 'STALE_OBSTACLE_SCAN'
        if self._scan is None:
            return self._scan_error or 'INVALID_OBSTACLE_SCAN'
        scan = self._scan
        decision = evaluate_scan(
            angle_min=scan.angle_min, angle_increment=scan.angle_increment,
            ranges=scan.ranges, range_min=scan.range_min, range_max=scan.range_max,
            forward_mps=command[0], angular_radps=command[2],
            clearance_m=float(self._p('clearance_m')),
            reaction_s=float(self._p('reaction_time_s')),
            brake_mps2=float(self._p('brake_decel_mps2')),
            side_margin_m=float(self._p('side_margin_m')),
            minimum_hits=int(self._p('scan_min_hits')),
            rotation_clearance_m=float(self._p('rotation_clearance_m')))
        return decision.reason if decision.blocked else None

    def _local_search_ready(self, now):
        if not bool(self._p('allow_local_search')):
            return False
        if self._odom_status is None or self._odom_status_rx is None:
            return False
        if now - self._odom_status_rx > float(self._p('status_timeout')):
            return False
        return self._odom_status.state == AdapterStatus.STATE_FRESH

    def _on_localization(self, msg):
        self._localization = msg
        self._localization_rx = time.monotonic()

    def _on_odom_status(self, msg):
        self._odom_status = msg
        self._odom_status_rx = time.monotonic()

    def _ready_reason(self, now):
        timeout = float(self._p('status_timeout'))
        if self._localization is None or self._localization_rx is None:
            return 'WAIT_LOCALIZATION'
        if now - self._localization_rx > timeout:
            return 'STALE_LOCALIZATION_STATUS'
        loc = self._localization
        if loc.map_id != self._p('map_id') or loc.map_version != self._p('map_version'):
            return 'MAP_IDENTITY_MISMATCH'
        if loc.state != LocalizationStatus.STATE_LOCALIZED or not loc.global_correction_valid:
            return 'WAIT_GLOBAL_RELOCALIZATION'
        if not loc.local_odom_fresh:
            return 'LOCAL_ODOM_NOT_FRESH'
        if self._odom_status is None or self._odom_status_rx is None:
            return 'WAIT_ODOM_ADAPTER'
        if now - self._odom_status_rx > timeout:
            return 'STALE_ODOM_STATUS'
        if self._odom_status.state != AdapterStatus.STATE_FRESH:
            return 'LOCAL_ODOM_NOT_FRESH'
        return 'READY'

    @staticmethod
    def _slew(current, target, max_delta):
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def _tick(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_tick, 0.1))
        self._last_tick = now
        state = self._ready_reason(now)
        command_timeout = float(self._p('command_timeout'))
        command_fresh = (self._command_rx is not None and
                         now - self._command_rx <= command_timeout)
        manual_fresh = (self._manual_rx is not None
                        and now - self._manual_rx <= command_timeout)
        # Manual local search remains separate from Nav2 global commands.
        # No global localization claim is made in this state.
        if manual_fresh and self._local_search_ready(now):
            state = 'LOCAL_SEARCH'
            target_command = self._manual_command
            limits = (float(self._p('search_max_linear_x')),
                      float(self._p('search_max_linear_y')),
                      float(self._p('search_max_angular_z')))
        elif state == 'READY' and command_fresh:
            target_command = self._command
            limits = (float(self._p('max_linear_x')),
                      float(self._p('max_linear_y')),
                      float(self._p('max_angular_z')))
        else:
            if state == 'READY':
                state = 'STALE_NAV_COMMAND'
            self._current = (0.0, 0.0, 0.0)
            target_command = None
            limits = None

        # This is the final software gate AFTER Nav2/local-search selection.
        # Scan failures/obstacles cause immediate zero, not slewed deceleration.
        scan_block = self._scan_gate(target_command or (0., 0., 0.), now)
        if scan_block is not None:
            state = scan_block
            self._current = (0., 0., 0.)
            target_command = None

        if target_command is None:
            output = self._current
        else:
            target = tuple(max(-limit, min(limit, value))
                           for value, limit in zip(target_command, limits))
            max_deltas = (float(self._p('max_linear_accel')) * dt,
                          float(self._p('max_linear_accel')) * dt,
                          float(self._p('max_angular_accel')) * dt)
            self._current = tuple(self._slew(current, desired, delta)
                                  for current, desired, delta in zip(
                                      self._current, target, max_deltas))
            output = self._current

        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = output
        self._cmd_pub.publish(msg)
        status = String()
        status.data = state
        self._status_pub.publish(status)
        self._state = state


def main(args=None):
    rclpy.init(args=args)
    node = FwminiNavigationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always send a final stop on orderly shutdown.
        if rclpy.ok():
            try:
                node._cmd_pub.publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
