"""Conservative forward LaserScan stop gate; ROS-independent for offline tests.

A 2D laser projection cannot classify people vs foliage. Any measured object in
the swept safety corridor is treated as an obstacle; unsupported reverse travel
is blocked. Numeric distances are candidate safety parameters, not certified.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, hypot, isfinite, sin


@dataclass(frozen=True)
class ScanDecision:
    blocked: bool
    reason: str
    hits: int
    stopping_distance_m: float


def evaluate_scan(*, angle_min, angle_increment, ranges, range_min, range_max,
                  forward_mps, angular_radps, body_half_length_m=0.2475,
                  body_half_width_m=0.18, side_margin_m=0.18,
                  clearance_m=0.30, reaction_s=0.30, brake_mps2=0.50,
                  minimum_hits=2, rotation_clearance_m=0.55):
    """Assess the route command against scan points in base_footprint.

    Corridor ends at front bumper + reaction distance + measured braking
    distance estimate + clearance; negative x is a prohibited reverse command.
    Pure rotations are checked against a conservative in-place sweep radius.
    Errors fail closed, never silently cast a damaged scan to all-clear.
    """
    values = (angle_min, angle_increment, range_min, range_max,
              forward_mps, angular_radps, body_half_length_m,
              body_half_width_m, side_margin_m, clearance_m, reaction_s,
              brake_mps2, rotation_clearance_m)
    if not all(isfinite(float(v)) for v in values):
        return ScanDecision(True, "INVALID_SCAN_GEOMETRY", 0, 0.)
    if (angle_increment <= 0 or range_min < 0 or range_max <= range_min
            or brake_mps2 <= 0 or reaction_s < 0 or clearance_m < 0
            or body_half_length_m <= 0 or body_half_width_m <= 0
            or side_margin_m < 0 or rotation_clearance_m < 0.1
            or type(minimum_hits) is not int or minimum_hits < 1):
        return ScanDecision(True, "INVALID_SCAN_GEOMETRY", 0, 0.)
    if ranges is None or not (1 <= len(ranges) <= 5000):
        return ScanDecision(True, "INVALID_SCAN_GEOMETRY", 0, 0.)
    if forward_mps < -0.001:
        return ScanDecision(True, "REVERSE_UNVERIFIED", 0, 0.)
    forward_mps = max(0.0, float(forward_mps))
    stop_x = (body_half_length_m + clearance_m
              + forward_mps * reaction_s
              + forward_mps ** 2 / (2.0 * brake_mps2))
    half_width = body_half_width_m + side_margin_m
    count = 0
    for i, distance in enumerate(ranges):
        if not isfinite(distance) or distance < range_min or distance > range_max:
            continue
        theta = angle_min + i * angle_increment
        x, y = distance * cos(theta), distance * sin(theta)
        in_front = 0.0 <= x <= stop_x and abs(y) <= half_width
        in_turn = (abs(angular_radps) > 0.001
                   and hypot(x, y) <= rotation_clearance_m)
        if in_front or in_turn:
            count += 1
            if count >= minimum_hits:
                return ScanDecision(True, "OBSTACLE_STOP", count, stop_x)
    return ScanDecision(False, "CLEAR", count, stop_x)
