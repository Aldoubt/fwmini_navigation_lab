"""Software-only, no ROS imports or hardware; field release still requires testing."""
import math
import unittest
from pathlib import Path

from fwmini_multi_nav_bringup.safety_scan import evaluate_scan

PI = math.pi


def sample_scan(point_list=(), *, n=721):
    ranges = [math.inf] * n
    a0, inc = -PI, 2*PI/(n-1)
    for x, y in point_list:
        bearing = math.atan2(y, x)
        idx = round((bearing - a0) / inc)
        ranges[idx] = math.hypot(x, y)
    return dict(angle_min=a0, angle_increment=inc, ranges=ranges,
                range_min=0.25, range_max=8.0,
                forward_mps=0.15, angular_radps=0.0)


class SafetyScanTests(unittest.TestCase):
    def test_empty_valid_scan_clear(self):
        result = evaluate_scan(**sample_scan())
        self.assertFalse(result.blocked)
        self.assertEqual(result.reason, 'CLEAR')

    def test_personlike_cluster_in_front_stops(self):
        result = evaluate_scan(**sample_scan([(0.50, -0.07), (0.50, 0.07)]))
        self.assertTrue(result.blocked)
        self.assertEqual(result.reason, 'OBSTACLE_STOP')

    def test_far_obstacle_clear_at_low_speed_but_blocked_at_one_mps(self):
        params = sample_scan([(1.30, -0.07), (1.30, 0.07)])
        low = evaluate_scan(**params)
        fast = evaluate_scan(**dict(params, forward_mps=1.0))
        self.assertFalse(low.blocked)
        self.assertTrue(fast.blocked)
        self.assertGreater(fast.stopping_distance_m, 1.8)

    def test_plant_near_front_not_ignored(self):
        result = evaluate_scan(**sample_scan([(0.44, -0.10), (0.44, 0.10)]))
        self.assertTrue(result.blocked)

    def test_outside_corridor_is_clear(self):
        self.assertFalse(evaluate_scan(**sample_scan([(0.50, 0.6),
                                                    (0.50, -0.6)])).blocked)

    def test_single_outlier_does_not_trip_min_hit_count(self):
        self.assertFalse(evaluate_scan(**sample_scan([(0.4, 0.05)])).blocked)
        self.assertTrue(evaluate_scan(
            **dict(sample_scan([(0.4, 0.05)]), minimum_hits=1)).blocked)

    def test_rotation_sweep_protection(self):
        params = sample_scan([(0.0, -0.38), (0.0, 0.38)])
        self.assertTrue(evaluate_scan(**dict(params, angular_radps=0.2)).blocked)

    def test_reverse_motion_is_rejected(self):
        result = evaluate_scan(**dict(sample_scan(), forward_mps=-0.04))
        self.assertTrue(result.blocked)
        self.assertEqual(result.reason, 'REVERSE_UNVERIFIED')

    def test_no_bad_scan_becomes_clear(self):
        params = sample_scan()
        for changes in [
            dict(angle_increment=0),
            dict(range_min=12.0),
            dict(ranges=[]),
            dict(ranges=[math.inf]*6000),
            dict(brake_mps2=0),
            dict(forward_mps=math.nan),
        ]:
            decision = evaluate_scan(**dict(params, **changes))
            self.assertTrue(decision.blocked)
            self.assertEqual(decision.reason, 'INVALID_SCAN_GEOMETRY')

    def test_final_guard_has_no_navigation_or_search_bypass(self):
        path = Path(__file__).resolve().parents[1] / (
            'fwmini_multi_nav_bringup/navigation_guard.py')
        source = path.read_text()
        tick = source.split('    def _tick(self):', 1)[1]
        self.assertIn('target_command = self._manual_command', tick)
        self.assertIn("target_command = self._command", tick)
        self.assertIn("scan_block = self._scan_gate(target_command or (0., 0., 0.), now)", tick)
        self.assertIn("self._current = (0., 0., 0.)", tick)
        self.assertLess(tick.index('target_command = self._manual_command'),
                        tick.index('scan_block ='))
        self.assertLess(tick.index('target_command = self._command'),
                        tick.index('scan_block ='))
        self.assertLess(tick.index('scan_block ='), tick.index('self._cmd_pub.publish(msg)'))

    def test_scan_callback_rejects_stale_stamp_or_wrong_frame(self):
        path = Path(__file__).resolve().parents[1] / (
            'fwmini_multi_nav_bringup/navigation_guard.py')
        src = path.read_text()
        self.assertIn('OBSTACLE_SCAN_FRAME_MISMATCH', src)
        self.assertIn('OBSTACLE_SCAN_STAMP_INVALID', src)
        self.assertIn('STALE_OBSTACLE_SCAN', src)


if __name__ == '__main__':
    unittest.main()
