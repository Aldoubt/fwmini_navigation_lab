"""Offline CAN and corrected-driver integration regressions (no ROS, no CAN writes)."""
import math
from pathlib import Path
import unittest

from fwmini_can_protocol import CTRL_FEEDBACK_ID, decode_motion_frame, encode_motion_command, integrate_body_twist

ROOT = Path(__file__).resolve().parents[1]

class MotionContract(unittest.TestCase):
    def test_wire_sign_crc_and_scale(self):
        for x,y,deg in [(0.1,0.2,10),(-0.1,-0.2,-10),(0,0,0),(-1.23,0.045,25)]:
            data=encode_motion_command(6,x,y,math.radians(deg),9)
            m=decode_motion_frame(CTRL_FEEDBACK_ID,data)
            self.assertAlmostEqual(m.vx_mps,x,places=3)
            self.assertAlmostEqual(m.vy_mps,y,places=3)
            self.assertAlmostEqual(m.wz_radps,math.radians(deg),places=3)
            self.assertEqual(m.counter,9)
            broken=bytearray(data); broken[0]^=8
            with self.assertRaises(ValueError): decode_motion_frame(CTRL_FEEDBACK_ID,broken)
    def test_lateral_world_integration(self):
        x,y,a=integrate_body_twist(0,0,math.pi/2,0,0.2,0,0.5)
        self.assertAlmostEqual(x,-0.1)
        self.assertAlmostEqual(y,0)
    def test_corrected_vendor_and_adapter_profile_pair(self):
        source=(ROOT/'vendor/FW-mini-ros2/yhs_can_control/src/yhs_can_control_node.cpp').read_text()
        config=(ROOT/'src/fwmini_odom_adapter/config/odom_adapter.yaml').read_text()
        self.assertIn('msg.ctrl_fb_y_linear = static_cast<float>',source)
        self.assertIn('>> 4)) / 1000;',source)
        self.assertIn('publish_odom(msg.ctrl_fb_x_linear, msg.ctrl_fb_y_linear',source)
        self.assertIn('twist_cov.twist.linear.y = linear_y;',source)
        self.assertIn('feedback_profile: mps_degps',config)
        self.assertNotIn('feedback_profile: reviewed_yhs_ros2',config)

if __name__=='__main__': unittest.main()
