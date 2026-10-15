"""Hardware-free ROS graph test. Starts only the adapter, never the CAN driver."""
import math
import time
import unittest
import launch
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from yhs_can_interfaces.msg import CtrlCmd
from launch_ros.actions import Node


@pytest.mark.launch_test
def generate_test_description():
    adapter = Node(
        package='fwmini_chassis_adapter', executable='fwmini_chassis_adapter_node',
        parameters=[{'command_topic': '/test_cmd/input',
                     'vendor_command_topic': '/test_cmd/output',
                     'cmd_timeout': 0.2, 'max_linear_x': 0.3,
                     'max_linear_y': 0.2, 'max_angular_z': 0.5}])
    return launch.LaunchDescription([adapter, launch_testing.actions.ReadyToTest()])


class CommandTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fake_command_tester')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
            if condition():
                return
        self.fail('ROS condition timed out')

    def test_command_gate_and_units(self):
        messages = []
        sub = self.node.create_subscription(CtrlCmd, '/test_cmd/output', messages.append, 10)
        pub = self.node.create_publisher(Twist, '/test_cmd/input', 10)
        service = self.node.create_client(SetBool, '/fwmini/enable_motion')
        self.assertTrue(service.wait_for_service(timeout_sec=5.0))
        self.wait(lambda: pub.get_subscription_count() > 0 and len(messages) > 0)

        def enable(value):
            future = service.call_async(SetBool.Request(data=value))
            self.wait(future.done)
            self.assertTrue(future.result().success)

        def zero(msg):
            return (msg.ctrl_cmd_x_linear == 0 and msg.ctrl_cmd_y_linear == 0
                    and msg.ctrl_cmd_z_angular == 0)

        command = Twist()
        command.linear.x = 1.0
        command.linear.y = -1.0
        command.angular.z = 1.0
        pub.publish(command)
        self.wait(lambda: len(messages) >= 3)
        self.assertTrue(all(zero(msg) for msg in messages))
        enable(True)
        messages.clear()
        pub.publish(command)
        self.wait(lambda: any(msg.ctrl_cmd_x_linear > 0 for msg in messages))
        output = next(msg for msg in messages if msg.ctrl_cmd_x_linear > 0)
        self.assertAlmostEqual(output.ctrl_cmd_x_linear, 0.3, places=6)
        self.assertAlmostEqual(output.ctrl_cmd_y_linear, -0.2, places=6)
        self.assertAlmostEqual(output.ctrl_cmd_z_angular, math.degrees(0.5), places=5)
        messages.clear()
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertTrue(messages)
        self.assertTrue(zero(messages[-1]))  # Inspect output after the actual deadline.
        pub.publish(command)
        messages.clear()
        self.wait(lambda: any(msg.ctrl_cmd_x_linear > 0 for msg in messages))
        enable(False)
        messages.clear()
        self.wait(lambda: len(messages) >= 2)
        self.assertTrue(all(zero(msg) for msg in messages))
        enable(True)  # Requires a new command, cannot replay the pre-disable command.
        messages.clear()
        self.wait(lambda: len(messages) >= 2)
        self.assertTrue(all(zero(msg) for msg in messages))
        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(pub)
        self.node.destroy_client(service)
