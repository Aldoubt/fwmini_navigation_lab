"""P4: synthetic camera -> image_proc -> REAL AprilTag -> adapters -> CtrlCmd.
No hardware driver. Synthetic static mount exists only in this test.
"""
import importlib.util
from pathlib import Path
import time
import unittest
import launch
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from fwmini_interfaces.msg import TagDetection
from yhs_can_interfaces.msg import CtrlCmd
from tf2_ros import StaticTransformBroadcaster
from launch_ros.actions import Node

spec = importlib.util.spec_from_file_location('fwmini_fixture', Path(__file__).with_name('fixture.py'))
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


@pytest.mark.launch_test
def generate_test_description():
    nodes = [
        Node(package='image_proc', executable='rectify_node',
             remappings=[('image', '/p4/color/image_raw'), ('image_rect', '/p4/color/image_rect'),
                         ('camera_info', '/p4/color/camera_info')]),
        Node(package='apriltag_ros', executable='apriltag_node',
             parameters=[{'family': '36h11', 'size': 0.16, 'max_hamming': 0,
                          'pose_estimation_method': 'pnp', 'image_transport': 'raw',
                          'qos_profile': 'sensor_data', 'detector.decimate': 1.0,
                          'tag.ids': [0], 'tag.frames': ['p4_tag'], 'tag.sizes': [0.16]}],
             remappings=[('image_rect', '/p4/color/image_rect'),
                         ('camera_info', '/p4/color/camera_info'),
                         ('detections', '/p4/raw_detections')]),
        Node(package='apriltag_detector_adapter', executable='apriltag_detector_adapter_node',
             parameters=[{'tag_frame': 'p4_tag', 'detections_topic': '/p4/raw_detections',
                          'output_topic': '/p4/tag_detection'}]),
        Node(package='tag_follow_controller', executable='tag_follow_controller_node',
             parameters=[{'detection_topic': '/p4/tag_detection',
                          'command_topic': '/p4/cmd_vel', 'extrinsics_verified': True}]),
        Node(package='fwmini_chassis_adapter', executable='fwmini_chassis_adapter_node',
             parameters=[{'command_topic': '/p4/cmd_vel',
                          'vendor_command_topic': '/p4/ctrl_cmd'}]),
    ]
    return launch.LaunchDescription(nodes + [launch_testing.actions.ReadyToTest()])


class Pipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('fake_femto_bolt')
        cls.tf = StaticTransformBroadcaster(cls.node)
        mount = TransformStamped()
        mount.header.stamp = cls.node.get_clock().now().to_msg()
        mount.header.frame_id = 'base_link'
        mount.child_frame_id = 'p4_color_optical'
        # Synthetic test axes: optical z -> base x, optical x -> base -y.
        mount.transform.rotation.x = -0.5
        mount.transform.rotation.y = 0.5
        mount.transform.rotation.z = -0.5
        mount.transform.rotation.w = 0.5
        cls.tf.sendTransform(mount)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait(self, condition, action=None, timeout=10.0):
        deadline, last_action = time.monotonic() + timeout, 0.0
        while time.monotonic() < deadline:
            if action and time.monotonic() - last_action >= 0.04:
                action()
                last_action = time.monotonic()
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if condition():
                return
        self.fail('P4 pipeline condition timed out')

    def test_image_to_vendor_command_and_losses(self):
        tags, twists, commands = [], [], []
        subscriptions = [
            self.node.create_subscription(TagDetection, '/p4/tag_detection', tags.append, 10),
            self.node.create_subscription(Twist, '/p4/cmd_vel', twists.append, 10),
            self.node.create_subscription(CtrlCmd, '/p4/ctrl_cmd', commands.append, 10)]
        image_pub = self.node.create_publisher(Image, '/p4/color/image_raw', 10)
        info_pub = self.node.create_publisher(CameraInfo, '/p4/color/camera_info', 10)

        def publish_image(show_tag=True):
            image = Image()
            image.header.stamp = self.node.get_clock().now().to_msg()
            image.header.frame_id = 'p4_color_optical'
            image.width, image.height = 640, 480
            image.encoding, image.step = 'mono8', 640
            image.data = fixture.image_bytes(show_tag)
            info = CameraInfo()
            info.header = image.header
            info.width, info.height = 640, 480
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            info.k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [600.0, 0.0, 320.0, 0.0, 0.0, 600.0, 240.0, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            info_pub.publish(info)
            image_pub.publish(image)

        self.wait(lambda: image_pub.get_subscription_count() > 0)
        self.wait(lambda: any(t.valid for t in tags), action=publish_image)
        detected = next(t for t in tags if t.valid)
        self.assertEqual(detected.id, 0)
        self.assertEqual(detected.header.frame_id, 'p4_color_optical')
        self.assertGreater(detected.pose.position.z, 0.9)
        self.assertLess(detected.pose.position.z, 1.5)
        self.assertGreater(detected.pose.position.x, 0)  # tag is to image right
        self.wait(lambda: len(commands) > 2, action=publish_image)
        self.assertTrue(all(c.ctrl_cmd_x_linear == 0 for c in commands))

        clients = []
        def enable(name, value):
            client = self.node.create_client(SetBool, name)
            clients.append(client)
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
            future = client.call_async(SetBool.Request(data=value))
            self.wait(future.done)
            self.assertTrue(future.result().success)

        enable('/fwmini/enable_tag_follow', True)
        enable('/fwmini/enable_motion', True)
        commands.clear()
        self.wait(lambda: any(c.ctrl_cmd_x_linear > 0 and c.ctrl_cmd_z_angular < 0
                             for c in commands), action=publish_image)
        moving = next(c for c in commands if c.ctrl_cmd_x_linear > 0)
        self.assertLessEqual(moving.ctrl_cmd_x_linear, 0.201)
        self.assertEqual(moving.ctrl_cmd_y_linear, 0)
        self.assertTrue(any(t.angular.z < 0 for t in twists))
        # White images exercise target-loss messages, not just source timeout.
        tags.clear()
        self.wait(lambda: any(not t.valid for t in tags),
                  action=lambda: publish_image(False))
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            publish_image(False)
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertEqual(commands[-1].ctrl_cmd_x_linear, 0)
        self.assertEqual(commands[-1].ctrl_cmd_z_angular, 0)
        commands.clear()
        self.wait(lambda: any(c.ctrl_cmd_x_linear > 0 for c in commands), action=publish_image)
        # Camera disconnect: no more image or CameraInfo publications.
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(commands[-1].ctrl_cmd_x_linear, 0)
        self.assertEqual(commands[-1].ctrl_cmd_z_angular, 0)
        commands.clear()
        self.wait(lambda: any(c.ctrl_cmd_x_linear > 0 for c in commands), action=publish_image)
        enable('/fwmini/enable_tag_follow', False)
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            publish_image()
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(commands[-1].ctrl_cmd_x_linear, 0)
        enable('/fwmini/enable_motion', False)
        for client in clients:
            self.node.destroy_client(client)
        for sub in subscriptions:
            self.node.destroy_subscription(sub)
        self.node.destroy_publisher(image_pub)
        self.node.destroy_publisher(info_pub)
