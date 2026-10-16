#!/usr/bin/env python3
"""SYNTHETIC TEST ONLY: publish stationary wheel odom and RGB-D, no /cmd_vel.

Only for an *isolated* ROS_DOMAIN_ID on ROS 2 Humble. Poses, camera intrinsics,
optical TF and RGB-D data are fabricated interface-test fixtures, NOT hardware
calibration evidence and NOT navigation performance measurements.
"""

import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class SyntheticSensor(Node):
    def __init__(self):
        super().__init__("fwmini_synthetic_sensor")
        self.width, self.height = 160, 120
        self.seq = 0
        self.odom = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.rgb = self.create_publisher(Image, "/fwmini/camera/color/image_raw", 10)
        self.depth = self.create_publisher(Image, "/fwmini/camera/depth/image_raw", 10)
        self.info = self.create_publisher(CameraInfo, "/fwmini/camera/color/camera_info", 10)
        self.depth_info = self.create_publisher(
            CameraInfo, "/fwmini/camera/depth/camera_info", 10
        )
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "fwmini_synthetic_optical"
        # Optical frame convention (-pi/2 roll, -pi/2 yaw); fixture only.
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        # Humble depthimage_to_laserscan defaults its scan header frame to
        # camera_depth_frame. An additional TEST-ONLY scanner TF is necessary
        # for Cartographer and SLAM Toolbox to consume the projected /scan.
        # This identity transform is never used in deployed robot code.
        laser_frame = TransformStamped()
        laser_frame.header.stamp = transform.header.stamp
        laser_frame.header.frame_id = "base_link"
        laser_frame.child_frame_id = "camera_depth_frame"
        laser_frame.transform.rotation.w = 1.0
        self.static_tf.sendTransform([transform, laser_frame])

        pixels = bytearray()
        depth = bytearray()
        for row in range(self.height):
            for col in range(self.width):
                checker = (col // 12 + row // 12) % 2
                pixels.extend((255 if checker else 80, 70 + row, 180))
                # Fabricated front obstacle depth at 1.8-2.1 m.
                millimeters = 1800 + int(300 * (0.5 + 0.5 * math.sin(col / 18)))
                depth.extend(struct.pack("<H", millimeters))
        self.rgb_data = bytes(pixels)
        self.depth_data = bytes(depth)
        self.timer = self.create_timer(0.1, self.send)

    def camera_info(self, stamp):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = "fwmini_synthetic_optical"
        msg.width, msg.height = self.width, self.height
        msg.distortion_model = "plumb_bob"
        msg.k = [145., 0., 80., 0., 145., 60., 0., 0., 1.]
        msg.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        msg.p = [145., 0., 80., 0., 0., 145., 60., 0., 0., 0., 1., 0.]
        return msg

    def image(self, stamp, encoding, step, data):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "fwmini_synthetic_optical"
        msg.width, msg.height = self.width, self.height
        msg.encoding = encoding
        msg.step = step
        msg.is_bigendian = False
        msg.data = data
        return msg

    def send(self):
        stamp = self.get_clock().now().to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.orientation.w = 1.
        self.odom.publish(msg)
        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = "base_link"
        tf.transform.rotation.w = 1.
        self.tf.sendTransform(tf)
        self.rgb.publish(self.image(stamp, "rgb8", self.width * 3, self.rgb_data))
        self.depth.publish(self.image(stamp, "16UC1", self.width * 2, self.depth_data))
        camera_info = self.camera_info(stamp)
        self.info.publish(camera_info)
        self.depth_info.publish(camera_info)
        self.seq += 1
        if self.seq % 30 == 0:
            self.get_logger().info("Published 30 synthetic RGB-D + odometry frames")


def main():
    rclpy.init()
    node = SyntheticSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
