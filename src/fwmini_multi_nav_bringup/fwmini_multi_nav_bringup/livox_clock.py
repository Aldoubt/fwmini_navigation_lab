"""Publish the synchronized MID360 IMU timestamp as the ROS simulation clock."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu


class LivoxClock(Node):
    """Map ROS time to the sensor clock while failing closed on clock reset."""

    def __init__(self):
        super().__init__('fwmini_livox_clock')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('clock_topic', '/clock')
        self._last_stamp_ns = 0
        self._clock_pub = self.create_publisher(
            Clock, self.get_parameter('clock_topic').value, 10)
        self._imu_sub = self.create_subscription(
            Imu, self.get_parameter('imu_topic').value,
            self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f"Publishing synchronized sensor time from "
            f"{self.get_parameter('imu_topic').value} to "
            f"{self.get_parameter('clock_topic').value}")

    def _on_imu(self, msg):
        stamp = msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0 or stamp_ns <= self._last_stamp_ns:
            return
        self._last_stamp_ns = stamp_ns
        clock = Clock()
        clock.clock = stamp
        self._clock_pub.publish(clock)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxClock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
