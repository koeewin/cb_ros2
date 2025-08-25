#!/usr/bin/env python3

"""
@file       imu_euler_node.py
@package    imu_utils
@brief      Simple ROS2 Node that subscribes to IMUEuler messages and republishes them.
"""

import rclpy
from rclpy.node import Node

from ception_msgs.msg import IMUEuler   # <- your custom Euler message


class ImuEulerNode(Node):
    def __init__(self):
        super().__init__('imu_euler_node')

        # Subscriber
        self.sub_imu = self.create_subscription(
            IMUEuler,
            '/diablo/sensor/ImuEuler',
            self.imu_callback,
            10
        )

        self.get_logger().info("IMU Euler Node started.")

    def imu_callback(self, msg: IMUEuler):
        # Print angles
        self.get_logger().info(
            f"Roll: {msg.roll:.3f}, Pitch: {msg.pitch:.3f}, Yaw: {msg.yaw:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuEulerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
