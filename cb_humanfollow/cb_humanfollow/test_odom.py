#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class OdomListener(Node):
    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation (quaternion -> yaw)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        theta = yaw
        # Log values
        self.get_logger().info(
            f"x: {x:.3f}, y: {y:.3f}, yaw: {theta:.3f} rad"
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
