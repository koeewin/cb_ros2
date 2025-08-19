import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Ensure this is the correct import for publishing odometry
from motion_msgs.msg import LegMotors  # Adjust this to the correct import path
from rclpy.callback_groups import ReentrantCallbackGroup
from motion_msgs.msg import MotionCtrl
from std_msgs.msg import Int8
import numpy as np

import math

class DiabloOdometry(Node):
    def __init__(self):

        super().__init__('diablo_odometry')

        callback_group = ReentrantCallbackGroup()

        self.wheel_encoder_subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.wheel_encoder_callback, 10, callback_group=callback_group)
        # self.odom_request_subscription = self.create_subscription(Int8, "diablo/odom_request", self.odom_request_callback, 2,callback_group=callback_group)
        
        self.odom_publisher = self.create_publisher(Odometry, 'diablo/odometry', 10, callback_group=callback_group)

        self.robot = Robot()  # Initialize the robot instance here
        # Initialize previous wheel positions
        self.prev_left_wheel_pos = None
        self.prev_right_wheel_pos = None

        self.odom_msg = Odometry()
        

    def wheel_encoder_callback(self, msg):
        left_wheel_enc_rev = msg.left_wheel_enc_rev
        right_wheel_enc_rev = msg.right_wheel_enc_rev

        # Calculate the current positions based on the encoder readings
        current_left_wheel_pos = msg.left_wheel_pos + left_wheel_enc_rev * 2 * math.pi
        current_right_wheel_pos = msg.right_wheel_pos + right_wheel_enc_rev * 2 * math.pi

        # Initialize previous positions on first callback
        if self.prev_left_wheel_pos is None or self.prev_right_wheel_pos is None:
            self.prev_left_wheel_pos = current_left_wheel_pos
            self.prev_right_wheel_pos = current_right_wheel_pos
            return  # Skip the rest of the callback to avoid erroneous initial movement

        # Calculate the change in wheel positions
        delta_left_wheel_pos = current_left_wheel_pos - self.prev_left_wheel_pos
        delta_right_wheel_pos = current_right_wheel_pos - self.prev_right_wheel_pos

        # Update the previous positions
        self.prev_left_wheel_pos = current_left_wheel_pos
        self.prev_right_wheel_pos = current_right_wheel_pos

        # Update odometry based on the change in encoder positions
        self.robot.updateOdometry(delta_left_wheel_pos, delta_right_wheel_pos)

        # Publish the odometry information
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'

        self.odom_msg.pose.pose.position.x = self.robot.Pose.x
        self.odom_msg.pose.pose.position.y = self.robot.Pose.y
        self.odom_msg.pose.pose.orientation.z = self.robot.Pose.theta  # Simplified, you may need quaternion
        
        self.odom_publisher.publish(self.odom_msg)
        
        # print for debugging
        self.get_logger().info(f'Position - x: {self.odom_msg.pose.pose.position.x}, y: {self.odom_msg.pose.pose.position.y}, theta: {self.odom_msg.pose.pose.orientation.z}')


    # def odom_request_callback(self, msg):
    #     if msg.data == 1:
    #         self.odom_publisher.publish(self.odom_msg)


class Robot:
    _R_WHEEL = 0.094
    _D_WHEEL = 0.482

    def __init__(self):
        self.Pose = Pose(0, 0, 0)
        self.tol = np.finfo(float).eps

    def updateOdometry(self, left_wheel_pos, right_wheel_pos):
        
        ds_left = self._R_WHEEL * left_wheel_pos
        ds_right = self._R_WHEEL * right_wheel_pos
        ds = (ds_left + ds_right) / 2
        dtheta = (ds_right - ds_left) / self._D_WHEEL
        ct = math.cos(self.Pose.theta)
        st = math.sin(self.Pose.theta)

        if abs(ct) < self.tol:
            ct = 0
        if abs(st) < self.tol:
            st = 0

        self.Pose.x += ds * ct
        self.Pose.y += ds * st
        self.Pose.theta += dtheta


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def main(args=None):

    rclpy.init(args=args)

    odometry = DiabloOdometry()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)

    executor.add_node(odometry)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
