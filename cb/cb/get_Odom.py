import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from motion_msgs.msg import LegMotors
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
import math

class DiabloOdometry(Node):
    def __init__(self):
        super().__init__('diablo_odometry')
        self.subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.listener_callback_encoder, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.robot = Robot()  # Initialize the robot instance here

        # Initialize previous wheel positions
        self.prev_left_wheel_pos = None
        self.prev_right_wheel_pos = None



    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to a quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def listener_callback_encoder(self, msg):
        left_wheel_enc_rev = msg.left_wheel_enc_rev
        right_wheel_enc_rev = msg.right_wheel_enc_rev

        left_wheel_vel = msg.left_wheel_vel * self.robot._R_WHEEL
        right_wheel_vel = msg.right_wheel_vel * self.robot._R_WHEEL

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
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.robot.Pose.x 
        odom_msg.pose.pose.position.y = self.robot.Pose.y 

        # Convert yaw (theta) to a quaternion for odometry message
        quaternion = self.euler_to_quaternion(0, 0, self.robot.Pose.theta)  # Roll, Pitch, Yaw
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Set linear velocity (forward velocity in m/s) and angular velocity (rotation rate in rad/s)
        odom_msg.twist.twist.linear.x = (left_wheel_vel + right_wheel_vel) / 2
        odom_msg.twist.twist.angular.z = (right_wheel_vel - left_wheel_vel) / self.robot._D_WHEEL

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Now broadcast the transform between odom and base_link
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'

        # Set the translation (position) of base_link relative to odom
        transform_stamped.transform.translation.x = self.robot.Pose.x
        transform_stamped.transform.translation.y = self.robot.Pose.y
        transform_stamped.transform.translation.z = 0.0  # Assuming planar motion

        # Set the rotation (orientation) of base_link relative to odom (as a quaternion)
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform_stamped)

        #position = odom_msg.pose.pose.position
        # self.get_logger().info(f'Position - x: {odom_msg.twist.twist.linear.x}, y: {position.y}, z: {odom_msg.twist.twist.angular.z}')


class Robot:
    _PI = 3.14159265359
    _R_WHEEL = 0.095
    _D_WHEEL = 0.483

    def __init__(self):
        self.Pose = Pose(0, 0, 0)

    def updateOdometry(self, delta_left_wheel_pos, delta_right_wheel_pos):
        ds_left = self._R_WHEEL * delta_left_wheel_pos
        ds_right = self._R_WHEEL * delta_right_wheel_pos
        ds = (ds_left + ds_right) / 2
        dtheta = (ds_right - ds_left) / self._D_WHEEL
        self.Pose.x += ds * math.cos(self.Pose.theta)
        self.Pose.y += ds * math.sin(self.Pose.theta)
        self.Pose.theta += dtheta


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


def main(args=None):
    rclpy.init(args=args)
    node = DiabloOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
