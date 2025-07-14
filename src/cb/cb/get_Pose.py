import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import Pose
import math

# Quellen:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

class CurrentPose(Node):
    def __init__(self):
        super().__init__('current_pose')

        # Initialize the TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for the current pose
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)

        # Timer to repeatedly fetch and publish pose
        self.timer = self.create_timer(0.05, self.timer_get_pose)

        # Frame names for transformation
        self.from_frame_rel = 'base_link'
        self.to_frame_rel = 'map'

        
        self.counter = 0
        self.list_stamp = []

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        t1_x = 2 * ((qw * qx) + (qy * qz))
        t2_x = 1 - (2 * ((qx * qx) + (qy * qy)))
        roll = math.atan2(t1_x, t2_x)

        t1_y = math.sqrt(1 + (2 * ((qw * qy) - (qx * qz))))
        t2_y = math.sqrt(1 - (2 * ((qw * qy) - (qx * qz))))
        pitch = (2 * math.atan2(t1_y, t2_y)) - (math.pi / 2)

        t1_z = 2 * ((qw * qz) + (qx * qy))
        t2_z = 1 - (2 * ((qy * qy) + (qz * qz)))
        yaw = math.atan2(t1_z, t2_z)

        return roll, pitch, yaw

    def timer_get_pose(self):
        """Timer callback to get and publish the current pose from TF."""
        try:
            # Lookup the latest transform between frames
            transform = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_frame_rel,
                rclpy.time.Time())
        except TransformException as error:
            # Log failure and return
            self.get_logger().info(
                f'transform failed: {self.from_frame_rel} to {self.to_frame_rel}: {error}')
            return

        # Extract translation
        x_trans = transform.transform.translation.x
        y_trans = transform.transform.translation.y
        z_trans = transform.transform.translation.z

        # Extract rotation as quaternion
        x_rot = transform.transform.rotation.x
        y_rot = transform.transform.rotation.y
        z_rot = transform.transform.rotation.z
        w_rot = transform.transform.rotation.w

        # Optional: convert to Euler angles if needed for debugging
        # roll, pitch, yaw = self.quaternion_to_euler(x_rot, y_rot, z_rot, w_rot)
        # self.get_logger().info(f'Pose {self.counter}: x: {x_trans}, y: {y_trans}, yaw: {yaw}')

        self.counter += 1

        # Fill the pose message
        pose_msg = Pose()
        pose_msg.position.x = x_trans
        pose_msg.position.y = y_trans
        pose_msg.position.z = z_trans
        pose_msg.orientation.x = x_rot
        pose_msg.orientation.y = y_rot
        pose_msg.orientation.z = z_rot
        pose_msg.orientation.w = w_rot

        # Publish the pose message
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
