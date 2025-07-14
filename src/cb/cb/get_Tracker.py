import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_ros import TransformException
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
import math

##Quellen: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html | https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles  ##

class CurrentTracker(Node):
    def __init__(self):
        super().__init__('current_tracker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(Pose, '/pose', 10)
        self.timer = self.create_timer(5.0, self.on_timer)
        self.from_frame_rel = 'tracker'
        self.to_frame_rel = 'map'
        self.from_frame_rel2 = 'tracker'
        self.to_frame_rel2 = 'marker_link'
        self.counter = 0
        self.list_stamp = []

    def quaternion_to_euler(self, qx, qy, qz, qw):
        #Convert quaternion to Euler angles
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

    def on_timer(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.to_frame_rel, self.from_frame_rel, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().info(f'transform failed: {self.from_frame_rel} to {self.to_frame_rel}: {error}')
            return
        
        x_trans = transform.transform.translation.x
        y_trans = transform.transform.translation.y
        z_trans = transform.transform.translation.z
        x_rot = transform.transform.rotation.x
        y_rot = transform.transform.rotation.y
        z_rot = transform.transform.rotation.z
        w_rot = transform.transform.rotation.w

        roll, pitch, yaw = self.quaternion_to_euler(x_rot,y_rot,z_rot,w_rot)

        #self.get_logger().info(f'teta: {yaw}')

        #self.get_logger().info(f'Pose {self.counter}: x: {x_trans}, y: {y_trans}, yaw: {yaw}')
        self.counter += 1
        
        # Translation
        translation = np.array([x_trans, y_trans, z_trans])
        
        # Quaternion: [x, y, z, w]
        quaternion = [x_rot, y_rot, z_rot, w_rot]
        
        # Rotation aus Quaternion
        rotation_matrix = R.from_quat(quaternion).as_matrix()  # 3x3 matrix
        
        # Homogene Transformationsmatrix (4x4)
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = translation*1000
        #transform_matrix = np.round(transform_matrix, 5)
        transform_matrix = np.round(transform_matrix,3)
        np.set_printoptions(precision=3, suppress=True)
        print(transform_matrix)
        
        try:
            transform2 = self.tf_buffer.lookup_transform(self.to_frame_rel2, self.from_frame_rel2, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().info(f'transform failed: {self.from_frame_rel} to {self.to_frame_rel}: {error}')
            return
        
        x_trans_tag = transform2.transform.translation.x
        y_trans_tag = transform2.transform.translation.y
        z_trans_tag = transform2.transform.translation.z
        x_rot_tag = transform2.transform.rotation.x
        y_rot_tag = transform2.transform.rotation.y
        z_rot_tag = transform2.transform.rotation.z
        w_rot_tag = transform2.transform.rotation.w
        
        # Translation
        translation2 = np.array([x_trans_tag, y_trans_tag, z_trans_tag])
        
        # Quaternion: [x, y, z, w]
        quaternion2 = [x_rot_tag, y_rot_tag, z_rot_tag, w_rot_tag]
        
        # Rotation aus Quaternion
        rotation_matrix2 = R.from_quat(quaternion2).as_matrix()  # 3x3 matrix
        
        # Homogene Transformationsmatrix (4x4)
        transform_matrix2 = np.eye(4)
        transform_matrix2[:3, :3] = rotation_matrix2
        transform_matrix2[:3, 3] = translation2*1000
        #transform_matrix = np.round(transform_matrix, 5)
        transform_matrix2 = np.round(transform_matrix2,3)
        print(transform_matrix2)
        
        print("----------------------------------")
        
def main(args=None):
    rclpy.init(args=args)
    node = CurrentTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()