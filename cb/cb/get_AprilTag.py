import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import os
import time
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import String


# ROS 2 node that subscribes to a camera image topic, detects AprilTags,
# publishes their estimated pose via TF, and publishes marker messages for visualization.

class CurrentAprilTag(Node):
    def __init__(self):
        super().__init__('current_apriltag')

        # Subscribe to the image topic from the camera
        self.image_sub = self.create_subscription(Image, '/image_PiCam', self.listener_callback_image, 10)

        # Publisher for marker informations
        self.landmark_pub = self.create_publisher(Marker, '/landmark', 2)

        # TF broadcaster for publishing the pose of the detected AprilTag
        self.tf_broadcaster = TransformBroadcaster(self)

        # CvBridge for converting between ROS image messages and OpenCV images
        self.bridge = CvBridge()

        # Resolve workspace directory from ROS2 package path
        self.dir_parts_ws = get_package_prefix('cb').split(os.sep)
        self.dir_ws = os.sep.join(self.dir_parts_ws[:-2])

        # Expected camera resolution
        self.DIM = (960, 540)

        # AprilTag side length in meters
        self.size_AprilTag = 0.086#0.062#0.1615

        # Locate directory with calibration matrices
        path_matrices_dir = os.path.join(self.dir_ws, 'src/cb_ros2/cb/camera_calib/camera_matrices')
        print(path_matrices_dir)

        if not os.path.isdir(path_matrices_dir):
            self.get_logger().error("Directory of matrices doesn't exist!")
        else:
            matrices_list = os.listdir(path_matrices_dir)

        if len(matrices_list) == 0:
            self.get_logger().error("No camera matrices found!")
        else:
            # Find most recently modified K (intrinsic) and D (distortion) matrices
            max_K = 0
            max_D = 0
            for matrix in matrices_list:
                matrix_time = os.path.getmtime(os.path.join(path_matrices_dir, matrix))
                if "K" in matrix:
                    if matrix_time > max_K:
                        max_K = matrix_time
                        matrix_K_newest = matrix
                elif "D" in matrix:
                    if matrix_time > max_D:
                        max_D = matrix_time
                        matrix_D_newest = matrix

        # Load the newest camera calibration matrices
        self.path_K = os.path.join(path_matrices_dir, matrix_K_newest)
        self.path_D = os.path.join(path_matrices_dir, matrix_D_newest)
        self.matrix_coefficients = np.load(self.path_K)
        self.distortion_coefficients = np.load(self.path_D)

        # Compute undistortion map and new camera matrix
        self.new_camera_matrix, self.RoI = cv2.getOptimalNewCameraMatrix(self.matrix_coefficients,self.distortion_coefficients,self.DIM, 1, self.DIM)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.matrix_coefficients,self.distortion_coefficients,None,self.new_camera_matrix,self.DIM,cv2.CV_16SC2)

        # Prepare AprilTag dictionary and detector parameters
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.arucoParams = cv2.aruco.DetectorParameters_create()


        self.publish_annotated_image = True
        # Publisher to publish images on the '/image' topic
        if self.publish_annotated_image:
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,  # good for video streams
                history=HistoryPolicy.KEEP_LAST,
                depth=10
                )

            self.image_pub = self.create_publisher(Image, '/image_PiCam/annotated', qos)

    def listener_callback_image(self, msg):
        """Callback function that processes incoming image messages."""
        try:
            # Convert the ROS2 image message to an OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Apply undistortion and cropping based on calibration
            frame = self.calib_frame(frame)

            # Optional live view for debugging
            #cv2.imshow("AprilTag Detection", frame)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Proceed with AprilTag detection
        self.detect_AprilTag(frame)

    def calib_frame(self, frame):
        """Applies camera undistortion and cropping to the input frame."""
        frame = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        frame = cv2.undistort(frame, self.matrix_coefficients, self.distortion_coefficients, None, self.new_camera_matrix)
        x, y, w, h = self.RoI
        return frame[y:y+h, x:x+w]

    def detect_AprilTag(self, frame):
        """Detects AprilTags in the input frame and publishes the pose via TF and Marker."""
        try:
            # Detect AprilTag markers in the frame
            corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame,self.arucoDict,parameters=self.arucoParams)
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # savin 2D marker corner points in message Point
            corner_vec = []
            for corner in corners[0][0]:
                point = Point()
                point.x = float(corner[0])
                point.y = float(corner[1])
                point.z = 0.0  # Not used
                corner_vec.append(point)
        except Exception:
            # No marker detected or failed detection
            return

        if len(corners) > 0:
            ids = ids.flatten()  # Flatten ID array

            for (markerCorner, markerID) in zip(corners, ids):
                # Estimate pose of the marker
                (rvec, tvec, _) = cv2.aruco.estimatePoseSingleMarkers(markerCorner,self.size_AprilTag,self.matrix_coefficients,self.distortion_coefficients)

                # Convert rotation vector to quaternion
                self.R_cam_tag, _ = cv2.Rodrigues(rvec)
                r = R.from_matrix(self.R_cam_tag)
                quaternions = r.as_quat()
                translation = tvec[0][0]

                # Create and broadcast a TransformStamped message
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = self.get_clock().now().to_msg()
                transform_stamped.header.frame_id = 'cam_link'
                transform_stamped.child_frame_id = 'marker_link'
                transform_stamped.transform.translation.x = translation[0]
                transform_stamped.transform.translation.y = translation[1]
                transform_stamped.transform.translation.z = translation[2]
                transform_stamped.transform.rotation.x = quaternions[0]
                transform_stamped.transform.rotation.y = quaternions[1]
                transform_stamped.transform.rotation.z = quaternions[2]
                transform_stamped.transform.rotation.w = quaternions[3]
                self.tf_broadcaster.sendTransform(transform_stamped)

                # Create a marker message
                marker_msg = Marker()
                marker_msg.id = int(ids[0])
                marker_msg.header.frame_id = "marker_link"
                marker_msg.header.stamp = self.get_clock().now().to_msg()
                marker_msg.pose.position.x = translation[0]
                marker_msg.pose.position.y = translation[1]
                marker_msg.pose.position.z = translation[2]
                marker_msg.pose.orientation.x = quaternions[0]
                marker_msg.pose.orientation.y = quaternions[1]
                marker_msg.pose.orientation.z = quaternions[2]
                marker_msg.pose.orientation.w = quaternions[3]
                marker_msg.points = corner_vec  # 2D pixel positions

                # Publish the marker
                self.landmark_pub.publish(marker_msg)

                if self.publish_annotated_image:
                     # Draw axes (use proper shapes)
                    try:
                        cv2.drawFrameAxes(
                            frame,
                            self.matrix_coefficients,
                            self.distortion_coefficients,
                            rvec[0][0], tvec[0][0],
                            0.06
                        )
                    except Exception:
                        pass

                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                    
                    cv2.imshow("AprilTag Detection", frame)
                    cv2.waitKey(1)
                    try:
                        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                        img_msg.header.stamp = self.get_clock().now().to_msg()
                        img_msg.header.frame_id = 'cam_link'
                        self.image_pub.publish(img_msg)
                    except Exception as e:
                        self.get_logger().warn(f'Failed to publish annotated image: {e}')



def main(args=None):
    rclpy.init(args=args)
    node = CurrentAprilTag()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
