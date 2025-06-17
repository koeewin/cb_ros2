import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import time
from cv_bridge import CvBridge

# ROS 2 node that captures images from a camera and publishes them to a ROS2 topic

class CurrentImage(Node):
    def __init__(self):
        super().__init__('current_image')

        # Publisher to publish images on the '/image' topic
        self.image_pub = self.create_publisher(Image, '/image', 10)

        # Camera source (0 = default webcam)
        self.source = 0

        # VideoCapture object from OpenCV
        self.capture = None

        # Desired resolution for the image stream
        self.resolution = [640, 480]

        # CvBridge object for converting between ROS2 and OpenCV images
        self.bridge = CvBridge()

        # Initialize camera settings
        self.initialize_camera()

        # If camera initialized successfully, start timer to grab frames
        if self.capture.isOpened():
            self.timer = self.create_timer(0.05, self.timer_get_image)

    def initialize_camera(self):
        """Initializes the camera and applies configuration settings."""
        self.capture = cv2.VideoCapture(self.source, cv2.CAP_V4L2)

        # Set camera resolution
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])

        # Optional: Uncomment for MJPG stream to reduce latency
        # self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # Set camera exposure
        self.capture.set(cv2.CAP_PROP_EXPOSURE, 100)

        # Set target FPS
        self.capture.set(cv2.CAP_PROP_FPS, 30)

        # Set buffer size to 1 frame to minimize latency
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def timer_get_image(self):
        """Callback function called by the timer to capture and publish a frame."""
        ret, frame = self.capture.read()

        if ret:
            # Convert OpenCV image to ROS2 Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="passthrough")

            # Publish the image message to the '/image' topic
            self.image_pub.publish(img_msg)

            # Optional for debugging:
            # cv2.imshow("Camera Feed", frame)
            # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
