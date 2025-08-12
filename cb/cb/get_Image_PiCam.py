import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from picamera2 import Picamera2
import cv2
import time
from cv_bridge import CvBridge

# ROS 2 node that captures images from a camera and publishes them to a ROS2 topic

class CurrentImage(Node):
    def __init__(self):
        super().__init__('current_image_PiCam')

        # ---- Parameters (Terminal: ros2 run pkg node --ros-args -p width:=1280 -p height:=720) ----
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('fps', 20)
        self.declare_parameter('af_mode', 0)     # 0: manual 1: auto
        self.declare_parameter('lens_position', 2.5) # manual focus position
        self.declare_parameter('preview', False)
        # ---- Get parameters ---- #
        self.w = int(self.get_parameter('width').value)
        self.h = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.af_mode = int(self.get_parameter('af_mode').value)
        self.lens_pos = float(self.get_parameter('lens_position').value)
        self.preview = bool(self.get_parameter('preview').value)

        # Publisher to publish images on the '/image' topic
        self.image_pub = self.create_publisher(Image, '/image_PiCam', 10)
        # CvBridge object for converting between ROS2 and OpenCV images
        self.bridge = CvBridge()

        self.initialize_camera()

        # If camera initialized successfully, start timer to grab frames
        try:
            _ = self.picam2.capture_array()  # quick sanity check
            period = 1.0 / float(20)
            self.timer = self.create_timer(period, self.timer_get_image)
            self.get_logger().info(f"Capture OK — timer @ {self.fps} Hz started.!!!")
        except Exception as e:
            self.get_logger().error(f"Camera not ready, no timer started: {e}")

    def initialize_camera(self):
        """Initializes the camera and applies configuration settings."""
         # ---- Picamera2 setup ----
        self.picam2 = Picamera2()
        camera_controls = {
        "AfMode": 0,            # start with manual in the config (we'll toggle after start)
        "LensPosition": -1,     # Set focus manually (range depends on lens)
        "FrameDurationLimits": (int(1e6/self.fps), int(1e6/self.fps)),  # Set fixed frame duration (in microseconds) = 30 fps
        }
         # Create video configuration with custom controls
        config = self.picam2.create_video_configuration(
        main={"size": (self.w, self.h), "format": "RGB888"},
        controls=camera_controls
        )

        self.picam2.configure(config)
        self.picam2.start()

        self.get_logger().info("Warming up camera...")
        #self.picam2.set_controls({"AfMode": 1})
        time.sleep(1.0)

        try:
            self.picam2.set_controls({"AfMode": self.af_mode, "LensPosition": self.lens_pos})
        except Exception as e:
            self.get_logger().warn(f"Could not set LensPosition: {e}")

    def timer_get_image(self):
        """Callback function called by the timer to capture and publish a frame."""
        """Capture a frame from Picamera2 and publish (and optionally preview)."""
        try:
            im = self.picam2.capture_array()  # HxWx3 uint8, RGB888
        except Exception as e:
            self.get_logger().warn(f"Picam capture failed: {e}")
            return
        
        # Basic validity check
        if im is not None and im.size > 0:
            frame = cv2.resize(im,(960,540))
        # Publish as ROS Image
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            #img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            img_msg = self.bridge.cv2_to_imgmsg(gray_frame, encoding='mono8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.image_pub.publish(img_msg)

            # Optional preview
            if self.preview:
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("PiCam Preview", bgr)
                cv2.waitKey(1)

    def destroy_node(self):
        try:
            if hasattr(self, "picam2"):
                self.picam2.stop()
                self.picam2.close()
            if self.preview:
                cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CurrentImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
