#!/usr/bin/env python3

"""
@file       vision_tasks.py

@package    vision_tasks
@brief      ROS2 Node for Real-Time Human and AprilTag Detection using OpenCV.

@details    This module defines the VisionTasks ROS2 node, which utilizes OpenCV and custom positioning modules 
            to detect humans and AprilTags in real-time using a connected camera. It operates with multithreaded 
            processing to ensure efficient and parallel detection workflows. The node publishes:
            - Human position as a `Point` message on the `/position/vision` topic.
            - AprilTag landmarks as `Marker` messages on the `/landmark` topic.
            - TF transformations for AprilTags on the `/tf` topic.

@date       2025
"""

import cv2
import rclpy

from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from cb_interfaces.msg import PositioningData

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import threading
import time

# Self-developed modules
try:
    # Relative import within the package
    from .positioning_utils.positioning import *
    from .positioning_utils.positioning_vision_human import *
    from .positioning_utils.positioning_vision_apriltag import *  
except ImportError:
    # Normal import when running directly
    from positioning_utils.positioning import *
    from positioning_utils.positioning_vision_human import *
    from positioning_utils.positioning_vision_apriltag import * 



## The height of the camera measured from the ground
CAMERA_HEIGHT = 0.5 

## Camera source
# 
# Possible values:
# - 0: Default camera (e.g., laptop webcam)
# - 1: External USB camera (if connected)
# - 2: Additional cameras (if available)
CAMERA_SRC = 0

CAMERA_RESOLUTION = (640, 480)


class VisionTasks(Node):
    """        
    @brief Main ROS2 node class that manages vision tasks for human and AprilTag tracking.
    This node class contains:
    - 2 publishers:
        - @ref position_vision_pub 
        - @ref landmark_pub 
    - 1 broadcaster:
        - @ref tf_broadcaster
    """
    
    ##  @name Constructor
    #   @{
    def __init__(self):
        """ 
        @brief Constructor initializing publishers, broadcasters, threads, camera and other instance variables.
        """
        super().__init__('vision_tasks')  

        ##  @name Publishers
        #   @{

        ## Publishes the human position to the `human/positionVision` message topic.
        self.position_vision_pub = self.create_publisher(PositioningData, 'human/positionVision', 10)

        ## Publishes the AprilTag landmarks to the `/landmark` message topic.
        self.landmark_pub = self.create_publisher(Marker, '/landmark', 2)       
        ##  @}    

        ##  @name Broadcasters
        #   @{

        ## Broadcasts the pose of the robot
        self.tf_broadcaster = TransformBroadcaster(self)
        ##  @}    
        
        ##  @name Positioning modules
        #   @{

        ## Positioning class for tracking the human
        self.positioningVisionHuman = PositioningVisionHuman(height_camera=CAMERA_HEIGHT)

        ## Positioning class for tracking the human AprilTag landmarks
        self.positioningVisionAprilTag = PositioningVisionAprilTag()
        ##  @}    

        ##  @name Camera Variables
        #   @{

        ## Video capture object for the camera
        #
        #  Assigned the result of `cv2.VideoCapture(@ref CAMERA_SRC)`. <br>
        #  See [OpenCV-Python Tutorials](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html) for more infomation.
        self.capture = None

        ## Store the captured frame each update.
        self.frame = None   
        ##  @}    

        ##  @name Boolean Flags
        #   @{

        ##  - Turns `False` when a new @ref frame is updated 
        #   - Turns `True` after the new @ref frame is processed for AprilTag detection
        self.apriltag_newframe_processed = True

        ##  - Turns `False` when a new @ref frame is updated 
        #   - Turns `True` after the new @ref frame is processed for Human detection
        self.human_newframe_processed = True
  
        ##  - Turns `False` if no AprilTag is detected 
        #   - Turns `True` if AprilTag is detected
        self.apriltag_detected = False

        ##  - Turns `False` if no human is detected 
        #   - Turns `True` if a human is detected
        self.human_detected = False

        ##  Manually set by user
        #  - `True` for visualization of human and AprilTag detection
        #  - `False` otherwise 
        self.show_ui = True 
        ##  @}    

        if self.show_ui:

            ##  @name Visualization Variables
            #   Only needed if @ref show_ui = `True`
            #   @{

            ## @ref apriltag_thread's frequency string 
            #
            #  Stores the frequency of the @ref apriltag_thread in form of a string
            #  for visualization, only needed when @ref show_ui = `True`
            self.aps_str = ''

            ## @ref human_thread's frequency string 
            #
            #  Stores the frequency of the @ref human_thread in form of a string
            #  for visualization, only needed when @ref show_ui = `True`
            self.hps_str = ''
        	##  @}    

        ##  @name Thread Locks
        #   @{

        ## Thread lock to prevent race condition
        self.lock = threading.Lock()
        ##  @}    

        self.initialize_camera()
        
        ##  @name Thread Sample Times
        #   @{

        ## Sample time for the @update_thread 
        #
        #  Set to 0 because the thread itself some time itself to update the @ref frame
        self.time_sleep_update = 0.0

        ## Sample time for the @ref apriltag_thread
        self.time_sleep_april = 0.01

        ## Sample time for the @human_thread
        self.time_sleep_human = 0.01
        ##  @}    
        
        ##  @name Threads
        #   @{

        ## Frame update thread
        #
        #  Constantly updates new @ref frame from the @ref capture object
        self.update_thread = threading.Thread(target=self.update, args=())
        self.update_thread.start()

        ## AprilTag detection thread
        #
        #  Waits for a new @ref frame to be updated by the @ref update_thread. <br>
        #  Once a new @ref frame is update (i.e. @ref apriltag_newframe_processed = False):
        #  - Performs AprilTag detection and sets @ref apriltag_newframe_processed to True.
        #  - If an AprilTag is detected (i.e. @ref apriltag_detected = True), 
        #    publishes the AprilTag landmark to the `/landmark` message topic.
        self.apriltag_thread = threading.Thread(target=self.apriltag_detection, args=())
        self.apriltag_thread.start()

        ## Human detection thread
        #
        #  Waits for a new @ref frame to be updated by the @ref update_thread. <br>
        #  Once a new @ref frame is update (i.e. @ref human_newframe_processed = False):
        #  - Performs Human detection and sets @ref human_newframe_processed to True.
        #  - publishes the human position to the `/position/vision` message topic.
        self.human_thread = threading.Thread(target=self.human_detection, args=())
        self.human_thread.start()
        ##  @}    
        
    ##   @}

    
    ##  @name Thread Callbacks
    #   Callbacks used by the three main threads: the @ref update_thread, the @ref apriltag_thread and the @ref human_thread.
    #   @{

    def update(self):
        """
        @brief  Continuously retrieves and updates frames from the camera stream in the @ref update_thread thread.

        @details
        This method is intended to run in the @ref update_thread thread. It continuously checks if a camera is available
        and updates internal state flags to indicate new data is available for the human and AprilTag detection threads.

        If the UI is enabled (@ref show_ui = `True`), the method also calculates and smooths the frames-per-second for display, 
        draws human tracking and AprilTag visualization overlays on the frame and displays the frame in an OpenCV window.

        @param self  The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.


        @par Example
        @code
        update_thread = threading.Thread(target=self.update)
        update_thread.start()
        @endcode
        """

        #while self.running and self.capture.isOpened():
        pre_f_time = 0
        fps_history = []

        while self.capture.isOpened():
            new_f_time = time.time() # starting time of the frame
            ret = self.get_frame()

            if ret:
                self.apriltag_newframe_processed = False
                self.human_newframe_processed = False

            if self.show_ui:
                fps = 1/(new_f_time - pre_f_time)
                pre_f_time = new_f_time
                fps = int(fps)
                fps = self.ps_average(fps_history, fps)
                if fps >= 10:
                    fps_str = 'fps:'+str(fps)
                else:
                    fps_str = 'fps:0'+str(fps)

                with self.lock:
                    frame = self.frame.copy()                

                if self.human_detected:
                    self.positioningVisionHuman.draw_tracks_on_frame(frame)

                if self.apriltag_detected:
                    self.positioningVisionAprilTag.draw_on_frame(frame, self.positioningVisionAprilTag.corners, self.positioningVisionAprilTag.ids)

                # print fpx
                frame = cv2.putText(frame,fps_str,(7,70),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
                frame = cv2.putText(frame,self.aps_str,(7,110),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
                frame = cv2.putText(frame,self.hps_str,(7,150),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
                cv2.namedWindow('frame', cv2.WND_PROP_FULLSCREEN)
                cv2.imshow("frame", frame)


                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    cv2.destroyAllWindows()
                    break

            time.sleep(self.time_sleep_update)


    def apriltag_detection(self):
        """
        @brief  Continuously performs AprilTags detection on new video frames in the @ref apriltag_thread thread.

        @details
        This method is intended to run in the @ref apriltag_thread thread. It constantly checks 
        if a new frame has been made available (by checking if the @ref apriltag_newframe_processed flag is `False`). If a new frame is present, 
        it performs AprilTag detection using the @ref positioningVisionAprilTag module. 
        Upon successful detection, it retrieves the tag's pose and corner coordinates, then:

        - Publishes a transform between the camera and the tag using @ref broadcast_tf.
        - Publishes a custom marker message using @ref publish_landmark.

        If the UI is enabled (@ref show_ui = `True`), the method also calculates and smooths the looping frequency in Hz 
        (loops per second), and updates the @ref aps_str string for visualization purposes.

        @param self  The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.


        @par Example
        @code
        apriltag_thread = threading.Thread(target=self.apriltag_detection)
        apriltag_thread.start()
        @endcode
        """
        pre_a_time = 0
        aps_history = []

        while True:
            new_a_time = time.time() # starting time of the frame

            if not self.apriltag_newframe_processed: 
                with self.lock:
                    frame = self.frame.copy()
                self.apriltag_detected = self.positioningVisionAprilTag.detect_AprilTag(frame)
                self.apriltag_newframe_processed = True

                if not self.apriltag_detected:
                    self.get_logger().info("No AprilTag detected!")
                    
                    if self.show_ui:
                        aps = 1/(new_a_time - pre_a_time)
                        pre_a_time = new_a_time
                        aps = int(aps)
                        aps = self.ps_average(aps_history, aps)
                        if aps >= 10:
                            self.aps_str = 'AprilTag:'+str(aps)
                        else:
                            self.aps_str = 'AprilTag:0'+str(aps)
                    
                    time.sleep(self.time_sleep_april) 
                    
                    continue
                
                if self.positioningVisionAprilTag.corners is not None and len(self.positioningVisionAprilTag.corners) > 0:
                    corner_vec = list()
                    for corner in self.positioningVisionAprilTag.corners[0][0]:
                        point = Point()
                        point.x = float(corner[0])
                        point.y = float(corner[1])
                        point.z = 0.0 #irrelevant
                        corner_vec.append(point)

                        quaternion = self.positioningVisionAprilTag.quaternion
                        translation = self.positioningVisionAprilTag.translation
                        ids = self.positioningVisionAprilTag.ids

                    self.broadcast_tf(quaternion,translation)
                    self.publish_landmark(quaternion, translation, int(ids[0]), corner_vec) #TODO: always publish first marker?
            
            if self.show_ui:
                aps = 1/(new_a_time - pre_a_time)
                pre_a_time = new_a_time
                aps = int(aps)
                aps = self.ps_average(aps_history, aps)
                if aps >= 10:
                    self.aps_str = 'AprilTag:'+str(aps)
                else:
                    self.aps_str = 'AprilTag:0'+str(aps)

            time.sleep(self.time_sleep_april)

    def human_detection(self):
        """
        @brief  Continuously performs human detection on new video frames in the @ref human_thread thread.

        @details
        This method is intended to run in the @ref human_thread thread. It constantly checks 
        if a new frame has been made available (by checking if the @ref human_newframe_processed flag is `False`). If a new frame is present, 
        it performs human detection using the @ref positioningVisionHuman module, estimates the human's angle and distance,
        and publishes the resulting position as a `geometry_msgs.msg.Point` to the `/position/vision` message topic.

        If the UI is enabled (@ref show_ui = `True`), the method also calculates and smooths the looping frequency in Hz 
        (loops per second), and updates the @ref hps_str string for visualization purposes.

        @param self  The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.

        @par Example
        @code
        human_thread = threading.Thread(target=self.human_detection)
        human_thread.start()
        @endcode
        """

        pre_h_time = 0
        hps_history = []
        while 1:

            new_h_time = time.time() # starting time of the frame

            if not self.human_newframe_processed:
                with self.lock:
                    frame = self.frame.copy()
                self.human_detected = self.positioningVisionHuman.estimate_angle_and_distance(frame)
                self.human_newframe_processed = True
            
                vision_msg = PositioningData()
                vision_msg.x = float(self.positioningVisionHuman.get_x())
                vision_msg.y = float(self.positioningVisionHuman.get_y())
                vision_msg.angle = float(self.positioningVisionHuman.get_angle())
                vision_msg.distance = float(self.positioningVisionHuman.get_distance())
                self.position_vision_pub.publish(vision_msg)
            
            if self.show_ui:
                hps = 1/(new_h_time - pre_h_time)
                pre_h_time = new_h_time
                hps = int(hps)
                hps = self.ps_average(hps_history, hps)
                if hps >= 10:
                    self.hps_str = 'Human:'+str(hps)
                else:
                    self.hps_str = 'Human:0'+str(hps)
            
            time.sleep(self.time_sleep_human) 
    ##  @}

    ##  @name Camera Utils
    #   Utility methods used for the camera
    #   @{
    def initialize_camera(self):
        """ 
        @brief Initializes the camera and changes the resolution.
        @param self The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.
        """
        self.capture = cv2.VideoCapture(CAMERA_SRC)
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
        #self.capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        #self.capture.set(cv2.CAP_PROP_EXPOSURE, 300) #200 d 600 for night
        self.capture.set(cv2.CAP_PROP_FPS, 20)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE,1)

    def get_frame(self):
        """ 
        @brief Get new @ref frame from the @ref capture object, this method is called inside the @ref update_thread
        @param self The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.
        """
        with self.lock:
            ret, self.frame = self.capture.read()

        return ret
    
    ##  @}


    ##  @name ROS2 Utils
    #   Utility methods used for ROS2 applications
    #   @{
    def broadcast_tf(self, quaternion, translation):
        """ 
        @brief  Method called by the @ref apriltag_detection to publish transformation updates on the `/tf` message topic.    

        @param  self        The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.
        @param  quaternion  A quaternion [x, y, z, w] representing the rotation from the camera frame to the marker frame. <br>
        @param  translation A 3D translation vector [x, y, z] representing the position of the marker relative to the camera.
        
        @par Example
        @code
        self.broadcast_tf([0, 0, 0.7071, 0.7071], [1.0, 0.0, 0.5])
        @endcode

        @note 
        - The transform is published on the `/tf` topic using the `TransformStamped` message.
        - The frames `cam_link` and `marker_link` defined in the code should correspond to the coordinate frames defined in your system.
        """

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'cam_link'
        transform_stamped.child_frame_id = 'marker_link'
        transform_stamped.transform.translation.x = translation[0]
        transform_stamped.transform.translation.y = translation[1]
        transform_stamped.transform.translation.z = translation[2]
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform_stamped)

    def publish_landmark(self, quaternion, translation, ID, corner_vec):
        """ 
        @brief  Method called by the @ref apriltag_detection to publish Apriltag updates on the `/landmark` message topic.    

        @param  self        The instance of the @ref VisionTasks class, used to access its attributes and methods from within the class.
        @param  quaternion  A quaternion [x, y, z, w] representing the rotation from the camera frame to the marker frame. 
        @param  translation A 3D translation vector [x, y, z] representing the position of the marker relative to the camera.
        @param  ID          ID of the AprilTag marker.
        @param  corner_vec  A list of geometry_msgs.msg.Point objects representing the 2D image-plane coordinates of the four corners 
                            of a detected AprilTag, with a fixed z-value of 0.0.

        
        @par Example
        @code
        from geometry_msgs.msg import Point

        corner_vec = [
                    Point(x=320.5, y=240.5, z=0.0),
                    Point(x=400.5, y=240.5, z=0.0),
                    Point(x=400.5, y=300.5, z=0.0),
                    Point(x=320.5, y=300.5, z=0.0)
                    ]
        @endcode

        @note 
        - The frame `marker_link` defined in the code should correspond to the coordinate frames defined in your system.
        """
        marker_msg = Marker()
        marker_msg.id = ID
        marker_msg.header.frame_id = "marker_link"
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.pose.position.x = translation[0]
        marker_msg.pose.position.y = translation[1]
        marker_msg.pose.position.z = translation[2]
        marker_msg.pose.orientation.x = quaternion[0]
        marker_msg.pose.orientation.y = quaternion[1]
        marker_msg.pose.orientation.z = quaternion[2]
        marker_msg.pose.orientation.w = quaternion[3]
        marker_msg.points = corner_vec

        self.landmark_pub.publish(marker_msg)
    ##  @}

    ##  @name Visualization Utils
    #   Utility methods for visualization, run when @ref show_ui = True.
    #   @{
    def ps_average(self, ps_history, current_ps_estimate):
        """
        @brief  Computes the moving average of pose estimation confidence values (or similar scalar estimates).

        @details
        This method maintains a history buffer of the most recent 30 pose-related scalar values (e.g., estimation confidence,
        probability scores, etc.). Each time it is called, the current value is added to the buffer and the oldest value 
        is discarded if the buffer exceeds 30 entries. It then returns the integer mean of the values in the buffer.

        @param  self                  The instance of the class where this method is defined.
        @param  ps_history            A list storing the previous scalar estimates. This list is modified in-place.
        @param  current_ps_estimate  The most recent scalar estimate (e.g., a confidence score or a prediction quality value).

        @return The integer average of the scalar values stored in the buffer (ps_history).

        @note
        - This function ensures that the history is limited to the latest 30 entries.
        - The result is cast to an integer using `int()` for downstream processing or threshold comparisons.
        
        @par Example
        @code
        ps_history = []
        current_value = 85
        avg = self.ps_average(ps_history, current_value)
        @endcode
        """
        ps_history.append(current_ps_estimate)
        if len(ps_history) > 30:
            ps_history.pop(0)
        return int(np.mean(ps_history))
    ##  @}

def main(args=None):
    rclpy.init(args=args)
    node = VisionTasks()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
      
        

if __name__ == '__main__':
    main()