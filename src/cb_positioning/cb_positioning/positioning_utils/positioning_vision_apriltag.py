"""
@file       positioning_vision_apriltag.py
@package    positioning_vision_apriltag
@brief      AprilTags detection with OpenCV's ArUco marker utilities

@details    This module provides the PositioningVisionAprilTag class, which implements vision-based 
            positioning using AprilTags and OpenCV's ArUco marker utilities. It is designed to work 
            within a ROS2-based system for detecting the spatial pose (position and orientation) 
            of AprilTags in a camera feed.

            @par Features:
            - Loads the latest camera calibration matrices (intrinsics and distortion) from a specified directory.
            - Undistorts input frames based on calibration data.
            - Detects AprilTags using the 36h11 dictionary.
            - Estimates the pose (rotation and translation) of detected tags.
            - Converts rotation matrices to quaternions for robotics applications.
            - Annotates the frame with marker ID, pose axes, and other visual aids.

            @par Usage:
            The PositioningVisionAprilTag class provided in this module is typically used as part of a larger localization or navigation system.
            You can use the `detect_AprilTag()` method on each video frame to obtain pose estimates,
            and `draw_on_frame()` to visualize the results.
            @code
            from positioning.positioning_vision_apriltag import *

            tag_detector = PositioningVisionAprilTag()
            detected = tag_detector.detect_AprilTag(frame)
            tag_detector.draw_on_frame(frame, tag_detector.corners, tag_detector.ids)
            @endcode

            @par Dependencies:
            - OpenCV (cv2) with aruco module
            - NumPy
            - SciPy

            @par Class:
            PositioningVisionAprilTag ( Positioning ):
                A subclass of `Positioning` that encapsulates AprilTag-based localization logic.

"""

import cv2
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
try:
    from .positioning import Positioning  # Relative import within the package
except ImportError:
    from positioning import Positioning  # Normal import when running directly

from ament_index_python.packages import get_package_prefix

import time
import numpy as np
#from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation as R

import systemConfig

BASE_PATH = systemConfig.Base_path  # Linux example

# Define other paths relative to BASE_PATH
CALIBRATION_PATH = os.path.join(BASE_PATH, "camera-parameters-apriltag")

class PositioningVisionAprilTag(Positioning):    
    """
     AprilTag-based vision positioning using OpenCV ArUco utilities.
    """

    def __init__(self):
        """
        Constructor initializes camera parameters and sets up ArUco detector.
        """
        super().__init__()

        # self.dir_parts_ws = get_package_prefix('carrierbot').split(os.sep)
        # self.dir_ws = os.sep.join(self.dir_parts_ws[:-2])

        self.DIM = (640, 480)
        self.size_AprilTag = 0.1615  # meters

        self.corners = None
        self.quaternion = None
        self.translation = None

        self.apriltag_detected = False

        # find newest matrices
        if os.path.isdir(CALIBRATION_PATH) == 0:
            print("Directory of matrices doesn't exist!")
        else:
            matrices_list = os.listdir(CALIBRATION_PATH)
        if len(matrices_list) == 0:
            print("No maps found!")
        else:
            max_K = 0
            max_D = 0
            for matrix in matrices_list:
                matrix_time = os.path.getmtime(os.path.join(CALIBRATION_PATH,matrix))
                if "K" in matrix:
                    if matrix_time > max_K:
                        max_K = matrix_time
                        matrix_K_newest = matrix
                elif "D" in matrix:
                    if matrix_time > max_D:
                        max_D = matrix_time
                        matrix_D_newest = matrix

        # load newest matrices
        self.path_K = os.path.join(CALIBRATION_PATH,matrix_K_newest)
        self.path_D = os.path.join(CALIBRATION_PATH,matrix_D_newest)
        self.matrix_coefficients = np.load(self.path_K)
        self.distortion_coefficients = np.load(self.path_D)

        self.new_camera_matrix, self.RoI = cv2.getOptimalNewCameraMatrix(self.matrix_coefficients,self.distortion_coefficients,self.DIM,1,self.DIM)
        self.map1, self.map2 = cv2.initUndistortRectifyMap(self.matrix_coefficients,self.distortion_coefficients,None,self.new_camera_matrix,self.DIM,cv2.CV_16SC2)

        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

    def calib_frame(self, frame):
        """
        @brief Applies calibration to undistort the input frame.
        @param self  The instance of the @ref PositioningVisionAprilTag class, used to access its attributes and methods from within the class.
        @param frame The input camera frame.
        @return Undistorted and cropped frame.
        """
        frame = cv2.remap(frame,self.map1,self.map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        frame = cv2.undistort(frame,self.matrix_coefficients,self.distortion_coefficients,None,self.new_camera_matrix)
        x,y,w,h = self.RoI
        frame = frame[y:y+h, x:x+w]

        return frame

    
    def detect_AprilTag(self, frame):
        """
        @brief Detects AprilTags in the provided frame and estimates pose.
        @param self  The instance of the @ref PositioningVisionAprilTag class, used to access its attributes and methods from within the class.
        @param frame The input frame from a calibrated camera.
        @return True if tags are detected, otherwise False.
        """
        apriltag_detected = False
        frameAprilTag = self.calib_frame(frame)

        try:
            self.corners, self.ids,_ = cv2.aruco.detectMarkers(frameAprilTag, self.arucoDict, parameters = self.arucoParams)

            if len(self.corners) > 0:    
                apriltag_detected = True    
                # flatten the ArUco IDs list
                self.ids = self.ids.flatten()
                print(self.ids)
                
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(self.corners, self.ids):
                    # pose estimation
                    (self.rvec, self.tvec, markerPoints) = cv2.aruco.estimatePoseSingleMarkers(markerCorner, self.size_AprilTag, self.matrix_coefficients, self.distortion_coefficients)
                
                    R_cam_tag, _ = cv2.Rodrigues(self.rvec)
                    r = R.from_matrix(R_cam_tag)
                    self.quaternion = r.as_quat()
                    self.translation = self.tvec[0][0] 
            
        except Exception as e:
            return apriltag_detected

        return apriltag_detected
    
    def draw_on_frame(self, frame, corners, ids):   
        """ 
        @brief Draws pose and ID annotations on the detected AprilTag in the frame.
        @param self  The instance of the @ref PositioningVisionAprilTag class, used to access its attributes and methods from within the class.
        @param frame The frame to draw on.
        @param corners Detected corners of the AprilTags.
        @param ids Detected IDs of the AprilTags.
        """
        for (markerCorner, markerID) in zip(corners, ids): 
            frame = cv2.drawFrameAxes(frame, self.matrix_coefficients, self.distortion_coefficients, self.rvec, self.tvec, 0.02)
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            (topLeft, topRight, bottomRight, bottomLeft) =  markerCorner.reshape((4, 2))
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            frame = cv2.rectangle(frame, topLeft, bottomRight, (0, 255, 0), 2)


            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            
            # draw the ArUco marker ID on the frame
            ID_str = 'ID:' + str(markerID)
            frame = cv2.putText(frame, ID_str,
                        (bottomRight[0]-60, bottomRight[1] ),
            cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 2)

            # rotation_matrix = np.array([[0, 0, 0, 0],
            #                             [0, 0, 0, 0],
            #                             [0, 0, 0, 0],
            #                             [0, 0, 0, 1]],
            #                             dtype=float)
            # rotation_matrix[:3, :3], _ = cv2.Rodrigues(self.rvec)

            # rotation_matrix[0][3] = self.tvec[0][0][0]
            # rotation_matrix[1][3] = self.tvec[0][0][1]
            # rotation_matrix[2][3] = self.tvec[0][0][2]
            # rotation_matrix[3][0] = np.double(0)
            # rotation_matrix[3][1] = np.double(0)
            # rotation_matrix[3][2] = np.double(0)
            # rotation_matrix[3][3] = np.double(1)

            # T_str = np.array2string(rotation_matrix, precision = 2,  sign='+' ,floatmode='maxprec_equal')
            # T_str= T_str.replace('[','',).replace(']','')
            # dy = 25
            # for i, txt in enumerate (T_str.split('\n')):
            #     if i == 0:
            #         y = bottomLeft[1] -55 + i*dy
            #         txt = ' '+txt
            #         frame = cv2.putText(frame, txt,
            #         (bottomRight[0], y),
            #         cv2.FONT_HERSHEY_PLAIN,
            #         1.5, (0, 255, 255), 2)
                    
            #     y = bottomLeft[1] -55 + i*dy        
            #     frame = cv2.putText(frame, txt,
            #         (bottomRight[0], y),
            #         cv2.FONT_HERSHEY_PLAIN,
            #         1.5, (0, 255, 255), 2)
            #     rotation_matrix = rotation_matrix.flatten()

            px = int(self.matrix_coefficients[0,2])
            py = int(self.matrix_coefficients[1,2])
            frame = cv2.arrowedLine(frame, (px,py), (px+100,py), (0, 0, 255), 2,tipLength=0.15)
            frame = cv2.putText(frame,'X',(px+110,py+20),cv2.FONT_HERSHEY_PLAIN,2,(0, 0, 255),2)
            frame = cv2.arrowedLine(frame, (px,py), (px,py+100), (0, 255, 0), 2,tipLength=0.15)
            frame = cv2.putText(frame,'Y',(px-30,py+120),cv2.FONT_HERSHEY_PLAIN,2,(0, 255, 0),2)

    
        
        





