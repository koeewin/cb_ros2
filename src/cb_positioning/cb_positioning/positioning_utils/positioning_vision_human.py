"""
@file       positioning_vision_human.py

@package    positioning_vision_human
@brief      Human detection, tracking, and position Estimation with Coral Edge TPU

@details    This module provides the PositioningVisionHuman class, which implements a real-time computer vision pipeline for detecting, tracking, 
            and estimating the position of humans using a calibrated fisheye camera and a Coral Edge TPU.

            @par Features:
            - Uses a quantized MobileDet SSD model accelerated by the Edge TPU for object detection.
            - Tracks detected humans across frames using the SORT (Simple Online Realtime Tracking) algorithm.
            - Computes the 3D position (X, Z) of detected individuals based on the camera's intrinsic parameters.
            - Estimates:
                - Horizontal distance (Z)
                - Lateral displacement (X)
                - Distance to target (in meters)
                - Relative angle (in radians)
            - Applies an affine transformation for additional calibration correction.
            - Supports real-time visualization.

            @par Usage:
            - Intended for embedded or robotic systems where tracking human position relative to the camera is essential.
            @code
            from positioning.positioning_vision_human import *

            human_detector = PositioningVisionHuman()
            detected = human_detector.estimate_angle_and_distance(frame)
            human_detector.draw_tracks_on_frame(frame)
            @endcode

             @par Dependencies:
            - Coral Edge TPU and PyCoral
            - OpenCV (with fisheye support)
            - NumPy, SciPy, and SORT tracking module
            - Pre-calibrated camera intrinsics (K, D) stored as .npy files

            @par Class:
            PositioningVisionHuman ( Positioning ):
                A subclass of Positioning that for vision-based human tracking.
"""

import argparse
import cv2
import serial
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from collections import deque
import statistics
try:
    from .positioning import Positioning  # Relative import within the package
except ImportError:
    from positioning import Positioning  # Normal import when running directly
try:
    from .sort import *
except ImportError:
    from sort import *

from pycoral.utils.edgetpu import list_edge_tpus
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.utils.edgetpu import run_inference
from pycoral.adapters.common import input_size
from pycoral.adapters.common import set_resized_input
from pycoral.adapters.common import input_details
from pycoral.adapters.detect import get_objects

import time
import numpy as np
#from ament_index_python.packages import get_package_prefix
from scipy.spatial.transform import Rotation as R

import systemConfig

#BASE_PATH = "C:/Users/zhang/Desktop/human-position-publisher/positioning"
BASE_PATH = systemConfig.Base_path  # Linux example

# Define other paths relative to BASE_PATH
LABEL_PATH = os.path.join(BASE_PATH, "coral-models", "coco_labels.txt")
MODEL_PATH = os.path.join(BASE_PATH, "coral-models", "ssdlite_mobiledet_coco_qat_postprocess_edgetpu.tflite")
CALIBRATION_PATH = os.path.join(BASE_PATH, "camera-parameters-human")


WINDOW_SIZE = 10


def load_model():
        """
        @brief Loads the Edge TPU model for inference.
        @return An initialized Edge TPU interpreter.
        """
        if list_edge_tpus():
            # initialize tf interpreter
            interpreter = make_interpreter(MODEL_PATH)
            interpreter.allocate_tensors()
            return interpreter
        else:
            print("No USB Accelerator connected!")
            exit()

def filter_object(objectI):
        """
        @brief Filters object detection results to retain only humans.
        @param objectI A detected object.
        @return True if the object is a human; False otherwise.
        """
        if objectI.id == 0:
            return True
        
        return False

def limit(value, lower_limit, upper_limit):
        """
        @brief Clamps a value between a lower and upper limit.
        @param value The input value.
        @param lower_limit Minimum bound.
        @param upper_limit Maximum bound.
        @return Clamped value.
        """
        if value < lower_limit:
            return lower_limit
        elif value > upper_limit:
            return upper_limit
        
        return value

class PositioningVisionHuman(Positioning):
    """
    @brief Class for human detection, tracking, and 3D localization using vision and Coral TPU.
    @details Uses a quantized MobileDet SSD with Edge TPU and SORT for tracking. Reprojects pixel coordinates 
    to real-world positions and outputs relative angle and distance of tracked person.
    """

    labels = read_label_file(LABEL_PATH)
    K = np.load(glob.glob(os.path.join(CALIBRATION_PATH, 'K*.npy'))[0])	# camera matrix
    D = np.load(glob.glob(os.path.join(CALIBRATION_PATH, 'D*.npy'))[0])	# distortion coefficients

    interpreter = load_model()
    inference_size = input_size(interpreter)
    #inference_size = (320, 320) # for testing purposes, use fixed size
    

    # Transformation matrix (affine transform) for additional correction
    A = np.array([[1.02525856, 0.03633815, -0.00136207], [-0.03460797, 1.03667567, -0.14860082]])

    def __init__(self, height_camera, **options):
        """
        @brief Initializes the PositioningVisionHuman instance.
        @param height_camera Camera's mounting height.
        @param options Optional parameters for detection thresholds, resolution, etc.
        """
        super().__init__()

        self.human_detected = False

        # Required attributes
        self.height_camera = height_camera

        # Optional attributes for function get_object 
        self.get_object_threshold = options.get('get_object_threshold', 0.36)
        self.get_object_top_k = options.get('get_object_top_k', 3)

        # Create Tracker
        self.tracker = Sort(
            options.get('tracker_max_age', 5), options.get('tracker_min_hits', 3), options.get('tracker_iou_threshold', 0.3)
            )

        self.resolution = options.get('resolution', (640, 480))

        self.scale = (self.resolution[0] / PositioningVisionHuman.inference_size[0],
                    self.resolution[1] / PositioningVisionHuman.inference_size[1])


        # camera matrix used after undistortion of image
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            PositioningVisionHuman.K, PositioningVisionHuman.D, self.resolution, np.eye(3), options.get('balance', 1)
            )    
        
        # mapping constants for undistortion of camera image
        self.mapx, self.mapy = cv2.fisheye.initUndistortRectifyMap(
            PositioningVisionHuman.K, PositioningVisionHuman.D, np.eye(3), new_K, self.resolution, cv2.CV_16SC2
            )    
        
        # Extract values from camera matrix for reprojection (calculation of 3d point in world coordinates from pixel coordinates)
        self.f_x = new_K[0,0]	# focal length in x [pixel]
        self.f_y = new_K[1,1]	# focal length in y [pixel]
        self.c_x = new_K[0,2]	# x coordinate of optical center [pixel] 
        self.c_y = new_K[1,2]	# y coordinate of optical center [pixel]

        # framerate time
        self.pre_f_time = 0
        self.new_f_time = 0

        #inference time
        self.inference_start = 0
        self.inference_stop = 0

        self.human_objects = None
        self.capture = None
        self.detections = None
        self.frameHuman = None
        self.inference_frame = None
         
        self.fps_history = []

        self.tracks = None
        self.matched_tracks = None

        self.x_history = deque([], WINDOW_SIZE)
        self.z_history = deque([], WINDOW_SIZE)

        # M is the point on lower edge of bbox, it is centered horizontally on this edge
        self.uM = 0.0 # horizontal center of bbox
        self.vM = 0.0


    def format_for_mot(self):
        """
        @brief Formats the detected human objects for input to the SORT tracker.
        """
        self.detections = np.empty((0, 5))

        for obj in self.human_objects:
            element = np.array([obj.bbox.xmin, obj.bbox.ymin, obj.bbox.xmax,
                                obj.bbox.ymax, obj.score]).reshape((1, 5))
            self.detections = np.append(self.detections, element, axis=0)

    

    def match_objects_with_tracks(self):
        """
            Matches objects to tracks by comparing bounding box intersection.
            This is used to reassign object id and score to the track, which is lost during mot_tracker.update().
        """
        self.matched_tracks = np.empty((0, 7))
        
        for track in self.tracks:
            t_x0 = track[0].item()
            t_y0 = track[1].item()
            t_x1 = track[2].item()
            t_y1 = track[3].item()
            t_id = track[4].item()
            area_track = (t_x1 - t_x0) * (t_y1 - t_y0)
            iou_max = 0

            # find object corresponding to track by calculating intersection over union
            for obj in self.human_objects:
                o_x0 = obj.bbox.xmin
                o_y0 = obj.bbox.ymin
                o_x1 = obj.bbox.xmax
                o_y1 = obj.bbox.ymax
                area_object = (o_x1 - o_x0) * (o_y1 - o_y0)
                area_overlap = (min(o_x1, t_x1) - max(o_x0, t_x0)) * (min(o_y1, t_y1) - max(o_y0, t_y0))
                area_union = area_track + area_object - area_overlap
                iou = area_overlap / area_union
                if iou > iou_max:
                    iou_max = iou
                    o_id = obj.id
                    o_score = obj.score

            # create array and append to matched tracks
            match = np.array([t_x0, t_y0, t_x1, t_y1, t_id, o_id, o_score]).reshape((1, 7))
            self.matched_tracks = np.append(self.matched_tracks, match, axis=0)



    def draw_tracks_on_frame(self, frame):
        """
        @brief Draws bounding boxes and info (distance, angle) for tracked objects on the frame.
        @param frame The current video frame.
        """
        
        if self.matched_tracks is not None and self.matched_tracks.any():
            if not self.matched_tracks.size == 0:
                track_id = self.matched_tracks[:,4].copy()
                follow_id = track_id.min().copy()

        #         print('tracks: ',tracks)
        #         print('track size = ',np.size(tracks,axis=0))
        #         print('track id = ',track_id)
        #         print('follow id = ', follow_id)

                for track in self.matched_tracks:
                    track = track.copy()
                    # format box text
                    print('id current =', track[4].item())
                    print(track[4].item()== follow_id)
                    #print('Follow ID = ', follow_id)
                    if track[4].item() == follow_id:
                        score = track[6].item()
                        object_id = track[5].item()
                        label = PositioningVisionHuman.labels.get(object_id, object_id)
                        track_id = int(track[4].item())
                        percent = int(100 * score)
                        box_text = '{} {}: {}%'.format(label, track_id, percent)
                        # distance_text =  "{:.0f} cm".format(int(distance/10))
                        distance_text = "{:.2f} m".format(self._distance)
                        try:
                            angle_text = "{:.2f} deg".format(self._angle.item()/np.pi*180)
                        except:
                            angle_text = "{:.2f} deg".format(self._angle/np.pi*180)

                        # scale points to resolution size
                        track[0] *= self.scale[0]
                        track[1] *= self.scale[1]
                        track[2] *= self.scale[0]
                        track[3] *= self.scale[1]

                        x0 = int(track[0].item())
                        y0 = int(track[1].item())
                        x1 = int(track[2].item())
                        y1 = int(track[3].item())

                        print(x0)
                        # draw bounding box
                        frame = cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)                        


                        # box description
                        #frame = cv2.putText(frame, box_text, (x0, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                        frame = cv2.putText(frame, distance_text, (x0, y0 + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                        frame = cv2.putText(frame, angle_text, (x0, y0 + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)
                        
                        frame = cv2.circle(frame, (self.uM, self.vM), radius=5, color=(20, 10, 220), thickness=-1)

                        # draw time on frame
                        inference_time_text = 'Inference: {:.2f} ms'.format((self.inference_stop - self.inference_start) * 1000)
                        frame = cv2.putText(frame, inference_time_text, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                        
    

    def calib_frame(self, frame):
        """
        @brief Undistorts the input frame using precomputed camera maps.
        @param frame Input distorted frame.
        @return Undistorted frame.
        """
        frame = cv2.remap(frame, self.mapx, self.mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        return frame
    
    def estimate_angle_and_distance(self, frame):   
        """
        @brief Estimates human position (angle and distance) from the camera.
        @param frame Current image frame.
        @return True if a human is detected; False otherwise.
        """
          

        # format frame
        frameHuman = self.calib_frame(frame.copy())         
        self.inference_frame = cv2.resize(frameHuman, PositioningVisionHuman.inference_size)
        # perform inference and measure time
        self.inference_start = time.monotonic()
        run_inference(PositioningVisionHuman.interpreter, self.inference_frame.tobytes())
        self.inference_stop = time.monotonic()

        # get detections
        objs = get_objects(PositioningVisionHuman.interpreter, self.get_object_threshold)[:self.get_object_top_k]
        
        humanobjs = filter(filter_object, objs)
        self.human_objects = list(humanobjs)

        # get tracks
        self.format_for_mot()
        self.tracks = self.tracker.update(self.detections)
        
        self.match_objects_with_tracks()

        if not self.matched_tracks.size == 0:  
            self.human_detected = True      
            track_id = self.matched_tracks[:,4]
            follow_id = track_id.argmin()
           
            # Retrieve pixel values for upper left (u1,v1) and bottom right (u4,v4) corner of bounding box 
            u1 = int(self.matched_tracks[follow_id,0].item())*self.scale[0]
            v1 = int(self.matched_tracks[follow_id,1].item())*self.scale[1]
            
            u4 = int(self.matched_tracks[follow_id,2].item())*self.scale[0]
            v4 = int(self.matched_tracks[follow_id,3].item())*self.scale[1]
            
            #p_m = np.array([[0.5*(u4+u1)],[0.5*(v4+v1)],[1]]) # midpoint/center of bbox
            # M is point on lower edge of bbox, it is centered horizontally on this edge
            self.uM = int(0.5 * (u4 + u1)) # horizontal center of bbox
            self.vM = int(v4)
            
            # 3D Point in camera coordinates P_c = [X_c, h_c, Z_c], calculated from pixel coordinates and projection equations
            Z_c = self.f_y * self.height_camera / (self.vM - self.c_y)
            X_c = (self.uM - self.c_x) * Z_c / self.f_x
            
            P_c_2d_hom = np.array([X_c, Z_c, 1])            # 2d (plane on the floor), homogeneous representation of 3d world point for transformation 
            P_c_2d_hom_corrected = np.matmul(PositioningVisionHuman.A,P_c_2d_hom)  # apply affine transform for additional correction
            
            X = float(P_c_2d_hom_corrected[0])
            self.x_history.appendleft(X)
            X = statistics.median(self.x_history)
            Z = float(P_c_2d_hom_corrected[1])
            self.z_history.appendleft(Z)
            Z = statistics.median(self.z_history)        
            
            self._distance = np.sqrt(Z**2 + X**2)
            
            if self.uM < 320:
                #angle = np.arctan(abs(X/Z))/np.pi*180
                self._angle = np.arctan(abs(X/Z))
            else:
                #angle = -np.arctan(abs(X/Z))/np.pi*180
                self._angle = -np.arctan(abs(X/Z))
                
            self._distance = limit(self._distance, 0, 50)
            self._angle = limit(self._angle, -2*np.pi, 2*np.pi)
            Z = limit(Z, 0, 50)
            
        else:
            self.human_detected = False
            self._distance = 0
            self._angle = 0
            Z = 0
            X = 0
            self.uM = 0
            self.vM = 0
        
        self._x = Z # Z from camera coordinate system corresponds to x in robot's coordinate System         
        self._y = -X    

        #if show_ui:
        #    frameHuman = self.draw_tracks_on_frame(frame)

        return self.human_detected
     

def main():
    show_ui = True
       
    # run inference loop
    positioningVisionHuman = PositioningVisionHuman(height_camera=0.47)
  
    positioningVisionHuman.initialize_camera()

    while positioningVisionHuman.capture.isOpened():
        # initialize camera

        #new_f_time = time.time() # starting time of the frame
        # capture frame
        ret = positioningVisionHuman.get_frame()
        #frame = cv2.flip(frame, 0) # flip image if camera is mounted upside down
        # catch no capture
        if not ret:
            break

        
        positioningVisionHuman.estimate_angle_and_distance()

        print(f"x: {positioningVisionHuman.get_x()}, y: {positioningVisionHuman.get_y()}")

        if show_ui:
            # draw tracks on frame
            positioningVisionHuman.draw_tracks_on_frame()            
            

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                cv2.destroyAllWindows()
                break        

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()




