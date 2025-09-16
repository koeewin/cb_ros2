import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import time
from motion_msgs.msg import MotionCtrl
from nav_msgs.msg import Odometry  
from rclpy.callback_groups import ReentrantCallbackGroup
import math

#from Paths.path import gen_path, arclength
import warnings
#from visualize_path import cv2_plot_path
import cv2
import atexit
import threading, queue

# atexit.register(cv2.destroyAllWindows)
# plot_queue = queue.Queue()
# def plotting_loop():
#     while True:
#         path = plot_queue.get()
#         if path is None:
#             break
#         cv2_plot_path(path)
# threading.Thread(target=plotting_loop, daemon=True).start()

class HumanPositionFollowing(Node):

    def __init__(self):
        super().__init__('human_path_following_controller')
        self.DEBUG = False  # Set to True for debugging output
        self.first_time = True  # Flag to send mode_mark on first command 

        # Choose self.control method
        self.control = "PID0"  # Options: PID, PID0

        self.ctrlMsgs = MotionCtrl()
        self.vel_cmd_publisher_ = self.create_publisher(MotionCtrl,"/diablo/MotionCmd_follow",2)   

        # multithreaded callback group for subscriptions
        self.callback_group = ReentrantCallbackGroup()
        self.human_position_subscription = self.create_subscription(
            Point,
            '/human/positionFuse',
            self.human_position_callback,
            10,
            callback_group=self.callback_group) 
        
        # initialize the controllers
        self.trans_controller = Controller(0.85, 0.025, 0.0, 1.0, 1.0)
        self.rot_controller = Controller(1.2,0.0,-1.6,1.6)
        # intialize the velocities
        self.wRef = 0.0
        self.vRef = 0.0
        self.d_stop = 0.65
        self.d_follow = 1.2
        
    
    # Callback for human position updates
    def human_position_callback(self, msg):

        dx = msg.x  # human position in robot body frame
        dy = msg.y
        d_rel = np.array([dx, dy])
        distance_to_human = np.linalg.norm(d_rel[:2])
        angle = math.atan2(dy,dx)
        if abs(angle) <= 0.12:
            angle = 0.0
        if self.DEBUG:
                print("dx, dy, angle:", dx, dy, angle) # Debugging output for current position relative to the human
            
        #self.path_storage = self.find_forward_points()
        self.vRef = 0.0
        self.wRef = 0.0
        
        if distance_to_human > self.d_stop:
            # follow the human
            if dx > self.d_follow: # perform the following control
                print("STATE ===> Follow the human") 
                if self.control == "PID":
                    # Perform PID self.control
                    vRef = 3 * np.linalg.norm(self.path_storage[:, 0])  # Linear velocity [m/s]
                    phi = np.arctan2(self.path_storage[1, 0], self.path_storage[0, 0])
                    wRef = 5 * phi  # Angular velocity [rad/s]

                    vRef = np.clip(vRef, self.v_min, self.v_max)
                    wRef = np.clip(wRef, self.omega_min, self.omega_max)

                elif self.control == "PID0":                 
                    self.rot_controller.put(angle)
                    self.trans_controller.put(abs(distance_to_human-self.d_follow))

                    self.rot_controller.run()
                    self.trans_controller.run()

                    self.vRef = self.trans_controller.get()
                    self.wRef = self.rot_controller.get()

            # rotate to find the human
            elif (distance_to_human> self.d_stop and distance_to_human < self.d_follow) or dx < self.d_follow: #dx > self.d_stop and dx < self.d_follow: # rotate to finde the human
                print("STATE ===> Rotate to find the human")
                if abs(angle) <= 0.6:
                    angle = 0.0
                    self.wRef = 0.0
                else:
                    self.trans_controller.reset()
                    if angle > 0: self.wRef = 0.6
                    else: self.wRef = -0.6

                self.vRef = 0.0

        else:
            print("STATE ===> STOP") 
            self.vRef = 0.0
            self.wRef = 0.0
                
        #### ======== Publish the motion command ======== ####
        self.ctrlMsgs.mode_mark = False

        # only first message should have mode_mark=True (robot will stand up)
        if self.first_time == True:
            self.ctrlMsgs.mode_mark = True
            self.first_time = False

        # set control modes
        self.ctrlMsgs.mode.stand_mode = True
        self.ctrlMsgs.mode.height_ctrl_mode = True
        self.ctrlMsgs.mode.pitch_ctrl_mode = True

        # set fixed up/pitch values
        self.ctrlMsgs.value.up = 1.0
        self.ctrlMsgs.value.pitch = 0.0

        # default: no motion
        self.ctrlMsgs.value.forward = self.vRef
        self.ctrlMsgs.value.left = self.wRef

        # publish motion command
        self.vel_cmd_publisher_.publish(self.ctrlMsgs)
        #if self.DEBUG:
        #self.get_logger().info(f'Publishing: forward={self.ctrlMsgs.value.forward:.3f}, left={self.ctrlMsgs.value.left:.3f}')

## ======== srart of helper functions ==========

class Controller:
    def __init__(self, p, iTs, lowerLimit=float('-inf'), upperLimit=float('inf'), aw=0.0):
        self._u = 0.0
        self._xI = 0.0
        self._p = p
        self._aw = aw
        self._y = 0.0
        self._ySat = 0.0
        self._iTs = iTs
        self._lowerLimit = lowerLimit
        self._upperLimit = upperLimit

    def put(self, e):
        self._u = e

    def run(self):
        self._xP = self._p * self._u
        self._dy = self._ySat - self._y
        self._xI = self._iTs * self._y + self._xI + self._dy * self._aw
        self._y = self._xP + self._xI
        self._ySat = self.limit(self._y, self._lowerLimit, self._upperLimit)

    def get(self):
        return self._ySat

    @staticmethod
    def limit(x, lower, upper):
        if x < lower:
            return lower
        elif x > upper:
            return upper
        else:
            return x

    def reset(self):
        self._dy = 0.0
        self._u = 0.0
        self._y = 0.0
        self._ySat = 0.0
        self._xI = 0.0
        self._xP = 0.0

def main(args=None):

    for i in range(1):
        print(i)
        time.sleep(1)

    rclpy.init(args=args)

    human_path_following_controller = HumanPositionFollowing()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(human_path_following_controller)

    print("Start")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    human_path_following_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
