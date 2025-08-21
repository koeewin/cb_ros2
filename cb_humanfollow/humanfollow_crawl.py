import numpy as np
import matplotlib.pyplot as plt
from mpc_optimizer import MpcOptimizer
from transform import body_to_world, Tz, Rz
from differential_drive import DifferentialDrive
from  visualize import  Visualize
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import time
from motion_msgs.msg import MotionCtrl
from nav_msgs.msg import Odometry  
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int8
import math
from motion_msgs.msg import LegMotors  # Adjust this to the correct import path
from loess.loess_1d import loess_1d
from Paths.path import gen_path, arclength
import warnings


class HumanPathFollowing(Node):

    def __init__(self):
        super().__init__('human_path_following_controller')
        self.DEBUG = False  # Set to True for debugging output
        self.first_time = True  # Flag to send mode_mark on first command 

        # Choose self.control method
        self.control = "PID0"  # Options: MPC, PID, PID0

        self.odom_msg = Odometry()

        self.robot = Robot()  # Initialize the robot instance here
        # Initialize previous wheel positions
        self.prev_left_wheel_pos = None
        self.prev_right_wheel_pos = None


        # Define differential drive kinematics parameters
        R = 0.094  # Wheel radius [m]
        L = 0.482  # Wheelbase [m]
        self.dd = DifferentialDrive(R, L)

        # Simulation parameters
        self.sampleTime = 0.06          # Sample time [s], equals 20 Hz
        initPose = np.array([0, 0, 0])  # Initial pose (x, y, theta) of the robot
        self.currentPose = initPose
        self.lastPose = initPose

        self.path_storage = np.zeros((2, 1))
        self.path_storage_smooth = np.zeros((2, 1))
        self.total_length = 0.0

        # Define self.control and velocity limits
        self.numPos = 20  # Number of stored self.positions corresponding to a 1 m distance to human
        self.v_max = 0.8
        self.omega_max = np.pi/4.0
        self.v_min = 0.0
        self.omega_min = -self.omega_max

        # === Setup MPC if selected ===
        if self.control == "MPC":
            N = 5
            u = np.zeros((N, 2))

            self.optimizeProblem = MpcOptimizer(N, self.sampleTime, u,
                                        WeightX=100, WeightY=7, WeightTheta=0.12,
                                        WeightV=0.01, WeightOmega=0.03,
                                        WeightVLast=0.0, WeightOmegaLast=0.0,
                                        VelLim=[self.v_min, self.v_max], AngVelLim=[self.omega_min, self.omega_max])
            self.optimizeProblem.setup_problem()

        # Simulation loop
        self.velB = np.array([0, 0, 0])
        self.odom_received = False

        self.ctrlMsgs = MotionCtrl()
        self.vel_cmd_publisher_ = self.create_publisher(MotionCtrl,"diablo/MotionCmd",2)   

        # multithreaded callback group for subscriptions
        self.callback_group = ReentrantCallbackGroup()
        self.human_position_subscription = self.create_subscription(
            Point,
            '/human/positionFuse',
            self.human_position_callback,
            10,
            callback_group=self.callback_group) 
        
        self.wheel_encoder_subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.wheel_encoder_callback, 10, callback_group=self.callback_group)
        
        # initialize the controllers
        self.trans_controller = Controller(0.6, 0.025, 0.0, 1.0, 1.0)
        self.rot_controller = Controller(1.0,0.0,-1.5,1.5)
        # intialize the velocities
        self.wRef = 0.0
        self.vRef = 0.0
        
        self.heightset = 0
        
    
    # Callback for human position updates
    def human_position_callback(self, msg):

        start_time = time.time()

        # Update the odometry message with the current pose from parallel thread
        self.lastPose = self.currentPose

        self.currentPose = np.array([self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.orientation.z])
        if self.DEBUG:
            print("Current Pose:", self.currentPose)

        # receive the human position from the message
        dx = msg.x
        dy = msg.y
        d_rel = np.array([dx, dy])
        if self.DEBUG:
            print("inital reading from msg dx, dy:", dx, dy) # Debugging output for current position relative to the human

        # Construction of the path storage
        current_position = self.currentPose[:2] # current position (x,y) of the robot in the world frame
        last_position = self.lastPose[:2]       # last position (x,y) of the robot in the world frame 
        
        # Following three lines are to find current_T_last, first find 
        # current_Translation_last: Position (x,y) of the last pose w.r.t. the current pose in the current frame
        current_position_last = - np.linalg.inv(Rz(self.currentPose[2])) @ np.append((current_position - last_position), 1) 
        # construct the current_pose_last: [x,y,theta] in the current frame
        current_pose_last = np.append(current_position_last[:2], self.lastPose[2] - self.currentPose[2])
        # current_T_last
        current_T_last = Tz(current_pose_last[2], current_pose_last)
        if self.DEBUG:
            print("current_T_last:", current_T_last)

        if self.path_storage.size > 0:
            # -- my be delete this part later --
            #print("Before transform, path_storage shape:", self.path_storage.shape)
            # tmp = np.vstack([self.path_storage,
            #         np.zeros((1, self.path_storage.shape[1])),
            #         np.ones((1, self.path_storage.shape[1]))])
            #print("After vstack shape:", tmp.shape)
            #print("current_T_last shape:", current_T_last.shape)

            transformed = current_T_last @ np.vstack([self.path_storage, np.zeros((1, self.path_storage.shape[1])), np.ones((1, self.path_storage.shape[1]))])
            self.path_storage = transformed[:2, :]
            #print("Final path_storage shape:", self.path_storage.shape)
        else: 
            print("path_storage is empty, skipping transform.")
            warnings.warn("path_storage is empty, skipping transform.")
            return
        
        # Calculate the total length of the path
        differences = np.diff(self.path_storage_smooth,axis=1)
        distances = np.sqrt(np.sum(differences**2, axis=0))
        self.total_length = np.sum(distances)

        if self.DEBUG:
            print("Path storage after transformation:", self.path_storage)
            print("Total length of path storage:", self.total_length)
        
            
 
        
        ## This part is for storing the path following condistion may be deleted later
        # If the distance to the last point is greater than 2 cm and the relative distance
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:
        #if np.linalg.norm(d_rel[:2]) > 0.5

        # Calculate parameters for storing the path
        dist2laststorage = np.linalg.norm(self.path_storage[:, -1] - d_rel[:2])
        dist2human = np.linalg.norm(d_rel[:2])
        lenpathstorage = self.total_length
        if self.DEBUG:
            print("Distance to last storage point:", dist2laststorage)
            print("Distance to human:", dist2human)
            print("Length of path storage:", lenpathstorage)
            print("Path storage with new points:", self.path_storage)
       
        # Path storage condition:
        if self.path_storage.shape[1] > 0:  # es gibt schon gespeicherte Punkte
            if dist2laststorage > 2e-2 and (dist2human > 1.3 or lenpathstorage > 1.5):
                self.path_storage = np.hstack([self.path_storage, d_rel[:2].reshape(-1, 1)])

            
        #self.path_storage = self.find_forward_points()
        self.vRef = 0.0
        self.wRef = 0.0
        
        #if np.linalg.norm(d_rel[:2]) > 0.5: # if the distance to the human is greater than 0.5 m
        #dx = msg.x
        if dx > 1.39:
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:     // Uncomment this line to enable the condition
        #if self.total_length > 1.3:
        
        #if self.path_storage.shape[1] > self.numPos and (np.linalg.norm(d_rel[:2]) > 1.5 or self.total_length > 1.5):
            #start_time = time.time()
            #print(".")
            #print(np.linalg.norm(d_rel[:2]))
            #print(self.total_length)
            #print(".")
            #self.path_storage = self.find_forward_points()
            #self.path_storage_smooth, self.total_length = self.reconstruct()
            #print(time.time()-start_time)
        
        
            #self.path_storage_smooth = self.find_forward_points()
          
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:
            #start_time = time.time()
            if self.control == "MPC":
                x_pos = self.path_storage[0,:]
                y_pos = self.path_storage[1,:]
                xnew = np.linspace(np.min(x_pos), np.max(x_pos), 10)

                if len(np.unique(x_pos)) < 3:
                    print("Skipping LOESS: not enough unique x values")
                    # Fallback: einfach Originalwerte benutzen
                    xout2, yout2, weights2 = x_pos, y_pos, None
                else:
                    xout2, yout2, weights2 = loess_1d(x_pos, y_pos, xnew, frac=0.5, degree=1)
                
                self.path_storage_smooth = np.row_stack((xout2, yout2))
            else:   # which is PID or PID0
                x_pos = dx
                y_pos = dy
            if self.DEBUG:
                print("dx, dy:", dx, dy) # Debugging output for current position relative to the human

        

            if self.control == "MPC":
                # Perform MPC optimization
                #st = time.time()
                self.optimizeProblem.solve_problem(self.path_storage_smooth, [self.vRef,self.wRef])
                #print(time.time() - st)
                self.vRef = self.optimizeProblem.Controls[0, 0]  # Linear velocity [m/s]
                self.wRef = self.optimizeProblem.Controls[0, 1]  # Angular velocity [rad/s]
            elif self.control == "PID":
                # Perform PID self.control
                vRef = 3 * np.linalg.norm(self.path_storage[:, 0])  # Linear velocity [m/s]
                phi = np.arctan2(self.path_storage[1, 0], self.path_storage[0, 0])
                wRef = 5 * phi  # Angular velocity [rad/s]

                vRef = np.clip(vRef, self.v_min, self.v_max)
                wRef = np.clip(wRef, self.omega_min, self.omega_max)
            elif self.control == "PID0":

                x = x_pos
                y = y_pos

                angle = math.atan2(y,x)
                if abs(angle) <= 0.08:
                    angle = 0.0
                
                self.rot_controller.put(angle)
                self.trans_controller.put(abs(x-1.5))

                self.rot_controller.run()
                self.trans_controller.run()

                self.vRef = self.trans_controller.get()
                self.wRef = self.rot_controller.get()
               

            if self.path_storage.shape[1]>1:
                self.path_storage = self.path_storage[:, 1:]
                #self.path_storage = self.find_forward_points()
                if self.DEBUG:            
                    print("Path storage after removal:", self.path_storage)
                
        else:
            self.vRef = 0.0
            self.wRef = 0.0
        
        
        #if not self.heightset:
        #    self.heightset = 1
        #    self.generMsgs(mode_mark=True,stand_mode=False)
        #    self.vel_cmd_publisher_.publish(self.ctrlMsgs)           
           
        print("vRef =========", self.vRef, ", wRef =========", self.wRef)

        
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
        self.ctrlMsgs.value.forward = 0.0
        self.ctrlMsgs.value.left = 0.0

        self.ctrlMsgs.value.forward = self.vRef
        self.ctrlMsgs.value.left = self.wRef

        # publish motion command
        self.vel_cmd_publisher_.publish(self.ctrlMsgs)
        #if self.DEBUG:
        self.get_logger().info(f'Publishing: forward={self.ctrlMsgs.value.forward:.3f}, left={self.ctrlMsgs.value.left:.3f}')

## ======== srart of helper functions ==========

    def wheel_encoder_callback(self, msg):
        left_wheel_enc_rev = msg.left_wheel_enc_rev
        right_wheel_enc_rev = msg.right_wheel_enc_rev

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
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'

        self.odom_msg.pose.pose.position.x = self.robot.Pose.x
        self.odom_msg.pose.pose.position.y = self.robot.Pose.y
        self.odom_msg.pose.pose.orientation.z = self.robot.Pose.theta  # Simplified, you may need quaternion
        
        
        # print for debugging
        # self.get_logger().info(f'Position - x: {self.odom_msg.pose.pose.position.x}, y: {self.odom_msg.pose.pose.position.y}, theta: {self.odom_msg.pose.pose.orientation.z}')

    def reconstruct(self, target_distance=0.05, n_loess_points=10, frac=0.7, degree=1):
        """
        Smooth the path with LOESS and resample to ensure approximately equal Euclidean distances
        while minimizing computational cost.
        """

        # Extract x and y positions
        x_pos = self.path_storage[0, :]
        y_pos = self.path_storage[1, :]
        
        # Limit the number of points for LOESS smoothing
        xnew = np.linspace(np.min(x_pos), np.max(x_pos), n_loess_points)
        xout, yout, _ = loess_1d(x_pos, y_pos, xnew, frac=frac, degree=degree)
        
        # Compute Euclidean distances along the smoothed path
        dx = np.diff(xout)
        dy = np.diff(yout)
        distances = np.sqrt(dx**2 + dy**2)
        cumulative_distances = np.insert(np.cumsum(distances), 0, 0)  # Start with zero
        
        # Calculate the number of points based on target_distance
        total_distance = cumulative_distances[-1]
        num_points = int(total_distance // target_distance) + 1
        
        # Use fewer target distances to reduce resampling
        target_cumulative_distances = np.linspace(0, total_distance, num_points)
        
        # Interpolate x and y coordinates at target distances
        x_resampled = np.interp(target_cumulative_distances, cumulative_distances, xout)
        y_resampled = np.interp(target_cumulative_distances, cumulative_distances, yout)
        
        # Return the resampled path
        return np.row_stack((x_resampled, y_resampled)), total_distance
        
    def find_forward_points(self):
    
        #Extract x and y of waypoints
        x_coords, y_coords = self.path_storage[0, :], self.path_storage[1, :]

        # Find the first point ahead of the robot
        for i in range(x_coords.shape[0]) :             
            if x_coords[i] > 0.1 and np.linalg.norm(self.path_storage[:, i]) > 0.2: 
                # Return the remaining path and the next waypoint 
                pruned_path = self.path_storage[:, i:] 
                return pruned_path
                 
        # No points ahead 
        print(1)
        return np.empty((2, 0))

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

class Robot:
    _R_WHEEL = 0.094
    _D_WHEEL = 0.482

    def __init__(self):
        self.Pose = Pose(0, 0, 0)
        self.tol = np.finfo(float).eps

    def updateOdometry(self, left_wheel_pos, right_wheel_pos):
        
        ds_left = self._R_WHEEL * left_wheel_pos
        ds_right = self._R_WHEEL * right_wheel_pos
        ds = (ds_left + ds_right) / 2
        dtheta = (ds_right - ds_left) / self._D_WHEEL
        ct = math.cos(self.Pose.theta)
        st = math.sin(self.Pose.theta)

        if abs(ct) < self.tol:
            ct = 0
        if abs(st) < self.tol:
            st = 0

        self.Pose.x += ds * ct
        self.Pose.y += ds * st
        self.Pose.theta += dtheta


class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

def main(args=None):

    for i in range(1):
        print(i)
        time.sleep(1)

    rclpy.init(args=args)

    human_path_following_controller = HumanPathFollowing()

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
