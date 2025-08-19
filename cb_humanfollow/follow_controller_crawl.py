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


class HumanPathFollowing(Node):

    def __init__(self):
        super().__init__('human_path_following_controller') 

        # Choose self.control method
        self.control = "MPC"  # Options: MPC, PID, PID0

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
        self.sampleTime = 0.06  # Sample time [s], equals 20 Hz
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

        # Setup MPC if selected
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
        self.vel_cmd_publisher_ = self.create_publisher(MotionCtrl,"/diablo/MotionCmd",2)   

        self.callback_group = ReentrantCallbackGroup()
        self.human_position_subscription = self.create_subscription(
            Point,
            '/human/positionFuse',
            self.human_position_callback,
            10,
            callback_group=self.callback_group) 
        
        self.wheel_encoder_subscription = self.create_subscription(LegMotors, '/diablo/sensor/Motors', self.wheel_encoder_callback, 10, callback_group=self.callback_group)
        
        
        self.trans_controller = Controller(0.6, 0.001, 0.0, 1.0, 1.0)
        self.rot_controller = Controller(1.0,0.0,-1.5,1.5)

        self.wRef = 0.0
        self.vRef = 0.0
        
        self.heightset = 0
        
    

    def human_position_callback(self, msg):

        start_time = time.time()
        self.lastPose = self.currentPose

        self.currentPose = np.array([self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, self.odom_msg.pose.pose.orientation.z])
        
        # Calculate relative distance between current pose and operator
        dx = msg.x
        dy = msg.y

        d_rel = np.array([dx, dy])

        current_position = self.currentPose[:2]
        last_position = self.lastPose[:2]

        current_position_last = - np.linalg.inv(Rz(self.currentPose[2])) @ np.append((current_position - last_position), 1) # Position (x,y) of the last pose w.r.t. the current pose in the current frame

        current_pose_last = np.append(current_position_last[:2], self.lastPose[2] - self.currentPose[2])

        current_T_last = Tz(current_pose_last[2], current_pose_last)
        self.path_storage = current_T_last @ np.vstack([self.path_storage, np.zeros((1, self.path_storage.shape[1])), np.ones((1, self.path_storage.shape[1]))])
        self.path_storage = self.path_storage[:2, :]
    

        differences = np.diff(self.path_storage_smooth,axis=1)
        distances = np.sqrt(np.sum(differences**2, axis=0))
        self.total_length = np.sum(distances)
        
            
        #print(self.total_length)
 
        

        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:
        #if np.linalg.norm(d_rel[:2]) > 0.5:
        #if np.linalg.norm(self.path_storage[:, -1] - d_rel[:2]) > 2e-2 and (np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3):
        
        #if np.linalg.norm(self.path_storage[:, -1] - d_rel[:2]) > 2e-2 and np.linalg.norm(d_rel[:2]) > 0.1:
        #        self.path_storage = np.hstack([self.path_storage, d_rel[:2].reshape(-1, 1)])
        #print(self.path_storage)
            
        #self.path_storage = self.find_forward_points()
        self.vRef = 0.0
        self.wRef = 0.0
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:
        if True:
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
 
            x_pos = self.path_storage[0,:]
            y_pos = self.path_storage[1,:]

            #xnew = np.linspace(np.min(x_pos), np.max(x_pos), 10)
            #xout2, yout2, weights2 = loess_1d(x_pos, y_pos, xnew, frac=0.5, degree=1)

            #self.path_storage_smooth = np.row_stack((xout2, yout2))
        

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
                x = dx
                y = dy

                angle = math.atan2(y,x)
                if abs(angle) <= 0.08:
                    angle = 0.0
                
                self.rot_controller.put(angle)
                self.trans_controller.put(x)

                self.rot_controller.run()
                self.trans_controller.run()

                self.vRef = self.trans_controller.get()
                self.wRef = self.rot_controller.get()

            #if self.path_storage.shape[1] > self.numPos:
            if True:
                self.path_storage = self.path_storage[:, 1:]
                #self.path_storage = self.find_forward_points()
                
        else:
            self.vRef = 0.0
            self.wRef = 0.0
        
        # Calculate motor velocities and perform forward kinematics
        # wL, wR = self.dd.inverse_kinematics(vRef, wRef)
        # v, w = self.dd.forward_kinematics(wL, wR)
        
        #if not self.heightset:
        #    self.heightset = 1
        #    self.generMsgs(mode_mark=True,stand_mode=False)
        #    self.vel_cmd_publisher_.publish(self.ctrlMsgs)           
           
      

        #self.generMsgs(forward=self.vRef, left=self.wRef)
        mctrl_msg = MotionCtrl()
        mctrl_msg.value.forward = self.vRef
        mctrl_msg.value.left = self.wRef

        self.vel_cmd_publisher_.publish(mctrl_msg)

        self.get_logger().info('Publishing: v:"%f", w:"%f"' % (float(mctrl_msg.value.forward), float(mctrl_msg.value.left)))


    # def odom_callback(self, msg):
    #     self.lastPose = self.currentPose

    #     self.currentPose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.z])
    #     # self.odom_received = True
        #print(time.time()-start_time)
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


    def generMsgs(self, forward=None,left=None,roll=None,up=None,
                pitch=None,mode_mark=False,height_ctrl_mode = None,
                pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = False,
                jump_mode = False,dance_mode = None):
        self.ctrlMsgs.mode_mark = mode_mark
        self.ctrlMsgs.mode.jump_mode = jump_mode

        if dance_mode is not None:
            self.ctrlMsgs.mode.split_mode = dance_mode
        if forward is not None:
            self.ctrlMsgs.value.forward = forward
        if left is not None:
            self.ctrlMsgs.value.left = left
        if pitch is not None:
            self.ctrlMsgs.value.pitch = pitch
        if roll is not None:
            self.ctrlMsgs.value.roll = roll
        if up is not None:
            self.ctrlMsgs.value.up = up
        if height_ctrl_mode is not None:
            self.ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
        if pitch_ctrl_mode is not None:
            self.ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
        if roll_ctrl_mode is not None:
            self.ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
        if stand_mode is not None:
            self.ctrlMsgs.mode.stand_mode = stand_mode

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

    for i in range(5):
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
