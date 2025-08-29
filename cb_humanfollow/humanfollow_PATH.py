import numpy as np
import matplotlib.pyplot as plt
from mpc_optimizer import MpcOptimizer
from transform import body_to_world, Tz, Rz
from  visualize import  Visualize
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import time
from tf_transformations import euler_from_quaternion

from motion_msgs.msg import MotionCtrl
from nav_msgs.msg import Odometry  
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Int8
import math
from motion_msgs.msg import LegMotors  # Adjust this to the correct import path
from loess.loess_1d import loess_1d
from Paths.path import gen_path, arclength
import warnings
import threading
from rclpy.time import Time


class HumanPathFollowing(Node):

    def __init__(self):
        super().__init__('human_path_following_controller')
        self.DEBUG = False  # Set to True for debugging output
        self.first_time = True  # Flag to send mode_mark on first command 

        # Choose self.control method
        self.control = "PP"  # Options: MPC, PID, PID0
        self.d_follow = 1.2  # Distance to follow the human in meters
        self.d_stop = 0.65  # Distance to rotate around the human in meters

        self.odom_msg = Odometry()

        # Define differential drive kinematics parameters
        #R = 0.094  # Wheel radius [m]
        #L = 0.482  # Wheelbase [m]


        # Simulation parameters
        self.sampleTime = 0.05          # Sample time [s], equals 20 Hz
        initPose = np.array([0, 0, 0])  # Initial pose (x, y, theta) of the robot
        self.currentPose = initPose.copy()
        self.lastPose    = initPose.copy()
        self.pose_stamp  = Time()   # last odom stamp
        self.pose_lock   = threading.Lock()

        self.path_storage = np.zeros((2, 1))
        self.path_for_mpc = np.zeros((2, 1))
        self.path_storage_smooth = np.zeros((2, 1))
        self.total_length = 0.0

        # Define self.control and velocity limits
        self.numPos = 30  # Number of stored self.positions corresponding to a 1,5s/ m distance to human
        self.v_max = 0.8
        self.omega_max = np.pi/2.0
        self.v_min = 0.0
        self.omega_min = -self.omega_max
        self.MPC_Horizon = 10  # Prediction horizon for MPC
       

        # === Setup MPC if selected ===
        if self.control == "MPC":
            N = self.MPC_Horizon-2
            u = np.zeros((N, 2))

            self.optimizeProblem = MpcOptimizer(N, self.sampleTime, u,
                                        WeightX=100, WeightY=10, WeightTheta=0.12,
                                        WeightV=0.03, WeightOmega=0.03,
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
        
        # Subscription to odometrys
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        ) 
        
        # self.wheel_encoder_subscription = self.create_subscription(LegMotors, 'diablo/sensor/Motors', self.wheel_encoder_callback, 10, callback_group=self.callback_group)
        
        # initialize the controllers for PID0
        self.trans_controller = Controller(0.65, 0.025, 0.0, 1.0, 1.0)
        self.rot_controller = Controller(1.0,0.0,-1.5,1.5)
        # intialize the velocities
        self.wRef = 0.0
        self.vRef = 0.0
        
        self.heightset = 0


    def odom_callback(self, msg: Odometry):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Orientation (quaternion -> yaw)
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        with self.pose_lock:
            self.lastPose = self.currentPose.copy()
            self.currentPose = np.array([x, y, theta], dtype=float)
            self.pose_stamp = Time.from_msg(msg.header.stamp)
        
        # Log values
        if self.DEBUG:
            self.get_logger().info(
                f"x: {x:.3f}, y: {y:.3f}, yaw: {theta:.3f} rad"
            )

    
    # Callback for human position updates
    def human_position_callback(self, msg):

        start_time = time.time()

        # Update the odometry message with the current pose from parallel thread
        # Safely read a consistent snapshot
        with self.pose_lock:
            currentPose = self.currentPose.copy()
            lastPose    = self.lastPose.copy()
            pose_stamp   = self.pose_stamp
        if self.DEBUG:
            print("Current Pose:", currentPose)

        # receive the human position from the message
        dx = msg.x
        dy = msg.y
        d_rel = np.array([dx, dy])

        if self.DEBUG:
            print("inital reading from msg dx, dy:", dx, dy) # Debugging output for current position relative to the human

        # Construction of the path storage
        current_position = currentPose[:2] # current position (x,y) of the robot in the world frame
        last_position = lastPose[:2]       # last position (x,y) of the robot in the world frame 
        
        # Following three lines are to find current_T_last, first find 
        # current_Translation_last: Position (x,y) of the last pose w.r.t. the current pose in the current frame
        current_position_last = - np.linalg.inv(Rz(currentPose[2])) @ np.append((current_position - last_position), 1) 
        # construct the current_pose_last: [x,y,theta] in the current frame
        angle_diff = math.atan2(math.sin(lastPose[2] - currentPose[2]), math.cos(lastPose[2] - currentPose[2]))
        current_pose_last = np.append(current_position_last[:2], angle_diff)
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
            warnings.warn("path_storage is empty, skipping transform!!!.")
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
       
        # Path storage condition for adding new points at the end of the path storage:
        if self.path_storage.shape[1] > 0 :  # es gibt schon gespeicherte Punkte und nicht zu viele
            if dist2laststorage > 5e-2 and (dist2human > self.d_stop or lenpathstorage > 1.5):
                self.path_storage = np.hstack([self.path_storage, d_rel[:2].reshape(-1, 1)])
        if  self.path_storage.shape[1] > self.numPos: # zu viele Punkte, entferne den ältesten
            self.path_storage = self.path_storage[:, -self.numPos]
        
        
        
        # preprocess path storage for MPC
        path_for_mpc = self.path_storage.copy()
        n_cols = path_for_mpc.shape[1]
        if n_cols  <  self.MPC_Horizon: # If shorter, pad with last column
            last_col = path_for_mpc[:, -1].reshape(2, 1) # Take the last column
            repeat_cols = self.MPC_Horizon - n_cols # Repeat it enough times
            pad = np.repeat(last_col, repeat_cols, axis=1)
            path_for_mpc = np.hstack([path_for_mpc, pad])  # Stack with the original
        elif n_cols > self.MPC_Horizon:
            # If longer, trim to horizon
            path_for_mpc = path_for_mpc[:, :self.MPC_Horizon]

        # Now guaranteed shape is (2, self.MPC_Horizon)
        assert path_for_mpc.shape == (2, self.MPC_Horizon)
        print("Path Storage:", self.path_storage)
        print("Path for MPC:", path_for_mpc)

        #self.path_storage = self.find_forward_points()
        self.vRef = 0.0
        self.wRef = 0.0
        
        #if np.linalg.norm(d_rel[:2]) > 0.5: # if the distance to the human is greater than 0.5 m
    
        if dist2human > self.d_stop:
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:     // Uncomment this line to enable the condition
        #if self.total_length > 1.3:
        
        #if self.path_storage.shape[1] > self.numPos and (np.linalg.norm(d_rel[:2]) > 1.5 or self.total_length > 1.5):
         
        #if np.linalg.norm(d_rel[:2]) > 1.3 or self.total_length > 1.3:
            #start_time = time.time()

        # calculate general geometry information from positiong
            x_pos, y_pos = d_rel[0], d_rel[1]
            angle = math.atan2(y_pos,x_pos)
            if abs(angle) <= 0.08:
                angle = 0.0
            if self.DEBUG:
                print("dx, dy:", dx, dy) # Debugging output for current position relative to the human
           
            
            
            #self.control = "MPC"
            # preprocess the path storage with loess smoothing and resampling for MPC
            # if self.control == "MPC":
            #     x_pathpoints = self.path_storage[0,:]
            #     y_pathpoints = self.path_storage[1,:]
            #     x_interp = np.linspace(np.min(x_pathpoints), np.max(x_pathpoints), 10)

                #if len(np.unique(x_pos)) < 3:
                #    print("Skipping LOESS: not enough unique x values")
                    # Fallback: einfach Originalwerte benutzen
                #    x_pathpoints_smoothed, x_pathpoints_smoothed, weights2 = x_pathpoints, y_pathpoints, None
                #else:
                #    x_pathpoints_smoothed, x_pathpoints_smoothed, weights2 = loess_1d(x_pathpoints, y_pathpoints, x_interp, frac=0.5, degree=1)
                
                #self.path_storage_smooth = np.row_stack((x_pathpoints_smoothed, x_pathpoints_smoothed))
                #print(self.path_storage_smooth)
                #print(self.path_storage_smooth.shape[0])
                #print(self.path_storage_smooth.shape[1])
            
            # if self.path_storage_smooth.shape[0] >= self.numPos: 
            #     self.control = "MPC"
            #     print("Ensuring MPC is possible")
            # else:
            #     self.control = "PID"

          
            if dist2human > self.d_follow: # perform the path following control using either MPC or PID
                print("STATE ===> Follow the human") 
                if self.control == "MPC":
                    print("STATE ===> Follow the human=====>MPC") 
                    # Perform MPC optimization
                    #st = time.time()
                    #self.optimizeProblem.solve_problem(self.path_storage_smooth, [self.vRef,self.wRef])
                    v_last = self.ctrlMsgs.value.forward
                    w_last = self.ctrlMsgs.value.left
                    self.optimizeProblem.solve_problem(path_for_mpc, [v_last,w_last]) # this shopud be the last velocity
                    
                    self.vRef = self.optimizeProblem.Controls[0, 0]  # Linear velocity [m/s]
                    self.wRef = self.optimizeProblem.Controls[0, 1]  # Angular velocity [rad/s]
                elif self.control == "PID":
                    print("STATE ======>> Follow the human path =====>PID") 
                    # Perform PID self.control
                    vRef = 3 * np.linalg.norm(self.path_storage[:, 0])  # Linear velocity [m/s]
                    phi = np.arctan2(self.path_storage[1, 0], self.path_storage[0, 0])
                    wRef = 5 * phi  # Angular velocity [rad/s]

                    x_pathpoint, y_pathpoint = self.path_storage[:, 0]

                    vRef = np.clip(vRef, self.v_min, self.v_max)
                    wRef = np.clip(wRef, self.omega_min, self.omega_max)



                elif self.control == "PID0":                 
                    self.rot_controller.put(angle)
                    self.trans_controller.put(abs(x_pos-self.d_follow))

                    self.rot_controller.run()
                    self.trans_controller.run()

                    self.vRef = self.trans_controller.get()
                    self.wRef = self.rot_controller.get()
                
                elif self.control == "PP":
                    path_forward = self.discard_points_behind(self.path_storage, x_eps=0.5)
                    #P_fwd = self.discard_points_behind(self.path_storage, x_eps=0.5)

                    if P_fwd is None or P_fwd.shape[1] == 0:
                        # nothing ahead → stop safely
                        vRef, wRef = 0.0, 0.0
                    else:
                        vRef, wRef, target_pt, kappa, idx = self.pure_pursuit(
                            path_forward, 
                            Ld=0.8,
                            v_max=1.0,
                            omega_max=np.pi/1.7
                        )
                    self.vRef = vRef
                    self.wRef = wRef
                                    

                ## remove points if follow is run
                if self.path_storage.shape[1]>1:
                    self.path_storage = self.path_storage[:, 1:]
                    #self.path_storage = self.find_forward_points()
                   

            elif (dist2human>= self.d_stop and dist2human <= self.d_follow):#rotate to finde the human
                print("STATE ===> Rotate to find the human")
                if abs(angle) <= 0.5:
                    angle = 0.0
                    self.wRef = 0.0
                else:
                    self.trans_controller.reset()
                    if angle > 0: self.wRef = 0.8
                    else: self.wRef = -0.8

                    #self.wRef = self.rot_controller.get()+0.2
                self.vRef = 0.0
                self.path_storage = np.zeros((2, 1))
            
        else: # Robot stop
            print("STATE ===> STOP") 
            self.vRef = 0.0
            self.wRef = 0.0
            self.path_storage = np.zeros((2, 1))
        
                 

        
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
        self.ctrlMsgs.value.forward = 0.0
        self.ctrlMsgs.value.left = 0.0

        self.ctrlMsgs.value.forward = self.vRef
        self.ctrlMsgs.value.left = self.wRef

        # publish motion command
        self.vel_cmd_publisher_.publish(self.ctrlMsgs)
        #if self.DEBUG:
        self.get_logger().info(f'Publishing: forward={self.ctrlMsgs.value.forward:.3f}, left={self.ctrlMsgs.value.left:.3f}, time = {(time.time()-start_time):.2f}')
        
## ======== srart of helper functions ==========
    def discard_points_behind(path_2xN, x_eps=0.5):
        """
        Keep only points with x >= x_eps (i.e., in front of the robot).
        path_2xN: shape (2, N), points in robot frame.
        x_eps: small positive margin to avoid points right on the axle, e.g. 0.02 m.
        """
        if path_2xN is None or path_2xN.size == 0:
            return path_2xN
        mask = path_2xN[0, :] >= x_eps
        return path_2xN[:, mask]
    # ========== pp controller for path following ==========
    def pure_pursuit(path_2xN,
                 Ld=0.8,                 # lookahead distance [m]
                 v_max=1.0,              # max linear speed [m/s]
                 omega_max=np.pi/1.7,    # max |angular| speed [rad/s]
                 min_Ld=1e-3):
        """
        path_2xN: np.ndarray shape (2, N), points in the *current robot frame* (x forward, y left).
                Column 0 is the closest/following-first point in time (e.g., from your path_storage).
        Returns: vRef, wRef, target_pt (2,), kappa, target_idx
        """

        # --- Edge cases ---
        if path_2xN is None or path_2xN.size == 0 or path_2xN.shape[1] == 0:
            return 0.0, 0.0, np.array([0.0, 0.0]), 0.0, -1
        P = path_2xN

        # If only one point, use it as target (clamped to Ld along its direction)
        if P.shape[1] == 1:
            p = P[:, 0]
            r = np.linalg.norm(p)
            if r < min_Ld:
                return 0.0, 0.0, p, 0.0, 0
            target = p * (Ld / r)
            yL = target[1]
            kappa = 2.0 * yL / max(Ld**2, min_Ld**2)
            vRef = np.clip(v_max / (1.0 + 0.5 * abs(kappa)), 0.0, v_max)
            wRef = np.clip(kappa * vRef, -omega_max, omega_max)
            return vRef, wRef, target, kappa, 0

        # --- 1) Build cumulative arc-length along the polyline ---
        diffs = np.diff(P, axis=1)                          # (2, N-1)
        seglens = np.linalg.norm(diffs, axis=0)             # (N-1,)
        s = np.concatenate(([0.0], np.cumsum(seglens)))     # (N,)

        total_len = s[-1]
        if total_len < min_Ld:
            # Path is tiny; fall back to first point direction
            p = P[:, 0]
            r = np.linalg.norm(p)
            if r < min_Ld:
                return 0.0, 0.0, p, 0.0, 0
            target = p * (Ld / r)
            yL = target[1]
            kappa = 2.0 * yL / max(Ld**2, min_Ld**2)
            vRef = np.clip(v_max / (1.0 + 0.5 * abs(kappa)), 0.0, v_max)
            wRef = np.clip(kappa * vRef, -omega_max, omega_max)
            return vRef, wRef, target, kappa, 0

        # --- 2) Find the first point at/after the lookahead distance ---
        # If Ld exceeds path length, use the last point.
        if Ld >= total_len:
            target = P[:, -1]
            target_idx = P.shape[1] - 1
        else:
            idx = int(np.searchsorted(s, Ld, side='left'))      # 1..N-1
            # Interpolate between P[:, idx-1] and P[:, idx] to hit Ld exactly
            s0, s1 = s[idx-1], s[idx]
            denom = max(s1 - s0, min_Ld)
            t = (Ld - s0) / denom
            target = (1.0 - t) * P[:, idx-1] + t * P[:, idx]
            target_idx = idx

        # --- 3) Pure pursuit curvature (robot at origin, heading +x) ---
        Ld_eff = max(np.linalg.norm(target), min_Ld)
        yL = target[1]                              # lateral offset of the lookahead point
        kappa = 2.0 * yL / (Ld_eff**2)              # curvature

        # --- 4) Speeds: slow down on tight curvature, clip outputs ---
        vRef = np.clip(v_max / (1.0 + 0.5 * abs(kappa)), 0.0, v_max)  # forward-only; set to [-v_max,v_max] if you allow reverse
        wRef = np.clip(kappa * vRef, -omega_max, omega_max)

        return vRef, wRef, target, kappa, target_idx

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
