import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_prefix
from cb_interfaces.srv import FollowPath
from cb_interfaces.msg import Flags
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_ros import TransformBroadcaster
from tf2_geometry_msgs import do_transform_pose
import numpy as np
import math
import os
import csv
import time

## Sources: 
## https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html 
## https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html 
## message Landmark: http://wiki.ros.org/rviz/DisplayTypes/Marker

class PPcontroller(Node):
    def __init__(self):
        super().__init__('pp_control_adaptive')

        # Subscriber for flags 
        self.Flags_sub = self.create_subscription(Flags, 'cb/Flags', self.listener_callback_flags, 10)

        # Service named for path following
        self.service = self.create_service(FollowPath, '/follow_path', self.follow_path_callback)

        # Subscriber for robot pose
        self.pose_sub = self.create_subscription(Pose, '/pose', self.listener_callback_pose, 10)

        # Subscriber for AprilTag information
        self.landmark_sub = self.create_subscription(Marker, '/landmark', self.listener_callback_landmark, 2)

        # Publisher for calculated velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for flags
        self.Flags_pub = self.create_publisher(Flags, '/cb/Flags', 10)

        # Transform broadcaster for TF2 frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # Buffer and listener for TF2 transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initial values for marker timestamp and ID
        self.marker_timestamp = 0
        self.marker_ID = 0
        self.noMarker = 0

        # Width of camera frame
        self.frame_width = 540

        # Initial value for mean of the x-coordinates of AprilTag corners
        self.landmark_x_corner_mean = 0

        # Cancel flag
        self.cancel = False

        # Dynamic Lookahead Distance
        self.dynamic_Lf = 1.0

    
    def follow_path_callback(self, request, response):
        # Callback function triggered when the service FollowPath is called
        self.traj_file_path = request.traj_file
        self.backwards = request.backwards

        # Load the path, start and end IDs, and marker positions from csv-file
        self.taught_path, self.start_ID, self.end_ID, self.start_marker, self.end_marker = self.get_path()        
        
        # Check if the robot is at the start or end of the path based on the current marker detection 
        if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5: 
            if self.marker_ID == self.start_ID and self.backwards == True:
                response.started = False
                self.get_logger().error("wrong direction!")
                return response
            elif self.marker_ID == self.end_ID and self.backwards == False:
                response.started = False
                self.get_logger().error("wrong direction!")
                return response
            elif self.marker_ID != self.end_ID and self.marker_ID != self.start_ID:
                self.get_logger().error("AprilTag is not part of path!")
                response.started = False
                return response
        
        #If no AprilTag detected recently, could abort (commented out)
        else:
            self.get_logger().warning(f'No AprilTag detected! Diff: {self.get_clock().now().to_msg().sec - self.marker_timestamp}')
            self.get_logger().warning(f'Just repeat the path')
            self.noMarker = 1
            #response.started = False
            #return response
        
        # If the path should be driven backwards, invert the path array
        if self.backwards == True:
            self.taught_path = np.flip(self.taught_path, axis=0)

        # Initialize variables for the repeat
        self.previous_location = 0
        self.start_location = 0
        self.begin = True
        self.turned = False
        self.homing_state = 0
        self.arrived = False
        self.homed = False
        self.end_angle = 0.0
        self.corrected_pose = [0.0,0.0,1]
        self.wRef_last = 0.0
        self.backup = False
        self.search = True
        self.start_timer = 0
        self.dist_corrected_pose = 0.0
        self.corrected = False
        self.error = 0.0
        self.integral = 0.0
        self.derivat = 0.0
        self.error_old = 0.0

        # Timer that calls repeat-code
        self.timer = self.create_timer(0.1, self.timer_repeat)

        # Response if repeat started successfully
        response.started = True
        return response

    def timer_repeat(self):

        if self.arrived == False:
            # If not arrived yet: path following with pp-controller
            vRef, wRef, location = self.ppcontroller(self.taught_path, self.cur_pos_x, self.cur_pos_y, self.cur_ori_z)
        elif self.arrived == True and self.noMarker == 1:
            # If started without AprilTag detection: stop after condition is made and stop
            self.homed = True
        else:
            # If arrived: align robot with AprilTag
            vRef, wRef = self.align_AprilTag(self.cur_pos_x, self.cur_pos_y, self.cur_ori_z)
        
        cmd_msg = Twist()
        Flags_msg = Flags()

        if self.homed == False and self.cancel == False:
            # Drive if not homed (aligned) and not cancelled
            cmd_msg.linear.x = float(vRef)
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= float(wRef)

            # Publish velocity commands
            self.cmd_vel_pub.publish(cmd_msg)

            # Set and publish flags accordingly
            Flags_msg.turned_around = self.turned
            Flags_msg.arrived = self.arrived
            Flags_msg.homed = self.homed
            Flags_msg.cancel = False
            self.Flags_pub.publish(Flags_msg)

            # self.get_logger().info(f'Driving to Node: {location}')
        else:
            # If homed or cancelled, stop all motion
            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= 0.0

            # Publish stop command
            self.cmd_vel_pub.publish(cmd_msg)

            # Update flags
            Flags_msg.turned_around = self.turned
            Flags_msg.arrived = self.arrived
            Flags_msg.homed = self.homed
            Flags_msg.cancel = False
            self.Flags_pub.publish(Flags_msg)

            self.get_logger().info('final Node reached!')

            # Stop the timer and the repeat
            self.timer.cancel()
        
    def quaternion_to_euler(self, qx, qy, qz, qw):
        # Convert quaternion (qx, qy, qz, qw) to Euler angles (roll, pitch, yaw)

        t1_x = 2 * ((qw * qx) + (qy * qz))
        t2_x = 1 - (2 * ((qx * qx) + (qy * qy)))
        roll = math.atan2(t1_x, t2_x)

        t1_y = math.sqrt(1 + (2 * ((qw * qy) - (qx * qz))))
        t2_y = math.sqrt(1 - (2 * ((qw * qy) - (qx * qz))))
        pitch = (2 * math.atan2(t1_y, t2_y)) - (math.pi / 2)

        t1_z = 2 * ((qw * qz) + (qx * qy))
        t2_z = 1 - (2 * ((qy * qy) + (qz * qz)))
        yaw = math.atan2(t1_z, t2_z)

        return roll, pitch, yaw
    
    def listener_callback_flags(self, msg):
        # Callback for Flags subscription, updates the cancel flag
        self.cancel = msg.cancel
                

    def listener_callback_pose(self, msg):
        # Callback for Pose subscription, updates current position and orientation of the robot

        # Extract position data from the message
        self.cur_pos_x = msg.position.x
        self.cur_pos_y = msg.position.y
        self.cur_pos_z = msg.position.z

        # Quaternion of orientation from the message
        cur_ori_x_quat = msg.orientation.x
        cur_ori_y_quat = msg.orientation.y
        cur_ori_z_quat = msg.orientation.z
        cur_ori_w_quat = msg.orientation.w

        # Convert quaternion to Euler angles
        self.cur_ori_x, self.cur_ori_y, self.cur_ori_z = self.quaternion_to_euler(cur_ori_x_quat,cur_ori_y_quat,cur_ori_z_quat,cur_ori_w_quat)

    def listener_callback_landmark(self, msg):
        # Callback for Landmark subscription
        # Extract marker ID and timestamp
        self.marker_ID = msg.id
        self.marker_timestamp = msg.header.stamp.sec

        # Try to get transform from 'marker_link' to 'base_link'
        try:
            transform_markerlink_baselink = self.tf_buffer.lookup_transform('base_link', 'marker_link',rclpy.time.Time())

        except Exception as e:
            self.get_logger().error(f'transform failed: base_link to marker_link: {e}')
            return

        # Relative position of marker to base_link
        self.rel_marker_pos_x = transform_markerlink_baselink.transform.translation.x
        self.rel_marker_pos_y = transform_markerlink_baselink.transform.translation.y
        self.rel_marker_pos_z = transform_markerlink_baselink.transform.translation.z

        # Relative orientation of marker to base_link as quaternion
        rel_marker_ori_x_quat = transform_markerlink_baselink.transform.rotation.x
        rel_marker_ori_y_quat = transform_markerlink_baselink.transform.rotation.y
        rel_marker_ori_z_quat = transform_markerlink_baselink.transform.rotation.z
        rel_marker_ori_w_quat = transform_markerlink_baselink.transform.rotation.w

        # Convert quaternion to Euler angles
        self.rel_marker_ori_x, self.rel_marker_ori_y, self.rel_marker_ori_z = self.quaternion_to_euler(rel_marker_ori_x_quat,rel_marker_ori_y_quat,rel_marker_ori_z_quat,rel_marker_ori_w_quat)
        self.rel_marker_euler_angles = np.array([self.rel_marker_ori_x,self.rel_marker_ori_y,self.rel_marker_ori_z])
        
        # Corner points of the marker from the message
        landmark_corners = msg.points

        # Calculate mean of the x-coordinates of the four corners of the marker
        landmark_corner1_x = landmark_corners[0].x
        landmark_corner2_x = landmark_corners[1].x
        landmark_corner3_x = landmark_corners[2].x
        landmark_corner4_x = landmark_corners[3].x
        self.landmark_x_corner_mean = np.mean([landmark_corner1_x, landmark_corner2_x, landmark_corner3_x, landmark_corner4_x])


    def ppcontroller(self,taught_path,x,y,teta):
        Lf = 1.0#1.5 # look ahead distance (meters)
        Lf_start = 1.0#1.5 # look ahead distance for start (meters)
        Vc = 1.0; # desired velocity in [m/s]
        end_dist = 0.3 # Min distance between current Pose and last Node
        end_dist_location = 30 # Min number of nodes between current Pose and last Node
        max_node_skip = 50 # Max number of nodes, we can look further ahead
        turn_tol = 0.3 # tollerance for turning
        accel_distance = 200 # distance robot will be accelerating or breaking (number of nodes)
        wRef = 0.0 # default value
        accel_distance_meter = 5.0#5.0 # distance vor robot to accelerate or brake (meters)
        
        

        # Calculate distance from current position (x,y) to all waypoints in taught_path
        distance = np.sqrt((taught_path[:,1] - x)**2 + (taught_path[:,2] - y)**2)
        position = np.argmin(distance)  # Index of the closest waypoint to current position
        
        #diff = abs(distance - Lf)  # Difference between each distance and lookahead distance
        diff = abs(distance - self.dynamic_Lf)  # Difference between each distance and lookahead distance
        
        # Initialization block when controller starts
        if self.begin == True:
            diff = abs(distance - Lf_start)  # Use different lookahead distance for start
            self.start_location = position  # Save starting waypoint index
            self.previous_location = position
            self.begin = False  # Mark initialization complete
        
        # Find the next waypoint index ahead of the robot, within max_node_skip limit
        location = self.previous_location + np.argmin(diff[self.previous_location:(self.previous_location + max_node_skip)])
        
        # Clamp location index so it does not go out of range
        if location >= (np.size(taught_path, 0) - 1):
            location = (np.size(taught_path, 0) - 1)
        
        # Update previous location for next iteration
        self.previous_location = location
        
        # Extract the goal point (lookahead waypoint) coordinates
        gx = taught_path[location, 1]
        gy = taught_path[location, 2]
        
        # Calculate Euclidean distance to the lookahead point
        Ld = distance[location]
        
        # If robot is far away from path (over 4m), stop repeat
        if Ld > 4.0:
            self.homed = True
            return 0, 0, 0  # Stop moving
        
        # Distances to start and end of path, used for acceleration/deceleration profiles
        Ld_end = distance[np.size(taught_path, 0) - 1]
        Ld_start = distance[0]
        Ld_end_location = (np.size(taught_path, 0) - 1) - location  # Remaining nodes to path end
        
        # Try to get transform from 'map' frame to 'base_link' (robot frame)
        try:
            transform_baselink_map = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
            
            # Create a Pose object for the lookahead point in map frame
            next_node = Pose()
            next_node.position.x = gx
            next_node.position.y = gy
            next_node.position.z = 0.0
            next_node.orientation.w = 1.0
            
            # Transform the lookahead point into the robot's base_link frame
            next_node_transformed = do_transform_pose(next_node, transform_baselink_map)
        
        except Exception as e:
            self.get_logger().error(f'transform failed: map to baselink: {e}')
            return
        
        # Control logic depending on whether the robot has turned yet
        if self.turned == False and self.arrived == False:
            # Check if robot needs to turn on spot to align with path direction
            if next_node_transformed.position.y > 0 and (next_node_transformed.position.y > turn_tol or next_node_transformed.position.x < 0):
                vRef = 0.0
                wRef = 0.3  # Turn left
            elif next_node_transformed.position.y < 0 and (next_node_transformed.position.y < -turn_tol or next_node_transformed.position.x < 0):
                vRef = 0.0
                wRef = -0.3  # Turn right
            elif next_node_transformed.position.x < 0:
                vRef = 0.0
                wRef = -0.3  # Turn right
            else:
                vRef = 0.0
                wRef = 0.0
                self.turned = True  # Mark turning complete
        
        elif self.turned == True and self.arrived == False:
            # Adjust speed based on distance from start and end of path for acceleration/deceleration
            
            # Slow down near start of path
            if abs(self.start_location - position) < accel_distance and Ld_start < accel_distance_meter:
                speed_percentage = (accel_distance_meter - Ld_start) / accel_distance_meter
                Vc -= Vc * 0.8 * speed_percentage
            
            # Slow down near end of path
            if abs((np.size(taught_path, 0) - 1) - position) < accel_distance and Ld_end < accel_distance_meter:
                speed_percentage = (accel_distance_meter - Ld_end) / accel_distance_meter
                Vc -= Vc * 0.8 * speed_percentage
            # calculate the Curvature
            k = next_node_transformed.position.y / Ld**2

            r_min = 3.60  # Minimum radius of curvature
            if abs(k) > 1/r_min:
                
                Vc =  Vc/(r_min*abs(k))  # Limit speed if curvature is too high
            else:
                Vc = Vc
            #self.get_logger().warning(f'Current Curvature k is : {k}')
            
            #Apply Curvature base Velocity Adjustmen
            self.dynamic_Lf = Vc * 2
         
            
            # Calculate angular velocity using pure pursuit formula: w = 2*v*y / Ld^2
            wRef = 2 * Vc * next_node_transformed.position.y / Ld**2
            vRef = Vc  # Set linear velocity to desired speed
            self.get_logger().warning(f'--- vRef = {round(vRef,2)}, Kruemmung = {round(k,2)}, Vc = {round(Vc,2)}, wRef = {round(wRef,2)},--- Dy. Lf = {round(self.dynamic_Lf,2)}---') #, Ld_end = {round(Ld_end,2)}, Ld_end_location = {round(Ld_end_location,2)} ---')
        
        # Check if robot has arrived at the goal location based on distance thresholds
        if Ld_end < end_dist and Ld_end_location < end_dist_location and self.arrived == False:
            self.arrived = True
            self.end_angle = teta  # Store final orientation angle
        
        return vRef, wRef, location  # Return velocity commands and current lookahead location
    
    
    def align_AprilTag(self, x, y, teta):
        # Align the robot using detected AprilTags and a state machine for homing

        # values for PD-controller
        Kp = 0.5
        Kd = 0.1
        Ki = 0.0
        dt = 0.1
        
        if self.homing_state == 0:
            # State 0: Rotate slowly to search for the target AprilTag
            
            vRef = 0.0
            wRef = 0.1  # Rotate counterclockwise slowly
            
            # If orientation difference is large, and marker hasn't been seen recently, search in different direction
            if abs(teta - self.end_angle) > math.pi/2 and self.get_clock().now().to_msg().sec - self.marker_timestamp > 5 and self.search == True:
                self.homing_state = 1
            
            # If AprilTag was detected recently check alignment conditions
            if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == False and self.end_ID == self.marker_ID:
                # If the detected tag is on right half of image frame, switch to search state 1
                if self.landmark_x_corner_mean > self.frame_width/2:
                    self.homing_state = 1
                
                # If the detected tag is near center horizontally in image
                if abs(self.frame_width/2 - self.landmark_x_corner_mean) < 20:
                    # If orientation or distance correction needed, move to state 2
                    if (abs(self.rel_marker_ori_z - math.pi/2) > 0.2 or abs(self.rel_marker_pos_x) < 1.0) and self.corrected == False:
                        self.homing_state = 2
                    else:
                        # Otherwise, alignment is good, move to final state 5
                        self.homing_state = 5
            
            # Similar checks if robot is following path backwards 
            elif self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == True and self.start_ID == self.marker_ID:
                if self.landmark_x_corner_mean > self.frame_width/2:
                    self.homing_state = 1
                if abs(self.frame_width/2 - self.landmark_x_corner_mean) < 20:
                    if (abs(self.rel_marker_ori_z - math.pi/2) > 0.2 or abs(self.rel_marker_pos_x) < 1.0) and self.corrected == False:
                        self.homing_state = 2
                    else:
                        self.homing_state = 5
        
        elif self.homing_state == 1:
            # State 1: Rotate slowly in the opposite direction to find the tag
            
            self.search = False
            vRef = 0.0
            wRef = -0.1  # Rotate clockwise slowly
            
            # Check if AprilTag has been detected recently and correct alignment if near center
            if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == False and self.end_ID == self.marker_ID:
                # If the detected tag is on left half of image frame, switch to search state 0
                if self.landmark_x_corner_mean < self.frame_width/2:
                    self.homing_state = 0 
                     
                # If the detected tag is near center horizontally in image   
                if abs(self.frame_width/2 - self.landmark_x_corner_mean) < 20:
                    # If orientation or distance correction needed, move to state 2
                    if (abs(self.rel_marker_ori_z - math.pi/2) > 0.2 or abs(self.rel_marker_pos_x) < 1.0) and self.corrected == False:
                        self.homing_state = 2 
                    else:
                        # Otherwise, alignment is good, move to final state 5
                        self.homing_state = 5 
            
            # Similar checks if robot is following path backwards
            elif self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == True and self.start_ID == self.marker_ID:
                if self.landmark_x_corner_mean < self.frame_width/2:
                    self.homing_state = 0
                if abs(self.frame_width/2 - self.landmark_x_corner_mean) < 20:
                    if (abs(self.rel_marker_ori_z - math.pi/2) > 0.2 or abs(self.rel_marker_pos_x) < 1.0) and self.corrected == False:
                        self.homing_state = 2
                    else:
                        self.homing_state = 5
        
        elif self.homing_state == 2:
            # State 2: creat correction pose
            
            self.search = False
            
            # Start timer on first entry into this state
            if self.start_timer == 0:
                self.start_timer = self.get_clock().now().to_msg().sec
            
            vRef = 0.0
            wRef = 0.0  # Stop movement 
            
            # After 3 seconds, perform transform for correction pose
            if self.get_clock().now().to_msg().sec - self.start_timer > 3:
                
                # If tag detected recently and robot not backing up
                if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == False and self.end_ID == self.marker_ID:
                    try:
                        # Get transform from map frame to the marker frame
                        transform_map_markerlink = self.tf_buffer.lookup_transform('map', 'marker_link', rclpy.time.Time())
                        
                        # Create a backup point pose in marker frame
                        backup_point = Pose()
                        backup_point.position.x = 0.0
                        backup_point.position.y = 0.0
                        backup_point.position.z = 1.5
                        backup_point.orientation.x = 0.0
                        backup_point.orientation.y = 0.0
                        backup_point.orientation.z = 0.0
                        backup_point.orientation.w = 1.0
                        
                        # Transform the backup point pose into the map frame
                        self.corrected_pose = do_transform_pose(backup_point, transform_map_markerlink)
                    
                    except Exception as e:
                        self.get_logger().error(f'transform failed: map to baselink: {e}')
                        return
                    
                    # Decide if robot should back up based on relative position to correction pose
                    if self.rel_marker_pos_z < 1.3:
                        self.backup = True
                    else:
                        self.backup = False
                    
                    # Reset timer and move to next homing state
                    self.start_timer = 0
                    self.homing_state = 3
                
                # Similar logic if robot is following path backwards
                elif self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.backwards == True and self.start_ID == self.marker_ID:
                    try:
                        transform_map_markerlink = self.tf_buffer.lookup_transform('map', 'marker_link', rclpy.time.Time())
                        backup_point = Pose()
                        backup_point.position.x = 0.0
                        backup_point.position.y = 0.0
                        backup_point.position.z = 1.5
                        backup_point.orientation.x = 0.0
                        backup_point.orientation.y = 0.0
                        backup_point.orientation.z = 0.0
                        backup_point.orientation.w = 1.0
                        self.corrected_pose = do_transform_pose(backup_point, transform_map_markerlink)
                    except Exception as e:
                        self.get_logger().error(f'transform failed: map to baselink: {e}')
                        return
                    
                    if self.rel_marker_pos_z < 1.3:
                        self.backup = True
                    else:
                        self.backup = False
                    
                    self.start_timer = 0
                    self.homing_state = 3
                
                else:
                    # If no matching AprilTag is detected, error
                    self.get_logger().error("No matching AprilTag detected!")
                

        if self.homing_state == 3:
            vRef = 0.0
            wRef = 0.0
            try:
                # Lookup transform from 'base_link' to 'map' frame 
                transform_baselink_map = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
        
                # Transform the corrected pose from map frame to base_link frame
                corrected_pose_transformed = do_transform_pose(self.corrected_pose, transform_baselink_map)
        
            except Exception as e:
                self.get_logger().error(f'transform failed: map to baselink: {e}')
                return
        
            # Decide turning direction
            if corrected_pose_transformed.position.y > 0 and self.backup:
                vRef = 0.0
                wRef = -0.1  # Turn clockwise while backing up
                self.wRef_last = wRef
            elif corrected_pose_transformed.position.y > 0 and not self.backup:
                vRef = 0.0
                wRef = 0.1   # Turn counterclockwise while moving forward
                self.wRef_last = wRef
            elif corrected_pose_transformed.position.y < 0 and self.backup:
                vRef = 0.0
                wRef = 0.1   # Turn counterclockwise while backing up
                self.wRef_last = wRef
            elif corrected_pose_transformed.position.y < 0 and not self.backup:
                vRef = 0.0
                wRef = -0.1  # Turn clockwise while moving forward
                self.wRef_last = wRef
        
            # If robot is aligned close enough, stop rotation and switch to next homing state
            if abs(corrected_pose_transformed.position.y) < 0.05:
                vRef = 0.0
                wRef = 0.0
                # Calculate the distance from robot to correction pose
                self.dist_corrected_pose = math.sqrt(corrected_pose_transformed.position.x**2 + corrected_pose_transformed.position.y**2)
                self.homing_state = 4  
        
        if self.homing_state == 4:
            try:
                # Lookup transform from 'base_link' to 'map' frame 
                transform_baselink_map = self.tf_buffer.lookup_transform('base_link', 'map', rclpy.time.Time())
                
                # Transform the corrected pose from map frame to base_link frame
                corrected_pose_transformed = do_transform_pose(self.corrected_pose, transform_baselink_map)
            except Exception as e:
                self.get_logger().error(f'transform failed: map to baselink: {e}')
                return
        
            # Calculate distance to correction pose in base_link frame
            Ld = math.sqrt(corrected_pose_transformed.position.x**2 + corrected_pose_transformed.position.y**2)
        
            # Set linear velocity forward or backward depending on backup flag
            if self.backup:
                vRef = -0.05   # Move backwards slowly
            else:
                vRef = 0.05    # Move forward slowly
        
            # Calculate angular velocity to correct heading based on lateral error
            wRef = 2 * vRef * corrected_pose_transformed.position.y / Ld**2
        
            # If robot has moved at least halfway to target pose, switch homing state
            if math.sqrt(corrected_pose_transformed.position.x**2 + corrected_pose_transformed.position.y**2) < (self.dist_corrected_pose / 2):
                if self.wRef_last < 0:
                    self.homing_state = 0
                elif self.wRef_last > 0:
                    self.homing_state = 1
        
        if self.homing_state == 5:
            # Start timer at beginning of this state
            if self.start_timer == 0:
                self.start_timer = self.get_clock().now().to_msg().sec
                self.corrected = True
        
            vRef = 0.0
            wRef = 0.0
        
            # Wait 2 seconds
            if self.get_clock().now().to_msg().sec - self.start_timer > 2:
                # Check if marker 
                if (self.get_clock().now().to_msg().sec - self.marker_timestamp < 3 and
                    ((not self.backwards and self.end_ID == self.marker_ID) or
                     (self.backwards and self.start_ID == self.marker_ID))):
        
                    # Calculate error between center of frame and detected landmark
                    self.error = (self.frame_width / 2 - self.landmark_x_corner_mean) / 100
                    self.integral += self.error * dt
                    self.derivat = (self.error - self.error_old) / dt
        
                    # PID control for angular velocity
                    wRef = Kp * self.error + Kd * self.derivat + Ki * self.integral
                    self.error_old = self.error
        
                    # Move forward if marker is far enough away in x direction
                    if self.rel_marker_pos_x - 1.0 > 0.05:
                        vRef = 0.1
        
                    # If close to desired x and well centered, finish homing
                    elif self.rel_marker_pos_x - 1.0 <= 0.05 and abs(self.frame_width / 2 - self.landmark_x_corner_mean) < 20:
                        self.homed = True
                        self.search = False
        
                # If no marker detected for longer than 3 seconds, reset homing process
                if self.get_clock().now().to_msg().sec - self.marker_timestamp > 3:
                    self.homing_state = 0
                    self.search = True
                    self.end_angle = teta
                    self.start_timer = 0
        
        return vRef, wRef

            

    def get_path(self):
        # Check if file for traj was named
        if self.traj_file_path:
            self.get_logger().info(f'Follow path: {self.traj_file_path}')
        else:
            self.get_logger().error('No file for traj named!')            
        
        # read path from csv-file

        with open(self.traj_file_path, 'r', newline= '') as csv_file:
            reader = csv.reader(csv_file, delimiter=';')
            data = list(reader)
            csv_file.close()

        for row in data:
            if 'start_ID' in row[0]:
                start_ID = row[1]
            elif 'end_ID' in row[0]:
                end_ID = row[1]

        path = np.genfromtxt(self.traj_file_path, delimiter=';', skip_header=6, dtype=float)
        #path = np.genfromtxt(self.traj_file_path, delimiter=';', skip_header=2, dtype=float)

        return path, int(start_ID), int(end_ID), data[4], data[5]

def main(args=None):
    rclpy.init(args=args)
    node = PPcontroller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
