import rclpy
from rclpy.node import Node
from cb_interfaces.srv import EndPath
from cb_interfaces.srv import StartPath
from cb_interfaces.srv import RewritePath
from tf2_ros import TransformListener
from tf2_ros import Buffer
from visualization_msgs.msg import Marker
from pbstream.reader import PBstream_Reader
from collections import defaultdict
import numpy as np
import time
import math
import csv
import os

#Quelle: https://github.com/meyerjo/cartographer-read-pbstream | https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html##

class CartographerPath(Node):
    def __init__(self):
        super().__init__('current_path_sim')
        self.service = self.create_service(EndPath, '/end_traj', self.end_traj_callback)
        self.service = self.create_service(StartPath, '/start_traj', self.start_traj_callback)
        self.service = self.create_service(RewritePath, '/rewrite_traj', self.rewrite_traj_callback)
        self.landmark_sub = self.create_subscription(Marker, '/landmark', self.listener_callback_landmark, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.start_node = 0
        self.start_marker_ID = 0
        self.start_marker_pose_x = 0.0
        self.start_marker_pose_y = 0.0
        self.start_marker_pose_z = 0.0
        self.start_marker_ori_x = 0.0
        self.start_marker_ori_y = 0.0
        self.start_marker_ori_z = 0.0
        self.marker_timestamp = 0
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        #Convert quaternion to Euler angles
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
    
    def listener_callback_landmark(self, msg):
        # get general informations about marker
        self.marker_ID = msg.id
        self.marker_timestamp = msg.header.stamp.sec

        try:
            transform_markerlink_baselink = self.tf_buffer.lookup_transform('marker_link', 'base_link', rclpy.time.Time())

        except Exception as e:
            self.get_logger().error(f'transform failed: base_link to marker_link: {e}')
            return
        # get relative position to marker
        self.rel_marker_pos_x = transform_markerlink_baselink.transform.translation.x
        self.rel_marker_pos_y = transform_markerlink_baselink.transform.translation.y
        self.rel_marker_pos_z = transform_markerlink_baselink.transform.translation.z
        # get relative orientation to marker as quaternions
        rel_marker_ori_x_quat = transform_markerlink_baselink.transform.rotation.x
        rel_marker_ori_y_quat = transform_markerlink_baselink.transform.rotation.y
        rel_marker_ori_z_quat = transform_markerlink_baselink.transform.rotation.z
        rel_marker_ori_w_quat = transform_markerlink_baselink.transform.rotation.w
        # transform quaternions into euler angles
        self.rel_marker_ori_x, self.rel_marker_ori_y, self.rel_marker_ori_z = self.quaternion_to_euler(rel_marker_ori_x_quat,rel_marker_ori_y_quat,rel_marker_ori_z_quat,rel_marker_ori_w_quat)
    
    def start_traj_callback(self, request, response):
        self.pbstream_file= request.pbstream_path
        self.dir_traj_folder, pbstream_file = os.path.split(self.pbstream_file)
        loaded = defaultdict(list)
        with PBstream_Reader(self.pbstream_file) as reader:
            for msg in reader:
                fields = msg.ListFields()
                if len(fields) == 0:
                    continue
                for (field_descriptor, message) in fields:
                    loaded[field_descriptor.name].append(message)

        # pick list of trajectorys form last generated pbstream-file
        length = len(loaded['pose_graph'][0].trajectory)-1
        nodes = loaded['pose_graph'][0].trajectory[length].node  

        # pick index of last nodes of every trajectory  
        length_nodes = len(nodes)-1
        self.start_node = nodes[length_nodes].node_index
        if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5 and self.marker_ID == 0:
            self.start_marker_ID = self.marker_ID
            self.start_marker_pose_x = self.rel_marker_pos_x
            self.start_marker_pose_y = self.rel_marker_pos_y
            self.start_marker_pose_z = self.rel_marker_pos_z
            self.start_marker_ori_x = self.rel_marker_ori_x
            self.start_marker_ori_y = self.rel_marker_ori_y
            self.start_marker_ori_z = self.rel_marker_ori_z
            response.start_node_saved = True
        else:
            response.start_node_saved = True
            self.get_logger().warning(f'Home-AprilTag not detected! Diff: {self.get_clock().now().to_msg().sec - self.marker_timestamp}')
            self.get_logger().warning(f'This is a very serious error!!!!!!!')
        return response

    def end_traj_callback(self, request, response):
        error = False

        self.pbstream_file= request.pbstream_path
        self.dir_traj_folder, pbstream_file = os.path.split(self.pbstream_file)
        loaded = defaultdict(list)
        self.rows = list()
        with PBstream_Reader(self.pbstream_file) as reader:
            for msg in reader:
                fields = msg.ListFields()
                if len(fields) == 0:
                    continue
                for (field_descriptor, message) in fields:
                    loaded[field_descriptor.name].append(message)
        
        map_num = int(pbstream_file.split('_')[1].split('.')[0])
        """
        file_list = os.listdir(self.dir_traj_folder)
        max_traj_num = 0
        for file in file_list:
            if 'trajectory' in file:
                traj_num = int(file.split('_')[1].split('.')[0])
                if traj_num > max_traj_num:
                    max_traj_num = traj_num
        """

        # pick list of trajectorys form last generated pbstream-file
        length = len(loaded['pose_graph'][0].trajectory)-1
        nodes = loaded['pose_graph'][0].trajectory[length].node  
            
        self.rows.append(['map_num:',map_num])
        self.rows.append(['accuracy:',1])
        if self.get_clock().now().to_msg().sec - self.marker_timestamp < 5:
            
            self.rows.append(['start_ID:',self.start_marker_ID])
            self.rows.append(['end_ID',self.marker_ID])
            self.rows.append([self.start_marker_pose_x,self.start_marker_pose_y,self.start_marker_pose_z,self.start_marker_ori_x,self.start_marker_ori_y,self.start_marker_ori_z])
            self.rows.append([self.rel_marker_pos_x,self.rel_marker_pos_y,self.rel_marker_pos_z,self.rel_marker_ori_x,self.rel_marker_ori_y,self.rel_marker_ori_z])

        else:
            self.rows.append(['start_ID:',self.start_marker_ID])
            self.rows.append(['end_ID','1'])
            self.rows.append([self.start_marker_pose_x,self.start_marker_pose_y,self.start_marker_pose_z,self.start_marker_ori_x,self.start_marker_ori_y,self.start_marker_ori_z])
            self.rows.append(['end_transform','Flase'])
            error = False
            self.get_logger().warning(f'No AprilTag detected! Diff: {self.get_clock().now().to_msg().sec - self.marker_timestamp}')

        # pick list of nodes of every trajectory  
        for node in nodes:
            # Extract index, translation pose and rotation from each node
            index = node.node_index
            if index >= self.start_node:
                position = node.pose.translation 
                rotation = node.pose.rotation
                roll, pitch, yaw = self.quaternion_to_euler(rotation.x,rotation.y,rotation.z,rotation.w)
                # print(f"Node {index}: x={position.x}, y={position.y}, yaw={yaw}")
                self.rows.append([index,round(position.x,3),round(position.y,3),round(yaw,3)])

        self.cut_path()
        response.start_node = self.start_node
        response.final_node = index
        #response.traj_num = self.marker_ID
        response.traj_num = 1


        if error == False:
            #response.csv_path = self.generate_csv(self.marker_ID)
            response.csv_path = self.generate_csv(1)
            self.start_node = 0
            self.start_marker_ID = 0
            self.start_marker_pose_x = 0.0
            self.start_marker_pose_y = 0.0
            self.start_marker_pose_z = 0.0
            self.start_marker_ori_x = 0.0
            self.start_marker_ori_z = 0.0
            self.start_marker_ori_z = 0.0
        else:
            response.csv_path = "False"

        return response
    
    def rewrite_traj_callback(self, request, response):
        start_node_rewrite = request.start_node
        end_node_rewrite = request.end_node
        traj_num = request.traj_num
        self.pbstream_file= request.pbstream_path
        self.dir_traj_folder, pbstream_file = os.path.split(self.pbstream_file)
        loaded = defaultdict(list)
        self.rows = list()
        with PBstream_Reader(self.pbstream_file) as reader:
            for msg in reader:
                fields = msg.ListFields()
                if len(fields) == 0:
                    continue
                for (field_descriptor, message) in fields:
                    loaded[field_descriptor.name].append(message)
        
        map_num = int(pbstream_file.split('_')[1].split('.')[0])
        accuracy, start_ID, end_ID, start_marker, end_marker = self.read_and_delete_csv(traj_num)

        # pick list of trajectorys form last generated pbstream-file
        length = len(loaded['pose_graph'][0].trajectory)-1
        nodes = loaded['pose_graph'][0].trajectory[length].node  
            
        self.rows.append(['map_num:',map_num])
        self.rows.append(['accuracy:',accuracy+1])
        self.rows.append(['start_ID:',start_ID])
        self.rows.append(['end_ID',end_ID])
        self.rows.append(start_marker)
        self.rows.append(end_marker)
        
        # pick list of nodes of every trajectory  
        for node in nodes:
            # Extract index, translation pose and rotation from each node
            index = node.node_index
            if index >= start_node_rewrite and index <= end_node_rewrite:
                position = node.pose.translation 
                rotation = node.pose.rotation
                roll, pitch, yaw = self.quaternion_to_euler(rotation.x,rotation.y,rotation.z,rotation.w)
                # print(f"Node {index}: x={position.x}, y={position.y}, yaw={yaw}")
                self.rows.append([index,round(position.x,3),round(position.y,3),round(yaw,3)])

        response.csv_path = self.generate_csv(traj_num)

        return response

    def cut_path(self):
        tol = 0.1
        start_index = 0
        nprows = np.array(self.rows[6:])

        distance_start = np.sqrt((nprows[:,1]-nprows[0,1])**2 + (nprows[:,2]-nprows[0,2])**2)
        for index,distance in enumerate(distance_start):
            if distance > tol:
                start_index = index
                break

        last_index = len(self.rows[6:])-1
        end_index = last_index
        distance_end = np.sqrt((nprows[:,1]-nprows[last_index,1])**2 + (nprows[:,2]-nprows[last_index,2])**2) 
        for index,distance in enumerate(np.flip(distance_end, axis=0)):
            if distance > tol:
                end_index = last_index-index
                break
        self.rows = self.rows[:6] + self.rows[6+start_index:6+end_index]

    def generate_csv(self, traj_num):
        if os.path.isdir(self.dir_traj_folder) == 0:
            os.makedirs(self.dir_traj_folder)
        traj_csv_file = os.path.join(self.dir_traj_folder, f'trajectory_{traj_num}.csv')
        if os.path.isfile(traj_csv_file):
            os.remove(traj_csv_file)
            # self.get_logger().info("old file was overwritten!")
        
        with open(traj_csv_file, 'w', newline= '') as csv_file:
            writer = csv.writer(csv_file,delimiter=';')
            writer.writerows(self.rows)
            csv_file.close()
        return traj_csv_file
    
    def read_and_delete_csv(self, traj_num):
        if os.path.isdir(self.dir_traj_folder) == 0:
            self.get_logger().error("wrong path!")
        traj_csv_file = os.path.join(self.dir_traj_folder, f'trajectory_{traj_num}.csv')

        with open(traj_csv_file, 'r', newline= '') as csv_file:
            reader = csv.reader(csv_file, delimiter=';')
            data = list(reader)
            csv_file.close()

        for row in data:
            if 'accuracy' in row[0]:
                accuracy = int(row[1])
            elif 'start_ID' in row[0]:
                start_ID = row[1]
            elif 'end_ID' in row[0]:
                end_ID = row[1]
        
        os.remove(traj_csv_file)

        return accuracy, int(start_ID), int(end_ID), data[4], data[5]

def main(args=None):
    rclpy.init(args=args)
    node = CartographerPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()