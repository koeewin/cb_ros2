import rclpy
from rclpy.node import Node
from carrierbot_interfaces.msg import Flags
from sensor_msgs.msg import Joy
from cartographer_ros_msgs.srv import WriteState
from carrierbot_interfaces.srv import EndPath, StartPath, RewritePath, FollowPath, ChangeCtrlmode
from ament_index_python.packages import get_package_prefix
from visualization_msgs.msg import Marker
import os
import time

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # Subscriptions to input topics
        self.Flags_sub = self.create_subscription(Flags, '/carrierbot/Flags', self.listener_callback_flags, 10)
        self.panel_sub = self.create_subscription(Joy, '/carrierbot/Panel', self.listener_callback_panel, 10)
        self.landmark_sub = self.create_subscription(Marker, '/landmark', self.listener_callback_landmark, 2)

        # Publisher for sending cancel flag
        self.Flags_pub = self.create_publisher(Flags, '/carrierbot/Flags', 10)

        # Determine workspace path for file operations
        self.dir_parts_ws = get_package_prefix('carrierbot').split(os.sep)
        self.dir_ws = os.sep.join(self.dir_parts_ws[:-2])
        self.working_office = os.path.join(self.dir_ws,'src/carrierbot/path_files')

        # State machine variables
        self.state = 1
        self.substate = 0
        self.follow = False
        self.teach = False
        self.current_pbstream_file = ''
        self.current_csv_file = ''
        self.current_traj_num = 0
        self.start_end_nodes = dict()
        self.response = None

        # Button states
        self.button_up = 0
        self.button_right = 0
        self.button_down = 0
        self.button_left = 0
        self.button_e = 0
        self.button_f = 0

        # Button press counters
        self.counter_button_up = 0
        self.counter_button_right = 0
        self.counter_button_down = 0
        self.counter_button_left = 0
        self.counter_button_e = 0
        self.counter_button_f = 0

        # State flags
        self.service_called = False
        self.turned_around = False
        self.arrived = False
        self.homed = False
        self.backwards = False
        self.canceled = False

        # Landmark marker info
        self.marker_ID = None
        self.marker_timestamp = 0

        # Create service clients and wait for their availability
        self.client_WriteState = self.create_client(WriteState, '/write_state')
        while not self.client_WriteState.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service WriteState not available, waiting...')
        self.request_WriteState = WriteState.Request()

        self.client_StartTraj = self.create_client(StartPath, '/start_traj')
        while not self.client_StartTraj.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service StartTraj not available, waiting...')
        self.request_StartTraj = StartPath.Request()

        self.client_EndTraj = self.create_client(EndPath, '/end_traj')
        while not self.client_EndTraj.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service EndTraj not available, waiting...')
        self.request_EndTraj = EndPath.Request()

        self.client_RewriteTraj = self.create_client(RewritePath, '/rewrite_traj')
        while not self.client_RewriteTraj.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service RewriteTraj not available, waiting...')
        self.request_RewriteTraj = RewritePath.Request()

        self.client_FollowPath = self.create_client(FollowPath, '/follow_path')
        while not self.client_FollowPath.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service FollowPath not available, waiting...')
        self.request_FollowPath = FollowPath.Request()

        self.client_ChangeCtrlmode = self.create_client(ChangeCtrlmode, '/change_ctrlmode')
        while not self.client_ChangeCtrlmode.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service ChangeCtrlmode not available, waiting...')
        self.request_ChangeCtrlmode = ChangeCtrlmode.Request()

        # Start the state machine loop
        self.state_machine_timer = self.create_timer(0.05, self.state_machine)

    def state_machine(self):
        # Uncomment for debugging
        # print(f'state: {self.state}')
        # print(f'substate: {self.substate}')
        """
        #----------------------------------------------------------------------------------------------
        if self.state == 0:
            path_files_path = os.path.join(self.dir_ws,'src/carrierbot/path_files')
            path_files_offices = os.listdir(path_files_path)
            if self.substate == 0:
                self.get_logger().info("Soll ein neues office gestartet werden? (button e = Nein, button f = Ja)")
                self.substate = 1

            if self.substate == 1:
                if len(path_files_offices) < 3:
                    if self.button_f == 1 or len(path_files_offices) == 0:
                        new_office_path = os.path.join(path_files_path,f'office_{len(path_files_offices)+1}')
                        os.mkdir(new_office_path)
                        self.working_office = new_office_path
                        self.state = 1
                        self.substate = 0
                    elif self.button_e == 1:
                        self.substate = 2    
                else:
                    self.get_logger().info("Es kann kein neues office gestartet werden!")
                    self.substate = 2
            
            elif self.substate == 2:
                self.get_logger().info(f'W�hle ein Office aus zwischen 1 und {len(path_files_offices)}!')
                if self.button_left == 1 and len(path_files_offices) >= 1:
                    new_office_path = os.path.join(path_files_path,'office_1')
                    self.working_office = new_office_path
                    self.substate = 0
                    self.state = 1
                elif self.button_up == 1 and len(path_files_offices) >= 2:
                    new_office_path = os.path.join(path_files_path,'office_2')
                    self.working_office = new_office_path
                    self.substate = 0
                    self.state = 1
                elif self.button_right == 1 and len(path_files_offices) >= 3:
                    new_office_path = os.path.join(path_files_path,'office_3')
                    self.working_office = new_office_path
                    self.substate = 0
                    self.state = 1
        """   
        #----------------------------------------------------------------------------------------------
        if self.state == 1:
            # Substate 0: Switch to manual mode
            if self.substate == 0:
                self.request_ChangeCtrlmode.manuel = True
                self.request_ChangeCtrlmode.follow = False
                self.request_ChangeCtrlmode.repeat = False
                if not self.service_called:
                    self.call_service(self.client_ChangeCtrlmode, self.request_ChangeCtrlmode)
                    self.service_called = True
                if self.response is not None:
                    self.service_called = False
                    if self.response.changed:
                        self.response = None
                        self.follow = False
                        self.substate = 5

            # Substate 1: Switch to follow mode
            elif self.substate == 1:
                self.request_ChangeCtrlmode.manuel = False
                self.request_ChangeCtrlmode.follow = True
                self.request_ChangeCtrlmode.repeat = False
                if not self.service_called:
                    self.call_service(self.client_ChangeCtrlmode, self.request_ChangeCtrlmode)
                    self.service_called = True
                if self.response is not None:
                    self.service_called = False
                    if self.response.changed:
                        self.response = None
                        self.follow = True
                        self.substate = 5
            
            # Substate 2: Save current map state to pbstream file
            elif self.substate == 2:
                if not self.service_called:
                    file_list = os.listdir(self.working_office)
                    max_map_num = 0
                    # Find highest existing map number
                    for file in file_list:
                        if 'map' in file:
                            map_num = int(file.split('_')[1].split('.')[0])
                            max_map_num = max(max_map_num, map_num)
                    # Generate new filename and request write
                    self.current_pbstream_file = os.path.join(self.working_office, f'map_{max_map_num+1}.pbstream')
                    self.request_WriteState.filename = self.current_pbstream_file
                    self.call_service(self.client_WriteState, self.request_WriteState)
                    self.service_called = True
                if self.response is not None:
                    self.service_called = False
                    if self.response.status.code == 0:
                        self.get_logger().info(f'service called successfully! {self.response.status.message}')
                        self.response = None
                        self.substate = 3 if not self.teach else 4
                    else:
                        self.get_logger().error('service call failed!')

            # Substate 3: Save trajectory start point
            elif self.substate == 3:
                if os.path.exists(self.current_pbstream_file):
                    self.request_StartTraj.pbstream_path = self.current_pbstream_file
                    if not self.service_called:
                        self.call_service(self.client_StartTraj, self.request_StartTraj)
                        self.service_called = True
                    if self.response is not None:
                        self.service_called = False
                        if self.response.start_node_saved:
                            self.teach = True
                            self.get_logger().info('start node saved successfully')
                        else:
                            self.get_logger().error('service call failed!')
                        self.response = None
                        self.substate = 5

            # Substate 4: Save trajectory end point and finalize path
            elif self.substate == 4:
                if os.path.exists(self.current_pbstream_file):
                    self.request_EndTraj.pbstream_path = self.current_pbstream_file
                    if not self.service_called:
                        self.call_service(self.client_EndTraj, self.request_EndTraj)
                        self.service_called = True
                    if self.response is not None:
                        self.service_called = False
                        if "False" not in self.response.csv_path:
                            self.get_logger().info('start node saved successfully')
                            self.current_csv_file = self.response.csv_path
                            self.current_traj_num = self.response.traj_num
                            self.start_end_nodes[self.response.traj_num] = [self.response.start_node, self.response.final_node]
                            self.teach = False
                        else:
                            self.get_logger().error('service call failed!')
                        self.response = None
                        self.substate = 5

            # Substate 5: Wait for user input to change mode or initiate action
            elif self.substate == 5:
                files = os.listdir(self.working_office)
                csv_files = [f for f in files if "trajectory" in f]

                if self.button_e == 1:
                    self.substate = 1 if not self.follow else 0
                elif self.button_left == 1:
                    self.canceled = False
                    self.substate = 2
                elif self.button_up == 1:
                    self.backwards = True
                    self.follow = False
                    self.teach = False
                    self.substate = 0
                    self.state = 2
                elif self.button_right == 1:
                    self.backwards = False
                    self.follow = False
                    self.teach = False
                    self.substate = 0
                    self.state = 2
        #----------------------------------------------------------------------------------------------
        elif self.state == 2:
            
            if self.substate == 0:
                # Substate 0: change control mode to "repeat"
                self.request_ChangeCtrlmode.manuel = False
                self.request_ChangeCtrlmode.follow = False
                self.request_ChangeCtrlmode.repeat = True
        
                if self.service_called == False:
                    # Call the service to change control mode to repeat
                    self.call_service(self.client_ChangeCtrlmode, self.request_ChangeCtrlmode)
                    self.service_called = True
        
                if self.response != None:
                    self.service_called = False
                    
                    if self.response.changed == True and self.canceled == False:
                        # If mode change succeeded and not canceled, check marker conditions
                        self.response = None
                        if self.get_clock().now().to_msg().sec - self.marker_timestamp < 3:
                            # Marker was recently seen
                            if self.marker_ID == 0 and self.backwards == False:
                                # Enter selection mode for trajectory
                                self.substate = 1
                            elif self.marker_ID != 0 and self.backwards == True:
                                # Load specific trajectory based on marker ID
                                self.current_csv_file = os.path.join(self.working_office, f'trajectory_{self.marker_ID}.csv')
                                self.substate = 2
                            else:
                                # Return to state 1
                                self.state = 1
                                self.substate = 0
                        else:
                            # Marker outdated, abort
                            self.state = 1
                            self.substate = 0
        
                    elif self.response.changed == True and self.canceled == True:
                        # If canceled by user (button E), behave similarly, but retain cancellation flag
                        self.response = None
                        if self.get_clock().now().to_msg().sec - self.marker_timestamp < 3:
                            self.canceled = False
                            if self.marker_ID == 0 and self.backwards == False:
                                self.substate = 1
                            elif self.marker_ID != 0 and self.backwards == True:
                                self.current_csv_file = os.path.join(self.working_office, f'trajectory_{self.marker_ID}.csv')
                                self.substate = 2
                            else:
                                self.state = 1
                                self.substate = 0
                        else:
                            # Marker outdated, but start repeat because robot is in middle of the path
                            self.substate = 2
        
            elif self.substate == 1:
                # Substate 1: Let the user choose a trajectory manually
                files = os.listdir(self.working_office)
                csv_files = list()
                for file in files:
                    if "trajectory" in file:
                        csv_files.append(file)
        
                if len(csv_files):
                    self.get_logger().info(f'Waehle eine Trajectory aus zwischen 1 und {len(csv_files)}!')
                    # Select based on button pressed
                    if self.button_left == 1 and len(csv_files) >= 1:
                        self.current_traj_num = 1
                        self.current_csv_file = os.path.join(self.working_office, f'trajectory_1.csv')
                        self.substate = 2
                    elif self.button_up == 1 and len(csv_files) >= 2:
                        self.current_traj_num = 2
                        self.current_csv_file = os.path.join(self.working_office, f'trajectory_2.csv')
                        self.substate = 2
                    elif self.button_right == 1 and len(csv_files) >= 3:
                        self.current_traj_num = 3
                        self.current_csv_file = os.path.join(self.working_office, f'trajectory_3.csv')
                        self.substate = 2
                else:
                    # No trajectory files found, go back to state 1
                    self.substate = 0
                    self.state = 1
        
            elif self.substate == 2:
                # Substate 2: Start following the selected trajectory
                self.request_FollowPath.traj_file = self.current_csv_file
                self.request_FollowPath.backwards = self.backwards
        
                if self.service_called == False:
                    self.call_service(self.client_FollowPath, self.request_FollowPath)
                    self.service_called = True
        
                if self.response != None:
                    self.service_called = False
                    if self.response.started == True:
                        self.response = None
                        self.substate = 3
                    elif self.response.started == False:
                        self.response = None
                        self.state = 1
                        self.substate = 0
        
            elif self.substate == 3:
                # Substate 3: Wait for trajectory to finish or get canceled
                if self.button_e == 1:
                    # Cancel requested by user
                    self.canceled = True
                    Flags_msg = Flags()
                    Flags_msg.cancel = True
                    self.Flags_pub.publish(Flags_msg)
                    self.substate = 0
                    self.state = 1
        
                elif self.homed == True:
                    # Robot reached end position
                    new_trajs = list(self.start_end_nodes.keys())
                    Flags_msg = Flags()
                    Flags_msg.homed = False
                    self.Flags_pub.publish(Flags_msg)
        
                    if self.current_traj_num in new_trajs and self.canceled == False:
                        self.substate = 4
                    else:
                        self.canceled = False
                        self.substate = 0
                        self.state = 1
        
            elif self.substate == 4:
                # Substate 4: Save new map after successful repeat
                if self.service_called == False:
                    file_list = os.listdir(self.working_office)
                    max_map_num = 0
                    for file in file_list:
                        if 'map' in file:
                            map_num = int(file.split('_')[1].split('.')[0])
                            max_map_num = max(map_num, max_map_num)
        
                    self.current_pbstream_file = os.path.join(self.working_office, f'map_{max_map_num+1}.pbstream')
                    self.request_WriteState.filename = self.current_pbstream_file
                    self.call_service(self.client_WriteState, self.request_WriteState)
                    self.service_called = True
        
                if self.response != None:
                    self.service_called = False
                    if self.response.status.code == 0:
                        self.get_logger().info(f'service called successfully! {self.response.status.message}')
                        self.substate = 5
                        self.response = None
                    else:
                        self.get_logger().error('service call failed!')
        
            elif self.substate == 5:
                # Substate 5: Rewrite trajectory nodes to match updated map
                self.request_RewriteTraj.pbstream_path = self.current_pbstream_file
                self.request_RewriteTraj.traj_num = self.current_traj_num
                self.request_RewriteTraj.start_node = self.start_end_nodes[self.current_traj_num][0]
                self.request_RewriteTraj.end_node = self.start_end_nodes[self.current_traj_num][1]
        
                if self.service_called == False:
                    self.call_service(self.client_RewriteTraj, self.request_RewriteTraj)
                    self.service_called = True
        
                if self.response != None:
                    self.service_called = False
                    if self.response.csv_path:
                        # Use the updated trajectory file path
                        self.current_csv_file = self.response.csv_path
                        self.substate = 0
                        self.state = 1
                        self.response = None

    def call_service(self, client, request):
        # call service
        self.future = client.call_async(request)
               
        self.future.add_done_callback(self.service_callback)
    
    def service_callback(self, future):
        # check response
        if future.result():
            self.get_logger().info('service successful')
            self.response = future.result()   
        else:
            self.get_logger().error('service call failed!')
            self.response = None
        

    def listener_callback_flags(self, msg):
        # check flags of repeat 
        self.turned_around = msg.turned_around
        self.arrived = msg.arrived
        self.homed = msg.homed

    def listener_callback_landmark(self, msg):
        # get general informations about marker
        self.marker_ID = msg.id
        self.marker_timestamp = msg.header.stamp.sec
    
    def listener_callback_panel(self, msg):
        # read button input
        self.button_up = 0
        self.button_right = 0
        self.button_down = 0
        self.button_left = 0
        self.button_e = 0
        self.button_f = 0

        current_button_up = msg.buttons[0]
        current_button_right = msg.buttons[1]
        current_button_down = msg.buttons[2]
        current_button_left = msg.buttons[3]
        current_button_f = msg.buttons[4]
        current_button_e = msg.buttons[5]
        
        if current_button_up and self.counter_button_up < 3:
            self.counter_button_up += 1
        if current_button_right and self.counter_button_right < 3:
            self.counter_button_right += 1    
        if current_button_down and self.counter_button_down < 3:
            self.counter_button_down += 1
        if current_button_left and self.counter_button_left < 3:
            self.counter_button_left += 1
        if current_button_e and self.counter_button_e < 3:
            self.counter_button_e += 1
        if current_button_f and self.counter_button_f < 3:
            self.counter_button_f += 1
        
        if self.counter_button_up >= 3 and current_button_up == 0:
            self.counter_button_up = 0
            self.button_up = 1
        if self.counter_button_right >= 3 and current_button_right == 0:
            self.counter_button_right = 0
            self.button_right = 1
        if self.counter_button_down >= 3 and current_button_down == 0:
            self.counter_button_down = 0
            self.button_down = 1
        if self.counter_button_left >= 3 and current_button_left == 0:
            self.counter_button_left = 0
            self.button_left = 1
        if self.counter_button_e >= 3 and current_button_e == 0:
            self.counter_button_e = 0
            self.button_e = 1
        if self.counter_button_f >= 3 and current_button_f == 0:
            self.counter_button_f = 0
            self.button_f = 1




def main(args=None):
    rclpy.init(args=args)
    node = StateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()