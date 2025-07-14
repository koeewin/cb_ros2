import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from cb_interfaces.srv import ChangeCtrlmode

class Motionctrl_sim(Node):
    def __init__(self):
        super().__init__('motionctrl_sim')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.panel_sub = self.create_subscription(Joy, '/cb/Panel', self.listener_callback_panel, 10)
        self.service = self.create_service(ChangeCtrlmode, '/change_ctrlmode', self.change_ctrlmode_callback)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.cur_forw_vel_panel = 0
        self.cur_angl_vel_panel = 0
        self.sema = True

        self.manuel = False
        self.follow = False
        self.repeat = False
        
    def change_ctrlmode_callback(self, request, response):
        print("Hey I am called!!!")
        
        self.manuel = request.manuel
        self.follow = request.follow
        self.repeat = request.repeat

        print(self.manuel + self.follow + self.repeat)



        if self.manuel + self.follow + self.repeat != 1:
            response.changed = False
            self.manuel = False
            self.follow = False
            self.repeat = False
            print("No Valid Input")
        else:
            response.changed = True
   
        return response

    def listener_callback_panel(self, msg):
        # get current forward vel
        self.cur_forw_vel_panel = msg.axes[0]*0.3
        # get current angular vel
        self.cur_angl_vel_panel = msg.axes[1]*0.3
        #print(f'Forward Velocity: {self.cur_forw_vel_panel}, Angular Velocity: {self.cur_angl_vel_panel}')
    def on_timer(self):
        #write msg for diablo motion ctrl
        cmd_msg = Twist()
        # overwrite controlls with input from panel
        if abs(self.cur_forw_vel_panel) > 0.1 or abs(self.cur_angl_vel_panel) > 0.1:
            cmd_msg.linear.x = self.cur_forw_vel_panel
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= self.cur_angl_vel_panel
            
            self.sema = True
            
            self.cmd_vel_pub.publish(cmd_msg)

        elif abs(self.cur_forw_vel_panel) <= 0.1 and abs(self.cur_angl_vel_panel) <= 0.1 and self.sema == True:
            cmd_msg.linear.x = 0.0
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0

            cmd_msg.angular.x= 0.0
            cmd_msg.angular.y= 0.0
            cmd_msg.angular.z= 0.0

            self.sema = False
            
            self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Motionctrl_sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()