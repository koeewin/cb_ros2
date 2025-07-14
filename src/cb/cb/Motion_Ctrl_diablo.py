import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from motion_msgs.msg import MotionCtrl
from sensor_msgs.msg import Joy
from cb_interfaces.srv import ChangeCtrlmode

# ROS2 node to manage control modes and motion commands for the DIABLO robot
class Motionctrl_diablo(Node):
    def __init__(self):
        super().__init__('motionctrl_diablo')

        # Service to change the control mode (manuel, follow, repeat)
        self.service = self.create_service(ChangeCtrlmode, '/change_ctrlmode', self.change_ctrlmode_callback)

        # Subscriber to velocity commands from the pp-controller
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback_ppcontrl, 10)
        
        # Subscriber to manual input from the joystick
        self.panel_sub = self.create_subscription(Joy, '/cb/Panel', self.listener_callback_panel, 10)

        # Publisher for motion control commands to DIABLO robot
        self.mctrl_pub = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)

        # Timer callback to periodically send motion commands
        self.timer = self.create_timer(0.05, self.timer_motionctrl)

        # Initialize velocity values for different modes
        self.cur_forw_vel_ppcontrl = None
        self.cur_angl_vel_ppcontrl = None
        self.cur_forw_vel_panel = 0
        self.cur_angl_vel_panel = 0
        self.cur_forw_vel_follow = None
        self.cur_angl_vel_follow = None

        # Flag to send a special mode_mark on first command (lets robots stand up)
        self.first_time = True

        # Control mode flags (only one can be active)
        self.manuel = False
        self.follow = False
        self.repeat = False

    # Callback for service to change control mode
    def change_ctrlmode_callback(self, request, response):
        # Update mode flags
        self.manuel = request.manuel
        self.follow = request.follow
        self.repeat = request.repeat

        # Only one mode must be active
        if self.manuel + self.follow + self.repeat != 1:
            response.changed = False
            self.manuel = False
            self.follow = False
            self.repeat = False
        else:
            response.changed = True

        return response

    # Callback for velocity commands from pp-controller
    def listener_callback_ppcontrl(self, msg):
        self.cur_forw_vel_ppcontrl = msg.linear.x        
        self.cur_angl_vel_ppcontrl = msg.angular.z

    # Callback for joystick input
    def listener_callback_panel(self, msg):
        self.cur_forw_vel_panel = msg.axes[0] #  (msg.joystick.x - 128) / 230       
        self.cur_angl_vel_panel = msg.axes[1] # (msg.joystick.y - 128) / 230

    # Timer callback to build and publish motion command to DIABLO
    def timer_motionctrl(self):
        # print(f'repeat: {self.repeat}')
        # print(f'manuel: {self.manuel}')
        # print(f'follow: {self.follow}')
        
        # write msg for diablo motion ctrl
        mctrl_msg = MotionCtrl()
        mctrl_msg.mode_mark = False

        # only first message should have mode_mark=True (robot will stand up)
        if self.first_time == True:
            mctrl_msg.mode_mark = True
            self.first_time = False

        # set control modes
        mctrl_msg.mode.stand_mode = True
        mctrl_msg.mode.height_ctrl_mode = True
        mctrl_msg.mode.pitch_ctrl_mode = True

        # set fixed up/pitch values
        mctrl_msg.value.up = 1.0
        mctrl_msg.value.pitch = 0.0

        # default: no motion
        mctrl_msg.value.forward = 0.0
        mctrl_msg.value.left = 0.0

        # handle motion logic depending on active mode
        if self.repeat == True:
            if self.cur_forw_vel_ppcontrl is not None:
                mctrl_msg.value.forward = self.cur_forw_vel_ppcontrl
            if self.cur_angl_vel_ppcontrl is not None:
                mctrl_msg.value.left = self.cur_angl_vel_ppcontrl

        elif self.manuel == True:
            if self.cur_forw_vel_panel != 0 or self.cur_angl_vel_panel != 0:
                if abs(self.cur_forw_vel_panel) > 0.1:
                    mctrl_msg.value.forward = self.cur_forw_vel_panel
                if abs(self.cur_angl_vel_panel) > 0.1:
                    mctrl_msg.value.left = self.cur_angl_vel_panel

        elif self.follow == True:
            if self.cur_forw_vel_follow is not None:
                mctrl_msg.value.forward = self.cur_forw_vel_follow
            if self.cur_angl_vel_follow is not None:
                mctrl_msg.value.left = self.cur_angl_vel_follow

        # publish motion command
        self.mctrl_pub.publish(mctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Motionctrl_diablo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
