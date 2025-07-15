import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from motion_msgs.msg import MotionCtrl

from pid_controller import PIDController

# Constants (adjust as needed)
P_CONTROLLER_DIST = 1.2
ITS_CONTROLLER_DIST = 0.01
MAX_FOLLOW_VEL = 0.6
AW_CONTROLLER_DIST = -0.1

P_CONTROLLER_ANGLE = 2.5
ITS_CONTROLLER_ANGLE = 0.01
MAX_FOLLOW_ROTVEL = 0.3

class TrackerControllerNode(Node):
    def __init__(self):
        super().__init__('follow_controller_node')

        self.trans_controller = PIDController(
            P_CONTROLLER_DIST, ITS_CONTROLLER_DIST, 0.0, MAX_FOLLOW_VEL, AW_CONTROLLER_DIST
        )
        self.rot_controller = PIDController(
            P_CONTROLLER_ANGLE, ITS_CONTROLLER_ANGLE, -MAX_FOLLOW_ROTVEL, MAX_FOLLOW_ROTVEL
        )

        self.subscription = self.create_subscription(
            Point,
            '/human/positionUWB',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(MotionCtrl, '/diablo/MotionCmd', 10)

        self.get_logger().info("TrackerControllerNode started.")

    def listener_callback(self, msg: Point):
        x = msg.x
        y = msg.y

        self.trans_controller.put(x)
        self.trans_controller.run()

        self.rot_controller.put(y)
        self.rot_controller.run()

        cmd = MotionCtrl()
        cmd.value.forward = self.trans_controller.get()
        cmd.value.left = self.rot_controller.get()

        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TrackerControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()