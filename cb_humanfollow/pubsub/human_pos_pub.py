#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point

from Paths.path import gen_path, arclength


class HumanPositionPublisher(Node):
    def __init__(self):
        super().__init__('human_position_publisher')
        self.publisher_ = self.create_publisher(Point, 'human_position', 10)

        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Load route
        shape = "8"  # Options: Sinus, Sample, Straight, 8, rec
        route = gen_path(shape)
        if not shape == "rec":
            L = arclength(route[:, 0], route[:, 1], 'spline')[0]  # Calculate length of the path
            scale = 0.05 / (L / route.shape[0])  # Scale the path for a walking speed of 1 m/s
            self.waypoints = route * scale
        else:
            self.waypoints = route

        self.step = 1

    def timer_callback(self):

        msg = Point()
        msg.x = float(self.waypoints[self.step - 1, 0])
        msg.y =float(self.waypoints[self.step - 1, 1])

        if self.step < self.waypoints.shape[0]:            
            self.step += 1

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: x:"%f", y:"%f"' % (msg.x, msg.y))

    
def main(args=None):
    rclpy.init(args=args)

    human_position_publisher = HumanPositionPublisher()
    rclpy.spin(human_position_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    human_position_publisher.destroy_node()
    rclpy.shutdown()        
        

if __name__ == '__main__':
    main()
