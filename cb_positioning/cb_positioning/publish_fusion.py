#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from cb_interfaces.msg import PositioningData
from rclpy.executors import MultiThreadedExecutor
import math
import time
import threading

try:
    # Relative import within the package
    from .positioning_utils.positioning import *
    from .positioning_utils.positioning_fusion import *
except ImportError:
    # Normal import when running directly
    from positioning_utils.positioning import *
    from positioning_utils.positioning_fusion import *



class Fusion(Node):
    def __init__(self):
        super().__init__('position_fusion')
        self.uwb_sub = self.create_subscription(PositioningData, '/human/positionUWB', self.uwb_callback, 10)
        self.vision_sub = self.create_subscription(PositioningData, 'human/positionVision', self.vision_callback, 10)
        self.fusion_publisher = self.create_publisher(Point, '/human/positionFuse', 10)
        self.timer_callback = self.create_timer(0.05, self.fuse)
        self.uwb_pos = PositioningData()
        self.vision_pos = PositioningData()
        self.positioningFusion = PositioningFusion()
        self.lock = threading.Lock()

    def uwb_callback(self, msg):
        with self.lock:
            self.uwb_pos = msg

    def vision_callback(self, msg):
        with self.lock:
            self.vision_pos = msg

    def fuse(self):
        with self.lock:
            # Considering offsets for UWB and Vision data
            OFFSET_X_UWB = -0.10
            OFFSET_X_VISION = 0.15

            uwb_pos = self.uwb_pos
            vision_pos = self.vision_pos
            
            uwb_x = uwb_pos.x + OFFSET_X_UWB
            uwb_y = uwb_pos.y
            vision_x = vision_pos.x + OFFSET_X_VISION
            vision_y = vision_pos.y

            uwb_distance = (uwb_x**2 + uwb_y**2)**0.5
            uwb_angle = math.atan2(uwb_y, uwb_x)

            vision_distance = (vision_x**2 + vision_y**2)**0.5
            vision_angle = math.atan2(vision_y, vision_x)

        # self.positioningFusion.put_measurements(uwb_pos.angle, uwb_pos.distance, 
        #                                         vision_pos.angle, vision_pos.distance) 
        self.positioningFusion.put_measurements(uwb_angle, uwb_distance, 
                                                vision_angle,  vision_distance)   
        self.positioningFusion.run() 

        msg = Point()
        msg.x = self.positioningFusion.get_x()
        msg.y = self.positioningFusion.get_y()
        self.fusion_publisher.publish(msg)
        self.get_logger().info('Publishing: x:"%f", y:"%f"' % (msg.x, msg.y))

      
def main(args=None):
    rclpy.init(args=args)
    node = Fusion()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()       

if __name__ == '__main__':
    main()
        
