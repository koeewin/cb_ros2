import rclpy
from rclpy.node import Node
from cb_interfaces.srv import ShowLed
from LEDControl import LEDControl  # Make sure this points to your LEDControl class

class LEDServiceNode(Node):
    def __init__(self):
        super().__init__('led_service_node')
        self.srv = self.create_service(ShowLed, 'show_led', self.handle_led_request)
        self.led = LEDControl('/dev/spidev0.0', 8)
        self.get_logger().info("LED Service Node ready. Waiting for requests...")

    def handle_led_request(self, request, response):
        mode = request.mode
        self.get_logger().info(f"Received LED mode request: {mode}")
        # Mode 0: Manual    Green   ID:0,1
        # Mode 1: Follow    Blue    ID:0,1
        # Mode 2: TeachPath [100, 100, 0]   ID:3,4,5,6  2Hz
        # Mode 9: Home/Auto [120 80 20]     Blue    ID:0,1  2Hz
        # Mode 10: Repeat/Auto [0,120,120]    ID:0,1 2Hz
        
        # Mode 3: Home      [0,100,0]    ID:3,4,5,6  2Hz   
        # Mode 4: PathSaved Green   ID:3,4,5,6  1Hz
        # Mode 5: SelectPath Blue   ID:2,3,4,5,6,7 1Hz
        # Mode 6: #1Selected Blue   ID:2,3 2Hz
        # Mode 7: #2Selected Blue   ID:4,5 2Hz
        # Mode 8: #3Selected Blue   ID:6,7 2Hz
        


        try:
            if mode == 0:
                # Mode 0: Upper 2 Green   ID:0,1
                self.led.stop_blink()  # Stop any ongoing blinking
                self.led.fill(0, 0, 0)
                self.get_logger().info(f"Previous Mode tured off and starting Manual: {mode}")
                self.led.fill_selected( [0,1], 0, 100, 0)
                response.result = 0
                response.success = True
            elif mode == 1:
                # Mode 1: Follow    Upper 2 Blue   ID:0,1
                # Mode 0: Set all LEDs to blue
                self.led.stop_blink()  # Stop any ongoing blinking
                self.led.fill(0, 0, 0)
                self.get_logger().info(f"Previous Mode tured off and starting Follow: {mode}")
                self.led.fill_selected( [0,1], 0, 0, 125)
                response.result = 0
                response.success = True

            elif mode == 2:
                #Mode 2: TeachPath Yellow blink 2Hz
                #self.led.fill(0, 0, 0)
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([0,1], frequency=2, color=[100, 100, 0], duration=None)
                response.result = 1
                response.success = True

            elif mode == 3: # Mode 3: Home Path     Blue    ID:3,4,5,6  2Hz
                #self.led.fill(0, 0, 0)
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.fill_selected( [0,1],100, 100, 0)
                self.led.blink([3,4,5,6], frequency=1, color=[0, 0, 125], duration=None)
                response.result = 1
                response.success = True
            elif mode == 4: # Mode 4: PathSaved Green   ID:3,4,5,6  1Hz
                #self.led.fill(0, 0, 0)
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([3,4,5,6], frequency=1, color=[0, 100, 0], duration=None)
                response.result = 1
                response.success = True

            elif mode == 5: #SelectPath Blue   ID:2,3,4,5,6,7 1Hz 
                #self.led.fill(0, 0, 0)
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([2,3,4,5,6,7], frequency=1, color=[0, 0, 125], duration=None)
                response.result = 1
                response.success = True

            elif mode == 6:  # Mode 6: #1Selected Blue   ID:2,3 2Hz
                
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([2,3], frequency=2, color=[0, 0, 150], duration=None)
                response.result = 1
                response.success = True
            elif mode == 7:  # Mode 7: #2Selected Blue   ID:4,5 2Hz
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([4,5], frequency=2, color=[0, 0, 150], duration=None)
                response.result = 1
                response.success = True
            elif mode == 8:  # Mode 8: #3Selected Blue   ID:6,7 2Hz
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([6,7], frequency=2, color=[0, 0, 150], duration=None)
                response.result = 1
                response.success = True
            elif mode == 9:  # Mode 9: Home/Auto [120, 80, 20] Blue ID:3,4,5,6 2Hz
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([0,1], frequency=2, color=[120, 80, 20], duration=None)
                response.result = 1
                response.success = True
            elif mode == 10:  # Mode 10: Repeat/Auto [0,120,120]    ID:0,1 2Hz
                self.led.stop_blink()
                self.get_logger().info(f"Previous Mode tured off and starting mode: {mode}")
                self.led.blink([0,1], frequency=2, color=[0, 120, 120], duration=None)
                response.result = 1
                response.success = True
            else:
                self.get_logger().warn(f"Unsupported mode: {mode}")
                self.led.fill(0, 0, 0)
                response.success = False
                response.result = -1

        except Exception as e:
            self.get_logger().error(f"Error handling LED request: {e}")
            response.success = False
            response.result = -999

        return response


def main(args=None):
    rclpy.init(args=args)
    node = LEDServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
