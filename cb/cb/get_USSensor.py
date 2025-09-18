import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial
import threading


class UltrasonicPublisher(Node):
    def __init__(self):
        super().__init__('us_sensor_publisher')

        # === Sensor-Konfiguration ===
        self.min_range_m = 0.02 #Meter
        self.max_range_m = 2.50 #Meter
        self.fov_rad = 0.2618  # 15° in rad

        # === ROS2 Publisher ===
        self.pub_left = self.create_publisher(Range, '/sensor_us/left', 10)
        self.pub_front = self.create_publisher(Range, '/sensor_us/front', 10)
        self.pub_right = self.create_publisher(Range, '/sensor_us/right', 10)

        # === Serielle Verbindung ===
        try:
            self.ser = serial.Serial('/dev/ttyAMA10', 9600, timeout=1) # /dev/ttyAMA10
            self.get_logger().info('Serielle Verbindung hergestellt.')
        except serial.SerialException as e:
            self.get_logger().error(f'Serielle Verbindung fehlgeschlagen: {e}')
            raise e

        # === Separater Thread liest kontinuierlich vom Arduino ===
        self.serial_thread = threading.Thread(target=self.read_serial_data_loop, daemon=True)
        self.serial_thread.start()

    def read_serial_data_loop(self):
        while rclpy.ok():
            
            try:
                #line = self.ser.readline().decode('utf-8').strip()
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                
                if not line:
                    continue

                values = [int(v) for v in line.strip().split('\t')]
                if len(values) != 3:
                    
                    continue

                

                self.publish_range(self.pub_left, values[2], 'ultrasonic_left')
                self.publish_range(self.pub_front, values[1], 'ultrasonic_front')
                self.publish_range(self.pub_right, values[0], 'ultrasonic_right')

            except Exception as e:
                self.get_logger().error(f'Serienfehler: {e}')

    def publish_range(self, publisher, cm_value, frame_id):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.fov_rad
        msg.min_range = self.min_range_m
        msg.max_range = self.max_range_m

        range_m = cm_value / 100.0
        # If the measured distance is less than 3 cm, set it to max_range
        if cm_value < 3:
            range_m = self.max_range_m
        msg.range = min(max(range_m, self.min_range_m) ,self.max_range_m)

        publisher.publish(msg)
        #self.get_logger().info(f"Published {frame_id}: {range_m:.2f} m")
        
def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
