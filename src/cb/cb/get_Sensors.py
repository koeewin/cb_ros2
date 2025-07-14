import rclpy
from rclpy.node import Node
import serial
import time
import threading

from cb_interfaces.msg import Positioning
#from cb_interfaces.msg import Panel
from sensor_msgs.msg import Joy
# Message constants
MSG_CMD_SEND_HUMAN_POSITION = 0xAA
MSG_LEN_PiComm = 21
MSG_END = 0xFF

# Byte positions
SP_DIST_HIGH_BYTE = 2
SP_DIST_LOW_BYTE = 3
SP_ANGLE_SIGN = 4
SP_ANGLE_HIGH_BYTE = 5
SP_ANGLE_LOW_BYTE = 6
SP_Y_HIGH_BYTE = 7
SP_Y_LOW_BYTE = 8
SP_X_SIGN = 9
SP_X_HIGH_BYTE = 10
SP_X_LOW_BYTE = 11
SP_FORWARD = 12
SP_LEFT = 13
SP_HOME = 14
SP_EMPTY = 15
SP_TOP = 16
SP_TOP_R = 17
SP_TOP_L = 18
SP_TOP_BOT = 19

class UWBSerialReader(Node):
    def __init__(self):
        super().__init__('uwb_serial_reader')
        self.declare_parameter('serial_port', '/dev/ttyAMA10') # Raspberry Pi5 Uart-Connector mini-jst /dev/ttyAMA10
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = 250000

        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=0.005)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            raise e

        # Publishers
        self.position_pub = self.create_publisher(Positioning, '/human/positionUWB', 10)
        self.panel_pub = self.create_publisher(Joy, '/cb/Panel', 10)

        # Store latest values
        self.latest_positioning = Positioning()
        self.latest_panel = Joy()

        # Timers
        #self.position_timer = self.create_timer(0.02, self.publish_position)  # 50 Hz
        self.panel_timer = self.create_timer(0.05, self.publish_panel)       # 20 Hz

        # Internal buffer
        self._serial_buffer = bytearray()
        
        # Start serial reader thread
        self._serial_thread = threading.Thread(target=self.serial_read_thread, daemon=True)
        self._serial_thread.start()
        self.log_counter = 0
        self._human_position_counter = 0

    def serial_read_thread(self):
        """Top-level periodic callback that reads and parses serial data."""
        while rclpy.ok():
            msg = self.receive_message()
            if msg:
                self.parse_message(msg)
            time.sleep(0.012)
        

    def receive_message(self):
        """Low-level serial byte reader with end marker + length check."""
        try:
            # Read a fixed chunk
            byte_to_read = min(self.serial_connection.in_waiting or 0, 32)
            if byte_to_read:
                self._serial_buffer.extend(self.serial_connection.read(self.serial_connection.in_waiting))

                # Trim if buffer is too large
                if len(self._serial_buffer) > 2 * MSG_LEN_PiComm:
                    self._serial_buffer = self._serial_buffer[-MSG_LEN_PiComm:]

                # Search for valid message
                if len(self._serial_buffer) >= MSG_LEN_PiComm:
                    # Look for correct structure
                    end_index = self._serial_buffer.find(MSG_END.to_bytes(1, 'big'), MSG_LEN_PiComm - 1)
                    if end_index >= MSG_LEN_PiComm - 1:
                        msg_start = end_index - (MSG_LEN_PiComm - 1)
                        msg = self._serial_buffer[msg_start:end_index + 1]

                        # Remove consumed bytes
                        self._serial_buffer = self._serial_buffer[end_index + 1:]
                        return list(msg)
                    else:
                        # if no valid end found, shift buffer
                        self._serial_buffer.pop(0)
        except Exception as e:
            self.get_logger().error(f"Serial receive error: {e}")
        return None

    def parse_message(self, message):
        try:
            # Parse float values
            distance = ((message[SP_DIST_HIGH_BYTE] << 8) | message[SP_DIST_LOW_BYTE]) / 1000.0
            
            angle = ((message[SP_ANGLE_HIGH_BYTE] << 8) | message[SP_ANGLE_LOW_BYTE]) / 1000.0
            if message[SP_ANGLE_SIGN] == 0x01:
                angle = -angle

            x = ((message[SP_X_HIGH_BYTE] << 8) | message[SP_X_LOW_BYTE]) / 1000.0
            if message[SP_X_SIGN] == 0x01:
                x = -x

            y = ((message[SP_Y_HIGH_BYTE] << 8) | message[SP_Y_LOW_BYTE]) / 1000.0

            # Update latest_positioning
            self.latest_positioning = Positioning(x=x, y=y, angle=angle, distance=distance)

            # Update latest_panel
            self.latest_panel = Joy()
            self.latest_panel.axes = [
                float((message[SP_FORWARD]/ 128.0 - 1.0)),
                float((message[SP_LEFT]/ 128.0 - 1.0)),
            ]
            self.latest_panel.buttons = [
                message[SP_TOP],
                message[SP_TOP_R],
                message[SP_TOP_BOT],
                message[SP_TOP_L],
                message[SP_EMPTY],
                message[SP_HOME]  # This is the home button
            ]

            self.position_pub.publish(self.latest_positioning)
            self._human_position_counter += 1
            if self._human_position_counter >= 10:
                self.get_logger().info(f"Published Human Position:\n{self.latest_positioning}")
                self._human_position_counter = 0
        except Exception as e:
            self.get_logger().error(f"Failed to parse message: {e}")

    def publish_panel(self):
        """Publishes the panel message at 20 Hz"""
        self.panel_pub.publish(self.latest_panel)
        self.log_counter += 1
        if self.log_counter >=10:
            #self.get_logger().info(f"Published Panel:\n{self.latest_panel}")
            self.log_counter = 0



def main(args=None):
    rclpy.init(args=args)
    node = UWBSerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        if node.serial_connection.is_open:
            node.serial_connection.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
