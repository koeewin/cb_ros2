import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import tkinter as tk

AXIS_VALUE = 1
AXIS_VALUER = 1.2

class JoyGUINode(Node):
    def __init__(self):
        super().__init__('joy_gui_node')

        # Publisher
        self.panel_pub = self.create_publisher(Joy, '/cb/Panel', 10)

        # Initialize Joy message
        self.latest_panel = Joy()
        self.latest_panel.axes = [0.0, 0.0]   # [forward/backward, left/right]
        self.latest_panel.buttons = [0] * 6

        # Setup GUI
        self.root = tk.Tk()
        self.root.title("cb Joy Panel")

        # --- Directional controls ---
        direction_frame = tk.Frame(self.root)
        direction_frame.grid(row=0, column=0, columnspan=6, pady=10)

        tk.Button(direction_frame, text="Forward", width=10, command=self.move_forward).grid(row=0, column=1)
        tk.Button(direction_frame, text="Left", width=10, command=self.move_left).grid(row=1, column=0)
        tk.Button(direction_frame, text="Stop", width=10, command=self.stop_motion).grid(row=1, column=1)
        tk.Button(direction_frame, text="Right", width=10, command=self.move_right).grid(row=1, column=2)
        tk.Button(direction_frame, text="Backward", width=10, command=self.move_backward).grid(row=2, column=1)

        # --- Button panel (6 buttons) ---
        button_names = ['TOP', 'TOP_R', 'TOP_BOT', 'TOP_L', 'EMPTY', 'HOME']
        for i, name in enumerate(button_names):
            b = tk.Button(self.root, text=name, width=10,
                          command=lambda idx=i: self.set_button(idx))
            b.grid(row=2, column=i, padx=5, pady=10)
        self.latest_panel.buttons = [0] * 6
        self.button_press_counters = [0] * 6  # <-- new: counts remaining publish cycles

        # Start GUI-based publish loop (replaces ROS timer)
        self.root.after(100, self.publish_loop)

    # --- Movement control methods ---
    def move_forward(self):
        self.latest_panel.axes[0] = AXIS_VALUE
        self.latest_panel.axes[1] = 0.0

    def move_backward(self):
        self.latest_panel.axes[0] = -AXIS_VALUE
        self.latest_panel.axes[1] = 0.0

    def move_left(self):
        self.latest_panel.axes[0] = 0.0
        self.latest_panel.axes[1] = AXIS_VALUER

    def move_right(self):
        self.latest_panel.axes[0] = 0.0
        self.latest_panel.axes[1] = -AXIS_VALUER

    def stop_motion(self):
        self.latest_panel.axes = [0.0, 0.0]

    # --- Button logic ---
    def set_button(self, index):
        self.button_press_counters[index] = 3  # publish 3 times
        self.latest_panel.buttons[index] = 1
        self.get_logger().info(f"Button {index} pressed")
        self.root.after(100, lambda: self.reset_button(index))  # Auto-reset

    def reset_button(self, index):
        self.latest_panel.buttons[index] = 0

    # --- Publishing logic ---
    def publish_loop(self):
            # Update buttons based on counters
            for i in range(len(self.latest_panel.buttons)):
                if self.button_press_counters[i] > 0:
                    self.latest_panel.buttons[i] = 1
                    self.button_press_counters[i] -= 1
                else:
                    self.latest_panel.buttons[i] = 0

            # Publish the message
            self.panel_pub.publish(self.latest_panel)
            self.get_logger().info(f"Published Joy: {self.latest_panel}")

            self.root.after(100, self.publish_loop)  # Repeat every 100 ms

    # --- GUI main loop ---
    def run(self):
        self.root.mainloop()

# --- Main ---
def main(args=None):
    rclpy.init(args=args)
    node = JoyGUINode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
