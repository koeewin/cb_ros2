#!/usr/bin/env python3
import math
import numpy as np
import rclpy
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Change the import below to match your package/message name if needed.
# The user wrote "PositioningData.msg". If your message is "PositioningData.msg", adjust accordingly.
from cb_interfaces.msg import PositioningData   # noqa: F401
from geometry_msgs.msg import Point

class MultiTopicRecorder(Node):
    #python3 thisfile.py --x-nr 2 --y-nr 5
    def __init__(self, outfile: str = "", x_nr: int | None = None, y_nr: int | None = None):
        super().__init__('positiong_measurements')

        # ---- Parameters for output naming ----
        # keep simple attributes instead of ROS params
        self.outfile = outfile
        self.x_nr = x_nr
        self.y_nr = y_nr

        # QoS (20 Hz publishers -> small depth is fine)
        qos = QoSProfile(depth=10)

        # ---- Subscriptions ----
        self.sub_uwb = self.create_subscription(
            PositioningData, '/human/positionUWB', self.cb_uwb, qos)

        self.sub_vision = self.create_subscription(
            PositioningData, '/human/positionVision', self.cb_vision, qos)

        self.sub_fuse = self.create_subscription(
            Point, '/human/positionFuse', self.cb_fuse, qos)

        # ---- Latest-sample caches ----
        # UWB and Vision messages contain: x, y, angle, distance
        self.latest_u = {'x': None, 'y': None, 'phi': None, 'd': None}
        self.latest_v = {'x': None, 'y': None, 'phi': None, 'd': None}
        # Fuse message is geometry_msgs/Point: x, y (z ignored)
        self.latest_f = {'x': None, 'y': None}

        # ---- Recording buffer (600 x 12) ----
        self.max_rows = 300
        self.buffer = np.zeros((self.max_rows, 12), dtype=float)
        self.row_idx = 0
        self.recording = False

        # ---- 5s countdown before recording ----
        self.countdown_secs = 5
        self._count = self.countdown_secs
        self.get_logger().info(f"Starting {self.countdown_secs}s countdown before recording...")
        self.countdown_timer = self.create_timer(1.0, self._countdown_tick)

    # ---------- Subscriptions ----------
    def cb_uwb(self, msg: 'PositioningData'):
        self.latest_u['x'] = float(msg.x)
        self.latest_u['y'] = float(msg.y)
        self.latest_u['phi'] = float(msg.angle)
        self.latest_u['d'] = float(msg.distance)

    def cb_vision(self, msg: 'PositioningData'):
        self.latest_v['x'] = float(msg.x)
        self.latest_v['y'] = float(msg.y)
        self.latest_v['phi'] = float(msg.angle)
        self.latest_v['d'] = float(msg.distance)

    def cb_fuse(self, msg: Point):
        # Always keep the newest fused point
        self.latest_f['x'] = float(msg.x)
        self.latest_f['y'] = float(msg.y)

        # Only store rows when recording is active
        if not self.recording:
            return

        # Require we have at least one sample from each source
        if not self._have_all_sources():
            return

        if self.row_idx >= self.max_rows:
            return  # already full; saving handled after countdown

        # Build the 12-element row (tab-separated when saved):
        # [phi_u, d_u, x_u, y_u,  phi_v, d_v, x_v, y_v,  phi_f, d_f, x_f, y_f]
        x_u, y_u, phi_u, d_u = self.latest_u['x'], self.latest_u['y'], self.latest_u['phi'], self.latest_u['d']
        x_v, y_v, phi_v, d_v = self.latest_v['x'], self.latest_v['y'], self.latest_v['phi'], self.latest_v['d']
        x_f, y_f = self.latest_f['x'], self.latest_f['y']

        # Compute phi_f and d_f from fused (Point) values
        phi_f = math.atan2(y_f, x_f)
        d_f = float(np.linalg.norm([x_f, y_f]))

        row = np.array([
            phi_u, d_u, x_u, y_u,
            phi_v, d_v, x_v, y_v,
            phi_f, d_f, x_f, y_f
        ], dtype=float)
        
        row_print = np.array([
            x_u, y_u,
            x_v, y_v,
            x_f, y_f
        ], dtype=float)

        self.buffer[self.row_idx, :] = row
        self.row_idx += 1
        # ---- PRINT in one line ----
        print("\t".join(f"{v:.2f}" for v in row_print))

        # Stop and save when we've reached 600 rows
        if self.row_idx >= self.max_rows:
            self.get_logger().info("Collected 600 rows. Saving to file...")
            self._save_and_shutdown()

    # ---------- Countdown ----------
    def _countdown_tick(self):
        if self._count > 0:
            self.get_logger().info(f"Recording starts in {self._count}s...")
            self._count -= 1
            return

        # Once countdown finishes, begin recording
        self.countdown_timer.cancel()
        self.recording = True
        self.get_logger().info("Recording started. Collecting ~30s of data (600 rows).")

    # ---------- Helpers ----------
    def _have_all_sources(self) -> bool:
        # Ensure we have UWB, Vision, and Fuse samples
        if any(v is None for v in self.latest_u.values()):
            return False
        if any(v is None for v in self.latest_v.values()):
            return False
        if self.latest_f['x'] is None or self.latest_f['y'] is None:
            return False
        return True

    def _resolve_outfile(self) -> str:
        # priority: explicit -> x/y -> timestamp -> (ask user once if empty)
        name = (self.outfile or "").strip()
        if not name and self.x_nr is not None and self.y_nr is not None:
            name = f"measurement_x_nr_{self.x_nr}_nr_{self.y_nr}.txt"
        if not name:
            # prompt once
            try:
                name = input("Enter output filename (leave empty for timestamped): ").strip()
            except EOFError:
                name = ""
        if not name:
            stamp = self.get_clock().now().to_msg()
            name = f"measurement_{stamp.sec}_{stamp.nanosec}.txt"
        if not name.endswith(".txt"):
            name += ".txt"
        return name

    def _save_and_shutdown(self):
        # Header line with parameter names (no leading '#')
        header = "\t".join([
            "phi_u","d_u","x_u","y_u",
            "phi_v","d_v","x_v","y_v",
            "phi_f","d_f","x_f","y_f"
        ])

        # Use only the filled portion in case we ended early
        data = self.buffer[:self.row_idx, :]

        outfile = self._resolve_outfile()
        np.savetxt(
            outfile,
            data,
            fmt="%.3f",
            delimiter="\t",
            newline="\n",
            header=header,
            comments=""  # ensures header is written as the first plain line
        )

        self.get_logger().info(f"Saved {self.row_idx} rows to '{outfile}'. Shutting down.")
        # Optionally, shutdown the node after saving
        rclpy.shutdown()
    
def parse_cli():
    p = argparse.ArgumentParser(description="Record UWB/Vision/Fuse into a 12-col txt.")
    p.add_argument("--outfile", type=str, default="measurement_00_00.txt", help="Output filename (e.g., measurement_3_7.txt)")
    p.add_argument("--x-nr", type=int, default=None, help="x index for auto filename")
    p.add_argument("--y-nr", type=int, default=None, help="y index for auto filename")
    return p.parse_args()

def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
