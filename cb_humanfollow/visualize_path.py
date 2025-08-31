# file: path_plot_cv2.py

import numpy as np
import cv2
import time

def cv2_plot_path(path, scale=200, xlim=(-0.5, 3), ylim=(-2, 2), winname="Path"):
    """
    Plot 2xN path array into an OpenCV window.
    scale : pixels per unit
    xlim, ylim : axis ranges
    winname : OpenCV window name
    """
    h = int((ylim[1] - ylim[0]) * scale)
    w = int((xlim[1] - xlim[0]) * scale)
    img = np.ones((h, w, 3), dtype=np.uint8) * 255  # white background

    # map world to pixel coords
    def world2pix(x, y):
        px = int((x - xlim[0]) * scale)
        py = int((ylim[1] - y) * scale)   # invert y-axis
        return px, py

    # draw path
    pts = [world2pix(path[0, i], path[1, i]) for i in range(path.shape[1])]
    for i in range(len(pts) - 1):
        cv2.line(img, pts[i], pts[i+1], (255, 0, 0), 2)
    for pt in pts:
        cv2.circle(img, pt, 3, (0, 0, 255), -1)

    cv2.imshow(winname, img)
    cv2.waitKey(1)  # process GUI events

# ---------------------------
# Example usage if run directly
# ---------------------------
if __name__ == "__main__":
    hz = 20
    dt = 1.0 / hz

    # first 10 points: straight line
    p1 = np.column_stack([np.linspace(0, 2.2, 10),
                          np.linspace(0, 0.8, 10)])

    # next 10 points: smooth curve
    t  = np.linspace(0, 1, 10)
    x2 = 2.2 + 0.9*np.sin(0.9*t)
    y2 = 0.8 + 1.75*(1 - np.cos(0.9*t))
    p2 = np.column_stack([x2, y2])

    # final path: 2x20
    path = np.vstack([p1, p2]).T

    while True:
        cv2_plot_path(path)
        time.sleep(dt)
