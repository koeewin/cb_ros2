from visualize_path import cv2_plot_path
import time
import numpy as np

hz = 20
dt = 1.0 / hz

# first 10 points: straight line
#data = [
#    [3.563, 2.194, 1.419, 1.504, 1.551, 1.568, 1.549, 1.518, 1.501, 1.524,
#     -0.096, -0.096, 1.44, 1.351, 1.332, 1.254, 1.215, 1.192, 1.166, 1.138],
#    [-0.389, -0.215, 0.194, 0.215, 0.244, 0.272, 0.513, 0.747, 0.735, 0.751,
#     1.383, 1.359, 0.721, 0.712, 0.697, 0.701, 0.682, 0.676, 0.664, 0.649]
#]

data = [[0],[0]]

arr = np.array(data)

# final path: 2x20
path = arr

while True:
    cv2_plot_path(path)
    time.sleep(dt)