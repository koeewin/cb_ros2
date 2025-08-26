import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
import os

# Undistorted test image
width, height = 640, 480
balance = 1.0

m_u,m_v = 334,427
pitch_msg = -0.1
roll_msg  = -0.0434

Y_c   = 0.52        # height above ground [m] (positive)
pitch_reference = -0.032
roll_reference  = 0.0045
pitch_correction = 0.095
roll_correction  = 0.005

pitch_rad =  -(pitch_msg-pitch_reference+pitch_correction)
roll_rad  =  -(roll_msg-roll_reference+roll_correction)         

# Load calibration parameters (K,D) then get rectified intrinsics new_K
calibration_path = r'C:\Users\zhang\Documents\CarrierBot\Diablo Robot\1_Software_dev\cb_ros2\cb_positioning\cb_positioning\positioning_utils\camera-parameters-human'
K_files = glob.glob(os.path.join(calibration_path, 'K*.npy'))
D_files = glob.glob(os.path.join(calibration_path, 'D*.npy'))
if not K_files or not D_files:
    raise FileNotFoundError("No K*.npy or D*.npy found in calibration_path.")

K_raw = np.load(K_files[0])
D = np.load(D_files[0])
K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_raw, D, (width, height), np.eye(3), balance=balance)

# Extract intrinsics
f_x, f_y = K[0,0], K[1,1]
c_x, c_y = K[0,2], K[1,2]

# --- Camera pose (world frame with y pointing DOWN) ---

cam_X = 0.0         # camera world X position [m]
cam_Z = 0.0         # camera world Z position [m]

# --- Angles: pitch_deg>0 = tilt down, roll_deg>0 = roll right ---

theta = pitch_rad
phi   = roll_rad


#1. Normalize pixel coordinates
a = (m_u - c_x) / f_x
b = (m_v - c_y) / f_y

# 2) Rotate camera ray by R = R_x(theta) @ R_z(-phi)
#    (minus sign on roll makes roll-left positive in this y-down convention)
cos_phi, sin_phi = np.cos(phi), np.sin(phi)
x1 = a * cos_phi + b * sin_phi
y1 = -a * sin_phi + b * cos_phi
# z1 = 1  (implicit)

cos_t, sin_t = np.cos(theta), np.sin(theta)
dwx = x1
dwy = y1 * cos_t - sin_t
dwz = y1 * sin_t + cos_t

# Tilt-only ground intersection (y=0), sign convention matches your zero-tilt formulas
# 3) Ray–plane intersection with ground y=0
#    Camera center C = (cam_X, -Y_c, cam_Z); solve -Y_c + λ*dwy = 0 => λ = Y_c / dwy
eps = 1e-9
if np.any(np.abs(dwy) < eps):
    print("Warning: some rays are near the horizon (denominator ~ 0).")

lam = Y_c / dwy

# Ground intersection in world coordinates (same axes as camera: x right, y down, z forward)
X_c = cam_X + lam * dwx
Z_c = cam_Z + lam * dwz

print("X_c:",X_c)
print("Z_c:",Z_c)