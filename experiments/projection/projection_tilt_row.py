import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
import os

Robot_Model = "Diablo"
# True marker positions
m_X = np.array([0,0,0,0,-0.5,-0.5,-0.5,-0.5,-1,-1,-1,-1,0.5,0.5,0.5,0.5,1,1,1,1])
if Robot_Model == "Diablo":
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2]) + 0.225
else:
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2])

# Undistorted test image
width, height = 640, 480
balance = 1.0

if Robot_Model == "Diablo":
    m_u = np.array([328, 328, 328, 328, 168, 236, 262, 277,  12, 141, 196, 226, 480, 417, 391, 377, 621, 503, 454, 426])
    m_v = np.array([397, 334, 309, 293, 403, 335, 309, 294, 397, 334, 308, 293, 393, 332, 307, 292, 382, 329, 305, 291])
else:
    m_u = np.array([327, 327, 327, 327, 150, 230, 260, 276,   1, 127, 190, 223, 495, 423, 393, 378, 640, 513, 459, 429])
    m_v = np.array([425, 354, 326, 311, 429, 355, 326, 311, 429, 357, 327, 311, 420, 353, 325, 311, 413, 351, 324, 311])

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
Y_c   = 0.51        # height above ground [m] (positive)
cam_X = 0.0         # camera world X position [m]
cam_Z = 0.0         # camera world Z position [m]

# --- Angles: pitch_deg>0 = tilt down, roll_deg>0 = roll right ---
pitch_deg = -5.0
roll_deg  =  -1.0
theta = np.deg2rad(pitch_deg)
phi   = np.deg2rad(roll_deg)

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
# ------------------------------------------------------------

# Fit additional affine transform to account for deviation
src_points = np.array([[X_c[0], Z_c[0]],
                       [X_c[10], Z_c[10]],
                       [X_c[14], Z_c[14]]], dtype=np.float32)
dst_points = np.array([[m_X[0], m_Z[0]],
                       [m_X[10], m_Z[10]],
                       [m_X[14], m_Z[14]]], dtype=np.float32)
T = cv2.getAffineTransform(src_points, dst_points)

P_c = np.vstack([X_c, Z_c, np.ones_like(X_c)])
P_c_corrected = T @ P_c  # shape (2, N)

d_before = np.sqrt((X_c - m_X)**2 + (Z_c - m_Z)**2)
d_after  = np.sqrt((P_c_corrected[0,:] - m_X)**2 + (P_c_corrected[1,:] - m_Z)**2)
print("mean error before affine:", np.mean(d_before))
print("mean error after  affine:", np.mean(d_after))
print("Affine T:\n", T)

plt.scatter(m_X, m_Z, s=10, c='black', label='True')
plt.scatter(X_c, Z_c, s=50, c='blue', label='Projected')
plt.scatter(P_c_corrected[0], P_c_corrected[1], s=50, c='red', label='Projected + Affine')
plt.axis('equal')
plt.legend()
plt.show()
