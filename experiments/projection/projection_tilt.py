import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
import os

Robot_Model = "Diablo"
# True marker positions
m_X = np.array([0,0,0,0,-0.5,-0.5,-0.5,-0.5,-1,-1,-1,-1,0.5,0.5,0.5,0.5,1,1,1,1])
if Robot_Model == "Diablo":
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2]) + 0.180
else:
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2])

# Undistorted test image
width, height = 640, 480
balance = 1.0

if Robot_Model == "Diablo":
    m_u = np.array([330, 328, 328, 328, 167, 236, 263, 277,   9, 140, 195, 226, 483, 419, 391, 377, 623, 504, 455, 426])
    m_v = np.array([402, 338, 312, 297, 409, 339, 312, 297, 403, 339, 311, 296, 397, 336, 310, 296, 385, 332, 308, 294])
else:
    m_u = np.array([327, 327, 327, 327, 150, 230, 260, 276,   1, 127, 190, 223, 495, 423, 393, 378, 640, 513, 459, 429])
    m_v = np.array([425, 354, 326, 311, 429, 355, 326, 311, 429, 357, 327, 311, 420, 353, 325, 311, 413, 351, 324, 311])

# Load calibration parameters (K,D) then get rectified intrinsics new_K
calibration_path = r'C:\Users\zhang\Documents\CarrierBot\Diablo Robot\1_Software_dev\cb_ros2\cb_positioning\cb_positioning\positioning_utils\camera-parameters-human'
K_files = glob.glob(os.path.join(calibration_path, 'K*.npy'))
D_files = glob.glob(os.path.join(calibration_path, 'D*.npy'))
if not K_files or not D_files:
    raise FileNotFoundError("No K*.npy or D*.npy found in calibrationgit_path.")

K_raw = np.load(K_files[0])
D = np.load(D_files[0])
K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K_raw, D, (width, height), np.eye(3), balance=balance)

# Extract intrinsics
f_x, f_y = K[0,0], K[1,1]
c_x, c_y = K[0,2], K[1,2]

# Camera height
Y_c = 0.52  # [m]

# -------- Introduce tilt (pitch) about camera x-axis --------
pitch_rad = -0.085 # << set your tilt angle here (positive = camera pitched downward)
theta = pitch_rad # deg to rad

#1. Normalize pixel coordinates
a = (m_u - c_x) / f_x
b = (m_v - c_y) / f_y

# Tilt-only ground intersection (y=0), sign convention matches your zero-tilt formulas
den = b * np.cos(theta) - np.sin(theta)
eps = 1e-9
if np.any(np.abs(den) < eps):
    print("Warning: some points are near the horizon (denominator ~ 0). Results may be unstable.")

Z_c = Y_c * (b * np.sin(theta) + np.cos(theta)) / den
X_c = Y_c * a / den
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
