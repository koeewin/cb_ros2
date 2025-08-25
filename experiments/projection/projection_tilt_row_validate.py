import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
import os

Robot_Model = "Diablo"
pitchNr = "normal"  # "normal", "up", "down", "up2"
useRoll = True
useReference = True


Y_c   = 0.52        # height above ground [m] (positive)
pitch_reference = -0.032
roll_reference  = 0.0045
pitch_correction = 0.095
roll_correction  = 0.005

# True marker positions
m_X = np.array([0,0,0,0,-0.5,-0.5,-0.5,-0.5,-1,-1,-1,-1,0.5,0.5,0.5,0.5,1,1,1,1])
if Robot_Model == "Diablo":
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2]) + 0.27
else:
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2])

# Undistorted test image
width, height = 640, 480
balance = 1.0

if pitchNr == "normal":
    pitch_rad = -(-0.0722) if not useReference else -(-0.0722-pitch_reference+pitch_correction)
    roll_rad  = -(-0.0434) if not useReference else -(-0.0434-roll_reference+roll_correction)          
    m_u = np.array([331, 331, 332, 331, 183, 243, 268, 282,  37, 154, 204, 233, 475, 417, 393, 379, 610, 501, 454, 427])
    m_v = np.array([393, 337, 314, 299, 390, 334, 311, 297, 377, 328, 306, 293, 398, 340, 316, 301, 396, 342, 317, 302])
elif pitchNr == "up": # negative pitch
    pitch_rad = -(-0.207) if not useReference else -(-0.207-pitch_reference+pitch_correction)
    roll_rad  = -(-0.0434) if not useReference else -(-0.0434-roll_reference+roll_correction)          
    m_u = np.array([339, 338, 339, 338, 190, 250, 275, 288,  50, 162, 211, 240, 491, 428, 401, 387, 635, 517, 466, 437])
    m_v = np.array([431, 375, 349, 334, 429, 370, 346, 332, 413, 364, 341, 328, 444, 379, 352, 336, 442, 382, 354, 338])
elif pitchNr == "down": # positive pitch
    pitch_rad = -(0.1188) if not useReference else -(0.1188-pitch_reference+pitch_correction)
    roll_rad  = -(-0.0443) if not useReference else -(-0.0443-roll_reference+roll_correction)
    m_u = np.array([350, 349, 349, 348, 213, 264, 287, 300,  84, 182, 226, 253, 496, 437, 412, 387, 639, 527, 477, 449])
    m_v = np.array([345, 294, 272, 258, 336, 289, 268, 255, 322, 282, 263, 251, 355, 299, 275, 260, 357, 303, 277, 263])

elif pitchNr == "up2": # positive pitch
    pitch_rad = -(-0.082) if not useReference else -(-0.082-pitch_reference+pitch_correction)
    roll_rad  = -(-0.0433) if not useReference else -(-0.0433-roll_reference+roll_correction)
    m_u = np.array([345, 344, 344, 343, 204, 258, 282, 295,  72, 175, 221, 248, 492, 431, 405, 392, 636, 520, 471, 441])
    m_v = np.array([394, 340, 316, 302, 387, 335, 313, 300, 372, 328, 308, 296, 404, 344, 319, 304, 405, 348, 322, 306])
if useRoll is False:
    roll_rad = 0.0
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
# ------------------------------------------------------------

# Fit additional affine transform to account for deviation
src_points = np.array([[X_c[0], Z_c[0]],
                       [X_c[10], Z_c[10]],
                       [X_c[14], Z_c[14]]], dtype=np.float32)
dst_points = np.array([[m_X[0], m_Z[0]],
                       [m_X[10], m_Z[10]],
                       [m_X[14], m_Z[14]]], dtype=np.float32)
T = cv2.getAffineTransform(src_points, dst_points)

T_global = np.array(
 [[ 9.57462213e-01, -4.48312286e-04, -3.15355000e-02],
 [ 1.31302876e-02,  9.94825428e-01, -1.89673207e-02]])

P_c = np.vstack([X_c, Z_c, np.ones_like(X_c)])
P_c_corrected = T @ P_c  # shape (2, N)
P_global_corrected = T_global @ P_c  # shape (2, N)


d_before = np.sqrt((X_c - m_X)**2 + (Z_c - m_Z)**2)
d_after  = np.sqrt((P_c_corrected[0,:] - m_X)**2 + (P_c_corrected[1,:] - m_Z)**2)
print("mean error before affine:", np.mean(d_before))
print("mean error after  affine:", np.mean(d_after))
print("Affine T:\n", T)

plt.scatter(m_X, m_Z, s=10, c='black', label='True')
plt.scatter(X_c, Z_c, s=50, c='blue', label='Projected')
plt.scatter(P_c_corrected[0], P_c_corrected[1], s=50, c='red', label='Projected + Affine')
plt.scatter(P_global_corrected[0], P_global_corrected[1], s=50, c='green', label='Projected + glob. Affine')
plt.axis('equal')
plt.legend()
plt.show()
