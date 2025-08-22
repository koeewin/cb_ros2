import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2

# True marker positions
m_X = np.array([0,0,0,0,-0.5,-0.5,-0.5,-0.5,-1,-1,-1,-1,0.5,0.5,0.5,0.5,1,1,1,1])
m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2])

# Undistorted test image
width = 640
height = 480
balance = 1
m_u = np.array([327, 327, 327, 327, 150, 230, 260, 276,   1, 127, 190, 223, 495, 423, 393, 378, 640, 513, 459, 429])
m_v = np.array([425, 354, 326, 311, 429, 355, 326, 311, 429, 357, 327, 311, 420, 353, 325, 311, 413, 351, 324, 311])

# Load calibration parameters
calibration_path = '../camera-calibration/calibration-results/'
K = np.load(glob.glob(calibration_path + 'K*.npy')[0])	# camera matrix
D = np.load(glob.glob(calibration_path + 'D*.npy')[0])	# distortion coefficients
K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, (width, height), np.eye(3), balance=balance)
#print(K)

# Extract values from camera matrix
f_x = K[0,0]	# focal length in x [pixel]
f_y = K[1,1]	# focal length in y [pixel]
c_x = K[0,2]	# x coordinate of optical center [pixel] 
c_y = K[1,2]	# y coordinate of optical center [pixel]

# Define height of camera above ground
Y_c = 0.47 	# [m]

#Z_c = 1/c_y * (m_v[0] - f_y*Y_c)
#X_c = 1/f_x * (m_u[0] - c_x*Z_c)
Z_c = f_y*Y_c/(m_v-c_y)
X_c = (m_u-c_x)*Z_c / f_x

#print(Z_c)
#print(X_c)

# Fit additional affine transform to account for deviation
src_points = np.array([[[X_c[0], Z_c[0]]],[[X_c[10], Z_c[10]]],[[X_c[14], Z_c[14]]]], dtype=np.float32)
dst_points = np.array([[[m_X[0], m_Z[0]]],[[m_X[10], m_Z[10]]],[[m_X[14], m_Z[14]]]], dtype=np.float32)
T = cv2.getAffineTransform(src_points, dst_points)
#print(T)
P_c = np.array([X_c, Z_c, np.ones(X_c.shape[0])])
P_c_corrected = np.matmul(T,P_c)

d = np.sqrt((X_c - m_X)**2 + (Z_c - m_Z)**2)
print(np.mean(d)) 
d = np.sqrt((P_c_corrected[0,:] - m_X)**2 + (P_c_corrected[1,:] - m_Z)**2)
print(np.mean(d))
print(T)

plt.scatter(m_X, m_Z, s=10, c='black')
plt.scatter(X_c, Z_c, s=50, c='blue')
plt.scatter(P_c_corrected[0], P_c_corrected[1], s=50, c='red')
plt.show()
