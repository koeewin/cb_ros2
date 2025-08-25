import numpy as np
import matplotlib.pyplot as plt
import glob
import cv2
import os

# ---------------------- Config ----------------------
Robot_Model      = "Diablo"
useRoll          = True
useReference     = True

Y_c              = 0.52      # camera height above ground [m] (positive)
pitch_reference  = -0.032
roll_reference   = 0.0045
pitch_correction = 0.095
roll_correction  = 0.005

width, height    = 640, 480
balance          = 1.0

# Per-case weights for the global affine (down gets less weight)
case_weights = {
    "normal": 1.0,
    "up":     0.6,
    "down":   0.2,   # de-emphasize "down" case
    "up2":    0.2,
}

# True marker positions
m_X = np.array([0,0,0,0,-0.5,-0.5,-0.5,-0.5,-1,-1,-1,-1,0.5,0.5,0.5,0.5,1,1,1,1], dtype=float)
if Robot_Model == "Diablo":
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2], dtype=float) + 0.27
else:
    m_Z = np.array([0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2,0.5,1,1.5,2], dtype=float)

# ---------------------- Calibration ----------------------
calibration_path = r'C:\Users\zhang\Documents\CarrierBot\Diablo Robot\1_Software_dev\cb_ros2\cb_positioning\cb_positioning\positioning_utils\camera-parameters-human'
K_files = glob.glob(os.path.join(calibration_path, 'K*.npy'))
D_files = glob.glob(os.path.join(calibration_path, 'D*.npy'))
if not K_files or not D_files:
    raise FileNotFoundError("No K*.npy or D*.npy found in calibration_path.")

K_raw = np.load(K_files[0])
D = np.load(D_files[0])
K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
    K_raw, D, (width, height), np.eye(3), balance=balance
)
f_x, f_y = float(K[0,0]), float(K[1,1])
c_x, c_y = float(K[0,2]), float(K[1,2])

# ---------------------- Case data ----------------------
def get_case_data(name: str):
    if name == "normal":
        pitch_meas = -0.0722; roll_meas = -0.0434
        m_u = np.array([331, 331, 332, 331, 183, 243, 268, 282,  37, 154, 204, 233, 475, 417, 393, 379, 610, 501, 454, 427], dtype=float)
        m_v = np.array([393, 337, 314, 299, 390, 334, 311, 297, 377, 328, 306, 293, 398, 340, 316, 301, 396, 342, 317, 302], dtype=float)
    elif name == "up":
        pitch_meas = -0.207;  roll_meas = -0.0434
        m_u = np.array([339, 338, 339, 338, 190, 250, 275, 288,  50, 162, 211, 240, 491, 428, 401, 387, 635, 517, 466, 437], dtype=float)
        m_v = np.array([431, 375, 349, 334, 429, 370, 346, 332, 413, 364, 341, 328, 444, 379, 352, 336, 442, 382, 354, 338], dtype=float)
    elif name == "down":
        pitch_meas =  0.1188; roll_meas = -0.0443
        m_u = np.array([350, 349, 349, 348, 213, 264, 287, 300,  84, 182, 226, 253, 496, 437, 412, 387, 639, 527, 477, 449], dtype=float)
        m_v = np.array([345, 294, 272, 258, 336, 289, 268, 255, 322, 282, 263, 251, 355, 299, 275, 260, 357, 303, 277, 263], dtype=float)
    elif name == "up2":
        pitch_meas = -0.082;  roll_meas = -0.0433
        m_u = np.array([345, 344, 344, 343, 204, 258, 282, 295,  72, 175, 221, 248, 492, 431, 405, 392, 636, 520, 471, 441], dtype=float)
        m_v = np.array([394, 340, 316, 302, 387, 335, 313, 300, 372, 328, 308, 296, 404, 344, 319, 304, 405, 348, 322, 306], dtype=float)
    else:
        raise ValueError(f"Unknown case: {name}")
    return pitch_meas, roll_meas, m_u, m_v

def resolve_angle(measured, reference, correction, use_reference: bool):
    # angle_rad = -(measured) if not use_reference
    #           = -(measured - reference + correction) if use_reference
    return -(measured if not use_reference else measured - reference + correction)

# ---------------------- Geometry ----------------------
def project_points(m_u, m_v, pitch_rad, roll_rad):
    a = (m_u - c_x) / f_x
    b = (m_v - c_y) / f_y

    # R = R_x(theta) @ R_z(-phi) in y-down convention
    cos_phi, sin_phi = np.cos(roll_rad), np.sin(roll_rad)
    x1 = a * cos_phi + b * sin_phi
    y1 = -a * sin_phi + b * cos_phi

    cos_t, sin_t = np.cos(pitch_rad), np.sin(pitch_rad)
    dwx = x1
    dwy = y1 * cos_t - sin_t
    dwz = y1 * sin_t + cos_t

    eps = 1e-9
    if np.any(np.abs(dwy) < eps):
        print("Warning: some rays are near the horizon (denominator ~ 0).")

    lam = Y_c / dwy
    X_c = lam * dwx
    Z_c = lam * dwz
    return X_c, Z_c

def apply_affine(T, X_c, Z_c):
    P = np.vstack([X_c, Z_c, np.ones_like(X_c)])   # (3, N)
    Pc = T @ P                                     # (2, N)
    return Pc[0], Pc[1]

# Weighted LS for single global 2x3 affine using all cases
def estimate_affine2d_weighted(per_case_list, case_weights_dict):
    """
    Solve for T (2x3) minimizing sum_i w_i * || [x_i', z_i']^T - T * [x_i, z_i, 1]^T ||^2
    where weights come from the case each point belongs to.
    """
    rows = []
    rhs  = []
    wrs  = []  # weights per equation

    for entry in per_case_list:
        name = entry["case"]
        w = float(case_weights_dict.get(name, 1.0))
        Xc, Zc = entry["X_c"], entry["Z_c"]
        mX, mZ = entry["m_X"], entry["m_Z"]
        for x, z, x_t, z_t in zip(Xc, Zc, mX, m_Z):
            # x' = a11*x + a12*z + a13
            rows.append([x, z, 1.0, 0.0, 0.0, 0.0])
            rhs.append(x_t)
            wrs.append(w)
            # z' = a21*x + a22*z + a23
            rows.append([0.0, 0.0, 0.0, x, z, 1.0])
            rhs.append(z_t)
            wrs.append(w)

    A = np.asarray(rows, dtype=np.float64)   # (2N, 6)
    b = np.asarray(rhs,  dtype=np.float64)   # (2N,)
    w = np.asarray(wrs,  dtype=np.float64)   # (2N,)

    # Apply weights via sqrt(w)
    sw = np.sqrt(w)
    Aw = A * sw[:, None]
    bw = b * sw

    p, *_ = np.linalg.lstsq(Aw, bw, rcond=None)  # (6,)
    T = np.array([[p[0], p[1], p[2]],
                  [p[3], p[4], p[5]]], dtype=np.float64)
    return T

# ---------------------- Project all cases, fit ONE weighted global affine ----------------------
cases = ["normal", "up", "down", "up2"]

per_case = []   # stores dicts with projections & references for each case

for name in cases:
    pitch_meas, roll_meas, u, v = get_case_data(name)
    pitch_rad = resolve_angle(pitch_meas, pitch_reference, pitch_correction, useReference)
    roll_rad  = resolve_angle(roll_meas,  roll_reference,  roll_correction,  useReference) if useRoll else 0.0

    X_c, Z_c = project_points(u, v, pitch_rad, roll_rad)

    per_case.append({
        "case": name,
        "X_c": X_c, "Z_c": Z_c,
        "m_X": m_X, "m_Z": m_Z
    })

# Global weighted affine using all cases
T_global = estimate_affine2d_weighted(per_case, case_weights)
print("\n=== Global weighted affine (all cases) ===")
print("T_global (2x3):\n", T_global)

# ---------------------- Plot each case with the same global T ----------------------
fig, axes = plt.subplots(2, 2, figsize=(10, 9), constrained_layout=True)
axes = axes.ravel()

all_before = []
all_after  = []

for ax, entry in zip(axes, per_case):
    name = entry["case"]
    X_c, Z_c = entry["X_c"], entry["Z_c"]
    mX, mZ   = entry["m_X"], entry["m_Z"]

    # errors BEFORE global affine
    d_before = np.hypot(X_c - mX, Z_c - mZ)
    mean_before = float(d_before.mean())
    all_before.append(d_before)

    # apply global affine
    X_corr, Z_corr = apply_affine(T_global, X_c, Z_c)
    d_after = np.hypot(X_corr - mX, Z_corr - mZ)
    mean_after = float(d_after.mean())
    all_after.append(d_after)

    # print per-point distances for this case
    print(f"\n=== {name} (using GLOBAL WEIGHTED T) ===")
    print("Distances BEFORE (m):", np.array2string(d_before, precision=4, separator=", "))
    print(f"Mean BEFORE: {mean_before:.4f} m")
    print("Distances AFTER  (m):", np.array2string(d_after,  precision=4, separator=", "))
    print(f"Mean AFTER : {mean_after:.4f} m")

    # plot
    ax.scatter(mX, mZ, s=20, label='True (m_X, m_Z)')
    ax.scatter(X_c, Z_c, s=40, marker='x', label='Projected')
    ax.scatter(X_corr, Z_corr, s=40, label='Projected + Global (weighted) Affine')
    ax.set_title(f"{name} | mean err: {mean_before:.3f} → {mean_after:.3f} m")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')

plt.show()

# ---------------------- Overall stats ----------------------
if len(all_before) and len(all_after):
    all_before = np.concatenate(all_before)
    all_after  = np.concatenate(all_after)
    print("\n=== Overall (all cases, all points) ===")
    print(f"Mean BEFORE: {all_before.mean():.4f} m")
    print(f"Mean AFTER : {all_after.mean():.4f} m")
