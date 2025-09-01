import numpy as np
import matplotlib.pyplot as plt
def discretize_path(points: np.ndarray, ds: float = 0.5) -> np.ndarray:
    """
    Resample a 2xN path so that consecutive points are ~ds apart (default 0.5 m).
    - If N == 0: returns empty (2x0).
    - If N == 1: draws a straight line from (0,0) to the point and samples every ds.
    - If N == 2: straight line between the two points, sampled every ds.
    - If N >= 3: centripetal Catmull–Rom spline through the points, then arc-length resampled at ds.

    Parameters
    ----------
    points : np.ndarray
        Shape (2, N). N must be <= 30 (extra columns are ignored).
    ds : float
        Target spacing between consecutive points.

    Returns
    -------
    np.ndarray
        Shape (2, M) resampled path (M varies depending on total length).
    """
    if points.ndim != 2 or points.shape[0] != 2:
        raise ValueError("points must be a 2xN numpy array")
    if ds <= 0:
        raise ValueError("ds must be positive")

    # Enforce N <= 30 (ignore extras if present)
    P = np.array(points[:, :30], dtype=float, copy=True)
    N = P.shape[1]

    # Remove consecutive duplicates to avoid zero-length segments
    if N > 1:
        keep = [0]
        for i in range(1, N):
            if np.linalg.norm(P[:, i] - P[:, keep[-1]]) > 1e-12:
                keep.append(i)
        P = P[:, keep]
        N = P.shape[1]

    if N == 0:
        return np.zeros((2, 0))
    if N == 1:
        # Straight line from origin to the single point
        p0 = np.zeros(2)
        p1 = P[:, 0]
        L = np.linalg.norm(p1 - p0)
        if L < 1e-12:
            return P[:, :1]  # just the point at the origin
        s = np.arange(0.0, L + 1e-9, ds)
        if s[-1] < L - 1e-9:
            s = np.append(s, L)
        t = s / L
        path = (p0[:, None] * (1 - t) + p1[:, None] * t)
        return path

    if N == 2:
        # Straight line between two points
        p0, p1 = P[:, 0], P[:, 1]
        L = np.linalg.norm(p1 - p0)
        if L < 1e-12:
            return P[:, :1]
        s = np.arange(0.0, L + 1e-9, ds)
        if s[-1] < L - 1e-9:
            s = np.append(s, L)
        t = s / L
        path = (p0[:, None] * (1 - t) + p1[:, None] * t)
        return path

    # --- N >= 3: Catmull–Rom (centripetal) + arc-length resample ---
    # Build centripetal parameterization
    alpha = 0.5
    pts = P.T  # shape (N, 2) as row points
    t = np.zeros(N)
    for i in range(1, N):
        d = np.linalg.norm(pts[i] - pts[i-1])
        t[i] = t[i-1] + d**alpha

    # Helper: evaluate segment spline between i and i+1 given u in [0,1]
    def catmull_rom_segment(i, u):
        # control points with endpoint duplication (clamped)
        p0 = pts[max(i-1, 0)]
        p1 = pts[i]
        p2 = pts[min(i+1, N-1)]
        p3 = pts[min(i+2, N-1)]

        t0, t1, t2, t3 = t[max(i-1, 0)], t[i], t[i+1], t[min(i+2, N-1)]

        # Avoid zero denominators by small eps
        def tj_diff(a, b):
            d = (b - a)
            return d if abs(d) > 1e-12 else 1e-12

        # Tangents m1, m2 (centripetal formulation)
        m1 = (p2 - p0) / tj_diff(t0, t2) * (t2 - t1)
        m2 = (p3 - p1) / tj_diff(t1, t3) * (t2 - t1)

        # Cubic Hermite basis
        u2 = u*u
        u3 = u2*u
        h00 = 2*u3 - 3*u2 + 1
        h10 = u3 - 2*u2 + u
        h01 = -2*u3 + 3*u2
        h11 = u3 - u2
        return h00*p1 + h10*m1 + h01*p2 + h11*m2  # (2,)

    # Oversample the spline adaptively (at least ~5 samples per ds per segment)
    dense = [pts[0]]
    for i in range(0, N-1):
        seg_len_est = np.linalg.norm(pts[i+1] - pts[i])
        k = max(8, int(np.ceil(seg_len_est / ds) * 6))  # denser for smoother curvature
        for j in range(1, k+1):
            u = j / k
            dense.append(catmull_rom_segment(i, u))
    dense = np.array(dense)  # shape (M0, 2)

    # Compute cumulative arc length
    dxy = np.diff(dense, axis=0)
    seg = np.linalg.norm(dxy, axis=1)
    s_cum = np.concatenate(([0.0], np.cumsum(seg)))
    total_L = s_cum[-1]

    if total_L < 1e-12:
        # Degenerate: all points identical
        return P[:, :1]

    # Target sample distances
    s_target = np.arange(0.0, total_L + 1e-9, ds)
    if s_target[-1] < total_L - 1e-9:
        s_target = np.append(s_target, total_L)

    # Interpolate along arc length (component-wise linear over s)
    x = np.interp(s_target, s_cum, dense[:, 0])
    y = np.interp(s_target, s_cum, dense[:, 1])
    return np.vstack((x, y))

def cut_path(path: np.ndarray, d_cut: float) -> np.ndarray:
    """
    Cut a 2xN path to start from distance d_cut (measured from origin).
    Only considers the first one-third of points for cutting.
    If fewer than 3 points exist, considers all of them.

    Parameters
    ----------
    path : np.ndarray
        2xN array of path points [x; y].
    d_cut : float
        Distance from the origin to start cutting.

    Returns
    -------
    np.ndarray
        2xM array of the cut path.
    """
    if path.shape[1] < 2:
        return path  # nothing to cut if single point

    # cumulative arc length
    diffs = np.diff(path, axis=1)
    seg_lengths = np.linalg.norm(diffs, axis=0)
    cumdist = np.concatenate(([0], np.cumsum(seg_lengths)))

    # determine how many points to check
    if path.shape[1] < 3:
        check_idx = np.arange(path.shape[1])
    else:
        max_idx = max(1, path.shape[1] // 3)  # at least 1 point
        check_idx = np.arange(max_idx + 1)    # include boundary

    # find first index where distance >= d_cut, restricted to first 1/3
    valid = np.where(cumdist[check_idx] >= d_cut)[0]
    if valid.size > 0:
        cut_idx = valid[0]
        return path[:, cut_idx:]
    else:
        # no cut possible, return full path
        return path

def find_forward_points(self):

    #Extract x and y of waypoints
    x_coords, y_coords = self.path_storage[0, :], self.path_storage[1, :]

    # Find the first point ahead of the robot
    for i in range(x_coords.shape[0]) :             
        if x_coords[i] > 0.1 and np.linalg.norm(self.path_storage[:, i]) > 0.2: 
            # Return the remaining path and the next waypoint 
            pruned_path = self.path_storage[:, i:] 
            return pruned_path
                
    # No points ahead 
    print(1)
    return np.empty((2, 0))


import numpy as np

import numpy as np

def prune_prefix_behind(P: np.ndarray, pose: np.ndarray, x_percent: float) -> np.ndarray:
    """
    From a 2xN path P, check only the first `x_percent` of points w.r.t. the robot pose.
    Any points in that *checked prefix* that lie behind the robot's heading are removed.
    The remaining (1 - x_percent) tail of the path is kept as-is.

    Args:
        P        : 2xN array of path points in world frame
        pose     : [x, y, theta] in world frame (theta in radians)
        x_percent: fraction in [0, 1] of the *prefix* to check (by count)

    Returns:
        2xM array with the filtered prefix followed by the untouched tail (order preserved).
    """
    # --- Checks ---
    if P.ndim != 2 or P.shape[0] != 2:
        raise ValueError("P must be a 2xN numpy array (2 rows).")
    if pose.shape != (3,):
        raise ValueError("pose must be a 1D array [x, y, theta].")
    if not (0.0 <= x_percent <= 1.0):
        raise ValueError("x_percent must be in [0, 1].")

    N = P.shape[1]
    if N == 0 or x_percent == 0.0:
        return P.copy()  # nothing to check

    # How many points to check in the prefix (by count)
    n_check = int(np.ceil(x_percent * N))
    n_check = min(max(n_check, 0), N)  # clamp to [0, N]

    # Robot heading (unit) and relative vectors to prefix points
    x, y, theta = map(float, pose)
    hx, hy = np.cos(theta), np.sin(theta)

    # Split into prefix to check and tail to keep
    P_prefix = P[:, :n_check]
    P_tail   = P[:, n_check:]

    # For the prefix, keep only points in *front* (projection >= 0)
    V = P_prefix - np.array([[x], [y]])              # 2 x n_check
    proj = hx * V[0, :] + hy * V[1, :]               # length n_check
    keep_prefix_idx = np.flatnonzero(proj >= 0.0)    # indices to keep in prefix

    # Reassemble: filtered prefix + untouched tail
    if keep_prefix_idx.size == 0:
        return P_tail.copy()  # all checked prefix points were behind
    else:
        return np.hstack((P_prefix[:, keep_prefix_idx], P_tail))

def adjust_pathMPC(path: np.ndarray, N: int = 5) -> np.ndarray:
    """
    Adjust a 2xn path array to size 2x(N+2).
    
    - If path has fewer than N+2 columns: pad with last column.
    - If path has more: keep only the first N columns, then pad to N+2.
    """
    if path.shape[0] != 2:
        raise ValueError("Input must be a 2xn numpy array")

    n_cols = path.shape[1]
    target_cols = N + 2

    if n_cols < target_cols:
        # pad with last column
        last_col = path[:, -1].reshape(2, 1)
        repeat_cols = target_cols - n_cols
        pad = np.repeat(last_col, repeat_cols, axis=1)
        return np.hstack([path, pad])
    else:
        # take first N, then pad to N+2
        trimmed = path[:, :N]
        last_col = trimmed[:, -1].reshape(2, 1)
        pad = np.repeat(last_col, 2, axis=1)  # add 2 extra columns
        return np.hstack([trimmed, pad])
# ====================================================  ========================

def plot_path(path: np.ndarray):
    """
    Plot a 2xN path array (first row: x, second row: y).
    """
    if path.shape[0] != 2:
        raise ValueError("Path must be a 2xN numpy array")
    
    x, y = path[0, :], path[1, :]
    
    plt.figure()
    plt.plot(x, y, 'b.-', label="Path")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.xlim(-0.5, 5)       # x-axis range
    plt.ylim(-2, 2)   # y-axis range
    
    plt.grid(True)
    plt.legend()
    plt.show()


def generate_path(num_points=10, x_start_range=(0.2, 0.3), x_end=5.0, y_range=(-2, 2), seed=None):
    """
    Generate a 2xN path:
      - First point at (random in x_start_range, y=0)
      - X increases monotonically to x_end
      - Y random within y_range (first y is 0)
    """
    if seed is not None:
        rng = np.random.default_rng(seed)
    else:
        rng = np.random.default_rng()

    # First point
    x0 = rng.uniform(*x_start_range)
    y0 = 0.0

    # Build full x with num_points entries, starting at x0 and ending at x_end
    x = np.linspace(x0, x_end, num_points)

    # Build full y with num_points entries (first 0, rest random)
    y = np.empty(num_points, dtype=float)
    y[0] = y0
    y[1:] = rng.uniform(y_range[0], y_range[1], num_points - 1)

    return np.vstack((x, y))



if __name__ == "__main__":

    P = np.array([[0,1,2,3,4,5,6],
              [0,0,0,0,0,0,0]])

    print(adjust_pathMPC(P, N=5))   # (2, 7) → 2x(N+2)
    print(adjust_pathMPC(P[:, :1], N=5))  # shorter input → also (2, 7)
    
    #P = generate_path(3)

    # P = np.array([[0.3, 1.0, 2.0, 2.0, 0.0],
    #               [0.0, 0.5, 0.0, -1.0, -1.0]])
    # pose = np.array([1.0, 0.0, 0.0])  # at x=2, facing +x
    
    # print("P1 =", P)
    # #plot_path(P)
    # R1 = discretize_path(P, ds=0.2)
    # print("R1 =", R1)
    # out = prune_prefix_behind(R1, pose, x_percent=0.51)
    # RC1 = out
   
    # #plot_path(R1)
    # RC2 = cut_path(R1, d_cut=1.0)
    # print("RC1 =", RC1)
    # #plot_path(RC1)
    # fig, axes = plt.subplots(1, 3, figsize=(12, 4))
    # axes[0].plot(P[0, :], P[1, :], 'ro-', label="Original")
    # axes[0].set_title("P1: Original Path")

    # # Plot R1
    # axes[1].plot(R1[0, :], R1[1, :], 'b.-', label="Discretized")
    # axes[1].set_title("R1: Discretized Path")

    # # Plot RC1
    # axes[2].plot(RC2[0, :], RC2[1, :], 'g.-', label="Cut")
    # axes[2].set_title("RC1: Cut Path")

    # # Common formatting
    # for ax in axes:
    #     ax.set_xlabel("X")
    #     ax.set_ylabel("Y")
    #     ax.set_xlim(-1, 5)
    #     ax.set_ylim(-2, 2)
    #     ax.set_aspect('equal', adjustable='box')
    #     ax.grid(True)
    #     ax.legend()


    # plt.tight_layout()
    # plt.show()

    