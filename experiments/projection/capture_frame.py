import cv2
import numpy as np
import argparse
import glob
import os
from datetime import datetime

# --- Edit these paths to your calibration files ---
CALIBRATION_PATH = r"/home/cb/Desktop/cb_workspace/src/cb_ros2/cb_positioning/cb_positioning/positioning_utils/camera-parameters-human"  # contains K*.npy and D*.npy
# LABEL_PATH = r"C:\path\to\labels\coco_labels.txt"  # not used here, kept for your reference

# --- Camera settings ---
CAMERA_SRC = 0
CAMERA_RESOLUTION = (640, 480)

def initialize_camera():
    cap = cv2.VideoCapture(CAMERA_SRC)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_RESOLUTION[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_RESOLUTION[1])
    # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    # cap.set(cv2.CAP_PROP_EXPOSURE, 300)  # optional
    cap.set(cv2.CAP_PROP_FPS, 20)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

def load_undistort_maps(calib_dir, resolution, balance=1.0):
    """
    Loads K and D from *.npy and prepares fisheye undistortion maps.
    """
    k_files = glob.glob(os.path.join(calib_dir, "K*.npy"))
    d_files = glob.glob(os.path.join(calib_dir, "D*.npy"))
    if not k_files:
        raise FileNotFoundError(f"No K*.npy found in {calib_dir}")
    if not d_files:
        raise FileNotFoundError(f"No D*.npy found in {calib_dir}")

    K = np.load(k_files[0])
    D = np.load(d_files[0])

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, resolution, np.eye(3), balance=balance
    )
    mapx, mapy = cv2.fisheye.initUndistortRectifyMap(
        K, D, np.eye(3), new_K, resolution, cv2.CV_16SC2
    )
    return mapx, mapy

def main():
    parser = argparse.ArgumentParser(description="Live preview; press 'c' to capture. Optional undistortion.")
    parser.add_argument("--undistort", action="store_true",
                        help="Enable fisheye undistortion using K*.npy and D*.npy from CALIBRATION_PATH.")
    parser.add_argument("--balance", type=float, default=1.0,
                        help="Balance for new camera matrix (0..1), used only with --undistort.")
    args = parser.parse_args()

    cap = initialize_camera()

    # Prepare undistortion maps if requested
    mapx = mapy = None
    if args.undistort:
        try:
            mapx, mapy = load_undistort_maps(CALIBRATION_PATH, CAMERA_RESOLUTION, balance=args.balance)
            print(f"Undistortion enabled. Maps ready: mapx {mapx.shape} {mapx.dtype}, mapy {mapy.shape} {mapy.dtype}")
        except Exception as e:
            print(f"⚠️ Could not prepare undistortion maps: {e}")
            print("Proceeding without undistortion.")
            mapx = mapy = None

    print("📺 Press 'c' to capture current frame (saved as JPEG with timestamp).")
    print("❌ Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        # Optionally undistort
        display_frame = frame
        if mapx is not None and mapy is not None:
            display_frame = cv2.remap(frame, mapx, mapy, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        cv2.imshow("Live Camera" + (" (Undistorted)" if mapx is not None else ""), display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = f"capture_{'undist_' if mapx is not None else ''}{ts}.jpg"
            cv2.imwrite(fname, display_frame)
            print(f"✅ Saved frame to {fname}")

        elif key == ord('q'):
            print("👋 Exiting...")
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()