
import time
import cv2
import sys
import numpy as np
import serial
from picamera2 import Picamera2


# Status of program
cmdstop = "stop".encode('utf-8')
Status = True
debug_mode = False

matrix_coefficients = np.load('/home/cb/Desktop/cb_workspace/src/cb_ros2/cb/camera_calib/camera_matrices/K_PiCamV3_20250808_152053.npy')
print(matrix_coefficients)
distortion_coefficients = np.load('/home/cb/Desktop/cb_workspace/src/cb_ros2/cb/camera_calib/camera_matrices/D_PiCamV3_20250808_152053.npy')
print(distortion_coefficients)
# Set desired resolution and fps
video_w = 1920#2304#1536
video_h = 1080#1296#864
fps = 30
nFPS = 30

DIM=(video_w, video_h)
# read camera matrix
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix_coefficients,distortion_coefficients,DIM,1,DIM)

map1, map2 = cv2.initUndistortRectifyMap(matrix_coefficients, distortion_coefficients,None,newcameramtx, DIM, cv2.CV_16SC2)

# set apriltag dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
arucoParams = cv2.aruco.DetectorParameters_create()
#arucoParams = cv2.aruco.DetectorParameters()
#detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams) no more available in cv 4.6.0


# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")

picam2 = Picamera2()
print(picam2.sensor_modes)

camera_controls = {
    "AfMode": 0,             # 0 = manual focus (disable autofocus)
    "LensPosition": -1,     # Set focus manually (range depends on lens)
    "FrameDurationLimits": (int(1e6/fps), int(1e6/fps)),  # Set fixed frame duration (in microseconds) = 30 fps
}

camera_config = picam2.create_video_configuration(
    main={"size": (video_w, video_h), "format": "RGB888"},
    controls=camera_controls
)

picam2.align_configuration(camera_config)
picam2.configure(camera_config)
picam2.start()

picam2.set_controls({
"AfMode": 0,
"LensPosition": 2.5,
"ExposureTime": 10000,
})

#picam2.set_controls({"FrameRate": fps})
#min_exp, max_exp, default_exp = picam2.camera_controls["ExposureTime"]
#picam2.set_controls({"ExposureTime": 3000})
#print(min_exp, max_exp, default_exp)

time.sleep(1.0)

# framerate time
pre_f_time = 0
new_f_time = 0

fps_log = np.zeros(nFPS)
i_loop = 0
i_start = 0

# loop over the frames from the video stream
while Status == True:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 600 pixels
    time0 = time.time()

    im = picam2.capture_array()
    #frame = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(im,(960,540))
    # Convert Color space
    #frame = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    #frame = cv2.resize(im,(960,540))
    
    #frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    ## calcuation of FPS
    ## 1.type
#     new_f_time = time.time()
#     fps = 1/(new_f_time - pre_f_time)
#     fps = int(fps)
    
    ##2. type average
    new_f_time = time.time()
    fps = 1/(new_f_time - pre_f_time)
    fps = int(fps)
    
    i_loop = i_loop + 1
    fps_log = np.roll(fps_log,1)
    fps_log[0] = fps
    
    if (i_loop > nFPS):
        fps_mean = int(np.mean(fps_log))
        if fps >= 10:
            fps_str = 'fps:'+str(fps_mean)
        else:
            fps_str = 'fps:0'+str(fps_mean)
    
    pre_f_time = new_f_time

    
    
    # detect ArUco markers in the input frame
    corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters = arucoParams)
    print(ids)
    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list
        ids = ids.flatten()
        
        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            ###### new lines for pose estimation
            # pose estimation
            (rvec, tvec, markerPoints) = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.146, matrix_coefficients, distortion_coefficients)
            # Draw Axis
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.02)

            rotation_matrix = np.array([[0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],
                                [0, 0, 0, 1]],
                                dtype=float)
            rotation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)

            rotation_matrix[0][3] = tvec[0][0][0]
            rotation_matrix[1][3] = tvec[0][0][1]
            rotation_matrix[2][3] = tvec[0][0][2]
            rotation_matrix[3][0] = np.double(0)
            rotation_matrix[3][1] = np.double(0)
            rotation_matrix[3][2] = np.double(0)
            rotation_matrix[3][3] = np.double(1)
            
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            
            #draw camera coordinate

            
            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            
            # draw the ArUco marker ID on the frame
            ID_str = 'ID:' + str(markerID)
            cv2.putText(frame, ID_str,
                        (bottomRight[0]-60, bottomRight[1] ),
            cv2.FONT_HERSHEY_SIMPLEX,1, (0, 255, 0), 2)
            
            T_str = np.array2string(rotation_matrix, precision = 2,  sign='+' ,floatmode='maxprec_equal')
            T_str= T_str.replace('[','',).replace(']','')
            #T_str = np.array_repr(rotation_matrix, max_line_width = 5, floarmode='fixed',precision = 2, sign='')
            if debug_mode:
                print(T_str)
            
            
            dy = 25
            for i, txt in enumerate (T_str.split('\n')):
                if i == 0:
                    y = topRight[1] + 20 + i*dy
                    txt = ' '+txt
                    cv2.putText(frame, txt,
                    (bottomRight[0]-68, y),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.2, (0, 255, 255), 2)
                    
                y = topRight[1] + 20 + i*dy        
                cv2.putText(frame, txt,
                    (bottomRight[0]-68, y),
                    cv2.FONT_HERSHEY_PLAIN,
                    1.2, (0, 255, 255), 2)
                rotation_matrix = rotation_matrix.flatten()

    cv2.arrowedLine(frame, (480,291), (580,291), (0, 0, 255), 2,tipLength=0.15)
    cv2.putText(frame,'X',(540,365),cv2.FONT_HERSHEY_PLAIN,2,(0, 0, 255),2)
    cv2.arrowedLine(frame, (480,291), (480,394), (0, 255, 0), 2,tipLength=0.15)
    cv2.putText(frame,'Y',(450,474),cv2.FONT_HERSHEY_PLAIN,2,(0, 255, 0),2)
    # print fpx
    if (i_loop > nFPS):
        cv2.putText(frame,fps_str,(7,70),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    #time4 = time.time()
    #cv2.putText(frame,'read frame' + str(time1-time0),(7,100),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    #cv2.putText(frame,'detect' + str(time2-time1),(7,125),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    #cv2.putText(frame,'estimate'+str(time3-time2),(7,150),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    #cv2.putText(frame,'print'+str(time4-time3),(7,175),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    #cv2.putText(frame,str(time1-time0),(7,200),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    
    #cv2.putText(frame,'translation in [m]',(7,100),cv2.FONT_HERSHEY_PLAIN,2,(255, 255, 255),2)
    # show the output frame
    cv2.namedWindow('frame', cv2.WND_PROP_FULLSCREEN)
    #cv2.setWindowProperty('frame',cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)
    #cv2.resizeWindow('frame',720,405)
    cv2.imshow("frame", frame)

    
    key = cv2.waitKey(1) & 0xFF
    
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        cv2.destroyAllWindows()
        break

# do a bit of cleanup
#cap.release()
#out.release()

