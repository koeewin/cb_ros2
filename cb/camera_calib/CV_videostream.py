from picamera2 import Picamera2
import cv2
import time

# Set desired resolution
video_w = 1920#2304#1536
video_h = 1080#1296#864
fps = 20

# Initialize camera
with Picamera2() as picam2:
    # Configure the video stream
    camera_controls = {
        "AfMode": 0,             # 0 = manual focus (disable autofocus)
        "LensPosition": -1,     # Set focus manually (range depends on lens)
        "FrameDurationLimits": (int(1e6/fps), int(1e6/fps)),  # Set fixed frame duration (in microseconds) = 30 fps
    }

    # Create video configuration with custom controls
    config = picam2.create_video_configuration(
        main={"size": (video_w, video_h), "format": "RGB888"},
        controls=camera_controls
    )
    
    picam2.configure(config)
    picam2.start()
    picam2.set_controls({"AfMode": 1})
    print("Press 'q' in the video window to quit.")
    time.sleep(1)  # Optional delay to let camera warm up
    picam2.set_controls({
    "AfMode": 0,
    "LensPosition": 2.5
    })

    while True:
        # Capture a frame
        frame = picam2.capture_array()

        # Convert BGR (from camera) to RGB for display (OpenCV expects BGR)
        # Actually, since OpenCV uses BGR by default, you may not need to convert
        # If colors look off, uncomment the next line
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Show the frame in a window
        #flipped = cv2.flip(frame, 0)
        cv2.imshow("Picamera2 Video Stream", frame)

        # Break the loop if user presses 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup
    cv2.destroyAllWindows()
