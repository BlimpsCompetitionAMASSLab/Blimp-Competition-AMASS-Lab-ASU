import cv2
import time

# Open the camera using OpenCV and libcamera
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Could not open camera.")
    exit()

# Set the camera resolution if needed (e.g., 640x480 or 1280x720)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# Read and display frames from the camera
while True:
    ret, frame = camera.read()
    if not ret:
        print("Failed to grab frame.")
        break

    cv2.imshow("Raspberry Pi Camera", frame)

    # Press 'q' to quit the video feed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()
