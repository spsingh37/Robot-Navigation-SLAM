import cv2
import time

'''
This script is to test the functionality of the camera connected to your system.
If return ID:
Usage: python3 test_camera.py
'''

def camera_pipeline(i, w, h):
        """
        Generates a GStreamer pipeline string for capturing video from an NVIDIA camera.

        Parameters:
        i (int): The sensor ID of the camera.
        w (int): The width of the video frame in pixels.
        h (int): The height of the video frame in pixels.
        """
        return "nvarguscamerasrc sensor_id=%d ! \
        video/x-raw(memory:NVMM), \
        width=%d, height=%d, \
        format=(string)NV12, \
        framerate=30/1 ! \
        nvvidconv \
        flip-method=0  ! \
        video/x-raw, \
        format=(string)BGRx ! \
        videoconvert ! \
        video/x-raw, \
        format=(string)BGR ! \
        appsink" % (i, w, h)


def test_camera(camera_id):
    cap = cv2.VideoCapture(camera_pipeline(camera_id, 1280, 720))
    while True:
        ret, frame = cap.read()
        if not ret:
            cap.release()
            return False
        else:
            cap.release()
            return True

valid_id = -1
for i in range(0, 2):  # Adjust the range based on how many cameras you expect
    is_valid = test_camera(i)
    if is_valid:
        valid_id = i

print("================================")
if (valid_id == -1):
    print(f"Cannot read your camera!")
else:
    print(f"Your camera ID {valid_id} is functional!")
print("================================")