from flask import Flask, Response
import cv2
import time
import atexit

"""
This script only displays the video live stream to browser.
This is a simple program to headlessly check if the camera work
visit: http://your_mbot_ip:5001/video
"""

class Camera:
    def __init__(self, camera_id, width, height):
        self.cap = cv2.VideoCapture(self.camera_pipeline(camera_id, width, height))

    def camera_pipeline(self, i, w, h):
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

    def generate_frames(self):
        if not self.cap.isOpened():
            raise IOError("Cannot open cam")

        while True:
            success, frame = self.cap.read()
            if not success:
                break

            # Encode the frame
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    def cleanup(self):
        print("Releasing camera resources")
        if self.cap and self.cap.isOpened():
            self.cap.release()

app = Flask(__name__)
camera = Camera(0, 1280, 720)
atexit.register(camera.cleanup)

@app.route('/video')
def video():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)
