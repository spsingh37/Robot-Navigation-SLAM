from flask import Flask, Response, render_template, request, jsonify
import threading
import cv2
import time
import atexit
import numpy as np
import signal
import os

"""
This script displays the video live stream to browser with a button "save image".
When you click the button, the current frame will be saved to "/images"
visit: http://your_mbot_ip:5001
"""

class Camera:
    def __init__(self, camera_id, width, height):
        self.cap = cv2.VideoCapture(self.camera_pipeline(camera_id, width, height))
        self.image_count = 0
        self.lock = threading.Lock()
        self.frame = None
        self.running = True
        self.update_thread = threading.Thread(target=self.update_frame, args=())
        self.update_thread.start()

    def camera_pipeline(self, i, w, h):
        """
        Generates a GStreamer pipeline string for capturing video from an NVIDIA camera.

        Parameters:
        i (int): The sensor ID of the camera.
        w (int): The width of the video frame in pixels.
        h (int): The height of the video frame in pixels.
        """
        return f"nvarguscamerasrc sensor_id={i} ! \
        video/x-raw(memory:NVMM), \
        width=1280, height=720, \
        format=(string)NV12, \
        framerate=30/1 ! \
        nvvidconv \
        flip-method=0 ! \
        video/x-raw, \
        format=(string)BGRx, \
        width={w}, height={h} !\
        videoconvert ! \
        video/x-raw, \
        format=(string)BGR ! \
        appsink"

    def update_frame(self):
        while self.running:
            success, frame = self.cap.read()
            if not success:
                self.running = False
            with self.lock:
                self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame if self.frame is not None else None

    def generate_frames(self):
        while self.running:
            frame = self.get_frame()
            if frame is not None:
                # Encode the frame 
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                        b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                # If no frame is captured, we just continue the loop and try again
                continue

    def cleanup(self):
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join()  # Wait for the frame update thread to finish
        if self.cap and self.cap.isOpened():
            self.cap.release()
        print("Camera resources released")

def signal_handler(sig, frame):
    print('Shutting down gracefully...')
    camera.cleanup()  # Clean up camera resources
    os._exit(0)  # Exit the program
    
app = Flask(__name__)

@app.route('/')
def video_page():
    return render_template('image_save_page.html')

@app.route('/video')
def video():
    return Response(camera.generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/save-image', methods=['POST'])
def save_image():
    # Extract the image save path from the POST request
    camera.image_count += 1
    save_path = "images/image"+str(camera.image_count)+".jpg"
    frame = camera.get_frame()
    if frame is not None:
        # Save the captured frame to the specified path
        cv2.imwrite(save_path, frame)
        return jsonify({"message": "Image saved successfully", "path": save_path}), 200
    else:
        return jsonify({"message": "Failed to capture image"}), 500

if __name__ == '__main__':
    # image width and height here should align with apriltag_streamer.py
    camera_id = 0
    image_width = 1280
    image_height = 720
    camera = Camera(camera_id, image_width, image_height)
    atexit.register(camera.cleanup)
    signal.signal(signal.SIGINT, signal_handler)  # Register the signal handler
    app.run(host='0.0.0.0', port=5001)
