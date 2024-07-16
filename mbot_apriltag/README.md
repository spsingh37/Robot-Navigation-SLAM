# mbot_apriltag

## Description
This project offers tools for camera calibration and Apriltag pose estimation, specifically tailored for use with the MBot equipped with a **Jetson Nano**.

The provided scripts enable users to:
- Stream live camera feeds directly to a browser, allowing for real-time viewing and interaction.
- Publish and subscribe to Apriltag LCM messages.
## Installation
### Install Apriltag Library
1. Directly install from the official [github repo](https://github.com/AprilRobotics/apriltag):
    ```bash
    git clone https://github.com/AprilRobotics/apriltag.git
    ```

2. Following the install [instruction](https://github.com/AprilRobotics/apriltag) there:
    ```bash
    cd apriltag
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    sudo cmake --build build --target install
    ```

### Clone this repo
```bash
git clone https://github.com/MBot-Project-Development/mbot_apriltag.git
```

## Usage and Features
### Run scripts
- `python3 video_streamer.py`
    - Visit `http://your_mbot_ip:5001/video`
    - Only shows the video stream to test your camera
- `python3 save_image.py`
    - Visit `http://your_mbot_ip:5001`
    - Show the video stream to save image to `/images` for camera calibration
- `python3 camera_calibration.py`
    - Use the images from `/images` and output calibration result as `cam_calibration_data.npz`. The result will be used directly by apriltag_streamer.py you don't have to modify anything.
- `python3 apriltag_streamer.py`
    - Visit `http://your_mbot_ip:5001/video`
    - It runs apriltag detection, when tag is detected, pose estimation will be printed on the screen.
- `python3 apriltag_lcm_publisher.py`
    - Publish apriltag lcm message over `MBOT_APRILTAG_ARRAY`
- `python3 apriltag_lcm_subscriber.py`
    - Listen to `MBOT_APRILTAG_ARRAY` for apriltag lcm message

### Troubleshooting
If encounter error during runtime:"ImportError: libapriltag.so.3: cannot open shared object file: No such file or directory"

1. Verify the Installation Location

    This file is typically installed in a directory like /usr/local/lib or /usr/lib. 
    ```bash
    ls /usr/local/lib | grep libapriltag
    ```
    ```bash
    ls /usr/lib | grep libapriltag
    ```
    - If there is output showing "libapriltag.so.3", we move to the next step

2. Update the Library Cache

    If the library is installed in a standard location but still not found, the system's library cache may need to be updated. Run ldconfig to refresh the cache:

    ```bash
    sudo ldconfig
    ```

## Authors and maintainers
- The original author of this project is Shaw Sun.
- The current maintainer of this project is Shaw Sun. Please direct all questions regarding support, contributions, and issues to the maintainer. The maintainer is responsible for overseeing the project's development, reviewing and merging contributions, and ensuring the project's ongoing stability and relevance.
