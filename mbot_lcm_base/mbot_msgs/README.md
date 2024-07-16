# mbot_msgs
Message definitions for the MBot

## Installation

To compile and install these messages from source, in the root of this repo, do:
```bash
cmake ..
make
sudo make install
```

*Note:* The Python messages will be installed in the current Python 3 library path. If you want to install the messages in a virtual environment, make sure it is activated before running `make install`.

To uninstall, in the build folder, do: `sudo xargs rm < install_manifest.txt`

## Usage

To use the messages in another CMake package (i.e. for C or C++ code), find it in the CMakeLists.txt:

```cmake
find_package(mbot_lcm_msgs REQUIRED)
```

## mbot_firmware

Currently, the files mbot_lcm_msgs_serial.c and mbot_lcm_msgs_serial.h need to be copied over to mbot_firmware/comms/src and include respectively.