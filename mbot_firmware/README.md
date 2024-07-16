# mbot_firmware
Firware to run on the RPi Pico for the MBot.

## Installation

After cloning the repo, cd into the mbot_firmware directory.
Then, run the setup script:

```bash
./setup.sh
```

Which will install dependencies (requires sudo password) and initialize the submodules.

If setup.sh is not executable by default, do the following to enable it:

```bash
sudo chmod +x setup.sh
```

## Building

Build as follows:
```bash
mkdir build
cd build
cmake ..
make
```

## Uploading firmware to the pico
There are a few ways to upload firmware, using the bootload partition on the pico, using picotool or using the upload.sh script and openocd.

### Upload Script
Uplpad using the upload.sh script.  You'll need a cable plugged in between the GPIO ports on the Jetson or RPi and the BTLD and RUN pins on the MBot Control Board.

```bash
./upload /build/src/mbot.uf2
```

### Bootloader
Plug in the pico while holding the BOOTLOAD button.  The pico will be mounted as a drive.  Copy the mbot.uf2 file over to the drive.

### picotool
Plug in the pico while holding the BOOTLOAD button.  Run:
```bash
picotool load build/src/mbot.uf2
picotool reboot
```

### upload_swd.sh and openocd
[TODO: Check if we can run on pin7 and pin11 instead]
Run the upload script which uses openocd.  This does not require puting the Pico into bootloader mode.  You must have the SWD wires (SWDIO and SWCLK) connected to GPIO 24 (Pin 18), GND (Pin 20) and GPIO 25 (Pin 22) on the Raspberry Pi.  Note, when using the upload script and openocd, you upload the .elf firmware file, not the .uf2 firmware file, they are just a different format, but the same firmware.
 ```bash
 upload.sh build/src/mbot.elf
 ```

## Using GDB with the Piso and Raspberry Pi

If the SWD wires are connected to a Raspberry Pi (see above section on main branch), you can use GDB on the Pico to help debug your current firmware or test program.

Note: make sure that the elf file was built using the `-DCMAKE_BUILD_TYPE=Debug` flag. If the file was built for Release, the optimization means most things won't be visible when you use the print command in gdb later.

## Installing openocd and gdb-multiarch
Note: We should add this to the setub.sh script if it is run on a RPi
```bash
sudo apt-get install libftdi-dev gdb-multiarch
git clone https://github.com/raspberrypi/openocd.git --recursive --branch rp2040 --depth=1
cd openocd
./bootstrap
./configure --enable-ftdi --enable-sysfsgpio --enable-bcm2835gpio
make -j4
sudo make install
```

Build as normal, then upload using:
```bash
upload_swd.sh path/to/elf/file.elf
```

In one window, run:
```bash
debug.sh path/to/elf/file.elf
```

Which starts up the GDB server and blocks this terminal until you Ctrl+C to quit the server. In another terminal, run:

```bash
debug_attach.sh path/to/elf/file.elf
```

Which attaches to the server and stops the program at the start of the main function, from where you can start debugging with GDB commands.

[TODO: get this working in VSCode]
