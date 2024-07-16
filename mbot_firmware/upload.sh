#!/bin/bash

EXPECTED_ARGS=2
E_BADARGS=65

if [ $# -lt 1 ] || [ $# -gt 2 ]; 
then
  echo "Usage: $0 [load | run | flash | disable] <uf2_file>"
  exit $E_BADARGS
fi

OPERATION=$1
UF2_FILE=$2

# GPIO Numbers, 
# on Jetson, pin 7 is gpio216 and pin 11 is gpio50
# on RPi pin 7 is gpio4, pin 11 is gpio17
if grep -q "Raspberry Pi" /proc/device-tree/model; then
  echo "Detected Raspberry Pi, entering $1 mode..."
  BTLD_PIN=4
  RUN_PIN=17
elif grep -q "NVIDIA Jetson" /proc/device-tree/model; then
  echo "Detected NVIDIA Jetson, entering $1 mode..."
  BTLD_PIN=50
  RUN_PIN=216
else
  echo "ERROR: Unknown hardware!"
  exit 1
fi

# Check if BTLD_PIN is exported and set direction if necessary
if [ ! -e /sys/class/gpio/gpio$BTLD_PIN ]; then
    echo $BTLD_PIN > /sys/class/gpio/export
    sleep 0.1
    echo out > /sys/class/gpio/gpio$BTLD_PIN/direction
elif [ "$(cat /sys/class/gpio/gpio$BTLD_PIN/direction)" != "out" ]; then
    echo out > /sys/class/gpio/gpio$BTLD_PIN/direction
fi

# Check if RUN_PIN is exported and set direction if necessary
if [ ! -e /sys/class/gpio/gpio$RUN_PIN ]; then
    echo $RUN_PIN > /sys/class/gpio/export
    sleep 0.1
    echo out > /sys/class/gpio/gpio$RUN_PIN/direction
elif [ "$(cat /sys/class/gpio/gpio$RUN_PIN/direction)" != "out" ]; then
    echo out > /sys/class/gpio/gpio$RUN_PIN/direction
fi

sleep 0.1

case "$OPERATION" in
    "load"|"run"|"flash"|"disable")
        # notice that for all cases, we consdier the RUN_PIN is high
        # which set by services/mbot_start_networking.py 
        case "$OPERATION" in
            "load")
                echo "Loading action for $UF2_FILE..."
                if [ -n "$UF2_FILE" ]; then
                    echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                    sleep 0.1
                    echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
                    sleep 0.5
                    echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                    sleep 0.5
                    sudo picotool load $UF2_FILE
                    sleep 0.5
                else
                    echo "Missing UF2 file for 'load' operation."
                fi
                ;;
            "run")
                echo "Running action for loaded UF2_FILE..."
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 0.1
                echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 1
                ;;
            "flash")
                echo "Flashing action for $UF2_FILE..."
                if [ -n "$UF2_FILE" ]; then
                    echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                    sleep 0.1
                    echo 0 > /sys/class/gpio/gpio$BTLD_PIN/value
                    sleep 0.5
                    echo 1 > /sys/class/gpio/gpio$RUN_PIN/value
                    sleep 0.5
                    sudo picotool load $UF2_FILE
                    sleep 0.5
                    echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
                    sleep 0.5
                    sudo picotool reboot
                else
                    echo "Missing UF2 file for 'flash' operation."
                fi
                ;;
            "disable")
                echo "Disable action..."    
                echo 1 > /sys/class/gpio/gpio$BTLD_PIN/value
                sleep 0.5
                echo 0 > /sys/class/gpio/gpio$RUN_PIN/value
                sleep 1
                ;;
        esac
        ;;
    *)
        echo "'$OPERATION' is not a valid operation. Please provide one of the following operations: load, run, flash, disable"
        ;;
esac
