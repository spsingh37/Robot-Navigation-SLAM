# MBot LCM Base

This repo contains some essential subpackages needed by all other MBot applications running off LCM:
* [mbot_lcm_serial](mbot_lcm_serial): LCM-to-serial communication with the MBot control board.
* [mbot_msgs](mbot_msgs): Message type definitions for LCM.

## Fast Install

To build and install code and services, you can use the install script:
```bash
./scripts/install.sh
```
You do not need to follow any of the other build steps if you use the install script.

## Build instructions

```bash
mkdir build && cd build
cmake ..
make
```

You will likely want to install the MBot message types so they can be used across the system for other packages. To do this, do `sudo make install`.

Finally, in order for the message definitions to be accessible on the terminal for lcm-spy in the future, run this command:
```bash
echo 'export CLASSPATH=$CLASSPATH:"/usr/local/share/java/mbot_lcm_msgs.jar"' >> ~/.bashrc
```

## Uninstalling

You can uninstall with:
```bash
cd build
sudo xargs rm < install_manifest.txt
```
