#!/bin/bash
set -e  # Quit on error.

echo "Building the MBot Autonomy code..."
echo
# Build the code.
if [ ! -d "build/" ]; then
    mkdir build
fi
cd build
cmake ..
make
sudo make install
cd ..

SERVICE_LIST="mbot-motion-controller
              mbot-slam"

# Copy the services.
for serv in $SERVICE_LIST
do
    sudo cp services/$serv.service /etc/systemd/system/$serv.service
done

# Enable the services.
sudo systemctl daemon-reload
for serv in $SERVICE_LIST
do
    sudo systemctl enable $serv.service
    sudo systemctl restart $serv.service
done

# Success message.
echo
echo "Installed and enabled the following services:"
echo
for serv in $SERVICE_LIST
do
    echo "    $serv.service"
done
echo

echo "Done!"
