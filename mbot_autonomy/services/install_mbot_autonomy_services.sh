#!/bin/bash
set -e  # Quit on error.
sudo cp mbot-motion-controller.service /etc/systemd/system/
sudo cp mbot-slam.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable mbot-slam.service
sudo systemctl enable mbot-motion-controller.service
