#!/bin/bash

set -e

echo "This script will configure your system"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

PACKAGE_PATH=/home/user/ros2_ws/src/adaptive-snowsampler

# Configure systemd service
echo "Copy systemd service"
cp -vf $PACKAGE_PATH/systemd/adaptive-snowsampler.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/microxrce_agent.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/rosbag-record.service /etc/systemd/system/

systemctl enable adaptive-snowsampler.service
systemctl enable microxrce_agent.service
systemctl enable rosbag-record.service

sudo systemctl start adaptive-snowsampler.service
sudo systemctl start microxrce_agent.service
