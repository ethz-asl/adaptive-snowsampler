#!/bin/bash

set -e

echo "This script will configure your system"

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

PACKAGE_PATH=/home/user/catkin_ws/src/adaptive-snowsampler

# Configure systemd service
echo "Copy systemd service"
cp -vf $PACKAGE_PATH/systemd/adaptive-snowsampler.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/mavlink-router.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/rosbag-record.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/ssp-bridge.service /etc/systemd/system/
cp -vf $PACKAGE_PATH/systemd/system-monitor.service /etc/systemd/system/

systemctl enable adaptive-snowsampler.service
systemctl enable mavlink-router.service
systemctl enable ssp-bridge.service
systemctl enable system-monitor.service

sudo systemctl start adaptive-snowsampler.service
sudo systemctl start mavlink-router.service
sudo systemctl start ssp-bridge.service
sudo systemctl start system-monitor.service