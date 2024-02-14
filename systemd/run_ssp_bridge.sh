#!/bin/bash

set -e

source $HOME/ros2_ws/install/setup.bash
ros2 launch ssp_bridge launch.xml
