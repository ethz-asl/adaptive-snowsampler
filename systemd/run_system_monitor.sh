#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch system_monitor_ros system_monitor.launch
