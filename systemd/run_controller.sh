#!/bin/bash

set -e

source $HOME/ros2_ws/install/setup.bash
ros2 launch adaptive_snowsampler launch_nuc.xml
