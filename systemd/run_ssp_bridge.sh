#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch ssp_bridge run.launch
