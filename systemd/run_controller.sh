#!/bin/bash

set -e

source $HOME/catkin_ws/devel/setup.bash
roslaunch adaptive_snowsampler run.launch
