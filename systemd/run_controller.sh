#!/bin/bash

set -e

source $HOME/ros2_ws/install/setup.bash
<<<<<<< HEAD
ros2 launch adaptive_snowsampler launch_nuc.xml
=======
ros2 launch adaptive_snowsampler adaptive_snowsampler.launch
>>>>>>> 6d4bc1e74f26c130ceeebe4a1f3ae44725bd7f9b
