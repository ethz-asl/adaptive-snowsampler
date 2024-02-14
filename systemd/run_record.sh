#!/bin/bash

set -e


gst-launch-1.0 -e rtspsrc location=rtsp://127.0.0.1:8553/stream protocols=tcp ! rtph264depay ! h264parse ! mp4mux ! filesink location=~/rosbag/onboard_$(date +"%Y_%m_%d_%H_%M_%S").mp4&

source $HOME/ros2_ws/install/setup.bash
ros2 bag record -o ~/rosbag/bag_$(date +"%Y_%m_%d_%H_%M_%S") /snowsampler/landing_leg_angle /target_slope /target_normal /tf /fmu/out/vehicle_global_position /fmu/out/vehicle_attitude /elevation_map
