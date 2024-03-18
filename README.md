<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# adaptive_snowsampler

[![ROS1 Build Test](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/actions/workflows/build_test.yml/badge.svg)](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/actions/workflows/build_test.yml)

![rviz](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/assets/5248102/117a296d-01ad-4209-bec7-fb14267628e0)
## Installation
```
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/Jaeyoung-Lim/adaptive-snowsampler.git -b ros1
git clone https://github.com/ethz-asl/grid_map_geo.git
git clone https://github.com/ethz-asl/mav_comm.git
git clone https://github.com/ethz-asl/terrain-navigation.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin build 


# post building tasks
echo 'ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="fc5f", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-actuonix.rules #setting usb permissions
sudo usermod -aG dialout user # add user to the dailout group
sudo reboot # needed for the usb permissions to take effect

# installation of GeographicLib dependency
cd ~
git clone https://github.com/mavlink/mavros.git
sudo apt install geographiclib-tools libgeographic-dev
sudo ~/mavros/mavros/scripts/install_geographiclib_datasets.sh 

```

## Running the code
Run the code with the following launch file
```
source ~/ros2_ws/install/setup.bash
ros2 launch adaptive_snowsampler launch.xml
```

## Testing with PX4 Software-In-The-Loop(SITL) simulation

Run the simulation instance
```
export PX4_HOME_LAT=46.785479
export PX4_HOME_LON=9.846803
export PX4_HOME_ALT=2301.23
make px4_sitl gz_x500
```

Run the micro-ros-agent
```
micro-ros-agent udp4 --port 8888
```
Run the node
```
ros2 launch adaptive_snowsampler launch.xml
```

## Setting the leg angle manually:
```
rosservice call /snowsampler/set_landing_leg_angle  "35.0"
```

## Running the ground station

To control the vehicle from the ground, we need to connect to the ROS Master on the drone.
Run rviz with the following command.
```
ROS_MASTER_URI=http://172.30.132.111:11311
roslaunch snowsampler_rviz run.launch
```
