<img align="right" height="60" src="https://user-images.githubusercontent.com/5248102/126074528-004a32b9-7911-486a-9e79-8b78e6e66fdc.png">

# adaptive_snowsampler

[![ROS1 Build Test](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/actions/workflows/build_test.yml/badge.svg)](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/actions/workflows/build_test.yml)

![rviz](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/assets/5248102/117a296d-01ad-4209-bec7-fb14267628e0)
## Installation
```
cd ~
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Jaeyoung-Lim/adaptive-snowsampler.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/ethz-asl/grid_map_geo.git
git clone https://github.com/ethz-asl/mav_comm.git -b ros2
git clone https://github.com/ethz-asl/terrain-navigation.git -b ros2
git clone https://github.com/ros-drivers/transport_drivers.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to snowsampler_rviz adaptive_snowsampler ssp_bridge


# post building tasks
echo 'ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="fc5f", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-actuonix.rules #setting usb permissions
sudo usermod -aG dialout # add user to the dailout group
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
ros2 service call /snowsampler/set_landing_leg_angle snowsampler_msgs/srv/SetAngle '{"angle":35}'
```
