# adaptive_snowsampler

![simplescreenrecorder-2024-01-22_15 43 08-ezgif com-video-to-gif-converter](https://github.com/Jaeyoung-Lim/adaptive-snowsampler/assets/5248102/66a692aa-e3e2-44a6-bdb5-1abd35ce0e69)

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
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to adaptive_snowsampler

# post building tasks
echo 'ATTRS{idVendor}=="04d8", ATTRS{idProduct}=="fc5f", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-actuonix.rules #setting usb permissions
sudo usermod -aG dialout # add user to the dailout group
sudo reboot # needed for the usb permissions to take effect

# installation of GeographicLib dependency

cd ~
git clone https://github.com/mavlink/mavros/tree/ros2
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
