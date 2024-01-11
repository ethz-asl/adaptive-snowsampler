# adaptive_snowsampler

## Installation
```
cd ~
mkdir ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Jaeyoung-Lim/adaptive-snowsampler.git
git clone https://github.com/PX4/px4_msgs.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to adaptive_snowsampler
```

## Running the code
Run the code with the following launch file
```
ros2 launch adaptive_snowsampler launch.xml
```

## Testing with PX4 Software-In-The-Loop(SITL) simulation

Run the simulation instance
```
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
