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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
